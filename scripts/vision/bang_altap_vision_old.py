import os
import cv2
import sys
import time
import serial
import requests
import argparse
from datetime import datetime
import numpy as np
from shapely.geometry import Point, Polygon
from pathlib import Path
import json
import threading
import time

# Add path
realpath = os.path.abspath(__file__)
_sep = os.path.sep
realpath = realpath.split(_sep)
sys.path.append(os.path.join(realpath[0]+_sep, *realpath[1:realpath.index('rknn_model_zoo')+1]))

from py_utils.coco_utils import COCO_test_helper

OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = (640, 640)

CLASSES = ("bicycle", "bus", "car", "minibus", "motorcycle", "transjakarta", "truck")
VALID_VEHICLES = ["transjakarta"]
API_ENDPOINT = "http://localhost:3030/api/logs"

def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with object threshold."""
    box_confidences = box_confidences.reshape(-1)
    candidate, class_num = box_class_probs.shape

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)

    _class_pos = np.where(class_max_score * box_confidences >= OBJ_THRESH)
    scores = (class_max_score * box_confidences)[_class_pos]

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]

    return boxes, classes, scores

def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes."""
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep

def dfl(position):
    # Distribution Focal Loss (DFL)
    import torch
    x = torch.tensor(position)
    n, c, h, w = x.shape
    p_num = 4
    mc = c//p_num
    y = x.reshape(n, p_num, mc, h, w)
    y = y.softmax(2)
    acc_metrix = torch.tensor(range(mc)).float().reshape(1, 1, mc, 1, 1)
    y = (y*acc_metrix).sum(2)
    return y.numpy()

def box_process(position):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    grid = np.concatenate((col, row), axis=1)
    stride = np.array([IMG_SIZE[1]//grid_h, IMG_SIZE[0]//grid_w]).reshape(1,2,1,1)

    position = dfl(position)
    box_xy = grid + 0.5 - position[:,0:2,:,:]
    box_xy2 = grid + 0.5 + position[:,2:4,:,:]
    xyxy = np.concatenate((box_xy*stride, box_xy2*stride), axis=1)

    return xyxy

def post_process(input_data):
    boxes, scores, classes_conf = [], [], []
    default_branch = 3
    pair_per_branch = len(input_data)//default_branch

    for i in range(default_branch):
        boxes.append(box_process(input_data[pair_per_branch*i]))
        classes_conf.append(input_data[pair_per_branch*i+1])
        scores.append(np.ones_like(input_data[pair_per_branch*i+1][:,:1,:,:], dtype=np.float32))

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0,2,3,1)
        return _in.reshape(-1, ch)

    boxes = [sp_flatten(_v) for _v in boxes]
    classes_conf = [sp_flatten(_v) for _v in classes_conf]
    scores = [sp_flatten(_v) for _v in scores]

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    boxes, classes, scores = filter_boxes(boxes, scores, classes_conf)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s)

        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores

def setup_model(args):
    model_path = args.model_path
    if model_path.endswith('.pt') or model_path.endswith('.torchscript'):
        platform = 'pytorch'
        from py_utils.pytorch_executor import Torch_model_container
        model = Torch_model_container(args.model_path)
    elif model_path.endswith('.rknn'):
        platform = 'rknn'
        from py_utils.rknn_executor import RKNN_model_container 
        model = RKNN_model_container(args.model_path, args.target, args.device_id)
    elif model_path.endswith('onnx'):
        platform = 'onnx'
        from py_utils.onnx_executor import ONNX_model_container
        model = ONNX_model_container(args.model_path)
    else:
        assert False, "{} is not rknn/pytorch/onnx model".format(model_path)
    print('Model-{} is {} model, starting val'.format(model_path, platform))
    return model, platform
class VideoDetector:
    def __init__(self, model, platform, serial_port='/dev/ttyUSB0', show_output=False, save_output=False):
        self.model = model
        self.platform = platform
        self.co_helper = COCO_test_helper(enable_letter_box=True)
        self.first_loop = True
        self.current_video_name = None
        self.frame_count = 0
        self.working_area = None
        self.last_update = 0
        self.update_interval = 60
        self.show_output = show_output
        self.save_output = save_output
        self.output_writer = None
        self.frame_width = None
        self.frame_height = None
        
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            print(f"Serial connection established on {serial_port}")
        except:
            print("Warning: Could not establish serial connection")
            self.ser = None

        # Start working area update thread
        self.update_thread = threading.Thread(target=self.update_working_area_loop, daemon=True)
        self.update_thread.start()
        
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            print(f"Serial connection established on {serial_port}")
        except:
            print("Warning: Could not establish serial connection")
            self.ser = None

        self.working_area = None

    def setup_save_directory(self, video_path):
        """Setup directory for saving detection results"""
        if video_path:
            # Extract filename without extension
            video_name = Path(video_path).stem
            # Create directory if it doesn't exist
            save_dir = f"detection_results_{video_name}"
            os.makedirs(save_dir, exist_ok=True)
            return save_dir, video_name
        return None, None

    def save_detection_frame(self, frame, video_name, frame_num):
        """Save detection frame if in first loop"""
        if self.first_loop and video_name:
            save_dir = f"detection_results_{video_name}"
            save_path = os.path.join(save_dir, f"frame_{frame_num:04d}.jpg")
            cv2.imwrite(save_path, frame)
            # print(f"Saved detection result: {save_path}")

    def set_working_area(self, points):
        """Set the working area polygon"""
        self.working_area = Polygon(points)

    def is_in_working_area(self, box):
        """Check if the center of the box is in the working area"""
        if self.working_area is None:
            return False  # Changed from True to False to ensure we only detect in defined areas
        
        # Calculate center point of the box
        center_x = (box[0] + box[2]) / 2
        center_y = (box[1] + box[3]) / 2
        point = Point(center_x, center_y)
        
        return self.working_area.contains(point)
    
    def setup_video_writer(self, frame_width, frame_height, fps):
        """Setup video writer for output"""
        if self.save_output:
            output_filename = 'output.mp4'
            if self.current_video_name:
                output_filename = f'output_{self.current_video_name}.mp4'
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.output_writer = cv2.VideoWriter(
                output_filename, fourcc, fps,
                (frame_width, frame_height)
            )

    def draw_working_area(self, frame):
        """Draw working area on frame"""
        if self.working_area:
            # Get coordinates of the polygon
            coords = np.array(self.working_area.exterior.coords)
            # Convert to integer points
            pts = coords.reshape((-1, 1, 2)).astype(np.int32)
            # Draw filled polygon with semi-transparency
            overlay = frame.copy()
            cv2.fillPoly(overlay, [pts], (0, 255, 0))
            # Combine with original frame
            alpha = 0.3
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
            # Draw polygon outline
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

    def send_serial_message(self, message):
        """Send message through serial port"""
        if self.ser:
            try:
                self.ser.write(message.encode())
            except:
                print("Failed to send serial message")
    def load_working_area(self):
        """Load working area from API and save local copy"""
        try:
            print("Attempting to load working area from API...")
            response = requests.get('http://localhost:3030/api/working-area')
            print(f"API Response Status: {response.status_code}")
            print(f"API Response: {response.text}")
            
            if response.status_code == 200:
                config = response.json()
                
                # Save local copy
                try:
                    with open('working_area.json', 'w') as f:
                        json.dump(config, f, indent=2)
                    print("Successfully saved working_area.json")
                except Exception as e:
                    print(f"Error saving working_area.json: {str(e)}")
                
                if self.frame_width is None or self.frame_height is None:
                    print("Frame dimensions not set")
                    print(f"Current dimensions: {self.frame_width}x{self.frame_height}")
                    return False

                if config['type'] == 'polygon':
                    points = [(p['x'] * self.frame_width, p['y'] * self.frame_height) 
                            for p in config['polygon']]
                    self.working_area = Polygon(points)
                    print(f"Created polygon with points: {points}")
                else:  # boundingBox
                    box = config['bounding_box']
                    points = [(p['x'] * self.frame_width, p['y'] * self.frame_height) 
                            for p in box['points']]
                    self.working_area = Polygon(points)
                    print(f"Created bounding box with points: {points}")
                
                return True
                
        except requests.exceptions.ConnectionError:
            print("Could not connect to API server at http://localhost:3030")
        except requests.exceptions.RequestException as e:
            print(f"API request failed: {str(e)}")
        except Exception as e:
            print(f"Unexpected error loading working area: {str(e)}")
            print(f"Error type: {type(e)}")
            import traceback
            traceback.print_exc()
        
        # Try loading from local file if API fails
        try:
            if os.path.exists('working_area.json'):
                print("Found existing working_area.json, attempting to load...")
                with open('working_area.json', 'r') as f:
                    config = json.load(f)
                print("Successfully loaded working_area.json")
                
                if self.frame_width is None or self.frame_height is None:
                    print("Frame dimensions not set for local file")
                    return False

                if config['type'] == 'polygon':
                    points = [(p['x'] * self.frame_width, p['y'] * self.frame_height) 
                            for p in config['polygon']]
                    self.working_area = Polygon(points)
                    print(f"Created polygon from file with points: {points}")
                else:  # boundingBox
                    box = config['bounding_box']
                    points = [(p['x'] * self.frame_width, p['y'] * self.frame_height) 
                            for p in box['points']]
                    self.working_area = Polygon(points)
                    print(f"Created bounding box from file with points: {points}")
                return True
            else:
                print("No working_area.json file found")
        except Exception as e:
            print(f"Error loading from working_area.json: {str(e)}")
            import traceback
            traceback.print_exc()
        
        return False

    def update_working_area_loop(self):
        """Periodically update working area"""
        while True:
            self.load_working_area()
            time.sleep(self.update_interval)


    def is_in_working_area(self, box):
        """Check if the center of the box is in the working area"""
        if self.working_area is None:
            return False
        
        # Calculate center point of the box
        center_x = (box[0] + box[2]) / 2
        center_y = (box[1] + box[3]) / 2
        point = Point(center_x, center_y)
        
        return self.working_area.contains(point)

    def send_api_request(self, image, event_type, detection_class):
        """Send detection event to API with image upload"""
        try:
            # Convert OpenCV image to jpg format in memory
            _, img_encoded = cv2.imencode('.jpg', image)
            
            # Create file-like object
            files = {
                'image': ('detection.jpg', img_encoded.tobytes(), 'image/jpeg')
            }

            # Prepare form data
            data = {
                'plateNumber': 'Unidentified',
                'eventType': event_type,
                'location': 'Cawang, South Jakarta',
                'vehicleType': detection_class,
                'timestamp': datetime.now().isoformat(),
                'description': f'{detection_class} detected',
                'direction': 'UNKNOWN',  # Add direction if you have it
                'confidence': '95'  # Example confidence value
            }
            
            # Send multipart form request
            response = requests.post(
                API_ENDPOINT,
                files=files,
                data=data
            )

            if response.status_code == 201:
                print(f"Successfully logged {event_type} event")
            
        except Exception as e:
            pass  # Silently handle errors as requested
            
    def process_frame(self, frame):
        """Process a single frame and return annotated frame"""
        img_src = frame
        if img_src is None:
            return None

        height, width = img_src.shape[:2]

        img = self.co_helper.letter_box(im=img_src.copy(), new_shape=(IMG_SIZE[1], IMG_SIZE[0]), pad_color=(0,0,0))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        if self.platform in ['pytorch', 'onnx']:
            input_data = img.transpose((2,0,1))
            input_data = input_data.reshape(1,*input_data.shape).astype(np.float32)
            input_data = input_data/255.
        else:
            input_data = img

        outputs = self.model.run([input_data])
        boxes, classes, scores = post_process(outputs)

        valid_detected = False
        
        if boxes is not None:
            img_annotated = img_src.copy()
            
            # Draw working area first
            self.draw_working_area(img_annotated)
            
            real_boxes = self.co_helper.get_real_box(boxes)
            
            for box, score, cl, real_box in zip(boxes, scores, classes, real_boxes):
                if not self.is_in_working_area(real_box):
                    continue
                
                class_name = CLASSES[cl]
                top, left, right, bottom = [int(b) for b in real_box]
                cv2.rectangle(img_annotated, (top, left), (right, bottom), (255, 0, 0), 2)
                cv2.putText(img_annotated, f'{class_name} {score:.2f}',
                           (top, left - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                if class_name in VALID_VEHICLES:
                    valid_detected = True
                    print(f"Valid vehicle detected: {class_name} (confidence: {score:.2f})")
                    self.send_serial_message("x1")
                    self.send_api_request(img_annotated, "VALID", class_name)
                else:
                    self.send_api_request(img_annotated, "TRESPASSING", class_name)

            if not valid_detected:
                self.send_serial_message("x0")
            
            if self.output_writer:
                self.output_writer.write(img_annotated)
                
            return img_annotated
        return img_src

    def process_video(self, source=0):
        """Process video stream from file or camera"""
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print(f"Error: Could not open video source {source}")
            return

        # Set frame dimensions first
        self.frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        
        if self.save_output:
            self.setup_video_writer(self.frame_width, self.frame_height, fps)
        
        is_video_file = isinstance(source, str)
        if is_video_file:
            print(f"Processing video file: {source}")
            self.save_dir, self.current_video_name = self.setup_save_directory(source)
        else:
            print("Processing webcam feed")
            self.save_dir = None
            self.current_video_name = None

        # Load initial working area
        self.load_working_area()
        
        is_video_file = isinstance(source, str)
        
        if is_video_file:
            print(f"Processing video file: {source}")
            self.save_dir, self.current_video_name = self.setup_save_directory(source)
            print(f"Saving first loop results to: detection_results_{self.current_video_name}/")
        else:
            print("Processing webcam feed")
            self.save_dir = None
            self.current_video_name = None

        
        try:
            while True:
                ret, frame = cap.read()
                
                if not ret:
                    if is_video_file:
                        print("\nRestarting video...")
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        self.first_loop = False
                        self.frame_count = 0
                        ret, frame = cap.read()
                        if not ret:
                            print("Error: Could not read video file")
                            break
                    else:
                        print("Error: Could not read from camera")
                        break

                self.frame_count += 1
                processed_frame = self.process_frame(frame)
                
                if processed_frame is not None:
                    if self.show_output:
                        cv2.imshow('Detection', processed_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            cap.release()
            if self.output_writer:
                self.output_writer.release()
            if self.show_output:
                cv2.destroyAllWindows()
            if self.ser:
                self.ser.close()

def setup_video_detector(args):
    """Setup the video detector with the specified model"""
    model, platform = setup_model(args)
    return VideoDetector(model, platform, args.serial_port, args.show, args.save)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', type=str, required=True, help='model path')
    parser.add_argument('--target', type=str, default='rk3588', help='target RKNPU platform')
    parser.add_argument('--device_id', type=str, default=None, help='device id')
    parser.add_argument('--video_source', type=str, default='0', 
                        help='video source (0 for webcam, or path to video file)')
    parser.add_argument('--serial_port', type=str, default='/dev/ttyUSB0',
                        help='serial port for ESP32 communication')
    parser.add_argument('--show', action='store_true', default=False,
                        help='show video output')
    parser.add_argument('--save', action='store_true', default=False,
                        help='save video output')

    args = parser.parse_args()
    
    if args.video_source.isdigit():
        args.video_source = int(args.video_source)

    detector = setup_video_detector(args)
    detector.process_video(args.video_source)