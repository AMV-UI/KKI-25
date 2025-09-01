# import cv2
# import numpy as np
# import matplotlib.pyplot as plt
# from ultralytics import YOLO

# class TRTV8:
#     def __init__(self, model_path):
#         """Initialize the YOLOv8 model."""
#         self.model = YOLO(model_path)
#         self.classes_names = self.model.names  # Get class names
#         self.colors = plt.get_cmap('tab20b')(np.linspace(0, 1, len(self.classes_names)))[:, :3]  # Colors for classes

#     def preprocess_image(self, img):
#         """Preprocess the image for inference."""
#         img_resized = cv2.resize(img, (640, 640))  # Resize to 640x640 (or the model input size)
#         img_resized = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
#         return img_resized

#     def predict(self, img):
#         """Detect objects in the input image."""
#         img_resized = self.preprocess_image(img)
#         results = self.model(img_resized)  # Perform inference

#         boxes = []
#         scores = []
#         classes = []

#         for result in results:
#             for box in result.boxes:
#                 boxes.append(box.xyxy.cpu().numpy())  # Get bounding box coordinates
#                 scores.append(box.conf.cpu().numpy())  # Get confidence scores
#                 classes.append(box.cls.cpu().numpy())   # Get class IDs

#         boxes = np.array(boxes).reshape(-1, 4)
#         scores = np.array(scores).flatten()
#         classes = np.array(classes).flatten()

#         # Filter out low-confidence detections
#         conf_th = 0.25  # Confidence threshold
#         keep = scores >= conf_th
#         return boxes[keep], scores[keep], classes[keep]

#     def draw(self, frame, boxes, confs, clss):
#         """Draw bounding boxes and labels on the frame."""
#         for bb, cf, cl in zip(boxes, confs, clss):
#             cl = int(cl)
#             x_min, y_min, x_max, y_max = map(int, bb)
#             color = self.colors[cl]
#             cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
#             label = f"{self.classes_names[cl]} - {round(cf, 2)}"
#             cv2.putText(frame, label, (x_min, y_min - 10), 0, 0.75, (255, 255, 255), 2)

# # # Example usage:
# # if __name__ == "__main__":
# #     detector = YOLOv8Detector('yolov8n.pt')  # Load your YOLOv8 model

# #     # Read an image
# #     img = cv2.imread('path_to_image.jpg')

# #     # Perform detection
# #     boxes, scores, classes = detector.predict(img)

# #     # Draw results
# #     detector.draw(img, boxes, scores, classes)

# #     # Display the result
# #     cv2.imshow('Detections', img)
# #     cv2.waitKey(0)
# #     cv2.destroyAllWindows()



import ctypes
import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
import matplotlib.pyplot as plt
import rospy
from config import Param

# Load YOLOv8 TensorRT plugins
try:
    ctypes.cdll.LoadLibrary('./plugins/libyolo_layer.so')
except OSError as e:
    raise SystemExit('ERROR: failed to load ./plugins/libyolo_layer.so. '
                     'Did you forget to do a "make" in the "./plugins/" subdirectory?') from e


def _preprocess_yolo(img, input_shape):
    """Preprocess an image for YOLOv8 inference."""
    img = cv2.resize(img, (input_shape[1], input_shape[0]))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.transpose((2, 0, 1)).astype(np.float32)
    img /= 255.0
    return img


def _postprocess_yolo(trt_outputs, img_w, img_h, conf_th, nms_threshold):
    """Postprocess YOLOv8 outputs."""
    # Assume trt_outputs is a single tensor of shape [N, num_anchors, 85]
    # where 85 = 4 (bbox) + 1 (objectness score) + num_classes (80)
    
    detections = trt_outputs[0].reshape(-1, 85)
    detections = detections[detections[:, 4] >= conf_th]

    if len(detections) == 0:
        return np.zeros((0, 4)), np.zeros((0,)), np.zeros((0,))

    # Scale boxes back to image size
    detections[:, :4] *= np.array([img_w, img_h, img_w, img_h], dtype=np.float32)
    
    # Apply NMS
    boxes, scores, classes = [], [], []
    for cls_id in range(detections.shape[1] - 5):
        cls_detections = detections[detections[:, 5] == cls_id]
        keep = _nms_boxes(cls_detections, nms_threshold)
        boxes.append(cls_detections[keep])
    
    boxes = np.concatenate(boxes, axis=0)
    if boxes.size > 0:
        boxes = boxes[:, :4]
        scores = boxes[:, 4]
        classes = boxes[:, 5]

    return boxes.astype(np.int32), scores, classes


class HostDeviceMem(object):
    """Helper data class for managing host/device memory."""
    def __init__(self, host_mem, device_mem):
        self.host = host_mem
        self.device = device_mem

    def __del__(self):
        del self.device
        del self.host


def allocate_buffers(engine):
    """Allocate input/output buffers."""
    inputs = []
    outputs = []
    bindings = []
    stream = cuda.Stream()
    
    for binding in engine:
        binding_dims = engine.get_binding_shape(binding)
        size = trt.volume(binding_dims) * (1 if engine.binding_is_input(binding) else engine.max_batch_size)
        dtype = trt.nptype(engine.get_binding_dtype(binding))
        
        host_mem = cuda.pagelocked_empty(size, dtype)
        device_mem = cuda.mem_alloc(host_mem.nbytes)
        
        bindings.append(int(device_mem))
        if engine.binding_is_input(binding):
            inputs.append(HostDeviceMem(host_mem, device_mem))
        else:
            outputs.append(HostDeviceMem(host_mem, device_mem))
    
    return inputs, outputs, bindings, stream


class TrtYOLOv8(object):
    """Class to encapsulate YOLOv8 TensorRT model."""
    
    def __init__(self, model):
        """Initialize the model, engine, and context."""
        self.model = model
        self.trt_logger = trt.Logger(trt.Logger.INFO)
        self.engine = self._load_engine()
        self.input_shape = (640, 640)  # Adjust as necessary for your model
        self.inputs, self.outputs, self.bindings, self.stream = allocate_buffers(self.engine)
        self.inference_fn = self._get_inference_function()
        self.classes_names, self.inv_classes = self._read_class_names(f"{model}/classes.names")

    def _load_engine(self):
        TRTbin = f"{self.model}/yolo.trt"
        with open(TRTbin, 'rb') as f, trt.Runtime(self.trt_logger) as runtime:
            return runtime.deserialize_cuda_engine(f.read())

    def _get_inference_function(self):
        return do_inference_v2 if trt.__version__[0] >= '7' else do_inference

    def _read_class_names(self, path):
        names = {}
        with open(path, 'r') as data:
            for ID, name in enumerate(data):
                names[ID] = name.strip('\n')
        return names

    def predict(self, img):
        """Run inference on an input image."""
        img_resized = _preprocess_yolo(img, self.input_shape)
        self.inputs[0].host = np.ascontiguousarray(img_resized)
        
        trt_outputs = self.inference_fn(
            context=self.engine.create_execution_context(),
            bindings=self.bindings,
            inputs=self.inputs,
            outputs=self.outputs,
            stream=self.stream
        )

        boxes, scores, classes = _postprocess_yolo(
            trt_outputs, img.shape[1], img.shape[0], 
            rospy.get_param(Param.THRESHOLD, 0), nms_threshold=0.5
        )
        
        return boxes, scores, classes

    def draw(self, frame, boxes, scores, classes):
        """Draw bounding boxes on the image."""
        for box, score, cls in zip(boxes, scores, classes):
            x_min, y_min, x_max, y_max = box
            color = (0, 255, 0)  # Example color
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), color, 2)
            label = f"{self.classes_names[int(cls)]} {score:.2f}"
            cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

