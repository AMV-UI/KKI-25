
from ultralytics import YOLO
import cv2
import math
import rospy
from std_msgs.msg import String
import base64
import datetime
import time
from ably import AblyRealtime
import asyncio
from utils.config import Topic
from io import BytesIO

class ObjectDetector:
    def __init__(self, model_path, class_names, camera_index=0, width=640, height=480, fps=30):
        self.model = YOLO(model_path)
        self.class_names = class_names
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(1, fps)      # Set FPS
        self.cap.set(3, width)    # Set width
        self.cap.set(4, height)   # Set height
        
        self.image = rospy.Publisher("/kki24/vision/camera/show_green", String, queue_size=10)
        self.imageA = rospy.Publisher("/kki24/vision/camera/show_blue", String, queue_size=10)
        
        self.time = -1
        self.timeA = -1
        
        self.is_took_blue_box = False 
        self.is_took_green_box = False 

        self.dscPub = Topic.dsc.createPublisher()
        #self.dscFlagPub = Topic.dsc_flag.createPublisher()
    
    async def publish_gcs(self, image):
        ably = AblyRealtime('NXISOw.ePEc4A:nwlGvuHnw7KsO039takwqRoMjXUzetpa0iUkSeovXyk')
        await ably.connection.once_async('connected')
        print('connected to Ably')

        channel = ably.channels.get('amv-juara')

        result, encoded_image = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 10])

        if not result:
            rospy.logerr("Failed to encode frame to JPG")
            ably.close()
            return

        # Convert to base64                                
        base64_image = base64.b64encode(encoded_image).decode('utf-8')

        image_len = len(base64_image)

        print(f"sending_file {image_len}")
        await channel.publish('first', base64_image)

    def process_frame(self):
        success, img = self.cap.read()
        # print("In your mom")
        if not success:
            return None
        

        results = self.model(img, stream=True, task='detect', verbose=False)

        # red / green buoy
        max_red = -1
        max_green = -1
        red = { "x1":-1, "y1":-1, "x2":-1, "y2":-1}
        green = {"x1":-1, "y1":-1, "x2":-1, "y2":-1}

        is_detected_green_box = 0
        is_detected_blue_box = 0
        is_detected_green_buoy = 0 
        is_detected_red_buoy = 0
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Get bounding box coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
    
                # Confidence
                confidence = math.ceil((box.conf[0] * 100)) / 100
                # print("Confidence --->", confidence)

                # # Class name
                cls = int(box.cls[0])
                # print("Class name -->", self.class_names[cls])

                if confidence < 0.7:
                    continue

                # put box in cam
                color = (0, 0, 0)
                area = abs(x1 - x2)
                if(self.class_names[cls] == "redBuoy"):
                    is_detected_red_buoy = 1
                    color = (0, 0, 255)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(img, self.class_names[cls] + " " + str(confidence), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                
                    
                   
                    if(max_red == -1):
                        max_red = area
                        red = {
                            "x1": x1, "y1": y1, "x2": x2, "y2": y2
                        }
                        continue

                    if max_red < area:
                        max_red = area
                        red = {
                            "x1": x1, "y1": y1, "x2": x2, "y2": y2
                        }
                elif(self.class_names[cls] == "greenBuoy"):
                    is_detected_green_buoy = 1
                    color = (0, 255, 0)   
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(img, self.class_names[cls] + " " + str(confidence), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

                    if(max_green == -1):
                        max_green = area
                        green = {
                            "x1": x1, "y1": y1, "x2": x2, "y2": y2
                        }
                        continue

                    
                    if max_green < area:
                        max_green = area
                        green = {
                            "x1": x1, "y1": y1, "x2": x2, "y2": y2
                        }
                elif(self.class_names[cls] == "greenBox"):
                    green_area = abs(x1 - x2) * abs(y1 - y2)
                    color = (0, 69, 0)   
                    is_detected_green_box  = 1
                    # if(self.time == -1):
                    #     self.time = time.time()
                    # elif time.time() - self.time > 2 and not self.is_took_green_box:
                    #     self.time = -1   
                    #     self.is_took_green_box = True
                    #     result, encoded_image = cv2.imencode('.jpg', img)
                    #     if not result:
                    #         rospy.logerr("Failed to encode frame to JPG")
                    #         break
                    
                    #         # Convert to base64
                    #     base64_image = base64.b64encode(encoded_image).decode('utf-8')
                    #     self.image.publish(base64_image)
                    #     # self.image.publish(img)
                    #     cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                    #     cv2.putText(img, self.class_names[cls] + " " + str(confidence), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                    # self.image.publish(img)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(img, self.class_names[cls] + " " + str(confidence), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                elif(self.class_names[cls] == "blueBox"):
                    blue_area = abs(x1 - x2) * abs(y1 - y2)
                    color = (0, 0, 255)   
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(img, self.class_names[cls] + " " + str(confidence), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

                    is_detected_blue_box = 1
                    # if(self.timeA == -1):
                    #     self.timeA = time.time()
                    # elif time.time() - self.timeA > 2 and not self.is_took_blue_box:
                    #     self.timeA = -1   
                    #     self.is_took__box = True
                    #     result, encoded_image = cv2.imencode('.jpg', img)
                    #     if not result:
                    #         rospy.logerr("Failed to encode frame to JPG")
                    #         break
                    
                    #         # Convert to base64
                    #     base64_image = base64.b64encode(encoded_image).decode('utf-8')
                    #     self.imageA.publish(base64_image)
                    #     # self.image.publish(img)
                    #     cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                    #     cv2.putText(img, self.class_names[cls] + " " + str(confidence), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                else:
                    color = (0, 0, 0)
                    cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
                    cv2.putText(img, self.class_names[cls] + " " + str(confidence), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            if(is_detected_green_box == 0):
                self.time = -1
            if(is_detected_blue_box == 0):
                self.timeA = -1

            # print(f"Detected: Red Buoy: {is_detected_red_buoy}, Green Buoy: {is_detected_green_buoy}, Green Box: {is_detected_green_box}, Blue Box: {is_detected_blue_box}")

        # print(f"Detected Sex: Red Buoy: {is_detected_red_buoy}, Green Buoy: {is_detected_green_buoy}, Green Box: {is_detected_green_box}, Blue Box: {is_detected_blue_box}")


        ## print(f"Red {max_red} {red}")
        ## print(f"Green {max_green} {green}")
        # asyncio.run(self.publish_gcs(img))
        mid_red = red["x1"] + abs(red["x2"] - red["x1"]) / 2
        mid_red = int(mid_red)

        mid_green = green["x1"] + abs(green["x2"] - green["x1"]) / 2
        mid_green = int(mid_green)

        width  = self.cap.get(3) // 2  # float `width`
        height = self.cap.get(4) // 2 # float `height`
        
        width = int(width)
        height = int(height)

        
        color = (0, 0, 0)
        cv2.rectangle(img, (width, height), (width, height), color, 3)

        dsc = 0
        dsc_flag = False
        state = [is_detected_red_buoy, is_detected_green_buoy, is_detected_green_box, is_detected_blue_box]
        # print("test", state)
        if(max_red != -1 and max_green != -1):
            color = (255, 255, 255)   
            
            mid_x = (mid_green + mid_red) / 2
            mid_x = int(mid_x)

            mid_y = (max(red["y1"], green["y1"]) + min(red["y2"], green["y2"])) / 2
            mid_y = int(mid_y)

            dsc_x = abs(mid_x - width)
            dsc_y = abs(mid_y - height)
            dsc = math.sqrt(dsc_x * dsc_x + dsc_y * dsc_y)

            # kiri = 1, kanan = 0
            # dsc_flag = 0
            # if(mid_x < width):
            #     dsc_flag = 1
            dsc_flag = True # if both detected
            if(mid_x > width): 
                dsc = -dsc

            # elif(mid_X <)sc, is_detected 
            cv2.rectangle(img, (mid_x, mid_y), (mid_x, mid_y), color, 3)
            
            # self.dscPub.publish(dsc)
            #self.dscFlagPub.publish(dsc_flag)
            #self.image.publish(img)
        
        elif(max_green != -1):
            dsc = -200


        elif(max_red != -1):
            dsc = 200
        

        return img, dsc, state


    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def run(self):
        while True:
            frame = self.process_frame()
            if frame is None:
                break
            
            cv2.imshow('Webcam', frame)
            if cv2.waitKey(1) == ord('q'):
                break



