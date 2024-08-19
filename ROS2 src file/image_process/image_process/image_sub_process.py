# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from array_msgs.msg import Arrs
import cv2 # OpenCV library
import numpy as np
import time

import cv2
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision
  
class ImageSubscriberProcessed(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, 
            'video_frames', 
            self.listener_callback, 
            10)
        self.box_publisher = self.create_publisher(Arrs, "box_msgs", 10)
        self.COUNTER, self.FPS = 0, 0
        self.START_TIME = time.time()
        self.detection_frame = None
        # Visualization parameters
        self.row_size = 50  # pixels
        self.left_margin = 24  # pixels
        self.text_color = (0, 0, 0)  # black
        self.font_size = 1
        self.font_thickness = 1
        self.fps_avg_frame_count = 10

        self.detection_frame = None
        self.detection_result_list = []

        self.MARGIN = 10  # pixels
        self.ROW_SIZE = 30  # pixels
        self.FONT_SIZE = 1
        self.FONT_THICKNESS = 1
        self.TEXT_COLOR = (0, 0, 0)  # black
            
        self.br = CvBridge()

        def save_result(result: vision.ObjectDetectorResult, unused_output_image: mp.Image, timestamp_ms: int):

            # Calculate the FPS
            if self.COUNTER % self.fps_avg_frame_count == 0:
                self.FPS = self.fps_avg_frame_count / (time.time() - self.START_TIME)
                self.START_TIME = time.time()

            self.detection_result_list.append(result)
            self.COUNTER += 1

        # Initialize the object detection model
        base_options = python.BaseOptions("model_int8_200.tflite")
        options = vision.ObjectDetectorOptions(base_options=base_options,
                                                running_mode=vision.RunningMode.LIVE_STREAM,
                                                max_results=5, score_threshold=0.85,
                                                result_callback=save_result)
        self.detector = vision.ObjectDetector.create_from_options(options)

    def __del__(self):
        self.detector.close()

    def listener_callback(self, data):
        """
        Callback function.
        """
        self.get_logger().info('Receiving video frame')

        self.image = self.br.imgmsg_to_cv2(data)

        # Continuously capture images from the camera and run inference
        self.image = cv2.flip(self.image, 1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_image)

        # Run object detection using the model.
        self.detector.detect_async(mp_image, time.time_ns() // 1_000_000)

        # Show the FPS
        fps_text = 'FPS = {:.1f}'.format(self.FPS)
        text_location = (self.left_margin, self.row_size)
        current_frame = self.image
        cv2.putText(current_frame, fps_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                    self.font_size, self.text_color, self.font_thickness, cv2.LINE_AA)

        if self.detection_result_list:
            # print(detection_result_list)
            current_frame = self.visualize(current_frame, self.detection_result_list[0])
            self.detection_frame = current_frame
            self.detection_result_list.clear()

        if self.detection_frame is not None:
            cv2.imshow('object_detection', self.detection_frame)

        #cv2.imshow('frame', self.image)
        cv2.waitKey(1)
    
    def get_current_frame(self):
        return self.current_frame
    
    def visualize(
            self,
            image,
            detection_result
        ) -> np.ndarray:
        """Draws bounding boxes on the input image and return it.
        Args:
            image: The input RGB image.
            detection_result: The list of all "Detection" entities to be visualized.
        Returns:
            Image with bounding boxes.
        """
        box_msg = Arrs()
        # for detection in detection_result.detections:
        if len(detection_result.detections) != 0:
            for detect in detection_result.detections:
                # Draw bounding_box
                bbox = detect.bounding_box
                start_point = bbox.origin_x, bbox.origin_y
                end_point = bbox.origin_x + bbox.width, bbox.origin_y + bbox.height
                # Use the orange color for high visibility.
                cv2.rectangle(image, start_point, end_point, (0, 165, 255), 3)
                cv2.circle(image, (int(bbox.origin_x + bbox.width/2), int(bbox.origin_y + bbox.height/2)), 2, (255, 0, 0), 2)

                # Draw label and score
                category = detect.categories[0]
                category_name = category.category_name
                probability = round(category.score, 2)
                result_text = category_name + ' (' + str(probability) + ')'
                text_location = (self.MARGIN + bbox.origin_x,
                                    self.MARGIN + self.ROW_SIZE + bbox.origin_y)
                cv2.putText(image, result_text, text_location, cv2.FONT_HERSHEY_DUPLEX,
                                self.FONT_SIZE, self.TEXT_COLOR, self.FONT_THICKNESS, cv2.LINE_AA)
                cv2.line(image, (320, 0), (320, 480), (255, 0, 0), 1) 
                cv2.line(image, (0, 240), (640, 240), (255, 0, 0), 1)

                if category_name == "silo_0" or category_name == "silo_1" or category_name == "silo_2":
                    if bbox.width <= 560 and bbox.height <= 300:
                        box_msg.int_array_1.append(self.get_category(category_name))
                        box_msg.int_array_2.append(bbox.width)
                        box_msg.int_array_3.append(bbox.height)
                        box_msg.int_array_4.append(bbox.origin_x)
                        box_msg.int_array_5.append(bbox.origin_y)
                else:
                    box_msg.int_array_1.append(self.get_category(category_name))
                    box_msg.int_array_2.append(bbox.width)
                    box_msg.int_array_3.append(bbox.height)
                    box_msg.int_array_4.append(bbox.origin_x)
                    box_msg.int_array_5.append(bbox.origin_y)

        #Published bounding box object
        self.box_publisher.publish(box_msg)

        return image
    def get_category(self, name)->int:
        if name == "red_ball":
            return 1
        elif name == "purple_ball":
            return 2
        elif name == "silo_0":
            return 3
        elif name == "silo_1":
            return 4
        elif name == "silo_2":
            return 5
        elif name == "silo_3":
            return 6
        else:
            return 99
   
def main():
    rclpy.init()
    image_subscriber_processed = ImageSubscriberProcessed()
    try:
        rclpy.spin(image_subscriber_processed)
    except KeyboardInterrupt:
        pass
    image_subscriber_processed.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
