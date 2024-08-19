# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from std_msgs.msg import Int32MultiArray
import cv2 as cv # OpenCV library
import numpy as np
import time
  
class ImageSubscriberColorSeg(Node):
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber_color_seg')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.box_publisher = self.create_publisher(Int32MultiArray, "box_msgs", 1)
    self.subscription # prevent unused variable warning
    self.box_publisher 
       
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    #Blob detection variable
    self.font = cv.FONT_HERSHEY_SIMPLEX
    # Define range for red color in HSV
    self.lower_red1 = np.array([0,100,100])
    self.upper_red1 = np.array([40,255,255])
    self.lower_red2 = np.array([160,100,100])
    self.upper_red2 = np.array([179,255,255])
    # self.lower_blue = np.array([100, 100, 100])
    # self.upper_blue = np.array([130, 255, 255])
    # self.lower_blue = np.array([90, 50, 50])
    # self.upper_blue = np.array([130, 255, 255])
    # used to record the time when we processed last frame 
    self.prev_frame_time = 0
      
    # used to record the time at which we processed current frame 
    self.new_frame_time = 0
    
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')
  
    # Convert ROS Image message to OpenCV image
    frame = self.br.imgmsg_to_cv2(data)
    frame = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Masking
    # Threshold the HSV image to get only red colors
    mask1 = cv.inRange(frame, self.lower_red1, self.upper_red1)
    mask2 = cv.inRange(frame, self.lower_red2, self.upper_red2)
    mask = cv.bitwise_or(mask1, mask2)
    #masked_blue = cv.inRange(frame, self.lower_blue, self.upper_blue)
    _, masked_color = cv.threshold(mask, 127, 255, cv.THRESH_BINARY)
    result = cv.bitwise_and(frame, frame, mask=masked_color)
    result = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
    inv_mask = cv.bitwise_not(result)

    inv_mask = cv.erode(inv_mask, None, iterations=6)
    cv.imshow("Blob detection", inv_mask)
    def blob_detection(frame):
      #Blob detection parameter
      params = cv.SimpleBlobDetector_Params()

      params.minThreshold = 0
      params.maxThreshold = 300

      params.filterByArea = True
      params.minArea = 1200
      params.maxArea = 100000

      params.filterByCircularity = True
      params.minCircularity = 0.7

      params.filterByConvexity = False
      params.minConvexity = 0.5

      params.filterByInertia = False
      params.minInertiaRatio = 0.5

      detector = cv.SimpleBlobDetector_create(params)

      keypoints = detector.detect(frame)

      return keypoints

    # Blob Detection Method Call and Returning list/array
    keypoints = blob_detection(inv_mask)
    blobCount = len(keypoints)

    largest_blob_size = 0
    largest_blob_index = -1

    frame=cv.cvtColor(frame, cv.COLOR_HSV2BGR)
    box_msg = Int32MultiArray()

    # Loop through all keypoints to find the largest blob
    for i, kp in enumerate(keypoints):
        if kp.size > largest_blob_size:
            largest_blob_size = kp.size
            largest_blob_index = i

    # Draw circles and labels for all blobs
    for i, kp in enumerate(keypoints):
        # Get blob position and size
        blob_x, blob_y = kp.pt
        blob_size = kp.size

        # Determine color and label based on largest blob
        if i == largest_blob_index:
            color = (255, 0, 0)  # Red for largest blob
            label = "Lock"
        else:
            color = (0, 255, 0)  # Green for other blobs
            label = "Blob " + str(i + 1)

        # Write position and size
        text_pos = "X=" + "{:.2f}".format(blob_x) + ", Y=" + "{:.2f}".format(blob_y)
        text_size = "Size=" + "{:.2f}".format(blob_size)
        cv.putText(frame, text_pos, (int(blob_x), int(blob_y) - 10), self.font, 0.5, color, 1)
        cv.putText(frame, text_size, (int(blob_x), int(blob_y) + 20), self.font, 0.5, color, 1)

        # Draw circle to indicate the blob
        cv.circle(frame, (int(blob_x), int(blob_y)), int(blob_size / 2), color, 2)

        # Label the largest blob as "Lock"
        if i == largest_blob_index:
            box_msg.data = [0, int(blob_size), int(blob_size), int(blob_x), int(blob_y)]
            cv.putText(frame, label, (int(blob_x), int(blob_y) + 40), self.font, 0.5, color, 1)

    # Put blobCount text on the left corner
    cv.putText(frame, f"Blob Count: {blobCount}", (10, 30), self.font, 1, (255, 255, 255), 3)
     # time when we finish processing for this frame 
    self.new_frame_time = time.time() 
  
    # Calculating the fps 
  
    # fps will be number of frame processed in given time frame 
    # since their will be most of time error of 0.001 second 
    # we will be subtracting it to get more accurate result 
    fps = 1/(self.new_frame_time-self.prev_frame_time) 
    self.prev_frame_time = self.new_frame_time 
  
    # converting the fps into integer 
    fps = int(fps) 
  
    # converting the fps to string so that we can display it on frame 
    # by using putText function 
    fps = str(fps) 
  
    # putting the FPS count on the frame 
    cv.putText(frame, fps, (10, 85), self.font, 2, (100, 255, 0), 2, cv.LINE_AA) 

    self.box_publisher.publish(box_msg)
    # Display image
    cv.imshow("camera", frame)
     
    cv.waitKey(1)
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  image_subscriber_seg = ImageSubscriberColorSeg()
   
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber_seg)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber_seg.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()