# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from simple_pid import PID
from geometry_msgs.msg import Twist
import cv2 # OpenCV library
import numpy as np
  
class LineFollower(Node):
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
       
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
    self.publisher
       
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    #PID
    self.get_PID = PID(1, 0, 0, 8)

    #Camera frame setting
    self.step = 75
    self.idx = 0
    self.coord = [[0,0], [0,0]]
    self.check_val = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    self.check_sensor = [0,0,0]
    self.path = []
    
    #Line parameter
    self.check_point = []
    self.check_point.append((70, 400-200))
    self.check_point.append((90, 345-200))
    self.check_point.append((120, 305-200))
    for i in range(160, 520, 40):
        if i == 320 or i == 280 or i == 360:
            self.check_point.append((i, 240-200))
        else:
            self.check_point.append((i, 280-200))
    self.check_point.append((520, 305-200))
    self.check_point.append((550, 345-200))
    self.check_point.append((570, 400-200))
    self.cnt = 0
    self.idx = 0
    self.check = 0
    self.path = [['011', 15], ['110', 0], ['101', 0], ['000', 8]]
    
  def listener_callback(self, data):
    # Display the message on the console
    #self.get_logger().info('Receiving video frame')
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    #setting up points for perspective transformation
    TL = (50, 100)
    BL = (0, 472)
    TR = (590, 100)
    BR = (640, 472)
    cv2.circle(current_frame, TL, 5, (0,0,255), -1)
    cv2.circle(current_frame, BL, 5, (0,0,255), -1)
    cv2.circle(current_frame, TR, 5, (0,0,255), -1)
    cv2.circle(current_frame, BR, 5, (0,0,255), -1)

    #Applying perspective transformation
    pts1 = np.float32([TL, BL, TR, BR])
    pts2 = np.float32([[0,0], [0,480], [640, 0], [640, 480]])

    #Get transform frame
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    transformed_frame = cv2.warpPerspective(current_frame, matrix, (640, 480))
    frame = transformed_frame

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #Black HSV spectrum
    L = np.uint8([0, 0, 0])
    U = np.uint8([255, 205, 50])
    mask = cv2.inRange(hsv_frame, L, U)
    for i in self.check_point:
        cv2.circle(frame, i, 5, (255,0,0), -1)
        bgr = mask[i[1], i[0]]
        if bgr > 10:
            self.check_val[self.check_point.index(i)] = self.check_point.index(i)+1
        else:
            self.check_val[self.check_point.index(i)] = 0
    if self.check_val[6:9].count(0)<3: self.check_sensor[1] = 1 
    else: self.check_sensor[1]=0
    if self.check_val[9:12].count(0)<3: self.check_sensor[2] = 1
    else: self.check_sensor[2]=0
    if self.check_val[3:6].count(0)<3: self.check_sensor[0] = 1
    else: self.check_sensor[0]=0
    bitcount = len(self.check_val)-self.check_val.count(0)
    if bitcount == 0:
        position=8
    else:
        position = sum(self.check_val)/bitcount
    string_sensor = ''.join(str(i) for i in self.check_sensor)
    if string_sensor == self.path[self.idx][0]:
      self.check += 1
      if self.check > 5:
        self.cnt = 50
    if self.cnt>0:
      position = self.path[self.idx][1]
      if self.cnt == 1:
        self.idx += 1
        self.check = 0
      self.cnt -= 1
    print("Checkpoint : " + str(self.idx))
    err = self.get_PID(position)
    vel = Twist()
    vel.linear.y = 0.75
    vel.angular.z = err/7
    self.publisher.publish(vel)
    print(string_sensor)
     
    # Display image
    cv2.imshow("camera", frame)
    cv2.imshow("cam1", mask)
     
    cv2.waitKey(1)
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  lf = LineFollower()
   
  # Spin the node so the callback function is called.
  rclpy.spin(lf)
   
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  lf.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
