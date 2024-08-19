import rclpy
import time
import cv2
import numpy as np
from simple_pid import PID
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from array_msgs.msg import ObjectPosition
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray
import subprocess


class Main_Algorithm(Node):

    def find_cam(self, name):
        output = subprocess.run(["bash", "/home/blakasutha/scripts/cam_with_name", name], capture_output=True)
        dev_name = str(output.stdout)

        def find_first_number(text):
            for char in text:
                if char.isdigit():
                    return char
            return None

        return find_first_number(dev_name)
    

    def __init__(self):
        super().__init__("Main_Algorithm")
        self.get_logger().info(f'main-control running')


        self.MIN_SILO_W = 330
        self.MIN_BAll_W = 220
        self.CAMERA_SILO = int(self.find_cam("NYK"))
        self.CAMERA_BALL = int(self.find_cam("Pro"))
        print(f"ball: {self.CAMERA_BALL} silo: {self.CAMERA_SILO}")


        self.create_timer(0.005, self.main_loop)

        self.control = self.create_publisher(Twist, "cmd_vel", 10)
        self.to_pico_command = self.create_publisher(String, "com_msg", 10)
        self.activate_xi_val = self.create_publisher(Bool, "activate_task_zone_3", 10)
        self.cam_pub = self.create_publisher(Int8, "cam_index", 10)

        self.machine = self.create_subscription(String, "command_msg", self.get_state, 10)
        self.feedback_from_pico = self.create_subscription(String, "pico_feedback", self.get_feedback, 10)
        self.orientation = self.create_subscription(Pose, "IMU_angle", self.get_angle, 10)
        self.activate = self.create_subscription(Bool, "activate_task_zone_3", self.activation_feedback, 10)
        self.limit_msg = self.create_subscription(String, "crash_msg", self.isCrash, 10)
        self.ultrasonic = self.create_subscription(Float32MultiArray, "ultrasonic", self.get_distance_us, 10)
        self.obj_pos_sub = self.create_subscription(ObjectPosition, "pos_msgs", self.get_object_position, 10)

        #PID
        self.pid_angle = PID(0.4, 0.0, 0.015, setpoint=0, sample_time=0.005, output_limits=(-0.99, 0.99), auto_mode=True)
        self.pid_distance = PID(1.5, 0.015, 0.005, setpoint=0, output_limits=(-0.78, 0.78), auto_mode=True)
        self.pid_width = PID(0.75, 0.00, 0.0025, setpoint=0, output_limits=(-0.48, 0.48), auto_mode=True)
        self.pid_heading = PID(0.7, 0.0, 0.001, setpoint=0, output_limits=(-55, 55), auto_mode=True)
        self.US_PID_R = PID(0.325, 0.00, 0.005, setpoint=0, auto_mode=True, output_limits=(-0.6, 0.6))
        self.US_PID_L = PID(0.325, 0.00, 0.005, setpoint=0, auto_mode=True, output_limits=(-0.6, 0.6))
        self.US_PID_M = PID(1, 0.00, 0.00, setpoint=0, auto_mode=True, output_limits=(-0.6, 0.6))


        #Filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                        [0, 1, 0, 1],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = 1e-5 * np.eye(4, dtype=np.float32)
        
        self.imu_orientation = [0 for i in range(3)]
        self.get_box = [0 for i in range(5)]
        self.check_box = [0 for i in range(5)]

        self.machine_state = False
        self.activate_algorithm = False

        self.rotate = True
        self.put = False
        self.silo_vel = False

        self.command_msg = String()
        self.obj_pos = ObjectPosition()
        self.feedback_msg = ""
        self.devider = 2
        self.us_distance = [0, 0, 0]
        self.init_heading = [[90, 270]]
        self.param_team = [[-1, 1], [-1, -1], [1, -1], [1, 1]]
        self.ball_msg = ['object ball is RED', 'object ball is BLUE']
        self.go_silo = 0

        #Movement Sequence Control
        self.move_sequence = 0
        self.index = 0
        self.prevMove = 0
        self.prevCam = 0
        self.isObject = False
        self.idx = None
        self.count = 0
        self.putcount = 0
        self.prevTime = 0
        self.xi = Bool()
        self.noball = 0
        self.dec = False
        self.degree_rotate = 90
        self.count_1 = 0
        self.putball = False
        self.us_error = 0
        self.cam_index = Int8()
        self.cam_index.data = self.CAMERA_BALL

    def is_us_dist_valid(self, times, val, max):
        if val > max:
            self.us_error = self.us_error + 1
        print(self.us_error)
        return self.us_error < times

    def update_kalman(self, measurement):
        self.kalman.correct(measurement)
        prediction = self.kalman.predict()
        return prediction

    def get_distance_us(self, data: Float32MultiArray):
        if len(data.data)>0:
            self.us_distance[0] = data.data[0]
            self.us_distance[1] = data.data[1]
            self.us_distance[2] = data.data[2]
    
    def get_object_position(self, pos: ObjectPosition):
        self.get_logger().info("sub_objp")
        self.obj_pos = pos
        self.get_logger().info(str(self.obj_pos))

    
    def activation_feedback(self, msg: Bool):
        #pass
        if msg.data:
            self.activate_algorithm = msg.data
    
    def isCrash(self, crash: String):
        self.crash_msg = crash.data

    def get_angle(self, angle: Pose)->None:
        self.imu_orientation[0] = angle.orientation.x
        self.imu_orientation[1] = angle.orientation.y
        self.imu_orientation[2] = angle.orientation.z

    def get_feedback(self, feedback: String)->None:
        self.feedback_msg = feedback.data
    
    def get_state(self, state: String)->None:
        if state.data == "STOP":
            self.machine_state = False
            self.move_sequence = 0
            self.activate_algorithm = False
        else:
            self.machine_state = True
        if state.data == 'MULAI as RED_TEAM' or state.data == 'RETRY as RED_TEAM':
            self.index = 1
        elif state.data == 'MULAI as BLUE_TEAM' or state.data == 'RETRY as BLUE_TEAM':
            self.index = 0
        #self.activate_algorithm = True

    def get_rotate(self, setpoint: int):
        err = (540 + setpoint - self.imu_orientation[2])%360-180
        if abs(err)>1.5:
            w = (-self.pid_heading(err)/100, False)
        else:
            w = (-self.pid_heading(err)/100, True)
        return w

    def move_in_time(self, duration)->bool:
        if(self.millis()-self.prevTime<=duration):
            return False
        else:
            return True
        
    def millis(self)->int:
        return int(round(time.time() * 1000))
    
    def goto_ball(self, x_ : float, y_ : float)->Twist:
        vel = Twist()
        angle_err = (x_-320)/320
        dist_err = (y_-410)/410
        if self.rotate:
            if abs(angle_err)>0.15:
                vel.angular.z = self.pid_angle(angle_err)/2
            else:
                vel.angular.z = 0
                self.rotate = False
        else:
            if abs(angle_err) > 0.875:
                self.rotate = True
            else:
                if y_ > 150:
                    self.devider = 2
                    self.command_msg.data = 'take_ball'
                else:
                    self.devider = 1.5
                if y_ > 400:
                    print("ball is ahead")
                    self.move_sequence = 4
                else:
                    vel.linear.y = self.pid_distance(-dist_err)/self.devider
                    vel.linear.x = self.pid_width(angle_err)/self.devider
        return vel
    
    def goto_silo(self)->Twist:
        if (len(self.obj_pos.pos)!=0):
            measurement = self.update_kalman(np.array([self.obj_pos.pos[0], self.obj_pos.pos[1]], dtype=np.float32))
        self.xi.data = False
        vel = Twist()
        error_dist = ((self.us_distance[1] - 60)/60)
        if self.index == 0:
            vel.linear.x = self.US_PID_M(error_dist)
        elif self.index == 1:
          vel.linear.x = self.US_PID_M(error_dist)*(-1)
        vel.angular.z  = self.get_rotate(self.init_heading[0][self.index])
        # vel.angular.z  = 0
        # vel.linear.y = 0
        if len(self.obj_pos.pos) != 0:
            angle_err = (int(measurement[0])-300)/300
            dist_err = (int(self.obj_pos.pos[2])-280)/280
            if self.index == 0:
                vel.linear.y = self.pid_width(angle_err)*0.75*(-1)
            elif self.index == 1:
                vel.linear.y = self.pid_width(angle_err)*0.75
            # if self.obj_pos.pos[2] > 310:
            #     vel.linear.y = 0
            #     vel.linear.y = self.pid_distance(dist_err)*0.75
            error = self.obj_pos.pos[0] - 320
            print(f"error {error} error_dist {error_dist}")
            if self.us_distance[1] < 62:
                # self.move_sequence = 6
                self.us_error= self.us_error + 1
            else: 
                self.us_error = 0
            if error < abs(4) and self.us_error > 24:
                self.move_sequence = 6
                self.US_PID_M.reset()
                self.us_error = 0
        # else:
        #   vel.linear.y = 0
        return vel


        # if not self.put:
        # vel.angular.z, state = self.get_rotate(-90)
        # if self.get_box[1] > 110: 
        #     self.devider = 4
        # else:
        #     self.devider = 2
        # if self.get_box[1] > 100: 
        #     vel.linear.y = -self.pid_width(angle_err)*0.75
        # else:
        #     vel.linear.y = -self.pid_width(angle_err)/5
        # vel.linear.x = self.pid_distance(dist_err)/self.devider
        # self.get_logger().info(str(dist_err))
        # if self.get_box[1]>155 and self.get_box[1] != 1000:
        #     self.put = True
        #     self.devider = 2
        #     self.pid_angle.reset()
        #     self.pid_width.reset()
        #     self.pid_distance.reset()
        #     # self.command_msg.data = 'lift_up'
        #     vel.angular.z = 0
        #     vel.linear.y = 0
        #     vel.linear.x = 0
        #     self.get_logger().warn("finish")
        # self.get_logger().info("not put")
        # else:
        # vel.angular.z, state = self.get_rotate(-90)
        # vel.linear.y = -self.pid_width(angle_err)/self.devider
        # vel.linear.x = 0
        # if abs(angle_err)<0.017:
        #     self.count += 1
        # else:
        #     self.count = 0
        # if self.count > 24:
        #     if state:
        #     self.count = 0
        #     self.put = False
        #     vel.linear.x = 0
        #     vel.linear.y = 0
        #     self.move_sequence = 10
        #     self.go_silo += 1
        #     self.get_logger().info("put")
        # self.get_logger().info(str(dist_err))
        # return vel

    def main_loop(self):
        # self.get_logger().info(f"mainlooppppp {self.move_sequence}")
        new_vel = Twist()
        self.get_logger().info(str(self.obj_pos.pos))
        if self.machine_state:
            if self.activate_algorithm:
                self.prevCam = self.cam_index.data
                match self.move_sequence:
                    case 0:
                        self.command_msg.data = 'STOP_ROLL'
                        new_vel.angular.z, state = self.get_rotate(self.init_heading[0][self.index])
                        if state:
                            self.move_sequence = 1
                        # self.prevTime = self.millis()
                        # self.move_sequence += 1
                        self.get_logger().info("case 0")
                        
                    case 1:
                        self.xi.data = True
                        # if not self.move_in_time(3000):
                        self.cam_index.data = self.CAMERA_BALL
                        self.get_logger().info("case 1")
                        if len(self.obj_pos.pos)!=0:
                            x = self.obj_pos.pos[0]
                            y = self.obj_pos.pos[1]
                            new_vel = self.goto_ball(x, y)
                        else:
                            self.prevTime = self.millis()
                            new_vel.linear.x = 0
                            new_vel.linear.y = 0
                            new_vel.angular.z = 0
                            self.move_sequence = 3
                        
                        # if self.index == 0:
                        #     if self.feedback_msg =='object ball is RED':
                        #         self.command_msg.data = 'STOP_ROLL'
                        #         self.move_sequence = 3
                        #         new_vel.linear.x = 0
                        #         new_vel.linear.y = 0
                        #         new_vel.angular.z = 0
                        #     elif self.feedback_msg == "PURPLE, try to get rid" or self.feedback_msg == 'object ball is BLUE':
                        #         self.command_msg.data = 'put_ball'
                        #         self.move_sequence = 2
                        #         new_vel.linear.x = 0
                        #         new_vel.linear.y = 0
                        #         new_vel.angular.z = 0
                        #     else:
                        #         new_vel.linear.y = -0.3
                        #         new_vel.linear.x = 0 
                        #         new_vel.angular.z = 0
                        # if self.index == 1:
                        #     if self.feedback_msg =='object ball is BLUE':
                        #         self.command_msg.data = 'STOP_ROLL'
                        #         self.move_sequence = 3
                        #         new_vel.linear.x = 0
                        #         new_vel.linear.y = 0
                        #         new_vel.angular.z = 0
                        #     elif self.feedback_msg == "PURPLE, try to get rid" or self.feedback_msg == 'object ball is RED':
                        #         self.command_msg.data = 'put_ball'
                        #         self.move_sequence = 2
                        #         new_vel.linear.x = 0
                        #         new_vel.linear.y = 0
                        #         new_vel.angular.z = 0
                        #     else:
                        #         new_vel.linear.y = -0.3
                        #         new_vel.linear.x = 0
                        #         new_vel.angular.z = 0
                        
                        # self.get_logger().info(str(new_vel))
                        # self.get_logger().info("case 1")
                    
                    case 2:
                        if not self.move_in_time(2000):
                            new_vel.linear.x = 0
                            new_vel.linear.y = 0
                            new_vel.angular.z = 0
                        else:
                            self.command_msg.data = 'STOP_ROLL'
                            self.move_sequence = 1
                        self.get_logger().info("case 2")
                    case 3:
                        self.xi.data = False
                        #if not self.move_in_time(3000):
                        self.cam_index.data = self.CAMERA_SILO
                        self.get_logger().info("case 3")
                        if self.imu_orientation[0]>abs(5) or self.imu_orientation[1]>abs(5):
                            self.silo_vel = True
                        if self.silo_vel:
                            new_vel = self.goto_silo()  
                        else:
                            if self.index == 0:
                                new_vel.linear.x = 0.5
                                new_vel.linear.y = 0.0
                                new_vel.linear.z = 0.0
                            elif self.index == 1:
                                new_vel.linear.x = -0.5
                                new_vel.linear.y = 0.0
                                new_vel.linear.z = 0.0
                    case 4:
                        self.xi.data = False
                        if self.index == 0:
                            if self.feedback_msg =='object ball is RED':
                                if self.feedback_msg == 'take finish':
                                    self.command_msg.data = 'STOP_ROLL'
                                self.move_sequence = 3
                                new_vel.linear.x = 0
                                new_vel.linear.y = 0
                                new_vel.angular.z = 0
                            elif self.feedback_msg == "PURPLE, try to get rid" or self.feedback_msg == 'object ball is BLUE':
                                self.command_msg.data = 'put_ball'
                                self.prevTime = self.millis()
                                self.move_sequence = 2
                                new_vel.linear.x = 0
                                new_vel.linear.y = 0
                                new_vel.angular.z = 0
                            else:
                                new_vel.linear.x = 0
                                new_vel.linear.y = -0.1
                                new_vel.angular.z = 0

                        if self.index == 1:
                            if self.feedback_msg =='object ball is BLUE':
                                if self.feedback_msg == 'take finish':
                                    self.command_msg.data = 'STOP_ROLL'
                                self.move_sequence = 3
                                new_vel.linear.x = 0
                                new_vel.linear.y = 0
                                new_vel.angular.z = 0
                            elif self.feedback_msg == "PURPLE, try to get rid" or self.feedback_msg == 'object ball is RED':
                                self.command_msg.data = 'put_ball'
                                self.move_sequence = 2
                                self.prevTime = self.millis()
                                new_vel.linear.x = 0
                                new_vel.linear.y = 0
                                new_vel.angular.z = 0
                            else:
                                new_vel.linear.x = 0
                                new_vel.linear.y = -0.1
                                new_vel.angular.z = 0
                        self.get_logger().info("case 4")

                    case 5:
                        self.xi.data = False
                        new_vel.linear.x = 0.0
                        new_vel.angular.z, state = self.get_rotate(self.degree_rotate)
                        if state:
                            if self.degree_rotate<=0:
                                self.dec = False
                            elif self.degree_rotate>=360:
                                self.dec = True
                            if self.dec:
                                self.degree_rotate -= 45
                            else:
                                self.degree_rotate += 45
                        if len(self.obj_pos.pos)!=0:
                            self.noball += 1
                            if self.noball > 24:
                                self.noball = 0
                                self.xi.data = True
                                self.move_sequence = 1
                        else:
                            self.noball = 0
                        self.get_logger().info("case 5")
                        
                    case 6:
                        self.xi.data = False
                        error_dist = ((self.us_distance[1] - 11)/11)
                        if self.index == 0:
                            new_vel.linear.x = self.US_PID_M(error_dist)*0.75
                        elif self.index == 1:
                            new_vel.linear.x = self.US_PID_M(error_dist)*-0.75
                        new_vel.angular.z  = self.get_rotate(self.init_heading[0][self.index])

                        # if len(self.obj_pos.pos) != 0:
                            # angle_err = (int(self.obj_pos.pos[0])-300)/300
                            # dist_err = (int(self.obj_pos.pos[2])-280)/280
                        new_vel.linear.y = 0
                        # if self.obj_pos.pos[2] > 310:
                        #     vel.linear.y = 0
                        #vel.linear.y = self.pid_distance(dist_err)*0.75
                        if self.us_distance[1] <= 12:
                            # self.move_sequence = 6
                            self.us_error= self.us_error + 1
                        else: 
                            self.us_error = 0
                        if self.us_error > 16:
                            self.silo_vel = False
                            self.command_msg.data = 'STOP_ROLL'
                            self.us_error = 0
                            self.move_sequence = 7
                        print('done')
                        self.get_logger().info("case 6")
                    case 7:
                        new_vel.linear.y = 0
                        new_vel.linear.x = 0
                        new_vel.angular.z = 0
                        self.command_msg.data = 'put_ball'
                        if self.feedback_msg == 'throw ball finish':
                            self.prevTime = self.millis()
                            self.move_sequence = 8
                        self.get_logger().info("case 7")
                    case 8:
                        # mundur
                        self.cam_index.data = self.CAMERA_SILO
                        if not self.move_in_time(1_000):
                            self.command_msg.data = 'STOP_ROLL'
                        elif not self.move_in_time(3500):
                            if self.index == 0:
                                new_vel.linear.x = 0.7
                            elif self.index == 1:
                                new_vel.linear.x = -0.57
                        else:
                            self.move_sequence = 1
                        self.get_logger().info("case 8")
                    case 9:
                        self.get_logger().info("case 9")
                    case 10:
                        self.get_logger().info("case 10")
                    case 11:
                        self.get_logger().info("case 11")

                # if self.cam_index.data != self.prevCam:
                self.cam_pub.publish(self.cam_index)
                self.activate_xi_val.publish(self.xi)
                self.control.publish(new_vel)
                self.to_pico_command.publish(self.command_msg)
        else:
            self.move_sequence = 0
            self.activate_algorithm = False
            #cam_index.data = self.CAMERA_BALL
            self.command_msg.data = 'STOP_PICO'
            self.to_pico_command.publish(self.command_msg)

def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  object_following = Main_Algorithm()
   
  # Spin the node so the callback function is called.
  rclpy.spin(object_following)
  object_following.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()