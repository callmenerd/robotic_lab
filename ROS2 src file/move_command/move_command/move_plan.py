import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from simple_pid import PID
from std_msgs.msg import Float32MultiArray
import sys
import math
from std_msgs.msg import String
from std_msgs.msg import Bool
import time

class Movement_Plan(Node):
    def __init__(self):
        super().__init__("Trajectory_Node")

        self.subscriber = self.create_subscription(String, "command_msg", self.sub_callback,10)
        self.activate_task = self.create_publisher(Bool, "activate_task_zone_3", 10)
        self.state_machine = False

        #Initial variable
        self.init_pose = Pose()
        self.init_angle = 0
        self.x_to_goal = 0
        self.y_to_goal = 0

        #Realtime variable
        self.realtime_pose = Pose()
        self.distance = 0
        self.angle = 0

        #Enable add the setpoint in the beginning
        self.enable_setpoint = True
        self.enable_move = True

        #Path planning
        self.path_from_start = [[[0, 615, 0], [390, 630, 0], [385, 850, 0], [300, 850, 0]],
                                 [[150, 75, 0], [390, 85, 0], [391, 410, 0], [300, 410, 0]]]
        self.idx = 0
        self.start_index = 0

        #PID
        self.Heading_PID = PID(0.75, 0.0, 0, auto_mode=True, output_limits=(-49, 49)) #BMI 20 1 2
        self.Y_PID = PID(1, 0.005, 0.025, auto_mode=True, output_limits=(-69, 69))
        self.US_PID = PID(0.7, 0.00, 0.0135, setpoint=0, auto_mode=True, output_limits=(-0.69, 0.69))
        self.X_PID = PID(1, 0.005, 0.025, auto_mode=True, output_limits=(-69, 69)) #1.05 0.4

        #Communication
        self.command_to_pico = self.create_publisher(String, "com_msg", 10)
        self.move_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.heading_sub = self.create_subscription(Pose, "IMU_angle", self.callback_heading, 10)
        self.pose_sub = self.create_subscription(Pose, 'pose_msg', self.pose_callback, 10)
        self.ultrasonic = self.create_subscription(Float32MultiArray, "ultrasonic", self.get_distance_us, 10)
        self.create_timer(0.01, self.calc_vel)
        self.msg = String()
        self.activate = Bool()
        self.us_distance = [0,0]

        self.param = -10
        self.check_point = 0
    
    def get_distance_us(self, data: Float32MultiArray):
        if len(data.data)>0:
            self.us_distance[0] = data.data[0]
            self.us_distance[1] = data.data[1]

    def sub_callback(self, msg):
        if msg.data == "STOP":
            self.state_machine = False
            self.enable_move = False
            self.enable_setpoint = True
            self.idx = 0
        else :
            self.state_machine = True
            self.enable_move = True
            if msg.data == "RETRY":
                self.start_index = 1
            else :
                self.start_index = 0

    def callback_heading(self, msg: Pose):
        self.angle = msg.orientation.z

    def pose_callback(self, data):
        self.realtime_pose = data
    
    def calc_vel(self):
        new_vel = Twist()
        goal = Pose()
        if self.state_machine:
            goal.position.x = self.path_from_start[self.start_index][self.idx][0]
            goal.position.y = self.path_from_start[self.start_index][self.idx][1]
            goal.orientation.z = 0

            if self.enable_move:
                if self.enable_setpoint:
                    #Get initial variable value
                    self.init_pose.position.x = self.realtime_pose.position.x
                    self.init_pose.position.y = self.realtime_pose.position.y
                    self.init_angle = self.angle

                    #Get initial parameter
                    self.x_to_goal = (goal.position.x-self.init_pose.position.x)
                    self.y_to_goal = (goal.position.y-self.init_pose.position.y)
                    self.X_PID.setpoint = self.x_to_goal
                    self.Y_PID.setpoint = self.y_to_goal
                    self.Heading_PID.setpoint = goal.orientation.z

                    self.enable_setpoint = False
                    self.get_logger().info("Set distance move : " + str(round(self.X_PID.setpoint)) + "| angle : " + str(self.Y_PID.setpoint))
                Vx = self.X_PID((self.realtime_pose.position.x-self.init_pose.position.x))/100
                Vy = self.Y_PID((self.realtime_pose.position.y-self.init_pose.position.y))/100
                w = self.Heading_PID((self.angle-self.init_angle), 0.01)
                X = self.x_to_goal-(self.realtime_pose.position.x-self.init_pose.position.x)
                Y = self.y_to_goal-(self.realtime_pose.position.y-self.init_pose.position.y)
                self.get_logger().info(str(X) + " | "+str(Y) + " | W : " + str(w))
                #new_vel.linear.x = self.US_PID((self.us_distance[0]-35)/35)
                if self.idx < 3:
                    self.msg.data = "getready"
                if self.start_index == 0:
                    if self.idx == 0 and abs(Y) > 5:
                        new_vel.linear.y = Vy*0.85
                        new_vel.linear.x = self.US_PID((self.us_distance[0]-35)/35)
                    elif self.idx == 1 and abs(X) > 5:
                        new_vel.linear.x = Vx*2/3
                        new_vel.linear.y = -self.US_PID((self.us_distance[1]-35)/35)
                    elif self.idx == 2 and abs(Y) > 10:
                        new_vel.linear.x = 0.110
                        new_vel.linear.y = Vy
                    elif self.idx == 3 and abs(X) > 10:
                        new_vel.linear.x = Vx
                        new_vel.linear.y = Vy
                    else :
                        new_vel.linear.x = 0.0
                        new_vel.linear.y = 0.0
                        if self.idx <= 2:
                            self.idx+=1
                            self.US_PID.reset()
                            self.enable_setpoint = True
                            self.activate.data = False
                        else:
                            self.X_PID.reset()
                            self.Y_PID.reset()
                            self.Heading_PID.reset()
                            new_vel.linear.x = 0.0
                            new_vel.linear.y = 0.0
                            new_vel.angular.z = 0.0
                            self.enable_move = False
                            self.msg.data = 'standby'
                            self.get_logger().info("Goal Reached")
                            self.activate.data = True
                elif self.start_index == 1:
                    if self.idx == 0 and abs(X) > 5:
                        new_vel.linear.y = Vy*2/3
                        new_vel.linear.x = Vx*2/3
                    elif self.idx == 1 and abs(X) > 5:
                        new_vel.linear.x = Vx*2/3
                        new_vel.linear.y = -self.US_PID((self.us_distance[1]-35)/35)
                    elif self.idx == 2 and abs(Y) > 10:
                        new_vel.linear.x = Vx
                        new_vel.linear.y = Vy
                    elif self.idx == 3 and abs(X) > 25:
                        new_vel.linear.x = Vx
                        new_vel.linear.y = Vy
                        self.get_logger().info("Goal Stuck")
                    else :
                        self.get_logger().info("Goal Almost")
                        new_vel.linear.x = 0.0
                        new_vel.linear.y = 0.0
                        if self.idx <= 2:
                            self.idx+=1
                            self.US_PID.reset()
                            self.enable_setpoint = True
                            self.activate.data = False
                        else:
                            self.X_PID.reset()
                            self.Y_PID.reset()
                            self.Heading_PID.reset()
                            new_vel.linear.x = 0.0
                            new_vel.linear.y = 0.0
                            new_vel.angular.z = 0.0
                            self.enable_move = False
                            self.msg.data = 'standby'
                            self.get_logger().info("Goal Reached")
                            self.activate.data = True
                
                new_vel.angular.z = w/100
                self.move_pub.publish(new_vel)
                self.activate_task.publish(self.activate)
        else:
            self.activate.data = False
            self.msg.data = ''
        if not self.activate.data:
            self.command_to_pico.publish(self.msg)
            

def main(args = None):
    rclpy.init(args=args)
    node = Movement_Plan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
