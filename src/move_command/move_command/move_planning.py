import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from simple_pid import PID
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
        self.distance_to_goal = 0

        #Realtime variable
        self.realtime_pose = Pose()
        self.distance = 0
        self.angle = 0

        #Enable add the setpoint in the beginning
        self.enable_setpoint = True
        self.enable_move = True

        #Path planning
        self.path_from_start = [[[0, 630, 0], [405, 610, 0], [410, 900, 0], [410, 900, 0]],
                                 [[150, 85, 0], [405, 85, 0], [400, 430, 0], [39, 430, 0]],
                                 [[0, 630, 0], [-405, 610, 0], [-410, 900, 0], [-410, 900, 0]],
                                 [[-150, 85, 0], [-405, 85, 0], [-400, 430, 0], [-39, 430, 0]]]
        self.idx = 0
        self.start_index = 0

        #PID
        self.Heading_PID = PID(1.0, 0.0, 0.005, auto_mode=True, output_limits=(-49, 49)) #BMI 20 1 2
        self.Distance_PID = PID(5, 0, 0, auto_mode=True, output_limits=(-69, 69))
        self.Angle_PID = PID(1.75, 0.0, 0.0, auto_mode=True, output_limits=(-69, 69)) #1.05 0.4

        #Communication
        self.command_to_pico = self.create_publisher(String, "com_msg", 10)
        self.move_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.heading_sub = self.create_subscription(Pose, "IMU_angle", self.callback_heading, 10)
        self.pose_sub = self.create_subscription(Pose, 'pose_msg', self.pose_callback, 10)
        self.create_timer(0.01, self.calc_vel)
        self.msg = String()
        self.activate = Bool()

    def sub_callback(self, msg):
        if msg.data == "STOP":
            self.state_machine = False
            self.enable_move = False
            self.enable_setpoint = True
            self.idx = 0
        else :
            self.state_machine = True
            self.enable_move = True
            if msg.data == "RETRY as RED_TEAM":
                self.start_index = 1
            elif msg.data == "MULAI as RED_TEAM":
                self.start_index = 0
            elif msg.data == "RETRY as BLUE_TEAM":
                self.start_index = 3
            elif msg.data == "MULAI as BLUE_TEAM":
                self.start_index = 2

    def callback_heading(self, msg: Pose):
        self.angle = msg.orientation.z

    def pose_callback(self, data):
        self.realtime_pose = data
    
    def calc_vel(self):
        new_vel = Twist()
        goal = Pose()
        if self.state_machine:
            # goal.position.x = float(sys.argv[1])
            # goal.position.y = float(sys.argv[2])
            goal.position.x = self.path_from_start[self.start_index][self.idx][0]
            goal.position.y = self.path_from_start[self.start_index][self.idx][1]
            goal.orientation.z = 0

            if self.enable_move:
                self.msg.data = 'getready'
                if self.enable_setpoint:
                    #Get initial variable value
                    self.init_pose.position.x = self.realtime_pose.position.x
                    self.init_pose.position.y = self.realtime_pose.position.y
                    self.init_angle = self.angle

                    #Get initial parameter
                    self.distance_to_goal = math.sqrt((goal.position.x-self.init_pose.position.x)**2+(goal.position.y-self.init_pose.position.y)**2)
                    self.Distance_PID.setpoint = self.distance_to_goal
                    self.Angle_PID.setpoint = math.atan2((goal.position.y-self.init_pose.position.y), (goal.position.x-self.init_pose.position.x))*180/math.pi
                    self.Heading_PID.setpoint = goal.orientation.z

                    self.enable_setpoint = False
                    self.get_logger().info("Set distance move : " + str(round(self.Distance_PID.setpoint)) + "| angle : " + str(self.Angle_PID.setpoint))
                distance_riil = math.sqrt((self.realtime_pose.position.x-self.init_pose.position.x)**2+(self.realtime_pose.position.y-self.init_pose.position.y)**2)
                vel = self.Distance_PID(distance_riil, 0.01)
                w = self.Heading_PID((self.angle-self.init_angle), 0.01)
                correct_angle = self.Angle_PID(math.atan2((goal.position.y-self.realtime_pose.position.y), (goal.position.x-self.realtime_pose.position.x))*180/math.pi)

                if abs(self.distance_to_goal - distance_riil) >= 5:
                    new_vel.linear.x = abs(vel)/100 * math.cos((self.Angle_PID.setpoint)/180*math.pi)
                    new_vel.linear.y = abs(vel)/100 * math.sin((self.Angle_PID.setpoint)/180*math.pi)
                    self.get_logger().info(str(correct_angle) + " | "+str(self.distance_to_goal) + " | " 
                            + " | "+str(vel) + " | W : " + str(w))
                else :
                    new_vel.linear.x = 0.0
                    new_vel.linear.y = 0.0
                    # if self.idx == 1:
                    #     prevTime = time.time()
                    #     while(time.time()-prevTime <= 2):
                    #         new_vel.linear.x = 0.0
                    #         new_vel.linear.y = 0.0
                    #         self.move_pub.publish(new_vel)
                    if self.idx < 3:
                        self.idx+=1
                        self.enable_setpoint = True
                        self.activate.data = False
                    else:
                        self.Distance_PID.reset()
                        self.Angle_PID.reset()
                        self.Heading_PID.reset()
                        new_vel.linear.x = 0.0
                        new_vel.linear.y = 0.0
                        new_vel.angular.z = 0.0
                        self.enable_move = False
                        self.msg.data = 'standby'
                        self.get_logger().info("Goal Reached")
                        self.activate.data = True
                new_vel.angular.z = w/100
                self.command_to_pico.publish(self.msg)
                self.move_pub.publish(new_vel)
                self.activate_task.publish(self.activate)
        else:
            self.activate.data = False

def main(args = None):
    rclpy.init(args=args)
    node = Movement_Plan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()