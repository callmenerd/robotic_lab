import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from simple_pid import PID
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

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
                                 [[150, 85, 0], [150, 85, 90], [400, 230, 0], [700, 230, 0], [700, 130, 0], [150, 130, 0], [0, 100, 90], [0, 0, 0]],
                                 [[0, 630, 0], [-405, 610, 0], [-410, 900, 0], [-410, 900, 0]],
                                 [[-150, 85, 0], [-405, 85, 0], [-400, 430, 0], [-39, 430, 0]]] #[[150, 85, 0], [405, 85, 0], [400, 430, 0], [39, 430, 0]],
        self.idx = 0
        self.start_index = 0

        #Robot Dinamic Control
        self.myu = 0.25
        self.g = 9.8067
        self.max_acc = self.myu*self.g
        self.prev_vel = 0.0

        #PID
        self.Heading_PID = PID(1.0, 0.0, 0.005, auto_mode=True, output_limits=(-49, 49)) #BMI 20 1 2
        self.Distance_PID = PID(5, 0, 0, auto_mode=True, output_limits=(-99, 99))
        self.Angle_PID = PID(1.75, 0.0, 0.0, auto_mode=True, output_limits=(-99, 99)) #1.05 0.4

        #Communication
        self.command_to_pico = self.create_publisher(String, "com_msg", 10)
        self.move_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom_filtered = self.create_subscription(Odometry, 'sensor/odom_filtered', self.odom_callback, 10)
        self.create_timer(0.005, self.calc_vel)
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

    def odom_callback(self, msg: Odometry):
        self.realtime_pose.position.x = msg.pose.pose.position.x
        self.realtime_pose.position.y = msg.pose.pose.position.y
        self.angle = msg.pose.pose.orientation.z
    
    def calc_vel(self):
        new_vel = Twist()
        goal = Pose()
        if self.state_machine:
            goal.position.x = self.path_from_start[self.start_index][self.idx][0]
            goal.position.y = self.path_from_start[self.start_index][self.idx][1]
            goal.orientation.z = self.path_from_start[self.start_index][self.idx][2]

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
                vel = self.Distance_PID(distance_riil, 0.005)
                w = self.Heading_PID((self.angle), 0.005)
                correct_angle = self.Angle_PID(math.atan2((goal.position.y-self.realtime_pose.position.y), (goal.position.x-self.realtime_pose.position.x))*180/math.pi)

                if abs(self.distance_to_goal - distance_riil) >= 5:
                    new_vel.linear.x = abs(vel)/100 * math.cos(((self.Angle_PID.setpoint)/180*math.pi))
                    new_vel.linear.y = abs(vel)/100 * math.sin(((self.Angle_PID.setpoint)/180*math.pi))
                    self.get_logger().info(str(correct_angle) + " | "+str(self.distance_to_goal) + " | " 
                            + " | "+str(vel) + " | W : " + str(w))
                else :
                    new_vel.linear.x = 0.0
                    new_vel.linear.y = 0.0
                    if self.idx == 0:
                        self.Distance_PID.output_limits=(-99, 99)
                    else:
                        self.Distance_PID.output_limits=(-99, 99)
                    if self.idx < 1:
                        self.idx+=1
                        self.enable_setpoint = True
                        self.activate.data = False
                    else:
                        self.Distance_PID.reset()
                        self.Angle_PID.reset()
                        self.Heading_PID.reset()
                        new_vel.linear.x = 0.0
                        new_vel.linear.y = 0.0
                        #new_vel.angular.z = 0.0
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
            new_vel.linear.x = 0.0
            new_vel.linear.y = 0.0
            #new_vel.angular.z = 0.0
            self.move_pub.publish(new_vel)
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