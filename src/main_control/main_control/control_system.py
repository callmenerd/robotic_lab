import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from simple_pid.pid import PID

class TrajectoryControl(Node):
    def __init__(self):
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        super().__init__("Trajectory_Controller")

        #Publisher and Subscriber
        self.command = self.create_subscription(String, "command_msg", self.sub_callback,10)
        self.move_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.odom_filtered = self.create_subscription(Float32MultiArray, 'sensor/odom_filtered', self.odom_callback, qos_profile)
        self.odom = self.create_subscription(Float32MultiArray, 'sensor/odom', self.odom_, 10)

        #Create timer loop
        self.dt = 0.005
        self.create_timer(self.dt, self.timer_callback)

        self.state_machine = False
        #Enable add the setpoint in the beginning
        self.enable_setpoint = True
        self.enable_move = True

        #Point planning
        self.path_from_start = [[[0, 630, 0], [405, 610, 0], [410, 900, 0], [410, 900, 0]],
                                 [[0, 250, 180], [0, 0, 0], [0, 0, 180], [700, 230, 270], [700, 80, 270], [150, 80, 270], [0, 0, 0]],
                                 [[0, 630, 0], [-405, 610, 0], [-410, 900, 0], [-410, 900, 0]],
                                 [[-150, 85, 0], [-405, 85, 0], [-400, 430, 0], [-39, 430, 0]]] #[[150, 85, 0], [405, 85, 0], [400, 430, 0], [39, 430, 0]],
        self.idx = 0
        self.start_index = 0

        #Initial variable
        self.init_pose = np.array([0,0,0], np.float32)
        self.distance_to_goal = 0
        self.angle_to_goal = 0

        #Robot Dinamic Control
        self.pose_data = np.array([0,0,0], np.float32)
        self.twist_data = np.array([0,0,0], np.float32)
        self.myu = 0.25
        self.g = 9.8067
        #self.max_acc = self.myu*self.g*Ts/self.dt*(1-2.718281828459045**(-self.dt/(Ts/4)))
        self.max_acc = 0.495962/2
        self.rasio = 0.9
        self.max_a_lin = (self.max_acc*self.rasio)
        self.max_a_ang = (self.max_acc*np.sqrt(1-np.square(self.rasio)))
        self.V_real = 0.0
        self.W_real = 0.0

        #PID
        self.Heading_PID = PID(0.8, 0.0, 0.005, 0.0, auto_mode=True, output_limits=(-99, 99))
        self.Distance_PID = PID(2, 0, 0, 0.0, auto_mode=True, output_limits=(-99, 99))
        self.Angle_PID = PID(2, 0.0, 0.0, 0.0, auto_mode=True, output_limits=(-99, 99)) #1.05 0.4

    def sub_callback(self, msg: String):
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

    def odom_callback(self, msg: Float32MultiArray):
        if len(msg.data) > 0:
            self.pose_data[0] = msg.data[0]
            self.pose_data[1] = msg.data[1]
            self.pose_data[2] = msg.data[2]
        # self.twist_data[0] = msg.twist.twist.linear.x
        # self.twist_data[1] = msg.twist.twist.linear.y
        # self.twist_data[2] = msg.twist.twist.angular.z

    def odom_(self, msg: Float32MultiArray):
        self.twist_data[0] = msg.data[3]
        self.twist_data[1] = msg.data[4]
        self.twist_data[2] = msg.data[5]
        self.V_real = np.sqrt(np.square(self.twist_data[0]) + np.square(self.twist_data[1]))/100
        self.W_real = np.deg2rad(msg.data[5])/10

    def final_vel(self, V_desire, W_desire):
        a = (V_desire - self.V_real)
        alpha = (W_desire - self.W_real)
        a_final = min(abs(a), (self.max_acc))
        alpha_final = min(abs(alpha), (self.max_acc))
        if a>0 and alpha>0:
            return a_final, alpha_final
        elif a<0 and alpha>0:
            return -a_final, alpha_final
        elif a>0 and alpha<0:
            return a_final, -alpha_final
        elif a<0 and alpha<0:
            return -a_final, -alpha_final
        else:
            return 0

    def timer_callback(self):
        new_vel = Twist()
        # new_vel.angular.z = 0.4
        if self.state_machine:
            goal_x = self.path_from_start[self.start_index][self.idx][0]
            goal_y = self.path_from_start[self.start_index][self.idx][1]
            goal_z = self.path_from_start[self.start_index][self.idx][2]

            if self.enable_move:
                if self.enable_setpoint:
                    #Get initial variable value
                    self.init_pose[0] = self.pose_data[0]
                    self.init_pose[1] = self.pose_data[1]
                    self.init_pose[2] = self.pose_data[2]

                    self.distance_to_goal = np.sqrt(np.square(goal_x-self.init_pose[0])+np.square(goal_y-self.init_pose[1]))
                    self.angle_to_goal = np.arctan2((goal_y-self.init_pose[1]), (goal_x-self.init_pose[0]))

                    self.enable_setpoint = False

                distance = np.sqrt(np.square(self.pose_data[0]-self.init_pose[0])+np.square(self.pose_data[1]-self.init_pose[1]))
                angle = np.arctan2(round((goal_y-self.pose_data[1])), round((goal_x-self.pose_data[0])))
                Vr = abs(self.Distance_PID(distance-self.distance_to_goal))/100
                correct_angle = self.Angle_PID(self.angle_to_goal-angle)
                err = (540 + goal_z - self.pose_data[2])%360-180
                w = -self.Heading_PID(err)/100
                acc_lin, acc_ang = self.final_vel(Vr, w)
                Vel = self.V_real + acc_lin
                W_ = self.W_real + acc_ang
                new_vel.angular.z = W_
                if abs(self.distance_to_goal - distance) >= 1:
                    phi = self.angle_to_goal+correct_angle

                    Vx = Vel * np.cos(phi)
                    Vy = Vel * np.sin(phi)

                    # Vcorx = -2*(W_/23.58)*Vy
                    # Vcory = -2*(W_/23.58)*Vx

                    Vcorx = 2*np.square(W_)/23.58 #- np.square(W_)*23.5849
                    Vcory = -2*np.square(W_)/23.58 #- np.square(W_)*23.5849

                    new_vel.linear.x = (Vx+Vcorx)
                    new_vel.linear.y = (Vy+Vcory)

                    self.get_logger().info(str(Vx) + " | "+str(Vy) + " | " +str(Vcorx) + " | " 
                            + " | "+str(Vcory) + " | W : " + str(W_))
                else :
                    if self.idx < 1:
                        self.idx+=1
                        self.enable_setpoint = True
                    else:
                        self.Distance_PID.reset()
                        self.Angle_PID.reset()
                        self.Heading_PID.reset()
                        self.V_final = 0.0
                        #new_vel.angular.z = 0.0
                        new_vel.linear.x = 0.0
                        new_vel.linear.y = 0.0
                        self.enable_move = False
                        self.get_logger().info("Goal Reached")
        else:
            new_vel.angular.z = 0.0
            new_vel.linear.y = 0.0
            new_vel.linear.x = 0.0
        self.move_pub.publish(new_vel)
        #info_msg = "X: "+str(self.pose_data[0])+" | Y: "+str(self.pose_data[1])+" | T: "+str(self.pose_data[2])+" | Vx: "+str(self.twist_data[0])+" | Vy: "+str(self.twist_data[1])+" | W: "+str(self.twist_data[2])
        #self.get_logger().info(info_msg)

def main(args = None):
    rclpy.init(args=args)
    node = TrajectoryControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()