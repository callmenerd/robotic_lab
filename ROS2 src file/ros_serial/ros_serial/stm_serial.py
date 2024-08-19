#!/usr/bin/env python3
import rclpy
from pySerialTransfer import pySerialTransfer as txfer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Bool

# Massage format for serial transfer with STM32
class RECEIVE(object): #coordinate message receive from stm
    coord_x = 0.0
    coord_y = 0.0
    coord_w = 0.0
    cmd = ''
    team = ''

class TRANSMIT(object): #velocity message transmit to stm
    Vx = 0.0
    Vy = 0.0
    W = 0.0
    psi = 0.0

#Node for serial communication between STM32 with ROS2
class STMSerial(Node): 
    def __init__(self):
        #creating publisher and subscriber for movement command and initiate serial com
        super().__init__("stm_serial")
        self.create_timer(0.005, self.serialcom)
        self.get_logger().info("Serial com with stm node created!")
        self.link = txfer.SerialTransfer('/dev/stm32')
        self.link.open()
        self.prevState = ""
        self.stm_pub = self.create_publisher(Pose, "pose_msg", 10)
        self.command = self.create_publisher(String, "command_msg", 10)
        self.stm_sub = self.create_subscription(Twist, "cmd_vel", self.get_vel, 10)
        self.angle_sub = self.create_subscription(Pose, "IMU_angle", self.get_yaw, 10)
        self.activate = self.create_subscription(Bool, "activate_task_zone_3", self.activation_feedback, 10)
        self.activation_stat = False

    def activation_feedback(self, msg:Bool):
        self.activation_stat = msg.data
    
    def get_vel(self, msg: Twist):
        #assigning velocity command to massage that will be transmitted to stm32
        TRANSMIT.Vx = float(msg.linear.x)
        TRANSMIT.Vy = float(msg.linear.y)
        TRANSMIT.W = float(msg.angular.z)
    
    def get_yaw(self, msg: Pose):
        if self.activation_stat:
            TRANSMIT.psi = 0
        else:
            TRANSMIT.psi = msg.orientation.z

    def serialcom(self):
        #method for transmit and receiveing massage to and from stm32
        msg = Pose()
        cmd_msg = String()
        
        #SENDING MASSAGE
        send_size = 0
            
        vx_size = self.link.tx_obj(TRANSMIT.Vx, send_size) - send_size
        send_size += vx_size
            
        vy_size = self.link.tx_obj(TRANSMIT.Vy, send_size) - send_size
        send_size += vy_size
            
        w_size = self.link.tx_obj(TRANSMIT.W, send_size) - send_size
        send_size += w_size

        psi_size = self.link.tx_obj(TRANSMIT.psi, send_size) - send_size
        send_size += psi_size
            
        self.link.send(send_size)
        self.get_logger().info("Data to transmit = Vx: " + str(TRANSMIT.Vx) + " | Vy: "
                               + str(TRANSMIT.Vy) + " | W: " + str(TRANSMIT.W) + " | psi: " + str(TRANSMIT.psi))
        if(self.link.available()):
            #RECEIVE MESSAGE
            recSize = 0
            RECEIVE.coord_x = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.coord_y = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.coord_w = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.cmd = self.link.rx_obj(obj_type='c', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            RECEIVE.team = self.link.rx_obj(obj_type='c', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            msg.position.x = RECEIVE.coord_x
            msg.position.y = RECEIVE.coord_y
            msg.orientation.z = RECEIVE.coord_w

            self.stm_pub.publish(msg)
            hello_str = "Receive from STM = X: "+str(RECEIVE.coord_x)+" | Y: "+str(RECEIVE.coord_y)+" | W: "+str(RECEIVE.coord_w)+" | CMD: "+str(RECEIVE.team)
            self.get_logger().info(hello_str)
        elif self.link.status <= 0:
            if self.link.status == txfer.CRC_ERROR:
                self.get_logger().error('ERROR: CRC_ERROR')
            elif self.link.status == txfer.PAYLOAD_ERROR:
                self.get_logger().error('ERROR: PAYLOAD_ERROR')
            elif self.link.status == txfer.STOP_BYTE_ERROR:
                self.get_logger().error('ERROR: STOP_BYTE_ERROR')
            else:
                self.get_logger().error('ERROR: {}'.format(self.link.status))
        if self.prevState != str(RECEIVE.cmd):
            if str(RECEIVE.cmd) == "b'S'":
                cmd_msg.data = "STOP"
            elif str(RECEIVE.cmd) == "b'M'" and str(RECEIVE.team) == "b'\\x01'":
                cmd_msg.data = "MULAI as RED_TEAM"
            elif str(RECEIVE.cmd) == "b'M'" and str(RECEIVE.team) == "b'\\x00'":
                cmd_msg.data = "MULAI as BLUE_TEAM"
            elif str(RECEIVE.cmd) == "b'R'" and str(RECEIVE.team) == "b'\\x01'":
                cmd_msg.data = "RETRY as RED_TEAM"
            elif str(RECEIVE.cmd) == "b'R'" and str(RECEIVE.team) == "b'\\x00'":
                cmd_msg.data = "RETRY as BLUE_TEAM"
            self.command.publish(cmd_msg)
        self.prevState = str(RECEIVE.cmd)

def main(args = None):
    rclpy.init(args=args)
    serial_node = STMSerial()
    if rclpy.ok():
        rclpy.spin(serial_node)
    else:
        serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
