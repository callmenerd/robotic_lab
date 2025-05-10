import rclpy
from pySerialTransfer import pySerialTransfer as txfer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
import numpy as np

# Massage format for serial transfer with STM32
class RECEIVE(object): #coordinate message receive from stm
    timestamp = 0.0
    coord_x = 0.0
    coord_y = 0.0
    coord_w = 0.0
    Vx = 0.0
    Vy = 0.0
    Wr = 0.0
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
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10
        )
        #creating publisher and subscriber for movement command and initiate serial com
        super().__init__("stm_serial")
        self.create_timer(0.005, self.serialcom)
        self.get_logger().info("Serial com with stm node created!")
        self.link = txfer.SerialTransfer('/dev/stm32')
        self.link.open()
        self.prevState = ""
        self.stm_pub = self.create_publisher(Float32MultiArray, "sensor/odom", 10)
        self.command = self.create_publisher(String, "command_msg", 10)
        self.stm_sub = self.create_subscription(Twist, "cmd_vel", self.get_vel, 10)
        self.angle_sub = self.create_subscription(Float32MultiArray, "sensor/odom_filtered", self.get_yaw, qos_profile)
        self.activation_stat = False

    def activation_feedback(self, msg:Bool):
        self.activation_stat = msg.data
    
    def get_vel(self, msg: Twist):
        #assigning velocity command to massage that will be transmitted to stm32
        TRANSMIT.Vx = float(msg.linear.x)
        TRANSMIT.Vy = float(msg.linear.y)
        TRANSMIT.W = float(msg.angular.z)
    
    def get_yaw(self, msg: Float32MultiArray):
        if len(msg.data)>0:
            TRANSMIT.psi = msg.data[2]

    def serialcom(self):
        #method for transmit and receiveing massage to and from stm32
        msg = Float32MultiArray()
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
        # self.get_logger().info("Data to transmit = Vx: " + str(TRANSMIT.Vx) + " | Vy: "
        #                        + str(TRANSMIT.Vy) + " | W: " + str(TRANSMIT.W) + " | psi: " + str(TRANSMIT.psi))
        if(self.link.available()):
            #RECEIVE MESSAGE
            recSize = 0

            RECEIVE.timestamp = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.coord_x = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.coord_y = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.coord_w = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.Vx = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.Vy = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.Wr = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

            RECEIVE.cmd = self.link.rx_obj(obj_type='c', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            RECEIVE.team = self.link.rx_obj(obj_type='c', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']
            
            msg.data.append(RECEIVE.coord_x)
            msg.data.append(RECEIVE.coord_y)
            msg.data.append(RECEIVE.coord_w)
            msg.data.append(RECEIVE.Vx)
            msg.data.append(RECEIVE.Vy)
            msg.data.append(RECEIVE.Wr)

            self.stm_pub.publish(msg)
            hello_str = "Receive from STM = T: "+str(RECEIVE.timestamp)+" | X: "+str(RECEIVE.coord_x)+" | Y: "+str(RECEIVE.coord_y)+" | T: "+str(RECEIVE.coord_w)+" | Vx: "+str(RECEIVE.Vx)+" | Vy: "+str(RECEIVE.Vy)+" | W: "+str(RECEIVE.Wr)
            #self.get_logger().info(hello_str)
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