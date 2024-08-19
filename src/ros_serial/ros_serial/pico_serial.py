#!/usr/bin/env python3
import rclpy
from pySerialTransfer import pySerialTransfer as txfer
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

class feedback_cmd(object):
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    feed_cmd = ''
    left_dist = 0.0
    right_dist = 0.0
    mid_dist = 0.0

class command(object):
    mechanism_state = '1'
    cmd = 'S'

class PICO_serial(Node):
    def __init__(self):
        nomor = 1
        port = '/dev/pico'
        super().__init__("pico_serial")
        self.link = txfer.SerialTransfer(port)
        self.link.open()
        self.get_logger().info("Pico Serial COM has been created")
        self.create_timer(0.005, self.pico_serial)
        self.imu_pub = self.create_publisher(Pose, "IMU_angle", 10)
        self.feedback = self.create_publisher(String, "pico_feedback", 10)
        self.limit_msg = self.create_publisher(String, "crash_msg", 10)
        self.distance_us = self.create_publisher(Float32MultiArray, "ultrasonic", 10)
        self.send_cmd = self.create_subscription(String, "command_msg", self.get_command, 10)
        self.get_cmd = self.create_subscription(String, "com_msg", self.command, 10)
    
    def get_command(self, data: String):
        if data.data == "MULAI as RED_TEAM" or data.data == "MULAI as BLUE_TEAM":
            command.cmd = 'M'
        elif data.data == "RETRY as RED_TEAM" or data.data == "RETRY as BLUE_TEAM":
            command.cmd = 'R'
        else:
            command.cmd = 'S'
        self.get_logger().info(data.data)

    def command(self, data: String):
        if data.data == 'take_ball':
            command.mechanism_state = 'T'
        elif data.data == 'put_ball':
            command.mechanism_state = 'P'
        elif data.data == 'STOP_ROLL':
            command.mechanism_state = 'N'
        elif data.data == 'OKE!':
            command.mechanism_state = 'O'
        elif data.data == 'STOP_PICO':
            command.mechanism_state = 'Z'
        else:
            command.mechanism_state = '$'
        self.get_logger().info(command.mechanism_state)

    def get_response(self, feedback)->str:
        msg = ''
        if str(feedback) == "b'b'":
            msg = 'object ball is BLUE'
        elif str(feedback) == "b'r'":
            msg = 'object ball is RED'
        elif str(feedback) == "b'p'":
            msg = 'PURPLE, try to get rid'
        elif str(feedback) == "b'c'":
            msg = 'throw ball finish'
        elif str(feedback) == "b'!'":
            msg = 'take finish'
        else:
            msg = 'no_feedback'
        return msg
        
    def pico_serial(self):
        angle = Pose()

        #Sending section
        send_size = 0
        
        roll_size = self.link.tx_obj(command.mechanism_state, send_size) - send_size
        send_size += roll_size

        cmd_size = self.link.tx_obj(command.cmd, send_size) - send_size
        send_size += cmd_size
            
        self.link.send(send_size)
        #--end sending section--
        if self.link.available():
            recSize = 0
            feedback_cmd.roll = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.pitch = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.yaw = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.feed_cmd = self.link.rx_obj(obj_type='c', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']
            feedback_cmd.right_dist = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.left_dist = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.mid_dist = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            
            self.get_logger().info('from pico: {} {} {} {} {} {} {}'.format(feedback_cmd.roll, feedback_cmd.pitch, feedback_cmd.yaw, feedback_cmd.feed_cmd, feedback_cmd.right_dist, feedback_cmd.left_dist, feedback_cmd.mid_dist))
            angle.orientation.x = feedback_cmd.roll
            angle.orientation.y = feedback_cmd.pitch
            angle.orientation.z = feedback_cmd.yaw
            msg_pico = String()
            msg_pico.data = self.get_response(feedback_cmd.feed_cmd)
            self.get_logger().info('from pico: {}'.format(msg_pico.data))
            distance = Float32MultiArray()
            distance.data.append(feedback_cmd.left_dist)
            distance.data.append(feedback_cmd.mid_dist)
            distance.data.append(feedback_cmd.right_dist)
            limit = String()
            self.limit_msg.publish(limit)
            self.feedback.publish(msg_pico)
            self.imu_pub.publish(angle)
            self.distance_us.publish(distance)
        elif self.link.status <= 0:
            if self.link.status == txfer.CRC_ERROR:
                self.get_logger().info('ERROR: CRC_ERROR')
            elif self.link.status == txfer.PAYLOAD_ERROR:
                self.get_logger().info('ERROR: PAYLOAD_ERROR')
            elif self.link.status == txfer.STOP_BYTE_ERROR:
                self.get_logger().info('ERROR: STOP_BYTE_ERROR')
            else:
                self.get_logger().info('ERROR: {}'.format(self.link.status))
        else :
            self.get_logger().info("cek")                
            self.get_logger().info('ERROR: {}'.format(self.link.status))



def main(args = None):
    rclpy.init(args = args)
    node = PICO_serial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
