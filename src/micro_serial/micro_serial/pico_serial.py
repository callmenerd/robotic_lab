import rclpy
from pySerialTransfer import pySerialTransfer as txfer
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

class feedback_cmd(object):
    timestamp = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    gx = 0.0
    gy = 0.0
    gz = 0.0
    ax = 0.0
    ay = 0.0
    az = 0.0
    feed_cmd = ''

class command(object):
    mechanism_state = '1'
    cmd = 'S'

class PICO_serial(Node):
    def __init__(self):
        port = '/dev/pico'
        super().__init__("pico_serial")
        self.link = txfer.SerialTransfer(port)
        self.link.open()
        self.get_logger().info("Pico Serial COM has been created")
        self.create_timer(0.005, self.pico_serial)
        self.imu_pub = self.create_publisher(Float32MultiArray, "sensor/imu", 10)
        self.feedback = self.create_publisher(String, "pico_feedback", 10)
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
        angle = Float32MultiArray()

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
            feedback_cmd.timestamp = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.roll = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.pitch = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.yaw = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.gx = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.gy = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.gz = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.ax = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.ay = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.az = self.link.rx_obj(obj_type='f', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
            feedback_cmd.feed_cmd = self.link.rx_obj(obj_type='c', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']
            
            #self.get_logger().info('from pico: {} {} {} {} {} {} {} {} {} {}'.format(feedback_cmd.timestamp, feedback_cmd.roll, feedback_cmd.pitch,feedback_cmd.yaw, feedback_cmd.ax, feedback_cmd.ay, feedback_cmd.az, feedback_cmd.gx, feedback_cmd.gy, feedback_cmd.gz))

            angle.data.append(feedback_cmd.yaw)
            angle.data.append(feedback_cmd.gz)
            angle.data.append(feedback_cmd.ax)
            angle.data.append(feedback_cmd.ay)
            msg_pico = String()
            msg_pico.data = self.get_response(feedback_cmd.feed_cmd)
            #self.get_logger().info('to pico : {} {} {} {} {} {}'.format(command.X, command.Y, command.T, command.Vx, command.Vy, command.Wr))
            self.feedback.publish(msg_pico)
            self.imu_pub.publish(angle)
            #self.distance_us.publish(distance)
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
