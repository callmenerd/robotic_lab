#!/home/blakasutha/.pyenv/shims/python3 
import rclpy
import zmq
import json
import subprocess

from pynput import keyboard
from rclpy.node import Node
from array_msgs.msg import ObjectPosition
from std_msgs.msg import Int8
from std_msgs.msg import String

class ImageProcessRecv(Node):
    def __init__(self):
        super().__init__('image_process_recv')

        self.get_logger().info(f'img_recv running')


        self.box_publisher = self.create_publisher(ObjectPosition, 'pos_msgs', 10)
        self.create_subscription(String, "command_msg", self.get_state, 10)

        context = zmq.Context()
        self.zmq_pub = context.socket(zmq.PUB)
        self.zmq_pub.bind('tcp://127.0.0.1:2001')

        self.zmq_sub = context.socket(zmq.SUB)
        self.zmq_sub.connect('tcp://127.0.0.1:2000')
        self.zmq_sub.setsockopt(zmq.SUBSCRIBE, b'')

        self.cam_idx_sub = self.create_subscription(
            Int8,
            'cam_index',
            self.change_camera,
            10)

        self.reply = 'z'
        self.CAMERA_BALL = int(self.find_cam("Pro")) # BALL
        self.CAMERA_SILO = int(self.find_cam("NYK")) # SILO
        self.camera = self.CAMERA_BALL
        self.isRed = True
        self.timer = self.create_timer(0.001, self.timer_callback)
        self.priority_blue = {
            0: ['silo_bb'],
            1: ['silo_rr', 'silo_br', 'silo_rb'],
            2: ['silo_0'],
            3: ['silo_b'],
            4: ['silo_r'],
        }

        self.priority_red = {
            0: ['silo_rr'],
            1: ['silo_bb', 'silo_rb', 'silo_br'],
            2: ['silo_0'],
            3: ['silo_r'],
            4: ['silo_b'],
        }

        # Create a listener object
        # listener = keyboard.Listener(on_press=self.on_press)
        # listener.start()
        self.bounding_box=[]

    def find_priority(self, label):
        if self.isRed:
            for key, value in self.priority_red.items():
                if label in value:
                    return key
            return 100
        else:
            for key, value in self.priority_blue.items():
                if label in value:
                    return key
            return 100
        
    def find_cam(self, name):
        output = subprocess.run(["bash", "/home/blakasutha/scripts/cam_with_name", name], capture_output=True)
        dev_name = str(output.stdout)

        def find_first_number(text):
            for char in text:
                if char.isdigit():
                    return char
            return None

        return find_first_number(dev_name)
    
    def get_state(self, state: String):
        if state.data == "MULAI as RED_TEAM" or state.data == "RETRY as RED_TEAM":
            self.isRed = True
        elif state.data == "MULAI as BLUE_TEAM" or state.data == "RETRY as BLUE_TEAM":
            self.isRed = False

    def timer_callback(self):
        msg = json.loads(self.zmq_sub.recv_json())
        if self.reply != 'z':
            print(f"sending {self.reply}")
            self.zmq_pub.send_string(self.reply)
        obj = ObjectPosition()
        if len(msg) != 0:
            name = ''
            box = []
            if self.camera == self.CAMERA_BALL:
                w_values = [item["box"][2] for item in msg]
                max_w = max(w_values)
                max_w_index = w_values.index(max_w)
                name = msg[max_w_index]["name"]
                box = msg[max_w_index]["box"]
                obj.name = name
                obj.pos = box
                self.box_publisher.publish(obj)
            elif self.camera == self.CAMERA_SILO:
                def sort_by_label_priority(data, priorities):
                        def get_priority(label):
                            if self.isRed:
                                for priority, labels in priorities.items():
                                    if label in labels:
                                        return priority
                                return len(priorities)  # Default priority if not found
                        return sorted(msg, key=lambda item: get_priority(item['name']))

                sorted_data = []
                if self.isRed:
                    sort_by_label_priority(msg, self.priority_red)
                else:
                    sort_by_label_priority(msg, self.priority_blue)

                sel_msg = sorted_data[0]
                name = sel_msg["name"]
                box = sel_msg["box"]
                obj.name = name
                obj.pos = box
                self.box_publisher.publish(obj)
            print(f"name: {name} box: {box}")

    def change_camera(self, cam_idx):
        self.camera = cam_idx.data
        if cam_idx.data == self.CAMERA_BALL and self.isRed:
            self.get_logger().info(f'kirim {cam_idx}')
            self.zmq_pub.send_string('a')
        elif cam_idx.data == self.CAMERA_BALL and (not self.isRed):
            self.get_logger().info(f'kirim {cam_idx}')
            self.zmq_pub.send_string('b')
        elif cam_idx.data == self.CAMERA_SILO:
            self.get_logger().info(f'kirim {cam_idx}')
            self.zmq_pub.send_string('c')

        # self.get_logger().info(msg)
        # if(len(msg)!=0):

        # if self.reply != 'z':
        #     print(f"sending {self.reply}")
        #     self.zmq_pub.send_string(self.reply)

    def on_press(self, key):
        #print(f"You pressed: {key}")
        if hasattr(key, 'char'):
            if key.char == 'a' :
                self.reply = 'a'
            elif key.char == 'b':
                self.reply = 'b'
            elif key.char == 'c':
                self.reply = 'c'
            else:
                self.reply = 'z'
        else:
            self.reply = 'z'

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessRecv()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
