import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class Test_Com(Node):
    def __init__(self):
        super().__init__("Command_Test")
        self.command = self.create_publisher(String, "com_msg", 10)
        self.create_timer(0.001, self.timer_feedback)
        self.cmd = String()
    
    def timer_feedback(self):
        self.cmd.data = str(sys.argv[1])
        self.get_logger().info(self.cmd.data)
        self.command.publish(self.cmd)

def main(args = None):
    rclpy.init(args=args)
    node = Test_Com()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()