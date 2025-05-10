#!/usr/bin/env python3
import rclpy
import math
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
from matplotlib.animation import FuncAnimation
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

#Node for serial communication between STM32 with ROS2
class DisplayNode(Node): 
    def __init__(self):
        #creating publisher and subscriber for movement command and initiate serial com
        super().__init__("display_data")
        self.create_timer(0.005, self.timer_callback)
        self.get_logger().info("Serial com with stm node created!")

        self.create_subscription(Odometry, 'sensor/odom_filtered', self.odom_callback, 10)

        self.odom_data = [[0,0,0], [0,0,0]]
        self.imu_data = [[0,0,0], [0,0,0], [0,0,0]]

        # Parameters for data storage and plot
        self.time_window = 10.0  # Time window in seconds for real-time plot
        self.time_data = deque(maxlen=200)  # Stores time
        self.sensor_data_x = deque(maxlen=200)  # Stores sensor data x
        self.sensor_data_y = deque(maxlen=200)  # Stores sensor data y
        # Menggunakan properti sec dan nanosec secara langsung
        current_time = self.get_clock().now().to_msg()
        self.start_time = current_time.sec + current_time.nanosec * 1e-9

        
        # Setup matplotlib figure and axes
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.ax1.set_title("Sensor Data X over Time")
        self.ax1.set_xlabel("Time (s)")
        self.ax1.set_ylabel("Data X")
        self.ax2.set_title("Sensor Data Y over Time")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Data Y")
        
        # Initialize lines
        self.line1, = self.ax1.plot([], [], 'b-', label="Data X")
        self.line2, = self.ax2.plot([], [], 'g-', label="Data Y")
        
        # Real-time plot update
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50)

    def odom_callback(self, data: Odometry)->None:
        # Get current time
        current_time = self.get_clock().now().to_msg().sec_nanosec[0] - self.start_time
        self.time_data.append(current_time)
        
        self.sensor_data_x.append(data.pose.pose.position.x)
        self.sensor_data_y.append(data.pose.pose.position.y)

    def timer_callback(self):
        disp = str(self.odom_data[1][0])+" | "+str(self.odom_data[1][1])+" ~~ "+str(self.imu_data[1][0])+" | "+str(self.imu_data[1][1]*math.pi/180)
        self.get_logger().info(disp)

    def update_plot(self, frame):
        # Update line data
        self.line1.set_data(self.time_data, self.sensor_data_x)
        self.line2.set_data(self.time_data, self.sensor_data_y)
        
        # Adjust axis limits to show latest data within time window
        time_min = max(0, self.time_data[-1] - self.time_window) if self.time_data else 0
        self.ax1.set_xlim(time_min, time_min + self.time_window)
        self.ax1.set_ylim(min(self.sensor_data_x, default=0) - 1, max(self.sensor_data_x, default=1) + 1)
        
        self.ax2.set_xlim(time_min, time_min + self.time_window)
        self.ax2.set_ylim(min(self.sensor_data_y, default=0) - 1, max(self.sensor_data_y, default=1) + 1)

        return self.line1, self.line2

    def run_plot(self):
        plt.show()

def main(args = None):
    rclpy.init(args=args)
    serial_node = DisplayNode()
    # if rclpy.ok():
    #     rclpy.spin(serial_node)
    # else:
    #     serial_node.destroy_node()

    try:
        serial_node.run_plot()
    except KeyboardInterrupt:
        pass
    finally:
        serial_node.destroy_node()
        rclpy.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
