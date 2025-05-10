import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # ROS2 Subscriber and Publisher
        self.subscription_encoder = self.create_subscription(Float32MultiArray, 'sensor/odom', self.encoder_callback, 10)
        self.subscription_imu = self.create_subscription(Float32MultiArray, 'sensor/imu', self.imu_callback, 10)
        self.odom_filtered_pub = self.create_publisher(Odometry, 'sensor/odom_filtered', 10)
        self.create_timer(0.005, self.update_filter)

        # Extended Kalman filter state initialization
        # State vector [X, Y, θ, Vx, Vy, ω]
        self.x = np.zeros(6)
        # Covariance matrix
        self.P = np.eye(6) * 0.1
        # Process noise covariance (customizable)
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        # Measurement noise covariance for encoder and IMU (customizable)
        self.R_odom = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])
        self.R_imu = np.diag([0.1, 0.1, 0.2, 0.2])

        # Time step
        self.dt = 0.005
        
        # Measurement vector
        self.prev_z_odom = np.zeros(6)
        self.z_odom = np.zeros(6)
        self.z_imu = np.zeros(4)
        self.prev_yaw = 0
        
        # Flags to check if new data is available
        self.new_odom_data = False
        self.new_imu_data = False

    def encoder_callback(self, msg: Float32MultiArray):
        # Extract data from encoder
        if (len(msg.data) > 0):
            # Data encoder: [X, Y, theta, Vx, Vy, omega]
            self.prev_z_odom = self.z_odom
            self.z_odom = np.array(msg.data)
            self.new_odom_data = True

    def imu_callback(self, msg: Float32MultiArray):
        # Extract data from IMU
        if (len(msg.data) > 0):
            # Data IMU: [roll, pitch, yaw, wx, wy, wz, ax, ay, az]
            self.prev_yaw = self.z_imu[0]
            roll, pitch, yaw, wx, wy, wz, ax, ay, az = msg.data
            self.z_imu = np.array([yaw, wz, ax*100, ay*100])
            self.new_imu_data = True

    def update_filter(self):
        final_pose = Odometry()
        if not (self.new_odom_data and self.new_imu_data):
            return
        self.predict()
        slip_detected = self.detect_slip(self.z_odom, self.z_imu)
        # Perbarui dengan pengukuran dari encoder
        self.update(self.z_odom, 'odom', slip_detected)

        # Perbarui dengan pengukuran dari IMU
        self.update(self.z_imu, 'imu')

        final_pose.pose.pose.position.x = self.x[0]
        final_pose.pose.pose.position.y = self.x[1]
        final_pose.pose.pose.orientation.z = self.x[2]
        final_pose.twist.twist.linear.x = self.x[3]
        final_pose.twist.twist.linear.y = self.x[3]
        final_pose.twist.twist.angular.z = self.x[3]

        self.odom_filtered_pub.publish(final_pose)
        self.new_odom_data = False
        self.new_imu_data = False
        self.get_logger().info(str(slip_detected) + " | Vx: " + str(self.x[3]) + " Vy: " + str(self.x[4]) + " W: " + str(self.x[5]))


    def predict(self):
        # Prediksi posisi dan orientasi berdasarkan model gerak
        X, Y, theta, Vx, Vy, omega = self.x

        # Prediksi posisi berdasarkan kecepatan
        self.x[0] += (Vx * np.cos(theta) - Vy * np.sin(theta)) * self.dt
        self.x[1] += (Vx * np.sin(theta) + Vy * np.cos(theta)) * self.dt
        self.x[2] += omega * self.dt

        # Matriks transisi state
        F = np.eye(6)
        F[0, 3] = np.cos(theta) * self.dt
        F[0, 4] = -np.sin(theta) * self.dt
        F[1, 3] = np.sin(theta) * self.dt
        F[1, 4] = np.cos(theta) * self.dt
        F[2, 5] = self.dt

        # Perbarui kovarians
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, sensor_type, slip_detected=False):
        # Pembaruan state berdasarkan sensor
        if sensor_type == 'odom':
            H = np.eye(6)
            R = self.R_odom * (2 if slip_detected else 1)
        elif sensor_type == 'imu':
            # Hanya memperbarui [theta, omega, Vx, Vy]
            H = np.zeros((4, 6))
            H[0, 2] = 1  # yaw -> theta
            H[1, 5] = 1  # wz -> omega
            H[2, 3] = 1  # ax -> Vx
            H[3, 4] = 1  # ay -> Vy
            R = self.R_imu

            # Koreksi kecepatan dengan integrasi percepatan
            ax, ay = z[2], z[3]
            self.x[3] += ax * self.dt
            self.x[4] += ay * self.dt
            self.x[0] += 0.5 * ax * self.dt**2
            self.x[1] += 0.5 * ay * self.dt**2
            # if(abs(self.z_odom[5]-self.z_imu[1])>0.5):
            #     self.x[2] += (z[0]-self.prev_yaw)

        # Gain Kalman
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Perbarui estimasi state
        y = z - H @ self.x
        self.x += K @ y

        # Perbarui kovarians
        self.P = (np.eye(6) - K @ H) @ self.P
    
    def detect_slip(self, z_odom, z_imu):
        # Check difference between odom and IMU measurements for slip detection
        velocity_diff = np.linalg.norm([(z_odom[3]-self.prev_z_odom[3]) - z_imu[2], (z_odom[4]-self.prev_z_odom[4]) - z_imu[3]])
        slip_threshold = 50  # Set this threshold based on experimental data
        return velocity_diff > slip_threshold

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
