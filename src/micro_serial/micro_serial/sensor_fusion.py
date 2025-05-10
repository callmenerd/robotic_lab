import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        # Time step
        self.dt = 0.005

        # ROS2 Subscriber and Publisher
        self.subscription_encoder = self.create_subscription(Float32MultiArray, 'sensor/odom', self.encoder_callback, 10)
        self.subscription_imu = self.create_subscription(Float32MultiArray, 'sensor/imu', self.imu_callback, 10)
        self.odom_filtered_pub = self.create_publisher(Odometry, 'sensor/odom_filtered', 10)
        self.create_timer(self.dt, self.update_filter)

        # Extended Kalman filter state initialization
        # State vector [X, Y, θ, Vx, Vy, ω]
        self.x = np.zeros(6)
        # Covariance matrix
        self.P = np.diag([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        # Process noise covariance
        self.Q = np.diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
        # Measurement noise covariance for encoder and IMU (customizable)
        self.R_odom = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])
        self.R_imu = np.diag([0.1, 0.1, 0.2, 0.2])

        # Measurement vector
        self.z_odom = np.zeros(6)
        self.z_imu = np.zeros(4)
        self.prev_z_odom = np.zeros(6)
        self.curr_z_odom = np.zeros(6)
        self.curr_z_imu = np.zeros(4)
        self.prev_z_imu = np.zeros(4)

        self.final_coord = np.zeros(6)

        # Flags to check if new data is available
        self.new_odom_data = False
        self.new_imu_data = False

        self.timing1 = 0
        self.timing2 = 0

    def encoder_callback(self, msg: Float32MultiArray):
        # Extract data from encoder
        if (len(msg.data) > 0):
            # Data encoder: [X, Y, theta, Vx, Vy, omega]
            self.z_odom = np.array(msg.data)
            self.new_odom_data = True

    def imu_callback(self, msg: Float32MultiArray):
        # Extract data from IMU
        if (len(msg.data) > 0):
            # Data IMU: [roll, pitch, yaw, wx, wy, wz, ax, ay, az]
            yaw, wz, ax, ay = msg.data[0], msg.data[1], msg.data[2], msg.data[3]
            self.z_imu = np.array([yaw, wz, ax*100, ay*100])
            self.prev_z_odom = self.curr_z_odom
            self.curr_z_odom = self.z_odom
            self.new_imu_data = True
    
    def predict(self):
        # Prediksi posisi dan orientasi berdasarkan model gerak
        Vx, Vy, omega = self.x[3:]
        # Prediksi posisi berdasarkan kecepatan
        self.x[0] += Vx * self.dt
        self.x[1] += Vy * self.dt
        self.x[2] += omega * self.dt

        # Matriks transisi state
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        # Perbarui kovarians
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z, sensor_type, lin_slip_detected = False, ang_slip_detected = False):
        # Pembaruan state berdasarkan sensor
        if sensor_type == 'odom':
            H = np.eye(6)
            R = self.R_odom * (2 if lin_slip_detected else 1)
            if ang_slip_detected:
                R[2, 2] *= 10  # Elemen terkait yaw
                R[5, 5] *= 10  # Elemen terkait omega
            else:
                R[2, 2] *= 1  # Elemen terkait yaw
                R[5, 5] *= 1  # Elemen terkait omega
        elif sensor_type == 'imu':
            # Hanya memperbarui [theta, omega, Vx, Vy]
            H = np.zeros((4, 6))
            H[0, 2] = 1  # yaw -> theta
            H[1, 5] = 1  # wz -> omega
            H[2, 3] = 1  # ax -> Vx
            H[3, 4] = 1  # ay -> Vy
            R = self.R_imu
            if ang_slip_detected:
                R[0, 0] *= 0.1  # Elemen terkait yaw
                R[1, 1] *= 0.1  # Elemen terkait omega
            else:
                R[0, 0] *= 1  # Elemen terkait yaw
                R[1, 1] *= 1  # Elemen terkait omega
            # Koreksi kecepatan dengan integrasi percepatan
            ax, ay = z[2], z[3]
            theta = np.deg2rad(self.x[2])
            ax_global = ax * np.cos(theta) - ay * np.sin(theta)
            ay_global = ax * np.sin(theta) + ay * np.cos(theta)
            self.x[3] += ax_global * self.dt
            self.x[4] += ay_global * self.dt
            self.x[0] += 0.5 * ax_global * self.dt**2
            self.x[1] += 0.5 * ay_global * self.dt**2
            self.x[2] += (self.curr_z_imu[0]-self.prev_z_imu[0])

        # Gain Kalman
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Perbarui estimasi state
        y = z - H @ self.x
        self.x += K @ y
        # Normalize the orientation to [-pi, pi]
        self.x[2] = (self.x[2] + 180) % (360) - 180

        # Perbarui kovarians
        self.P = (np.eye(6) - K @ H) @ self.P

    def detect_linear_slip(self, z_odom, z_imu)->tuple:
        # Check difference between odom and IMU measurements for slip detection
        v_enc = np.array([(z_odom[3]-self.prev_z_odom[3]), (z_odom[4]-self.prev_z_odom[4])])  # Vx, Vy from state
        v_imu = np.array([z_imu[2], z_imu[3]])

        velocity_diff = np.linalg.norm(v_enc - v_imu)
        slip_threshold = 50  # Set this threshold based on experimental data
        return (abs(velocity_diff) > slip_threshold), velocity_diff
    
    def detect_angular_slip(self, z_odom, z_imu)->tuple:
        # Check difference between odom and IMU measurements for slip detection
        velocity_diff = (z_odom[5] - z_imu[1])
        slip_threshold = 5  # Set this threshold based on experimental data
        return (abs(velocity_diff) > slip_threshold), velocity_diff

    def update_filter(self):
        self.timing2 = self.timing1
        self.timing1 = time.time()*1000000
        final_pose = Odometry()
        if not (self.new_odom_data and self.new_imu_data):
            return
        self.prev_z_imu = self.curr_z_imu
        self.curr_z_imu = self.z_imu

        Is_lin_slip, lin_vel = self.detect_linear_slip(self.z_odom, self.curr_z_imu)
        Is_ang_slip, ang_vel = self.detect_angular_slip(self.z_odom, self.curr_z_imu)

        self.predict()
        # Perbarui dengan pengukuran dari encoder
        self.update(self.z_odom, 'odom', lin_slip_detected=Is_lin_slip, ang_slip_detected=Is_ang_slip)

        # Perbarui dengan pengukuran dari IMU
        self.update(self.z_imu, 'imu', lin_slip_detected=Is_lin_slip, ang_slip_detected=Is_ang_slip)

        final_pose.pose.pose.position.x = self.x[0]
        final_pose.pose.pose.position.y = self.x[1]
        final_pose.pose.pose.orientation.z = self.x[2]
        final_pose.twist.twist.linear.x = self.z_odom[3]
        final_pose.twist.twist.linear.y = self.z_odom[4]
        final_pose.twist.twist.angular.z = self.z_odom[5]

        self.odom_filtered_pub.publish(final_pose)

        self.new_odom_data = False
        self.new_imu_data = False
        self.get_logger().info(f"X: {self.timing1-self.timing2} | Y: {time.time()*1000000-self.timing2}")
        #self.get_logger().info(f"X: {self.x[0]:.2f} | Y: {self.x[1]:.2f} | Theta: {self.x[2]:.2f} | Lin_err: {lin_vel:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
