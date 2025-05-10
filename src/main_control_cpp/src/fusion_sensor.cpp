#include <chrono>
#include <functional>
#include <memory>
#include <cmath>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("sensor_fusion_node"), dt(0.005) {
        // Initialize Subscribers and Publisher
        subscription_encoder_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "sensor/odom", 10, std::bind(&SensorFusionNode::encoder_callback, this, std::placeholders::_1));
        
        subscription_imu_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "sensor/imu", 10, std::bind(&SensorFusionNode::imu_callback, this, std::placeholders::_1));
        
        odom_filtered_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("sensor/odom_filtered", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&SensorFusionNode::update_filter, this));

        // Initialize State Vector and Covariance Matrices
        x_ = VectorXd::Zero(6);
        P_ = MatrixXd::Identity(6, 6) * 0.1;
        Q_ = MatrixXd::Identity(6, 6) * 0.01;
        R_odom_ = MatrixXd::Identity(6, 6);
        R_odom_.diagonal() << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2;
        R_imu_ = MatrixXd::Identity(4, 4);
        R_imu_.diagonal() << 0.1, 0.1, 0.2, 0.2;

        z_odom_ = VectorXd::Zero(6);
        z_imu_ = VectorXd::Zero(4);

        new_odom_data_ = false;
        new_imu_data_ = false;
    }

private:
    double dt;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_encoder_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_imu_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_filtered_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    VectorXd x_;      // State Vector
    MatrixXd P_;      // Covariance Matrix
    MatrixXd Q_;      // Process Noise
    MatrixXd R_odom_; // Measurement Noise for Encoder
    MatrixXd R_imu_;  // Measurement Noise for IMU
    VectorXd z_odom_;
    VectorXd z_imu_;
    bool new_odom_data_;
    bool new_imu_data_;

    void encoder_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            // Buat array sementara dengan tipe double
            std::vector<double> odom_data(msg->data.begin(), msg->data.end());
            
            // Gunakan Eigen::Map untuk memetakan data
            Eigen::Map<const Eigen::VectorXd> z_odom_(odom_data.data(), odom_data.size());
            //z_odom_ = Eigen::Map<const VectorXd>(msg->data.data(), 6);
            new_odom_data_ = true;
        }
    }

    void imu_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!msg->data.empty()) {
            double yaw = msg->data[0];
            double wz = msg->data[1];
            double ax = msg->data[2], ay = msg->data[3];
            z_imu_ << yaw, wz, ax * 100.0, ay * 100.0;
            new_imu_data_ = true;
        }
    }

    void predict() {
        double theta = x_[2] * M_PI / 180.0;
        double ax_global = z_imu_[2] * cos(theta) - z_imu_[3] * sin(theta);
        double ay_global = z_imu_[2] * sin(theta) + z_imu_[3] * cos(theta);

        x_[0] += x_[3] * dt;
        x_[1] += x_[4] * dt;
        x_[2] += x_[5] * dt;
        x_[3] += ax_global * dt;
        x_[4] += ay_global * dt;
        x_[5] = z_imu_[1];

        MatrixXd F = MatrixXd::Identity(6, 6);
        F(0, 3) = dt;
        F(1, 4) = dt;
        F(2, 5) = dt;

        P_ = F * P_ * F.transpose() + Q_;
    }

    void update(const VectorXd &z, const std::string &sensor_type) {
        MatrixXd H;
        MatrixXd R;
        
        if (sensor_type == "odom") {
            H = MatrixXd::Identity(6, 6);
            R = R_odom_;
        } else if (sensor_type == "imu") {
            H = MatrixXd::Zero(4, 6);
            H(0, 2) = 1;
            H(1, 5) = 1;
            H(2, 3) = 1;
            H(3, 4) = 1;
            R = R_imu_;
        }

        MatrixXd S = H * P_ * H.transpose() + R;
        MatrixXd K = P_ * H.transpose() * S.inverse();

        x_ += K * (z - H * x_);
        x_[2] = fmod(x_[2] + 180.0, 360.0) - 180.0;
        P_ = (MatrixXd::Identity(6, 6) - K * H) * P_;
    }

    void update_filter() {
        if (!new_odom_data_ || !new_imu_data_) return;
        
        predict();
        
        update(z_odom_, "odom");
        update(z_imu_, "imu");

        nav_msgs::msg::Odometry final_pose;
        final_pose.pose.pose.position.x = x_[0];
        final_pose.pose.pose.position.y = x_[1];
        final_pose.pose.pose.orientation.z = x_[2];
        final_pose.twist.twist.linear.x = x_[3];
        final_pose.twist.twist.linear.y = x_[4];
        final_pose.twist.twist.angular.z = x_[5];

        odom_filtered_pub_->publish(final_pose);
        
        new_odom_data_ = false;
        new_imu_data_ = false;

        RCLCPP_INFO(this->get_logger(), "X: %.2f | Y: %.2f | Theta: %.2f", x_[0], x_[1], x_[2]);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
