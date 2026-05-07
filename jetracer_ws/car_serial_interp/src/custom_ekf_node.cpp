//
// Created using Gemini 3.1 pro on 09/04/2026. Reviewed and modified by Aleksander Røder.
//
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>
#include <memory>

class CustomEkfNode : public rclcpp::Node {
public:
  CustomEkfNode() : Node("custom_ekf_node"), x_(Eigen::Matrix<double, 6, 1>::Zero()),
                    P_(Eigen::Matrix<double, 6, 6>::Identity() * 1e-3) {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    output_topic_ = declare_parameter<std::string>("output_topic", "/odometry/filtered");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    use_imu_yaw_ = declare_parameter<bool>("use_imu_yaw", false);

    q_x_ = declare_parameter<double>("process_noise_x", 0.05);
    q_y_ = declare_parameter<double>("process_noise_y", 0.05);
    q_yaw_ = declare_parameter<double>("process_noise_yaw", 0.03);
    q_vx_ = declare_parameter<double>("process_noise_vx", 0.10);
    q_vy_ = declare_parameter<double>("process_noise_vy", 0.10);
    q_wz_ = declare_parameter<double>("process_noise_wz", 0.08);

    r_odom_x_ = declare_parameter<double>("odom_pose_noise_x", 0.02);
    r_odom_y_ = declare_parameter<double>("odom_pose_noise_y", 0.02);
    r_odom_yaw_ = declare_parameter<double>("odom_pose_noise_yaw", 0.05);
    r_odom_vx_ = declare_parameter<double>("odom_twist_noise_vx", 0.05);
    r_odom_vy_ = declare_parameter<double>("odom_twist_noise_vy", 0.05);
    r_odom_wz_ = declare_parameter<double>("odom_twist_noise_wz", 0.10);
    r_imu_yaw_ = declare_parameter<double>("imu_noise_yaw", 0.10);
    r_imu_wz_ = declare_parameter<double>("imu_noise_wz", 0.02);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&CustomEkfNode::odomCallback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 50, std::bind(&CustomEkfNode::imuCallback, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, 20);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void predict(const rclcpp::Time &stamp) {
    if (!initialized_) {
      last_stamp_ = stamp;
      return;
    }

    double dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      last_stamp_ = stamp;
      return;
    }

    const double yaw = x_(2);
    const double vx = x_(3);
    const double vy = x_(4);
    const double wz = x_(5);

    x_(0) += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    x_(1) += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
    x_(2) = normalizeAngle(x_(2) + wz * dt);

    Matrix6 F = Matrix6::Identity();
    F(0, 2) = (-vx * std::sin(yaw) - vy * std::cos(yaw)) * dt;
    F(0, 3) = std::cos(yaw) * dt;
    F(0, 4) = -std::sin(yaw) * dt;
    F(1, 2) = (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    F(1, 3) = std::sin(yaw) * dt;
    F(1, 4) = std::cos(yaw) * dt;
    F(2, 5) = dt;

    Matrix6 Q = Matrix6::Zero();
    Q(0, 0) = q_x_ * dt;
    Q(1, 1) = q_y_ * dt;
    Q(2, 2) = q_yaw_ * dt;
    Q(3, 3) = q_vx_ * dt;
    Q(4, 4) = q_vy_ * dt;
    Q(5, 5) = q_wz_ * dt;

    P_ = F * P_ * F.transpose() + Q;
    last_stamp_ = stamp;
  }

  void update(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
              bool angle_innovation = false, int angle_index = -1) {
    Eigen::VectorXd y = z - H * x_;
    if (angle_innovation && angle_index >= 0 && angle_index < y.size()) {
      y(angle_index) = normalizeAngle(y(angle_index));
    }

    const Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    x_(2) = normalizeAngle(x_(2));

    const Matrix6 I = Matrix6::Identity();
    P_ = (I - K * H) * P_;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double yaw_meas = 0.0;
    {
      tf2::Quaternion q;
      tf2::fromMsg(msg->pose.pose.orientation, q);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_meas);
    }

    if (!initialized_) {
      x_(0) = msg->pose.pose.position.x;
      x_(1) = msg->pose.pose.position.y;
      x_(2) = yaw_meas;
      x_(3) = msg->twist.twist.linear.x;
      x_(4) = msg->twist.twist.linear.y;
      x_(5) = msg->twist.twist.angular.z;
      initialized_ = true;
      last_stamp_ = msg->header.stamp;
      publish(msg->header.stamp);
      return;
    }

    predict(msg->header.stamp);

    Eigen::Matrix<double, 6, 1> z;
    z << msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        yaw_meas,
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.angular.z;

    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    R(0, 0) = covarianceOrDefault(msg->pose.covariance[0], r_odom_x_);
    R(1, 1) = covarianceOrDefault(msg->pose.covariance[7], r_odom_y_);
    R(2, 2) = covarianceOrDefault(msg->pose.covariance[35], r_odom_yaw_);
    R(3, 3) = covarianceOrDefault(msg->twist.covariance[0], r_odom_vx_);
    R(4, 4) = covarianceOrDefault(msg->twist.covariance[7], r_odom_vy_);
    R(5, 5) = covarianceOrDefault(msg->twist.covariance[35], r_odom_wz_);

    update(z, H, R, true, 2);
    publish(msg->header.stamp);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!initialized_) {
      return;
    }

    predict(msg->header.stamp);

    if (use_imu_yaw_) {
      tf2::Quaternion q;
      tf2::fromMsg(msg->orientation, q);
      double roll, pitch, yaw_meas;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_meas);

      Eigen::Matrix<double, 2, 1> z;
      z << yaw_meas, msg->angular_velocity.z;

      Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
      H(0, 2) = 1.0;
      H(1, 5) = 1.0;

      Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
      R(0, 0) = covarianceOrDefault(msg->orientation_covariance[8], r_imu_yaw_);
      R(1, 1) = covarianceOrDefault(msg->angular_velocity_covariance[8], r_imu_wz_);

      update(z, H, R, true, 0);
    } else {
      Eigen::Matrix<double, 1, 1> z;
      z << msg->angular_velocity.z;

      Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
      H(0, 5) = 1.0;

      Eigen::Matrix<double, 1, 1> R;
      R(0, 0) = covarianceOrDefault(msg->angular_velocity_covariance[8], r_imu_wz_);

      update(z, H, R, false, -1);
    }

    publish(msg->header.stamp);
  }

  double covarianceOrDefault(double value, double fallback) const {
    if (!std::isfinite(value) || value <= 0.0) {
      return fallback;
    }
    return value;
  }

  void publish(const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = x_(0);
    msg.pose.pose.position.y = x_(1);
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x_(2));
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.twist.twist.linear.x = x_(3);
    msg.twist.twist.linear.y = x_(4);
    msg.twist.twist.angular.z = x_(5);

    for (int i = 0; i < 36; ++i) {
      msg.pose.covariance[i] = 0.0;
      msg.twist.covariance[i] = 0.0;
    }

    msg.pose.covariance[0] = P_(0, 0);
    msg.pose.covariance[1] = P_(0, 1);
    msg.pose.covariance[6] = P_(1, 0);
    msg.pose.covariance[7] = P_(1, 1);
    msg.pose.covariance[35] = P_(2, 2);

    msg.pose.covariance[14] = 1e6;
    msg.pose.covariance[21] = 1e6;
    msg.pose.covariance[28] = 1e6;

    msg.twist.covariance[0] = P_(3, 3);
    msg.twist.covariance[7] = P_(4, 4);
    msg.twist.covariance[35] = P_(5, 5);
    msg.twist.covariance[14] = 1e6;
    msg.twist.covariance[21] = 1e6;
    msg.twist.covariance[28] = 1e6;

    odom_pub_->publish(msg);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = msg.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_(0);
      tf.transform.translation.y = x_(1);
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  bool use_imu_yaw_;

  double q_x_, q_y_, q_yaw_, q_vx_, q_vy_, q_wz_;
  double r_odom_x_, r_odom_y_, r_odom_yaw_, r_odom_vx_, r_odom_vy_, r_odom_wz_;
  double r_imu_yaw_, r_imu_wz_;

  bool initialized_ = false;
  rclcpp::Time last_stamp_;
  Vector6 x_;
  Matrix6 P_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomEkfNode>());
  rclcpp::shutdown();
  return 0;
}
