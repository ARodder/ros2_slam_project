#include "camera_node.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

using namespace std::chrono_literals;

CameraNode::CameraNode()
: Node("camera_node"), frame_count_(0)
{
  device_index_ = this->declare_parameter<int>("device_index", 0);
  width_ = this->declare_parameter<int>("width", 640);
  height_ = this->declare_parameter<int>("height", 480);
  fps_ = this->declare_parameter<double>("fps", 30.0);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_frame");

  image_pub_ = image_transport::create_publisher(this, "camera/image_raw");
  open_camera();

  const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(1.0 / std::max(fps_, 1.0)));
  timer_ = this->create_wall_timer(period, std::bind(&CameraNode::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "camera_node started");
}

bool CameraNode::open_camera()
{
  if (capture_.isOpened()) {
    capture_.release();
  }

  if (!capture_.open(device_index_, cv::CAP_V4L2)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to open camera device %d",
      device_index_);
    return false;
  }

  capture_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
  capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  capture_.set(cv::CAP_PROP_FPS, fps_);

  RCLCPP_INFO(
    this->get_logger(),
    "Opened camera device %d at %.0f FPS",
    device_index_,
    fps_);
  return true;
}

void CameraNode::on_timer()
{
  if (!capture_.isOpened() && !open_camera()) {
    return;
  }

  cv::Mat frame;
  if (!capture_.read(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Failed to read frame from camera device %d",
      device_index_);
    return;
  }

  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = frame_id_;
  msg.height = static_cast<uint32_t>(frame.rows);
  msg.width = static_cast<uint32_t>(frame.cols);
  msg.encoding = "bgr8";
  msg.is_bigendian = 0;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  msg.data.assign(frame.datastart, frame.dataend);

  image_pub_.publish(msg);
  ++frame_count_;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();
  return 0;
}
