#include "camera_node.hpp"

#include <chrono>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;

CameraNode::CameraNode()
: Node("camera_node"), frame_count_(0)
{
  image_pub_ = image_transport::create_publisher(this, "camera/image_raw");

  timer_ = this->create_wall_timer(500ms, std::bind(&CameraNode::on_timer, this));

  RCLCPP_INFO(this->get_logger(), "camera_node started");
}

void CameraNode::on_timer()
{
  cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(30, 30, 30));
  const int radius = 15;
  const int center_x = radius + static_cast<int>(frame_count_ % (frame.cols - 2 * radius));
  const int center_y = frame.rows / 2;
  cv::circle(frame, cv::Point(center_x, center_y), radius, cv::Scalar(0, 200, 255), cv::FILLED);
  cv::putText(
    frame,
    "frame: " + std::to_string(frame_count_),
    cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX,
    0.7,
    cv::Scalar(255, 255, 255),
    2);

  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "camera_frame";
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
