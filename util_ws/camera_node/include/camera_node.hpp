#ifndef CAMERA_NODE__CAMERA_NODE_HPP_
#define CAMERA_NODE__CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode();

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint32_t frame_count_;

  void on_timer();
};

#endif  // CAMERA_NODE__CAMERA_NODE_HPP_
