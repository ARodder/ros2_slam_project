#ifndef CAMERA_NODE__CAMERA_NODE_HPP_
#define CAMERA_NODE__CAMERA_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode();

private:
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint32_t frame_count_;

  void on_timer();
};

#endif  // CAMERA_NODE__CAMERA_NODE_HPP_
