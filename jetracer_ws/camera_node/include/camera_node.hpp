#ifndef CAMERA_NODE__CAMERA_NODE_HPP_
#define CAMERA_NODE__CAMERA_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <string>

class CameraNode : public rclcpp::Node
{
public:
  CameraNode();

private:
  image_transport::Publisher image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture capture_;
  std::uint32_t frame_count_;
  int device_index_;
  int width_;
  int height_;
  double fps_;
  std::string frame_id_;

  bool open_camera();
  void on_timer();
};

#endif  // CAMERA_NODE__CAMERA_NODE_HPP_
