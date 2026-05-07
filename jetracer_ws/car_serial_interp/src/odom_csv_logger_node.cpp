#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

class OdomCsvLoggerNode : public rclcpp::Node
{
public:
  OdomCsvLoggerNode() : Node("odom_csv_logger_node"), run_stamp_(makeTimestamp())
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    filtered_topic_ = declare_parameter<std::string>("filtered_topic", "/odometry/filtered");
    output_directory_ = resolveOutputDirectory(
      declare_parameter<std::string>("output_directory", "odom_logs"));
    const auto odom_filename = declare_parameter<std::string>("odom_filename", "odom.csv");
    const auto filtered_filename =
      declare_parameter<std::string>("filtered_filename", "odometry_filtered.csv");
    const bool timestamp_filenames = declare_parameter<bool>("timestamp_filenames", true);

    odom_output_path_ = output_directory_ / makeOutputFilename(odom_filename, timestamp_filenames);
    filtered_output_path_ =
      output_directory_ / makeOutputFilename(filtered_filename, timestamp_filenames);

    try {
      std::filesystem::create_directories(output_directory_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(
        get_logger(), "Could not create odometry CSV log directory '%s': %s",
        output_directory_.string().c_str(), e.what());
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(100),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        appendSample(*msg, odom_samples_);
      });

    filtered_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      filtered_topic_, rclcpp::QoS(100),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        appendSample(*msg, filtered_samples_);
      });

    RCLCPP_INFO(
      get_logger(), "Logging '%s' to '%s' and '%s' to '%s'",
      odom_topic_.c_str(), odom_output_path_.string().c_str(),
      filtered_topic_.c_str(), filtered_output_path_.string().c_str());
  }

  ~OdomCsvLoggerNode() override
  {
    writeLogs();
  }

private:
  struct OdomSample
  {
    int32_t stamp_sec;
    uint32_t stamp_nanosec;
    double stamp;
    double receipt_stamp;
    std::string frame_id;
    std::string child_frame_id;
    double pose_x;
    double pose_y;
    double pose_z;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_w;
    double roll;
    double pitch;
    double yaw;
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;
    std::array<double, 36> pose_covariance;
    std::array<double, 36> twist_covariance;
  };

  static std::string makeTimestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &now_time);
#else
    localtime_r(&now_time, &tm);
#endif

    std::ostringstream stream;
    stream << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return stream.str();
  }

  static std::filesystem::path resolveOutputDirectory(const std::string &directory)
  {
    if (directory.empty()) {
      return std::filesystem::absolute(".");
    }

    if (directory == "~" || directory.rfind("~/", 0) == 0) {
      const char *home = std::getenv("HOME");
      if (home != nullptr) {
        if (directory == "~") {
          return std::filesystem::absolute(home);
        }
        return std::filesystem::absolute(std::filesystem::path(home) / directory.substr(2));
      }
    }

    return std::filesystem::absolute(directory);
  }

  std::filesystem::path makeOutputFilename(const std::string &filename, bool timestamp) const
  {
    const std::filesystem::path path(filename);
    if (!timestamp) {
      return path;
    }

    std::string extension = path.extension().string();
    std::string stem = path.stem().string();
    if (stem.empty()) {
      stem = path.filename().string();
      extension.clear();
    }

    return path.parent_path() / (stem + "_" + run_stamp_ + extension);
  }

  void appendSample(
    const nav_msgs::msg::Odometry &msg,
    std::vector<OdomSample> &samples)
  {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    const auto &orientation = msg.pose.pose.orientation;
    const tf2::Quaternion quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    const rclcpp::Time receipt_time = now();
    OdomSample sample{
      msg.header.stamp.sec,
      msg.header.stamp.nanosec,
      rclcpp::Time(msg.header.stamp).seconds(),
      receipt_time.seconds(),
      msg.header.frame_id,
      msg.child_frame_id,
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z,
      orientation.x,
      orientation.y,
      orientation.z,
      orientation.w,
      roll,
      pitch,
      yaw,
      msg.twist.twist.linear.x,
      msg.twist.twist.linear.y,
      msg.twist.twist.linear.z,
      msg.twist.twist.angular.x,
      msg.twist.twist.angular.y,
      msg.twist.twist.angular.z,
      msg.pose.covariance,
      msg.twist.covariance
    };

    std::lock_guard<std::mutex> lock(samples_mutex_);
    samples.push_back(std::move(sample));
  }

  static std::string quoteCsv(const std::string &value)
  {
    std::string quoted;
    quoted.reserve(value.size() + 2);
    quoted.push_back('"');
    for (const char character : value) {
      if (character == '"') {
        quoted.push_back('"');
      }
      quoted.push_back(character);
    }
    quoted.push_back('"');
    return quoted;
  }

  static void writeHeader(std::ofstream &file)
  {
    file << "stamp_sec,stamp_nanosec,stamp,receipt_stamp,frame_id,child_frame_id,"
         << "pose_x,pose_y,pose_z,orientation_x,orientation_y,orientation_z,orientation_w,"
         << "roll,pitch,yaw,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z";

    for (int i = 0; i < 36; ++i) {
      file << ",pose_covariance_" << i;
    }
    for (int i = 0; i < 36; ++i) {
      file << ",twist_covariance_" << i;
    }
    file << '\n';
  }

  static void writeSample(std::ofstream &file, const OdomSample &sample)
  {
    file << sample.stamp_sec << ','
         << sample.stamp_nanosec << ','
         << sample.stamp << ','
         << sample.receipt_stamp << ','
         << quoteCsv(sample.frame_id) << ','
         << quoteCsv(sample.child_frame_id) << ','
         << sample.pose_x << ','
         << sample.pose_y << ','
         << sample.pose_z << ','
         << sample.orientation_x << ','
         << sample.orientation_y << ','
         << sample.orientation_z << ','
         << sample.orientation_w << ','
         << sample.roll << ','
         << sample.pitch << ','
         << sample.yaw << ','
         << sample.linear_x << ','
         << sample.linear_y << ','
         << sample.linear_z << ','
         << sample.angular_x << ','
         << sample.angular_y << ','
         << sample.angular_z;

    for (const double covariance : sample.pose_covariance) {
      file << ',' << covariance;
    }
    for (const double covariance : sample.twist_covariance) {
      file << ',' << covariance;
    }
    file << '\n';
  }

  void writeCsv(
    const std::filesystem::path &path,
    const std::vector<OdomSample> &samples) const
  {
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("failed to open " + path.string());
    }

    file << std::setprecision(17);
    writeHeader(file);
    for (const auto &sample : samples) {
      writeSample(file, sample);
    }
  }

  void writeLogs()
  {
    std::lock_guard<std::mutex> lock(samples_mutex_);
    if (logs_written_) {
      return;
    }
    logs_written_ = true;

    try {
      writeCsv(odom_output_path_, odom_samples_);
      writeCsv(filtered_output_path_, filtered_samples_);
      RCLCPP_INFO(
        get_logger(), "Wrote %zu samples to '%s' and %zu samples to '%s'",
        odom_samples_.size(), odom_output_path_.string().c_str(),
        filtered_samples_.size(), filtered_output_path_.string().c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to write odometry CSV logs: %s", e.what());
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_sub_;

  std::mutex samples_mutex_;
  std::vector<OdomSample> odom_samples_;
  std::vector<OdomSample> filtered_samples_;

  std::string odom_topic_;
  std::string filtered_topic_;
  std::filesystem::path output_directory_;
  std::filesystem::path odom_output_path_;
  std::filesystem::path filtered_output_path_;
  std::string run_stamp_;
  bool logs_written_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomCsvLoggerNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
