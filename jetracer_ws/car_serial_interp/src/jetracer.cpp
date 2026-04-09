#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <boost/asio.hpp>

// Protocol Definitions
#define HEAD1 0xAA
#define HEAD2 0x55
#define SEND_TYPE_VELOCITY    0x11
#define SEND_TYPE_PARAMS      0x12
#define SEND_TYPE_COEFFICIENT 0x13

using namespace std::chrono_literals;
using namespace boost::asio;

class JetracerNode : public rclcpp::Node
{
public:
    JetracerNode() : Node("jetracer"), io_service_(), serial_port_(io_service_)
    {
        // 1. Declare Parameters (replacing dynamic_reconfigure and param server)
        this->declare_parameter("port_name", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("publish_odom_transform", true);
        this->declare_parameter("linear_correction", 1.0);
        this->declare_parameter("servo_bias", 0);
        // PID params
        this->declare_parameter("kp", 350);
        this->declare_parameter("ki", 120);
        this->declare_parameter("kd", 50);
        // Coefficient params
        this->declare_parameter("coefficient_a", -0.016073);
        this->declare_parameter("coefficient_b", 0.176183);
        this->declare_parameter("coefficient_c", -23.428084);
        this->declare_parameter("coefficient_d", 1500.0);

        // 2. Initialize Variables from Parameters
        port_name_ = this->get_parameter("port_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        publish_odom_tf_ = this->get_parameter("publish_odom_transform").as_bool();
        linear_correction_ = this->get_parameter("linear_correction").as_double();
        servo_bias_ = this->get_parameter("servo_bias").as_int();

        kp_ = this->get_parameter("kp").as_int();
        ki_ = this->get_parameter("ki").as_int();
        kd_ = this->get_parameter("kd").as_int();

        float a = this->get_parameter("coefficient_a").as_double();
        float b = this->get_parameter("coefficient_b").as_double();
        float c = this->get_parameter("coefficient_c").as_double();
        float d = this->get_parameter("coefficient_d").as_double();

        // 3. Setup Serial Port
        try {
            serial_port_.open(port_name_);
            serial_port_.set_option(serial_port::baud_rate(baud_rate_));
            serial_port_.set_option(serial_port::flow_control(serial_port::flow_control::none));
            serial_port_.set_option(serial_port::parity(serial_port::parity::none));
            serial_port_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
            serial_port_.set_option(serial_port::character_size(8));
            RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", port_name_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            // In ROS2 we might want to shut down, but here we just log error
        }

        // 4. Setup Pubs/Subs/TF
        odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        imu_pub_  = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        lvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lvel", 10);
        rvel_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rvel", 10);
        lset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/lset", 10);
        rset_pub_ = this->create_publisher<std_msgs::msg::Int32>("motor/rset", 10);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&JetracerNode::cmd_callback, this, std::placeholders::_1));

        // 5. Initial Configuration Send
        // Sleep briefly to let MCU reset if needed, then send configs
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        send_coefficient(a, b, c, d);
        send_params(kp_, ki_, kd_, linear_correction_, servo_bias_);

        // 6. Parameter Callback (Dynamic Reconfigure replacement)
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&JetracerNode::on_parameter_change, this, std::placeholders::_1));

        // 7. Start Threads/Timers
        // Timer for sending velocity commands (50Hz)
        cmd_time_ = this->now();
        control_timer_ = this->create_wall_timer(
            20ms, std::bind(&JetracerNode::control_loop, this));

        // Thread for reading serial data
        reading_thread_ = std::thread(&JetracerNode::serial_read_task, this);
    }

    ~JetracerNode() {
        if (reading_thread_.joinable()) {
            reading_thread_.detach(); // Or handle graceful shutdown signal
        }
        serial_port_.close();
    }

private:
    // --- Member Variables ---
    io_service io_service_;
    serial_port serial_port_;
    std::string port_name_;
    int baud_rate_;

    // Publishers
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lvel_pub_, rvel_pub_, lset_pub_, rset_pub_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    // Timer & Callbacks
    rclcpp::TimerBase::SharedPtr control_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    std::thread reading_thread_;

    // State Variables
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    rclcpp::Time cmd_time_;

    bool publish_odom_tf_;
    int kp_, ki_, kd_, servo_bias_;
    float linear_correction_;

    // --- Serial Protocol Helpers ---
    uint8_t checksum(uint8_t* buf, size_t len) {
        uint8_t sum = 0x00;
        for(size_t i=0; i<len; i++) sum += *(buf + i);
        return sum;
    }

    void send_params(int p, int i, int d, float linear_correction, int servo_bias) {
        uint8_t buf[15];
        buf[0] = HEAD1;
        buf[1] = HEAD2;
        buf[2] = 0x0F; // length
        buf[3] = SEND_TYPE_PARAMS;
        buf[4] = static_cast<uint8_t>((p >> 8) & 0xFF);
        buf[5] = static_cast<uint8_t>(p & 0xFF);
        buf[6] = static_cast<uint8_t>((i >> 8) & 0xFF);
        buf[7] = static_cast<uint8_t>(i & 0xFF);
        buf[8] = static_cast<uint8_t>((d >> 8) & 0xFF);
        buf[9] = static_cast<uint8_t>(d & 0xFF);
        uint16_t lin_corr_int = static_cast<uint16_t>(linear_correction * 1000);
        buf[10] = static_cast<uint8_t>((lin_corr_int >> 8) & 0xFF);
        buf[11] = static_cast<uint8_t>(lin_corr_int & 0xFF);
        buf[12] = static_cast<uint8_t>((servo_bias >> 8) & 0xFF);
        buf[13] = static_cast<uint8_t>(servo_bias & 0xFF);
        buf[14] = checksum(buf, 14);

        try {
            write(serial_port_, buffer(buf,sizeof(buf)));
            RCLCPP_INFO(this->get_logger(), "SetParams: p=%d i=%d d=%d corr=%.2f bias=%d", 
                p, i, d, linear_correction, servo_bias);
        } catch (...) {}
    }

    void send_coefficient(float a, float b, float c, float d) {
        uint8_t buf[21];
        char* p;
        buf[0] = HEAD1;
        buf[1] = HEAD2;
        buf[2] = 0x15; // length
        buf[3] = SEND_TYPE_COEFFICIENT;
        p = (char*)&a;
        buf[4] = static_cast<uint8_t>(p[0]);
        buf[5] = static_cast<uint8_t>(p[1]);
        buf[6] = static_cast<uint8_t>(p[2]);
        buf[7] = static_cast<uint8_t>(p[3]);
        p = (char*)&b;
        buf[8] = static_cast<uint8_t>(p[0]);
        buf[9] = static_cast<uint8_t>(p[1]);
        buf[10] = static_cast<uint8_t>(p[2]);
        buf[11] = static_cast<uint8_t>(p[3]);
        p = (char*)&c;
        buf[12] = static_cast<uint8_t>(p[0]);
        buf[13] = static_cast<uint8_t>(p[1]);
        buf[14] = static_cast<uint8_t>(p[2]);
        buf[15] = static_cast<uint8_t>(p[3]);
        p = (char*)&d;
        buf[16] = static_cast<uint8_t>(p[0]);
        buf[17] = static_cast<uint8_t>(p[1]);
        buf[18] = static_cast<uint8_t>(p[2]);
        buf[19] = static_cast<uint8_t>(p[3]);
        buf[20] = checksum(buf, 20);

        try {
            write(serial_port_, buffer(buf,sizeof(buf)));
            RCLCPP_INFO(this->get_logger(), "SetCoefficient: a=%.4f b=%.4f c=%.4f d=%.4f", a, b, c, d);
        } catch (...) {}
    }

    void send_velocity(double x, double /* y */, double yaw) {
        int16_t linear_vel = static_cast<int16_t>(x * 1000);
        int16_t angular_vel = static_cast<int16_t>(yaw * 1000);
        static uint8_t tmp[11];
        tmp[0] = HEAD1;
        tmp[1] = HEAD2;
        tmp[2] = 0x0b;
        tmp[3] = SEND_TYPE_VELOCITY;
        tmp[4] = static_cast<uint8_t>((linear_vel >> 8) & 0xFF);
        tmp[5] = static_cast<uint8_t>(linear_vel & 0xFF);
        tmp[6] = 0;
        tmp[7] = 0;
        tmp[8] = static_cast<uint8_t>((angular_vel >> 8) & 0xFF);
        tmp[9] = static_cast<uint8_t>(angular_vel & 0xFF);
        tmp[10] = checksum(tmp, 10);

        try {
            write(serial_port_, buffer(tmp,sizeof(tmp)));
        } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "send_velocity failed: %s", e.what());
        }
    }

    // --- Callbacks ---
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        x_ = msg->linear.x;
        y_ = msg->linear.y; // Original code mapped linear.x to y as well? Kept as per original.
        yaw_ = msg->angular.z;
        //RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=(%.3f, %.3f), angular=%.3f",x_, y_, yaw_);
        cmd_time_ = this->now();
    }

    void control_loop() {
        rclcpp::Time current_time = this->now();
        // Timeout safety
        if ((current_time - cmd_time_).seconds() > 1.0) {
            x_ = 0.0;
            y_ = 0.0;
            yaw_ = 0.0;
        }
        send_velocity(x_, y_, yaw_);
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters) 
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        bool update_params = false;

        for (const auto &param : parameters) {
            if (param.get_name() == "kp") { kp_ = param.as_int(); update_params = true; }
            else if (param.get_name() == "ki") { ki_ = param.as_int(); update_params = true; }
            else if (param.get_name() == "kd") { kd_ = param.as_int(); update_params = true; }
            else if (param.get_name() == "servo_bias") { servo_bias_ = param.as_int(); update_params = true; }
            else if (param.get_name() == "linear_correction") { linear_correction_ = param.as_double(); update_params = true; }
        }

        if (update_params) {
            send_params(kp_, ki_, kd_, linear_correction_, servo_bias_);
        }

        return result;
    }

    // --- Serial Read Task ---
    void serial_read_task() {
        enum frameState { State_Head1, State_Head2, State_Size, State_Data, State_CheckSum, State_Handle };
        frameState state = State_Head1;

        uint8_t frame_size, frame_sum;
        uint8_t data[50];

        double imu_list[9];
        double odom_list[6];
        rclcpp::Time now_time, last_time;
        last_time = this->now();

        RCLCPP_INFO(this->get_logger(), "Start receive message");

        while(rclcpp::ok()) {
            if(!serial_port_.is_open()) {
                std::this_thread::sleep_for(1s);
                continue;
            }

            try {
                // State machine
                switch (state) {
                    case State_Head1:
                        frame_sum = 0x00;
                        read(serial_port_, buffer(&data[0], 1));
                        state = (data[0] == HEAD1 ? State_Head2 : State_Head1);
                        break;

                    case State_Head2:
                        read(serial_port_, buffer(&data[1], 1));
                        state = (data[1] == HEAD2 ? State_Size : State_Head1);
                        break;

                    case State_Size:
                        read(serial_port_, buffer(&data[2], 1));
                        frame_size = data[2];
                        state = State_Data;
                        break;

                    case State_Data:
                        read(serial_port_, buffer(&data[3], frame_size - 4));
                        state = State_CheckSum;
                        break;

                    case State_CheckSum:
                        read(serial_port_, buffer(&data[frame_size - 1], 1));
                        frame_sum = checksum(data, frame_size - 1);
                        state = (data[frame_size - 1] == frame_sum) ? State_Handle : State_Head1;
                        break;

                    case State_Handle:
                        now_time = this->now();

                        // IMU Decoding
                        imu_list[0]=((double)((int16_t)(data[4]*256+data[5]))/32768*2000/180*3.1415);
                        imu_list[1]=((double)((int16_t)(data[6]*256+data[7]))/32768*2000/180*3.1415);
                        imu_list[2]=((double)((int16_t)(data[8]*256+data[9]))/32768*2000/180*3.1415);
                        imu_list[3]=((double)((int16_t)(data[10]*256+data[11]))/32768*2*9.8);
                        imu_list[4]=((double)((int16_t)(data[12]*256+data[13]))/32768*2*9.8);
                        imu_list[5]=((double)((int16_t)(data[14]*256+data[15]))/32768*2*9.8);
                        imu_list[6]=((double)((int16_t)(data[16]*256+data[17]))/10.0);
                        imu_list[7]=((double)((int16_t)(data[18]*256+data[19]))/10.0);
                        imu_list[8]=((double)((int16_t)(data[20]*256+data[21]))/10.0);

                        // Publish IMU
                        {
                            sensor_msgs::msg::Imu imu_msg;
                            imu_msg.header.stamp = now_time;
                            imu_msg.header.frame_id = "base_imu_link";
                            imu_msg.angular_velocity.x = imu_list[0];
                            imu_msg.angular_velocity.y = imu_list[1];
                            imu_msg.angular_velocity.z = imu_list[2];
                            imu_msg.linear_acceleration.x = imu_list[3];
                            imu_msg.linear_acceleration.y = imu_list[4];
                            imu_msg.linear_acceleration.z = imu_list[5];

                            tf2::Quaternion q;
                            q.setRPY(0, 0, imu_list[8]/180.0*3.1415926);
                            imu_msg.orientation = tf2::toMsg(q);

                            imu_msg.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 0.05};
                            imu_msg.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e6};
                            imu_msg.linear_acceleration_covariance = {1e-2, 0, 0, 0, 0, 0, 0, 0, 0};
                            imu_pub_->publish(imu_msg);
                        }

                        // Odom Decoding
                        odom_list[0]=((double)((int16_t)(data[22]*256+data[23]))/1000); // x
                        odom_list[1]=((double)((int16_t)(data[24]*256+data[25]))/1000); // y
                        odom_list[2]=((double)((int16_t)(data[26]*256+data[27]))/1000); // yaw
                        odom_list[3]=((double)((int16_t)(data[28]*256+data[29]))/1000); // dx
                        odom_list[4]=((double)((int16_t)(data[30]*256+data[31]))/1000); // dy
                        odom_list[5]=((double)((int16_t)(data[32]*256+data[33]))/1000); // dyaw

                        // TF Broadcasting
                        if (publish_odom_tf_) {
                            geometry_msgs::msg::TransformStamped odom_trans;
                            odom_trans.header.stamp = now_time;
                            odom_trans.header.frame_id = "odom";
                            odom_trans.child_frame_id = "base_footprint";
                            odom_trans.transform.translation.x = odom_list[0];
                            odom_trans.transform.translation.y = odom_list[1];
                            odom_trans.transform.translation.z = 0.0;

                            tf2::Quaternion q;
                            q.setRPY(0, 0, odom_list[2]);
                            odom_trans.transform.rotation = tf2::toMsg(q);

                            odom_broadcaster_->sendTransform(odom_trans);
                        }

                        // Publish Odom
                        {
                            nav_msgs::msg::Odometry odom_msg;
                            odom_msg.header.stamp = now_time;
                            odom_msg.header.frame_id = "odom";
                            odom_msg.pose.pose.position.x = odom_list[0];
                            odom_msg.pose.pose.position.y = odom_list[1];
                            odom_msg.pose.pose.position.z = 0.0;
                            odom_msg.pose.covariance = {
                                0.02, 0,    0,    0,    0,    0,
                                0,    0.02, 0,    0,    0,    0,
                                0,    0,    1e6,  0,    0,    0,
                                0,    0,    0,    1e6,  0,    0,
                                0,    0,    0,    0,    1e6,  0,
                                0,    0,    0,    0,    0,    0.05
                              };

                            tf2::Quaternion q;
                            q.setRPY(0, 0, odom_list[2]);
                            odom_msg.pose.pose.orientation = tf2::toMsg(q);

                            odom_msg.child_frame_id = "base_footprint";
                            double dt = (now_time - last_time).seconds();
                            if (dt == 0) dt = 0.02; // Avoid division by zero

                            odom_msg.twist.twist.linear.x = odom_list[3]/dt;
                            odom_msg.twist.twist.linear.y = odom_list[4]/dt;
                            odom_msg.twist.twist.angular.z = odom_list[5]/dt;
                            odom_msg.twist.covariance = {
                                0.05, 0,    0,    0,    0,    0,
                                0,    0.05, 0,    0,    0,    0,
                                0,    0,    1e6,  0,    0,    0,
                                0,    0,    0,    1e6,  0,    0,
                                0,    0,    0,    0,    1e6,  0,
                                0,    0,    0,    0,    0,    0.10
                              };

                            // Covariances (simplified assignment)
                            // ... (fill if needed, mostly 0 or large values for unknown)
                            odom_pub_->publish(odom_msg);
                        }

                        // Motor Data
                        {
                            std_msgs::msg::Int32 m;
                            m.data = ((int16_t)(data[34]*256+data[35])); lvel_pub_->publish(m);
                            m.data = ((int16_t)(data[36]*256+data[37])); rvel_pub_->publish(m);
                            m.data = ((int16_t)(data[38]*256+data[39])); lset_pub_->publish(m);
                            m.data = ((int16_t)(data[40]*256+data[41])); rset_pub_->publish(m);
                        }

                        last_time = now_time;
                        state = State_Head1;
                        break;
                    default:
                        state = State_Head1;
                        break;
                }
            } catch (const std::exception &e) {
                // Serial read error, reset state
                state = State_Head1;
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JetracerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 
