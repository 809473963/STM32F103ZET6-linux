#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{
std::vector<double> parse_csv_doubles(const std::string & line)
{
  std::vector<double> values;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    try {
      values.push_back(std::stod(item));
    } catch (const std::exception &) {
      values.clear();
      return values;
    }
  }
  return values;
}
}  // namespace

class ChassisStateUdpBridgeNode : public rclcpp::Node
{
public:
  ChassisStateUdpBridgeNode() : Node("chassis_state_udp_bridge_node")
  {
    udp_port_ = this->declare_parameter<int>("udp_port", 15050);
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
    complementary_alpha_ = this->declare_parameter<double>("complementary_alpha", 0.98);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    voltage_pub_ = this->create_publisher<std_msgs::msg::Float32>("/chassis/battery_voltage", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    open_udp_socket();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ChassisStateUdpBridgeNode::poll_udp, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Listening for chassis state UDP on 0.0.0.0:%d. Expected CSV: "
      "vx,vy,vz,ax,ay,az,gx,gy,gz,voltage,flag_stop",
      udp_port_);
  }

  ~ChassisStateUdpBridgeNode() override
  {
    if (sock_fd_ >= 0) {
      close(sock_fd_);
    }
  }

private:
  void open_udp_socket()
  {
    sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
      throw std::runtime_error("Failed to create UDP socket");
    }

    int reuse = 1;
    setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(udp_port_));

    if (bind(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("Failed to bind UDP socket");
    }

    const int flags = fcntl(sock_fd_, F_GETFL, 0);
    fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  void poll_udp()
  {
    char buffer[512];
    sockaddr_in src{};
    socklen_t src_len = sizeof(src);

    while (true) {
      const ssize_t n = recvfrom(
        sock_fd_, buffer, sizeof(buffer) - 1, 0,
        reinterpret_cast<sockaddr *>(&src), &src_len);
      if (n <= 0) {
        break;
      }
      buffer[n] = '\0';
      handle_packet(std::string(buffer, static_cast<size_t>(n)));
    }
  }

  void handle_packet(const std::string & line)
  {
    const auto values = parse_csv_doubles(line);
    if (values.size() < 11) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Bad chassis packet: '%s'", line.c_str());
      return;
    }

    const auto now = this->now();
    double dt = 0.0;
    if (last_stamp_.nanoseconds() > 0) {
      dt = (now - last_stamp_).seconds();
    }
    last_stamp_ = now;

    const double vx = values[0];
    const double vy = values[1];
    const double vz = values[2];
    const double ax = values[3];
    const double ay = values[4];
    const double az = values[5];
    const double gx = values[6];
    const double gy = values[7];
    const double gz = values[8];
    const double voltage = values[9];

    update_pose(vx, vy, vz, dt);
    update_orientation_from_imu(ax, ay, az, gx, gy, gz, dt);

    publish_imu(now, ax, ay, az, gx, gy, gz);
    publish_odom(now, vx, vy, vz);
    publish_voltage(voltage);
    if (publish_tf_) {
      publish_transform(now);
    }
  }

  void update_pose(double vx, double vy, double wz, double dt)
  {
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }
    const double cos_yaw = std::cos(yaw_);
    const double sin_yaw = std::sin(yaw_);
    x_ += (vx * cos_yaw - vy * sin_yaw) * dt;
    y_ += (vx * sin_yaw + vy * cos_yaw) * dt;
    yaw_ = normalize_angle(yaw_ + wz * dt);
  }

  void update_orientation_from_imu(
    double ax, double ay, double az,
    double gx, double gy, double gz,
    double dt)
  {
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    const double accel_roll = std::atan2(ay, az);
    const double accel_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    const double alpha = std::clamp(complementary_alpha_, 0.0, 1.0);

    roll_ = alpha * (roll_ + gx * dt) + (1.0 - alpha) * accel_roll;
    pitch_ = alpha * (pitch_ + gy * dt) + (1.0 - alpha) * accel_pitch;
    yaw_ = normalize_angle(yaw_ + gz * dt * 0.0);  // yaw already comes from chassis vz integration.
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  tf2::Quaternion orientation_quat() const
  {
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    q.normalize();
    return q;
  }

  void publish_imu(
    const rclcpp::Time & stamp,
    double ax, double ay, double az,
    double gx, double gy, double gz)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = base_frame_;

    const auto q = orientation_quat();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();

    msg.angular_velocity.x = gx;
    msg.angular_velocity.y = gy;
    msg.angular_velocity.z = gz;
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    msg.orientation_covariance[0] = 0.05;
    msg.orientation_covariance[4] = 0.05;
    msg.orientation_covariance[8] = 0.10;
    msg.angular_velocity_covariance[0] = 0.02;
    msg.angular_velocity_covariance[4] = 0.02;
    msg.angular_velocity_covariance[8] = 0.02;
    msg.linear_acceleration_covariance[0] = 0.10;
    msg.linear_acceleration_covariance[4] = 0.10;
    msg.linear_acceleration_covariance[8] = 0.10;

    imu_pub_->publish(msg);
  }

  void publish_odom(const rclcpp::Time & stamp, double vx, double vy, double wz)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;
    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;

    const auto q = orientation_quat();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = wz;

    odom_pub_->publish(msg);
  }

  void publish_voltage(double voltage)
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(voltage);
    voltage_pub_->publish(msg);
  }

  void publish_transform(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    const auto q = orientation_quat();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }

  int udp_port_{15050};
  int sock_fd_{-1};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  bool publish_tf_{true};
  double complementary_alpha_{0.98};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  double roll_{0.0};
  double pitch_{0.0};
  rclcpp::Time last_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisStateUdpBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
