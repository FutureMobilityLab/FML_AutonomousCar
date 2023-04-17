#include "as5600driver/as5600driver.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

AS5600Driver::AS5600Driver()
    : Node("as5600publisher"), as5600_{std::make_unique<AS5600Sensor>()}, fir_filter(N_TAPS, TAP_COEFFS)
{
  // Declare parameters
  declareParameters();
  // Create publisher
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  std::chrono::duration<int64_t, std::milli> frequency = 10ms;
  timer_ = this->create_wall_timer(frequency, std::bind(&AS5600Driver::handleInput, this));
  steer_angle = 0.0;
  yaw_rate = 0.0;
  vel = 0.0;
  subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "cmd_ackermann", 10, std::bind(&AS5600Driver::steerCallback, this, _1));
}

void AS5600Driver::handleInput()
{
  // Initialize an empty odometry message and populate the header.
  auto message = nav_msgs::msg::Odometry();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  // Compute the velocity from absolute angle measurements.
  double current_velocity = as5600_->getVelocity();
  message.twist.twist.linear.x = fir_filter.filter(current_velocity);
  // message.twist.twist.linear.x = current_velocity;
  message.twist.twist.angular.z = getYawRate();

  // Publish the odometry message where only the linear x velocity
  // is nonzero.
  publisher_->publish(message);
}

void AS5600Driver::steerCallback(const ackermann_msgs::msg::AckermannDriveStamped & msg)
{
  steer_angle = msg.drive.steering_angle;
}

float AS5600Driver::getYawRate()
{
  yaw_rate = vel * tan(steer_angle)/0.404;
  return yaw_rate;
}

void AS5600Driver::declareParameters() //This is garbage perhaps
{
  this->declare_parameter<int>("frequency", 0.0);
  this->declare_parameter<bool>("filter", true);
  this->declare_parameter<float>("wheelbase", 0.404);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AS5600Driver>());
  rclcpp::shutdown();
  return 0;
}