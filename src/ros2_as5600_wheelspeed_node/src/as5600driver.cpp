#include "as5600driver/as5600driver.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

AS5600Driver::AS5600Driver()
    : Node("as5600publisher"), as5600_{std::make_unique<AS5600Sensor>()}
{
  // Declare parameters
  declareParameters();
  // Create publisher
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  std::chrono::duration<int64_t, std::milli> frequency = 100ms; //Used in MPU6050 node, unsure of application
  timer_ = this->create_wall_timer(frequency, std::bind(&AS5600Driver::handleInput, this));
}

void AS5600Driver::handleInput()
{
  auto message = nav_msgs::msg::Odometry();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  message.twist.twist.linear.x = as5600_->getVelocity();
  publisher_->publish(message);
}

void AS5600Driver::declareParameters()
{
  this->declare_parameter<int>("frequency", 0.0);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AS5600Driver>());
  rclcpp::shutdown();
  return 0;
}