#include "as5600driver/as5600driver.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

AS5600Driver::AS5600Driver()
    : Node("as5600publisher"), as5600_{std::make_unique<AS5600Sensor>()}
{
  // Declare parameters
  declareParameters();
  // Create Filter.
  fir_filter(N_TAPS, TAP_COEFFS);
  // Create publisher
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  timer_ = this->create_wall_timer(FREQUENCY, std::bind(&AS5600Driver::handleInput, this));
}

void AS5600Driver::handleInput()
{
  // Initialize an empty odometry message and populate the header.
  auto message = nav_msgs::msg::Odometry();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  // Compute the velocity from absolute angle measurements.
  message.twist.twist.linear.x = as5600_->getVelocity();
  if (this->get_parameter("filter")) {

  }
  // Publish the odometry message where only the linear x velocity
  // is nonzero.
  publisher_->publish(message);
}

void AS5600Driver::declareParameters()
{
  this->declare_parameter<int>("frequency", 0.0);
  this->declare_parameter<bool>("filter", true);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AS5600Driver>());
  rclcpp::shutdown();
  return 0;
}