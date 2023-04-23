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
  rclcpp::QoS FMLCarQoS(1);
  FMLCarQoS.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", FMLCarQoS);
  std::chrono::duration<int64_t, std::milli> frequency = 10ms;
  timer_ = this->create_wall_timer(frequency, std::bind(&AS5600Driver::handleInput, this));
  steer_angle = 0.0;
  yaw_rate = 0.0;
  vel = 0.0;
  subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "cmd_ackermann", FMLCarQoS, std::bind(&AS5600Driver::steerCallback, this, _1));
}

void AS5600Driver::handleInput()
{
  // Initialize an empty odometry message and populate the header.
  auto message = nav_msgs::msg::Odometry();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  // Compute the velocity from absolute angle measurements.
  double vel_raw = as5600_->getVelocity();
  vel = fir_filter.filter(vel_raw);
  message.twist.twist.linear.x = vel;
  // message.twist.twist.linear.x = current_velocity;
  message.twist.twist.angular.z = getYawRate();
  // std::cout << "YAW RATE:" << message.twist.twist.angular.z << std::endl;
  // Publish the odometry message where only the linear x velocity
  // is nonzero.
  publisher_->publish(message);
}

void AS5600Driver::steerCallback(const ackermann_msgs::msg::AckermannDriveStamped & msg)
{
  steer_angle = msg.drive.steering_angle;
}

double AS5600Driver::getYawRate()
{
  double slip_angle = atan2(0.199*tan(steer_angle),0.404);
  yaw_rate = vel * tan(steer_angle) * cos(slip_angle)/0.404;
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
