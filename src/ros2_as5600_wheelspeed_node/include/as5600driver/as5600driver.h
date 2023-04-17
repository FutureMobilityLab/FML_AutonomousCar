#ifndef AS5600DRIVER_H
#define AS5600DRIVER_H

#include "as5600driver/as5600sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "as5600driver/fir_filter.cpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
using std::placeholders::_1;

/**
 * @brief Provides methods for interfacing AS5600 hall-effect sensor 
 * to ROS2 network.
 */
class AS5600Driver : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new AS5600Driver instance.
   * This creates a publisher and publishes the velocity measurement 
   * at a set frequency.
   */
  AS5600Driver();

 private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<AS5600Sensor> as5600_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
  float steer_angle;
  float yaw_rate;
  float vel;

  /**
   * @brief Publishes the measured velocity from the AS5600 sensor.
   */
  void handleInput();
  float getYawRate();
  void steerCallback(const ackermann_msgs::msg::AckermannDriveStamped & msg);

  /**
   * @brief Sets the frequency parameter of the ROS node.
   */
  void declareParameters();

  // Node Constants.
  // Filter designed using MATLAB's fdesign.lowpass function. The specific commands are:
  // FIReq = fdesign.lowpass('N,Fc,Ap,Ast',10,0.2,0.5,40);
  // filterCoeff = design(FIReq,'equiripple','SystemObject',true);
  static const int N_TAPS = 21;
  const double TAP_COEFFS[N_TAPS] = {
    -0.0030,  0.0060,  0.0120,  0.0217,  0.0344,  
     0.0492,  0.0647,  0.0793,  0.0913,  0.0992,
     0.1019,  0.0992,  0.0913,  0.0793,  0.0647,
     0.0492,  0.0344,  0.0217,  0.0120,  0.0060, -0.0030};
  FIRFilter fir_filter;
};

#endif  // AS5600DRIVER_H