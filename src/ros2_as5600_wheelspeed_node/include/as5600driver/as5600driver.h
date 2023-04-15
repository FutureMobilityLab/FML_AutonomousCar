#ifndef AS5600DRIVER_H
#define AS5600DRIVER_H

#include "as5600driver/as5600sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "utils/fir_filter.cpp"

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

  /**
   * @brief Publishes the measured velocity from the AS5600 sensor.
   */
  void handleInput();

  /**
   * @brief Sets the frequency parameter of the ROS node.
   */
  void declareParameters();

  // Node Constants.
  static const std::chrono::duration<int64_t, std::milli> FREQUENCY = 10ms;
  // Filter designed using MATLAB's fdesign.lowpass function. The specific commands are:
  // FIReq = fdesign.lowpass('N,Fc,Ap,Ast',10,0.2,0.5,40);
  // filterCoeff = design(FIReq,'equiripple','SystemObject',true);
  static const N_TAPS = 11;
  const double TAP_COEFFS[N_TAPS] = {
    -2.4508e-04,  0.0257,  0.0707,  0.1313,  0.1843,
     0.2054,      0.1843,  0.1313,  0.0707,  0.0257, -2.4508e-04};
  FIRFilter fir_filter;
};

#endif  // AS5600DRIVER_H