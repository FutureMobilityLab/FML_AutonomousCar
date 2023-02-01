#ifndef AS5600DRIVER_H
#define AS5600DRIVER_H

#include "as5600driver/as5600sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class AS5600Driver : public rclcpp::Node {
 public:
  AS5600Driver();

 private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<AS5600Sensor> as5600_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleInput();
  void declareParameters();
};

#endif  // AS5600DRIVER_H