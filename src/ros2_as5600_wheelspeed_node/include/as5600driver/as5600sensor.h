/**
 * @file as5600sensor.h
 * @brief This file contains the AS5600Sensor class, which provides methods for reading the 
 * angle and velocity of an AS5600 magnetic sensor.
 */

#ifndef AS5600SENSOR_H
#define AS5600SENSOR_H

#include <string>
#include <unordered_map>
#include <chrono>

/**
 * @brief The AS5600Sensor class provides methods for reading the angle and velocity of an AS5600 magnetic sensor.
 */
class AS5600Sensor {
 public:
  /**
   * @brief Construct a new AS5600Sensor object.
   * The constructor opens the I2C device file descriptor and sets the device address.
   */
  AS5600Sensor();

  /**
   * @brief Destroy the AS5600Sensor object.
   * The destructor closes the I2C device file descriptor.
   */
  ~AS5600Sensor();

  /**
   * @brief Calculates the velocity of the AS5600 sensor based on the change in angle and time.
   * 
   * @return The velocity of the AS5600 sensor as a double in meters per second.
   */
  double getVelocity();
 
 private:
  /**
   * @brief Get the raw angle reading from the AS5600 sensor.
   * 
   * @return int16_t The raw angle reading.
   */
  int16_t getRawAngle();

  /**
   * @brief Calculate the delta angle between the current raw angle reading and the previous one.
   * 
   * @param raw_angle The current raw angle reading.
   * @return int16_t The delta angle.
   */
  int16_t getDelta(int16_t raw_angle);

  int16_t prev_raw_angle{getRawAngle()};
  bool dir_flag = 0;
  static const bool INCREASING_IS_FORWARD = 1;
  static const bool DECREASING_IS_FORWARD = 0;

  /**
   * @brief Get the time elapsed since the program started running.
   * 
   * @return double The time elapsed in milliseconds.
   */
  double getTime();

  /**
   * @brief Calculates the time difference between the current time and the previous time.
   * 
   * @param current_time The current time as a double in milliseconds.
   * @return The time difference between the current and previous time as a double in milliseconds.
   */
  double getTimeDelta(double current_time);
  double prev_time{getTime()};

  /**
   * @brief Sends the error number to std::cerr.
   */
  void reportError(int error);

  int file_;
  char filename_[11] = "/dev/i2c-1";

  std::chrono::time_point<std::chrono::steady_clock> runtime_start = std::chrono::steady_clock::now();

  // AS5600 registers and addresses (s. datasheet for details)
  static constexpr int AS5600_ADDRESS_DEFAULT = 0x36;
  static constexpr int RAW_ANGLE_REGISTER_MSB = 0x0C; // most significant bit
  static constexpr int RAW_ANGLE_REGISTER_LSB = 0x0D; // least significant bit
  static constexpr int ANGLE_REGISTER = 0x0F;
  static constexpr int MAGNET_STATUS_REGISTER = 0x0B;
  static constexpr int AGC_REGISTER = 0x1A;
  static constexpr int MAGNITUDE_REGISTER = 0x1C;
  static constexpr int CONFIG_REGISTER = 0x3F;

  // AS5600 Constants
  static const int MAX_ANGLE = 4096;
  static const int ROLL_OVER_CUTOFF = 3072;
  static const double PI = 3.14159;
  static const double SEC_TO_MS = 1.0 / 1000.0;
};

#endif  // AS5600SENSOR_H
