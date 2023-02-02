#ifndef AS5600SENSOR_H
#define AS5600SENSOR_H

#include <string>
#include <unordered_map>
#include <chrono>

class AS5600Sensor {
 public:
  AS5600Sensor();
  ~AS5600Sensor();

  double getVelocity();
 
 private:
  int16_t getRawAngle();
  int16_t getDelta(int16_t raw_angle);
  int16_t prev_raw_angle{getRawAngle()}; 

  double getTime();
  double getTimeDelta(double current_time);
  double prev_time{getTime()};

  void reportError(int error);

  int file_;
  char filename_[11] = "/dev/i2c-1";

  std::chrono::time_point<std::chrono::steady_clock> runtime_start = std::chrono::steady_clock::now();

  // AS5600 registers and addresses (s. datasheet for details)
  static constexpr int AS5600_ADDRESS_DEFAULT = 0x36;
  static constexpr int RAW_ANGLE_REGISTER_MSB = 0x0C;
  static constexpr int RAW_ANGLE_REGISTER_LSB = 0x0D;
  static constexpr int ANGLE_REGISTER = 0x0F;
  static constexpr int MAGNET_STATUS_REGISTER = 0x0B;
  static constexpr int AGC_REGISTER = 0x1A;
  static constexpr int MAGNITUDE_REGISTER = 0x1C;
  static constexpr int CONFIG_REGISTER = 0x3F;
};

#endif  // AS5600SENSOR_H
