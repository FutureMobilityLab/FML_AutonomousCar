#ifndef AS5600SENSOR_H
#define AS5600SENSOR_H

#include <string>
#include <unordered_map>

class AS5600Sensor {
 public:
  AS5600Sensor(int bus_number = 1);
  ~AS5600Sensor();

  double getRawAngle() const;
  double getVelocity();
  
  int16_t getDelta(int16_t raw_angle_1, int16_t raw_angle_2);

  void reportError(int error);

  int file_;
  char filename_[10] = "/dev/i2c-";

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
