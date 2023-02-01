#include "as5600driver/as5600sensor.h"

extern "C" {
#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <iostream>

AS5600Sensor::AS5600Sensor(int bus_number)
{
  // TODO: make char append cleaner
  filename_[9] = *std::to_string(bus_number).c_str();
  std::cout << filename_ << std::endl;
  file_ = open(filename_, O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open file descriptor! Check your bus number! Errno: "
              << strerror(errno);
    exit(1);
  }
  if (ioctl(file_, I2C_SLAVE, AS5600_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    exit(1);
  }
  // // Wake up sensor
  // int result = i2c_smbus_write_byte_data(file_, PWR_MGMT_1, 0);
  // if (result < 0) reportError(errno);
  // Read current ranges from sensor
}

AS5600Sensor::~AS5600Sensor() { close(file_); }

double AS5600Sensor::getRawAngle() const
{
  int16_t angle_msb = i2c_smbus_read_byte_data(file_, RAW_ANGLE_REGISTER_MSB);
  int16_t angle_lsb = i2c_smbus_read_byte_data(file_, RAW_ANGLE_REGISTER_LSB);
  int16_t raw_angle = angle_lsb | angle_msb << 8;

  return raw_angle;
}

int16_t AS5600Sensor::getDelta(int16_t raw_angle_1, int16_t raw_angle_2)
{
  int16_t delta_angle = raw_angle_2 - raw_angle_1;
  if(abs(delta_angle) > 2048)
  {
    delta_angle = 4096 - delta_angle;
  }
  return delta_angle;
}

double AS5600Sensor::getVelocity()
{
  int16_t raw_angle_reading_1 = getRawAngle();
  int16_t raw_angle_reading_2 = getRawAngle();
  int16_t delta_angle = getDelta(raw_angle_reading_1,raw_angle_reading_2);
  double delta_time = 0.01;
  double velocity = delta_angle /4096 * (13/37) * 3.14159 *.1143 / delta_time; //TODO: GET GOOD TIME
  return velocity;
}

void AS5600Sensor::reportError(int error) { std::cerr << "Error! Errno: " << strerror(error); }