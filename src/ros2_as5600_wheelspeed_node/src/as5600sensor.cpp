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

AS5600Sensor::AS5600Sensor()
{
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
}

AS5600Sensor::~AS5600Sensor() { close(file_); }

int16_t AS5600Sensor::getRawAngle()
{
  int16_t angle_msb = i2c_smbus_read_byte_data(file_, RAW_ANGLE_REGISTER_MSB);
  int16_t angle_lsb = i2c_smbus_read_byte_data(file_, RAW_ANGLE_REGISTER_LSB);
  int16_t raw_angle = angle_lsb | angle_msb << 8;
  return raw_angle;
}

int16_t AS5600Sensor::getDelta(int16_t raw_angle)
{
  int16_t delta_angle = (raw_angle - AS5600Sensor::prev_raw_angle);
  if(delta_angle > 2048)
  {
    delta_angle = 4096 - delta_angle;
  }
  if(delta_angle < -2048)
  {
    delta_angle = delta_angle + 4096;
  }
  //std::cout << "Raw Previous Angle:" << prev_raw_angle << std::endl;
  AS5600Sensor::prev_raw_angle = raw_angle;
  //std::cout << "Raw Angle:" << prev_raw_angle << std::endl;
  //std::cout << "Delta Angle:" << delta_angle << std::endl;
  return delta_angle;
}

double AS5600Sensor::getTime()
{
  std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_elapsed = current_time - AS5600Sensor::runtime_start;
  std::chrono::milliseconds time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed);
  return time_elapsed_ms.count();
}

double AS5600Sensor::getTimeDelta(double current_time)
{
  double dt = current_time - AS5600Sensor::prev_time; //seconds
  AS5600Sensor::prev_time = current_time;
  return dt;
}

double AS5600Sensor::getVelocity()
{
  int16_t raw_angle_reading = getRawAngle();
  int16_t delta_angle = getDelta(raw_angle_reading);
  double new_time = getTime();
  double delta_time = getTimeDelta(new_time);
  double velocity = ((double)delta_angle / 4096.0) * (13.0/37.0) * 3.14159 *0.1143 / (delta_time/1000.0);
  
  //std::cout << "Delta Angle:" << delta_angle << std::endl;
  //std::cout << "Time Step:" << delta_time << std::endl;
  std::cout << "Velocity:" << velocity << std::endl;
  return velocity;
}

void AS5600Sensor::reportError(int error) { std::cerr << "Error! Errno: " << strerror(error); }