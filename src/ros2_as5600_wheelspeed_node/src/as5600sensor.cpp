#include "as5600driver/as5600sensor.h"

// Include necessary C libraries
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

// Class constructor
AS5600Sensor::AS5600Sensor()
{
  std::cout << filename_ << std::endl;
  // Attempt to open the file descriptor for the I2C device
  file_ = open(filename_, O_RDWR);
  // Check if the file descriptor was successfully opened
  if (file_ < 0) {
    std::cerr << "Failed to open file descriptor! Check your bus number! Errno: "
              << strerror(errno);
    // If the file descriptor failed to open, exit the program
    exit(1);
  }
  // Set the device address to communicate with over I2C
  if (ioctl(file_, I2C_SLAVE, AS5600_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    // If the device address cannot be set, exit the program
    exit(1);
  }
}

// Class destructor
AS5600Sensor::~AS5600Sensor() { 
  // Close the file descriptor for the I2C device
  close(file_);
}

int16_t AS5600Sensor::getRawAngle()
{
  // Read the most significant byte of the raw angle register
  int16_t angle_msb = i2c_smbus_read_byte_data(file_, RAW_ANGLE_REGISTER_MSB);
  // Read the least significant byte of the raw angle register
  int16_t angle_lsb = i2c_smbus_read_byte_data(file_, RAW_ANGLE_REGISTER_LSB);
  // Combine the most and least significant bytes to form the raw angle reading
  int16_t raw_angle = angle_lsb | angle_msb << 8;
  // Return the raw angle reading
  return raw_angle;
}

int16_t AS5600Sensor::getDelta(int16_t raw_angle)
{
  // Calculate the difference between the current raw angle reading and the previous one
  int16_t delta_angle = (raw_angle - AS5600Sensor::prev_raw_angle);
  // Handle the case where the angle reading rolls over from the maximum value to the minimum value
  if(delta_angle > ROLL_OVER_CUTOFF)
  {
    delta_angle = delta_angle - MAX_ANGLE;
  }
  // Handle the case where the angle reading rolls over from the minimum value to the maximum value
  if(delta_angle < -ROLL_OVER_CUTOFF)
  {
    delta_angle = delta_angle + MAX_ANGLE;
  }
  // Invert the delta angle if the direction flag is set to "decreasing is forward"
  if (AS5600Sensor::dir_flag == DECREASING_IS_FORWARD)
  {
    delta_angle = delta_angle*-1.0;
  }
  // Update the previous raw angle reading to be the current one
  AS5600Sensor::prev_raw_angle = raw_angle;
  // Return the delta angle
  return delta_angle;
}

double AS5600Sensor::getTime()
{
  // get current time as a time point
  std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();

  // calculate the time elapsed since the runtime start
  std::chrono::duration<double> time_elapsed = current_time - AS5600Sensor::runtime_start;

  // convert time elapsed to milliseconds
  std::chrono::milliseconds time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed);

  // return the time elapsed in milliseconds as a double
  return time_elapsed_ms.count();
}

double AS5600Sensor::getTimeDelta(double current_time)
{
  double dt = current_time - AS5600Sensor::prev_time; //milliseconds
  AS5600Sensor::prev_time = current_time;
  return dt;
}

double AS5600Sensor::getVelocity()
{
  int16_t raw_angle_reading = getRawAngle();
  int16_t delta_angle = getDelta(raw_angle_reading);
  double new_time = getTime();
  double delta_time = getTimeDelta(new_time);
  // Calculate the velocity based on the change in angle and time
  double velocity = ((double)delta_angle / MAX_ANGLE) * (13.0/37.0) * PI * 0.1143 / (delta_time/1000.0);
  
  // Debugging statements to print out the delta angle, time step, and velocity
  //std::cout << "Delta Angle:" << delta_angle << std::endl;
  //std::cout << "Time Step:" << delta_time << std::endl;
  // std::cout << "Velocity:" << velocity << std::endl;
  return velocity;
}

void AS5600Sensor::reportError(int error) { 
  std::cerr << "Error! Errno: " << strerror(error); 
}