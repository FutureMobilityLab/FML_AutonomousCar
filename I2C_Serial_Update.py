#!/usr/bin/env python3

import time
import lgpio

MPU6050_Address = 0x68 # bus address
h_MPU = lgpio.i2c_open(1, MPU6050_Address)

#	MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# ESC_Address = 0x45
# h_ESC = lgpio.i2c_open(1, ESC_Address)

#	ESC Registers and their Address
# READ ADDRESSES
ESC_VOLTAGE = 0x0
ESC_RIPPLE = 0x1
ESC_CURRENT = 0x2
ESC_THROTTLE = 0x3
ESC_POWER = 0x4
ESC_SPEED = 0x5
ESC_TEMP = 0x6
ESC_BEC_VOLT = 0x7
ESC_BEC_CURRENT = 0x8
ESC_RAW_NTC = 0x9
ESC_RAW_LINEAR = 0x10
ESC_LINK_LIVE = 0x25
ESC_FAIL_SAFE = 0x26
ESC_E_STOP = 0x27
ESC_PACKET_IN = 0x28
ESC_PACKET_OUT = 0x29
ESC_CHECK_BAD = 0x30
ESC_PACKET_BAD = 0x31
#WRITE ADDRESSES
ESC_CMD_THROTTLE = 0x128		#0-65535
ESC_CMD_FAIL_SAFE = 0x129		#0-100
ESC_CMD_E_STOP = 0x130			#0/1
ESC_CMD_PACKET_IN = 0x131		#SETS TO 0
ESC_CMD_CHECK_BAD = 0x132		#SETS TO 0
ESC_CMD_PACKET_BAD = 0x133		#SETS TO 0

def MPU_Init():
	#write to sample rate register
	lgpio.i2c_write_byte_data(h_MPU, SMPLRT_DIV, 7)
	
	#Write to power management register
	lgpio.i2c_write_byte_data(h_MPU, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	lgpio.i2c_write_byte_data(h_MPU, CONFIG, 0)
	
	#Write to Gyro configuration register
	lgpio.i2c_write_byte_data(h_MPU, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	lgpio.i2c_write_byte_data(h_MPU, INT_ENABLE, 1)

def read_raw_data_mpu(addr):
	#Accelero and Gyro value are 16-bit
        high = lgpio.i2c_read_byte_data(h_MPU, addr)
        low = lgpio.i2c_read_byte_data(h_MPU, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

def read_raw_data_esc(addr):
	#All I2C Registers are 16-bit
        high = lgpio.i2c_read_byte_data(h_ESC, addr)
        low = lgpio.i2c_read_byte_data(h_ESC, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


MPU_Init()
print (" Reading Data of Gyroscope and Accelerometer")

while True:
	
	#Read Accelerometer raw value
	acc_x = read_raw_data_mpu(ACCEL_XOUT_H)
	acc_y = read_raw_data_mpu(ACCEL_YOUT_H)
	acc_z = read_raw_data_mpu(ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
	gyro_x = read_raw_data_mpu(GYRO_XOUT_H)
	gyro_y = read_raw_data_mpu(GYRO_YOUT_H)
	gyro_z = read_raw_data_mpu(GYRO_ZOUT_H)
	
	#Full scale range +/- 250 degree/C as per sensitivity scale factor
	Ax = acc_x/16384.0
	Ay = acc_y/16384.0
	Az = acc_z/16384.0
	
	Gx = gyro_x/131.0
	Gy = gyro_y/131.0
	Gz = gyro_z/131.0

	print ("Gx=%.3f" %Gx, "| Gy=%.3f" %Gy, "| Gz=%.3f" %Gz, u'\u00b0'+ "/s", "\tAx=%.3f" %Ax, "| Ay=%.3f" %Ay, "| Az=%.3f g" %Az) 	
	time.sleep(.1)


	# temperature = lgpio.i2c_read_word_data(h_ESC,ESC_TEMP)
	# print("Register reads:",temperature)

	

	# test = lgpio.i2c_read_device(h_ESC,3)
	# print(test)
	
	# i = i+1
	# throttle_cmd = 0

	# if i > 20:
	# 	throttle_cmd = 1000
	# if i > 40:
	# 	throttle_cmd = 0

	# lgpio.i2c_write_word_data(h_ESC,ESC_CMD_THROTTLE,throttle_cmd)


		