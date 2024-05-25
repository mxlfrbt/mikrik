//SPDX-License-Identifier: Apache-2.0
#ifndef DC_MOTOR_WIRING_PI_HPP_
#define DC_MOTOR_WIRING_PI_HPP_

#include <ros/ros.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <thread>
#include <chrono>

#define RPI_MAX_PWM_VALUE 100
#define _DFROBOT_MOTOR_DRIVER_ADDR 0x10
#define _REG_CTRL_MODE 0x03
#define _REG_ENCODER1_EN 0x04
#define _REG_ENCODER2_EN 0x09
#define _REG_MOTOR_PWM 0x0e

class DCMotorWiringPi {
public:
	DCMotorWiringPi(int8_t ctrl_reg, int8_t speed_reg);
	void cw(uint8_t speed);
	void ccw(uint8_t speed);
	void stop();
private:
	int8_t _fd;
	int8_t _ctrl_reg;
	int8_t _speed_reg;
	uint16_t protectOutput(uint8_t val);
};

DCMotorWiringPi::DCMotorWiringPi(int8_t ctrl_reg, int8_t speed_reg) {
	_ctrl_reg = ctrl_reg;
	_speed_reg = speed_reg;
	_fd = wiringPiI2CSetup(_DFROBOT_MOTOR_DRIVER_ADDR);
	if (_fd < 0) {
		ROS_ERROR("DCMotor wiringPi error: Failed to init I2C motor driver");
		throw std::runtime_error("");
	}
	ROS_INFO("DCMotor wiringPi: I2C motor driver setup");
    ROS_INFO("DCMotor wiringPi: Stop motor");
	// Set motor driver control mode
	wiringPiI2CWriteReg8(_fd, _REG_CTRL_MODE, 0);
	// Disable M1 left encoder
	wiringPiI2CWriteReg8(_fd, _REG_ENCODER1_EN, 0);
	// Disable M2 right encoder
	wiringPiI2CWriteReg8(_fd, _REG_ENCODER2_EN, 0);
	// Setup frequency
	  std::cout << "I2C communication successfully setup.\n";
	wiringPiI2CWriteReg8(_fd, _REG_MOTOR_PWM, 20);
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ROS_INFO("DCMotor wiringPi: Motor setup finished");
}

void DCMotorWiringPi::stop() {
  // send 5 to this register means to stop the motor
  int ret;

  ret = wiringPiI2CWriteReg8(_fd, _ctrl_reg, 5);
  /*if (ret < 0) {
	  ROS_ERROR("DCMotor wiringPi error: Failed to write wiringPiI2CWriteReg8 in stop function");
	  throw std::runtime_error("");
	}*/
}

void DCMotorWiringPi::cw(uint8_t speed) {
    // orientation forward ccw (left wheel example)
	// cw any wheel is 1
	int ret;

    ret = wiringPiI2CWriteReg8(_fd, _ctrl_reg, 1);
	/*if (ret < 0) {
	  ROS_ERROR("DCMotor wiringPi error: Failed to write wiringPiI2CWriteReg8 in cw function");
	  throw std::runtime_error("");
	}*/

    ret = wiringPiI2CWriteReg8(_fd, _speed_reg, protectOutput(speed));
	/*if (ret < 0) {
	  ROS_ERROR("DCMotor wiringPi error: Failed to write wiringPiI2CWriteReg8 in cw function");
	  throw std::runtime_error("");
	}*/
}

void DCMotorWiringPi::ccw(uint8_t speed) {
	// orientation forward ccw (left wheel example)
	// ccw any wheel is 2
	int ret;

    ret = wiringPiI2CWriteReg8(_fd, _ctrl_reg, 2);
	/*if (ret < 0) {
	  ROS_ERROR("DCMotor wiringPi error: Failed to write wiringPiI2CWriteReg8 in ccw function");
	  throw std::runtime_error("");
	}*/

    ret = wiringPiI2CWriteReg8(_fd, _speed_reg, protectOutput(speed));
	/*if (ret < 0) {
	  ROS_ERROR("DCMotor wiringPi error: Failed to write wiringPiI2CWriteReg8 in ccw function");
	  throw std::runtime_error("");
	}*/
}

uint16_t DCMotorWiringPi::protectOutput(uint8_t val) {
    return val > RPI_MAX_PWM_VALUE ? RPI_MAX_PWM_VALUE : val;
}

#endif // DC_MOTOR_WIRING_PI_HPP
