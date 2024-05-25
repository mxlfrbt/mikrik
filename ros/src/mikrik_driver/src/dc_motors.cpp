//SPDX-License-Identifier: Apache-2.0
#include "dc_motor_wiring_pi.hpp"
#include <std_msgs/Float64.h>

// for testing motors include
/*#include <thread>
#include <chrono>*/

#define MOTOR_1_PIN_D 4 // Wiring pi 7 = BCM 4
#define MOTOR_1_PIN_E 18 // Wiring pi 1 = BCM 18
#define MOTOR_2_PIN_D 12 // Wiring pi 26 = BCM 12
#define MOTOR_2_PIN_E 13 // Wiring pi 23 = BCM 13


#define _REG_MOTOR1_ORIENTATION 0x0f
#define _REG_MOTOR1_SPEED 0x10
#define _REG_MOTOR2_ORIENTATION 0x12
#define _REG_MOTOR2_SPEED 0x13

DCMotorWiringPi left_dc_motor(_REG_MOTOR1_ORIENTATION, _REG_MOTOR1_SPEED);
DCMotorWiringPi right_dc_motor(_REG_MOTOR2_ORIENTATION, _REG_MOTOR2_SPEED);

int16_t stop_count_left = 0;
int16_t stop_count_right = 0;

void leftMotorCallback(const std_msgs::Float64& msg) {
	int16_t pwm = msg.data;

	if ((pwm > 0) && (pwm <= 10.23)) {
		stop_count_left = 0;
		pwm = pwm * 9.77;
		left_dc_motor.ccw(abs(pwm));
	} else if ((pwm < 0) && (pwm >= -10.23)) {
		stop_count_left = 0;
		pwm = pwm * 9.77;
		left_dc_motor.cw(abs(pwm));
	} else if ((pwm == 0) && (stop_count_left == 0)) {
		stop_count_left = 1;
		left_dc_motor.stop();
	}
}

void rightMotorCallback(const std_msgs::Float64& msg) {
	int16_t pwm = msg.data;

	if ((pwm > 0) && (pwm <= 10.23)) {
		stop_count_right = 0;
		pwm = pwm * 9.77;
		right_dc_motor.cw(abs(pwm));
	} else if ((pwm < 0) && (pwm >= -10.23)) {
		stop_count_right = 0;
		pwm = pwm * 9.77; //9.77 = 100/10.23 replacement of redundant calculation in previous formula (pwm * 100)/10.23
		right_dc_motor.ccw(abs(pwm));
	} else if ((pwm == 0) && (stop_count_right == 0)) {
		stop_count_right = 1;
		right_dc_motor.stop();
	}
}

int main(int argc, char** argv) {
	// to test motors
	// don't forget uncomment includes
	/*int c = 0;
	while(c<5) {
		double pwm = 2.57;
		pwm = (pwm * 100)/10.23;
		left_dc_motor.ccw(pwm);
		 c++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	left_dc_motor.stop(); */
	ros::init(argc, argv, "dc_motors");
	ros::NodeHandle node;
	ros::Subscriber left_motor_target_vel_sub = node.subscribe("/mikrik/left_wheel/pwm", 1, &leftMotorCallback);
	ros::Subscriber right_motor_target_vel_sub = node.subscribe("/mikrik/right_wheel/pwm", 1, &rightMotorCallback);
	ros::spin();
	return 0;
}
