//SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <functional>
#include <ros/callback_queue.h>

#include "mikrik_hardware_interface.h"

typedef boost::chrono::steady_clock time_source;

void controlLoop(MikrikHardwareInterface& hardware, controller_manager::ControllerManager& cm, time_source::time_point& last_time) {
	time_source::time_point this_time = time_source::now();
	boost::chrono::duration<double> elapsed_duration = this_time - last_time;
	ros::Duration elapsed(elapsed_duration.count());
	last_time = this_time;

	hardware.updateJointsFromHardware(elapsed);
	cm.update(ros::Time::now(), elapsed);
	hardware.writeCommandsToHardware();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mikrik_base");
	ros::NodeHandle node;
	ros::NodeHandle private_node("~");

	int control_frequency;
	double max_wheel_angular_speed;

	private_node.param<int>("control_frequency", control_frequency, 1);
	private_node.param<double>("max_wheel_angular_speed", max_wheel_angular_speed, 1.0);

	MikrikHardwareInterface hardware(node, private_node, max_wheel_angular_speed);

	controller_manager::ControllerManager cm(&hardware, node);

	ros::CallbackQueue mikrik_queue;
	ros::AsyncSpinner mikrik_spinner(1, &mikrik_queue);

	time_source::time_point last_time = time_source::now();

	ros::TimerOptions control_timer(
		ros::Duration(1 / control_frequency),
		boost::bind(controlLoop, std::ref(hardware), std::ref(cm), std::ref(last_time)), &mikrik_queue);

	ros::Timer control_loop = node.createTimer(control_timer);

	mikrik_spinner.start();
	ros::spin();
	return 0;
}
