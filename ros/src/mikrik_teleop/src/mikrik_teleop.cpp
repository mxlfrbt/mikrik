//SPDX-License-Identifier: Apache-2.0
#include "mikrik_teleop.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "mikrik_teleop");
	ros::NodeHandle private_node("~");
	MikrikTeleop mikrikTeleop(private_node);
	ros::spin();
}
