#include <ros/ros.h>
#include "rosneuro_feedback_example/Wheel.h"

int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "wheel");

	rosneuro::Wheel wheel;

	if(wheel.configure() == false) {
		ROS_ERROR("Wheel configuration failed");
		ros::shutdown();
		return 1;
	}

	wheel.run();

	ros::shutdown();

	return 0;

}
