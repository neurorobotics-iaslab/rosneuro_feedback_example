#include <vector>
#include <string>
#include <ros/ros.h>
#include <rosneuro_msgs/NeuroOutput.h>

constexpr float RATELOOP = 16;
constexpr float SIN_AMPLITUDE = 1.0f; 
constexpr float SIN_FREQUENCY = 0.2f;
constexpr float SIN_SHIFT     = 0.0f;

float scale(float x, float xmin, float xmax, float ymin, float ymax) {
	return (ymax - ymin) * ( (x - xmin) / (xmax - xmin) ) + ymin;
}

float sinewave(float time, float frequency, float amplitude, float shift) {

	float y;
	frequency = frequency / RATELOOP;

	y = amplitude * std::sin(2.0f * M_PI * frequency  * time + shift);

	y = scale(y, -amplitude, amplitude, 0.0f, 1.0f);

	return y;
}


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "neuroinput");
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Rate r(RATELOOP);
	rosneuro_msgs::NeuroOutput msg;

	msg.class_labels = std::vector<std::string>({"class1", "class2"});
	msg.softpredict.data = std::vector<float>(2);

	pub = nh.advertise<rosneuro_msgs::NeuroOutput>("/output", 1);

	float y;
	int t = 0;

	while(ros::ok()) {


		y = sinewave(t, SIN_FREQUENCY, SIN_AMPLITUDE, SIN_SHIFT);

		msg.header.stamp = ros::Time::now();
		msg.softpredict.data.at(0) = y;
		msg.softpredict.data.at(1) = 1.0f - y;

		pub.publish(msg);

		ros::spinOnce();
		r.sleep();
		t ++;
	}

	ros::shutdown();

	return 0;

}
