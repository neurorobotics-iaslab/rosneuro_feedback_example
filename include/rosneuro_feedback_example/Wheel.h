#ifndef ROSNEURO_FEEDBACK_WHEEL_
#define ROSNEURO_FEEDBACK_WHEEL_

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroOutput.h>
#include "neurodraw/Ring.h"
#include "neurodraw/Arc.h"
#include "neurodraw/Cross.h"
#include "neurodraw/Line.h"
#include "neurodraw/Rectangle.h"
#include "neurodraw/Engine.h"
#include "neurodraw/Palette.h"
#include "neurodraw/EventKey.h"

namespace rosneuro {

class Wheel {

	public:
		Wheel(void);
		~Wheel(void);

		bool configure(void);

		void run(void);

		void update(float angle);

	private:
		void setup_scene(void);
		void on_received_data(const rosneuro_msgs::NeuroOutput& msg);
		void on_keyboard_event(const neurodraw::KeyboardEvent& event);

		float input2angle(float input);

	private:
		ros::NodeHandle 	nh_;
		ros::NodeHandle		p_nh_;
		ros::Subscriber 	sub_;

		neurodraw::Engine* 		engine_;
		neurodraw::Ring* 		ring_;
		neurodraw::Arc* 		arc_;
		neurodraw::Rectangle* 	mline_;
		neurodraw::Rectangle* 	lline_;
		neurodraw::Rectangle* 	rline_;
		neurodraw::Cross* 		cross_;
		bool user_quit_;

		float input_min_;
		float input_max_;
		float angle_min_;
		float angle_max_;



};


}

#endif
