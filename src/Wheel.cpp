#include "rosneuro_feedback_example/Wheel.h"

namespace rosneuro {

Wheel::Wheel(void) : p_nh_("~") {


	this->sub_ = this->nh_.subscribe("output", 1, &Wheel::on_received_data, this);
	
	this->engine_ = new neurodraw::Engine("Wheel");
	this->engine_->on_keyboard(&Wheel::on_keyboard_event, this);

	this->user_quit_ = false;

}

Wheel::~Wheel(void) {

	if(this->engine_ != nullptr)
		delete this->engine_;

}

bool Wheel::configure(void) {

	this->setup_scene();

	this->input_min_ = 0.0f;
	this->input_max_ = 1.0f;

	this->angle_min_ = 0.0f;
	this->angle_max_ = 180.0f;

	return true;

}

void Wheel::setup_scene(void) {

	this->cross_ = new neurodraw::Cross(0.3f, 0.05f);
	this->ring_  = new neurodraw::Ring(0.8f, 0.15f, neurodraw::Palette::grey);
	this->arc_   = new neurodraw::Arc(0.8f, 0.15f, 2.0 * M_PI / 3.0f, neurodraw::Palette::lightgrey);
	this->mline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::green);
	this->rline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::firebrick);
	this->lline_ = new neurodraw::Rectangle(0.01f, 0.15f, true, neurodraw::Palette::royalblue);

	this->arc_->rotate(30);
	this->mline_->move(0.0f, 0.725f);
	this->rline_->move(0.0f, 0.725f);
	this->lline_->move(0.0f, 0.725f);
	this->rline_->rotate(0.0f, 0.0f, 0.0f);
	this->lline_->rotate(180.0f, 0.0f, 0.0f);


	this->engine_->add(this->ring_);
	this->engine_->add(this->cross_);
	this->engine_->add(this->arc_);
	this->engine_->add(this->lline_);
	this->engine_->add(this->rline_);
	this->engine_->add(this->mline_);

}

void Wheel::update(float angle) {
	this->arc_->rotate(angle - 60.0f);
	this->mline_->rotate(angle, 0.0f, 0.0f);
}

void Wheel::run(void) {


	ros::Rate r(100);
	
	while(ros::ok() & this->user_quit_ == false) {

		ros::spinOnce();
		r.sleep();
	}

}

void Wheel::on_received_data(const rosneuro_msgs::NeuroOutput& msg) {

	float angle, input;
	input = msg.softpredict.data.at(0);
	angle = this->input2angle(input);
	this->update(angle);

}

void Wheel::on_keyboard_event(const neurodraw::KeyboardEvent& event) {

	if(event.state == 0)
		return;

	switch(event.sym) {
	    case neurodraw::EventKey::ESCAPE:
	   	 this->engine_->quit();
		 this->user_quit_ = true;
	   	 break;
	    case neurodraw::EventKey::a:
	   	 this->update(this->input2angle(1.0f));
	   	 break;
	    case neurodraw::EventKey::s:
	   	 this->update(this->input2angle(0.0f));
	   	 break;
	    case neurodraw::EventKey::r:
	   	 this->update(this->input2angle(0.5f));
	   	 break;
	}
}

float Wheel::input2angle(float input) {

	float b, a, xmax, xmin, angle;

	b    = this->angle_max_;
	a    = this->angle_min_;
	xmax = this->input_max_;
	xmin = this->input_min_;
	
	angle = (b-a) * ( (input - xmin) / (xmax - xmin) ) + a;

	return angle;

}

}
