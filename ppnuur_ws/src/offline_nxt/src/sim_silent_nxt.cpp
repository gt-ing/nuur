/**
 * Simulate the robot msgs/topics to try new functions offline
 * sim_silent_nxt.cpp /include/sim_silent_nxt.h
 *
 *  @date   09.12.2018
 *  @author GT
 */


// Header include
#include "sim_silent_nxt.h"

// ROS include
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>

// C++ std include
#include <string>

SimSilentNxt::SimSilentNxt(ros::NodeHandle _node) {
	node = _node;

	// Get parameter from ros node and set simulation values
	this->getParam();

	// Setup for publisher and subscriber
	this->subMotorCommand	= this->node.subscribe("motor_command", 10,
						&SimSilentNxt::getMotorCommand, this);
	this->subFakeRange	= this->node.subscribe("offline_sim/fake_range", 10,
						&SimSilentNxt::getFakeRange, this);

	this->pubSensorData	= this->node.advertise<offline_nxt::SensorData>("sensor_data", 10);

//	ros::spin();
}

SimSilentNxt::~SimSilentNxt() {}

void SimSilentNxt::getParam() {

	if(!this->node.getParam("/offline_simulation/motor_publish_frequency",
				this->motor_publish_frequency)) {
		this->motor_publish_frequency = 2;
		this->node.setParam("/offline_simulation/motor_publish_frequency",
					this->motor_publish_frequency);
	}

	if(!this->node.getParam("/offline_simulation/tick_A_per_second",
				this->tick_A_per_second)) {
		this->tick_A_per_second = 400;
		this->node.setParam("/offline_simulation/tick_A_per_second",
					this->tick_A_per_second);
	}

	if(!this->node.getParam("/offline_simulation/tick_B_per_second",
				this->tick_B_per_second)) {
		this->tick_B_per_second = 400;
		this->node.setParam("/offline_simulation/tick_B_per_second",
					this->tick_B_per_second);
	}

	if(!this->node.getParam("/offline_simulation/tick_A_correction",
				this->tick_A_correction)) {
		this->tick_A_correction = 1.0;
		this->node.setParam("/offline_simulation/tick_A_correction",
					this->tick_A_correction);
	}

	if(!this->node.getParam("/offline_simulation/tick_B_correction",
				this->tick_B_correction)) {
		this->tick_B_correction = 1.0;
		this->node.setParam("/offline_simulation/tick_B_correction",
					this->tick_B_correction);
	}

	if(!this->node.getParam("/offline_simulation/delay_time",
				this->delay_time)) {
		this->delay_time = 1.0;
		this->node.setParam("/offline_simulation/delay_time",
					this->delay_time);
	}

	if(!this->node.getParam("/offline_simulation/default_range",
				this->default_range)) {
		this->default_range = 0.255;
		this->node.setParam("/offline_simulation/default_range",
					this->default_range);
	}
}

bool SimSilentNxt::isStopMsg(const offline_nxt::MotorCommand& _msg) {
	std::string stop = "stop";
	if(stop.compare(_msg.name) != 0)	return false;
	else					return true;
}

bool SimSilentNxt::isDriveMsg(const offline_nxt::MotorCommand& _msg) {
	std::string drive = "drive";
	if(drive.compare(_msg.name) != 0)	return false;
	else					return true;
}

void SimSilentNxt::setDelay() {
	this->state = State::DELAY;
	this->time_end_delay = this->getRosTimeNow() + this->delay_time;
}

void SimSilentNxt::getMotorCommand(const offline_nxt::MotorCommand& _msg) {
	if(isStopMsg(_msg)) {
		this->next_effort_tick_A = 0.0;
		this->next_effort_tick_B = 0.0;
		this->setDelay();
		ROS_INFO("Get stop command");
	} else if(isDriveMsg(_msg)) {
		this->next_effort_tick_A = tick_A_per_second * tick_A_correction;
		this->next_effort_tick_B = tick_B_per_second * tick_B_correction;
		this->setDelay();
		ROS_INFO("Get drive command");
	}
}

void SimSilentNxt::getFakeRange(const offline_nxt::FakeRange& _msg) {
	this->use_default_range	= false;
	this->current_range	= (int)(_msg.range * 100);
	this->time_end_range	= this->getRosTimeNow() + _msg.time;
}

double SimSilentNxt::getRosTimeNow() {
	ros::Time time = ros::Time::now();
	return (time.sec * 1.0) + (time.nsec / 1000.0);
}

bool SimSilentNxt::isDelay(const double& _time_now) {
	return _time_now < this->time_end_delay;
}

void SimSilentNxt::updateToNextEffort() {
	this->effort_tick_A = this->next_effort_tick_A;
	this->effort_tick_B = this->next_effort_tick_B;
}

void SimSilentNxt::updateSimulation() {
	const double time_now = this->getRosTimeNow();
	ROS_INFO("UPDATE [%d]", this->state);
	// Update state flow
	if( this->state == State::DELAY && !this->isDelay(time_now) )  {
		this->state = State::DRIVE;
		this->updateToNextEffort();
		ROS_INFO("NEW STATE: [%d]", this->state);
	}

	// Update tick count
	if( this->state == State::STOP ) {
		this->current_tick_count_A = 0;
		this->current_tick_count_B = 0;
	} else {
		double update_length = time_now - this->last_update_time;
		this->current_tick_count_A = (int)(update_length * this->effort_tick_A);
		this->current_tick_count_B = (int)(update_length * this->effort_tick_B);

		ROS_INFO("T: %f", update_length);
		ROS_INFO("A: %f", this->effort_tick_A);
		ROS_INFO("B: %f", this->effort_tick_B);

		if(this->current_tick_count_A > 0 && this->current_tick_count_B > 0)
			this->last_update_time = time_now;
	}

	// Update range
	if( !this->use_default_range && (time_now > this->time_end_range) ) {
			this->use_default_range = true;
			this->current_range = (int)(this->default_range * 100);
	}

}

int SimSilentNxt::getTickByTime(const double& _time) {
	return (int)(_time * this->effort_tick_A);
}

offline_nxt::SensorData SimSilentNxt::getSensorDataMsg() {
	offline_nxt::SensorData msg;

	msg.tickCountA	= this->current_tick_count_A;
	msg.tickCountB	= this->current_tick_count_B;
	msg.distance	= this->current_range;

	return msg;
}

void SimSilentNxt::run() {
	ROS_INFO("RUN");
	this->state	 		= State::STOP;
	this->use_default_range 	= true;

	this->last_update_time		= this->getRosTimeNow();

	this->effort_tick_A		= 0.0;
	this->effort_tick_B		= 0.0;
	this->next_effort_tick_A	= 0.0;
	this->next_effort_tick_B	= 0.0;

	this->current_range		= this->default_range;

	this->time_end_delay		= this->getRosTimeNow();
	this->time_end_range		= this->getRosTimeNow();

	ros::Rate rate(this->motor_publish_frequency);

	while(true) {
		ROS_INFO("LOOP");
		this->updateSimulation();

		offline_nxt::SensorData msg = this->getSensorDataMsg();

		pubSensorData.publish(msg);

		ros::spinOnce();
		rate.sleep();
	}
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
	ros::init(argc, argv, "offlineNxtSimulation");
	ros::NodeHandle node;
	ROS_INFO("Init");
	SimSilentNxt simulation = SimSilentNxt(node);
	ROS_INFO("START");
	simulation.run();

	return 0;
}

