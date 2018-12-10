/**
 * Simulate the robot msgs/topics to try new functions offline
 * sim_silent_nxt.h
 *
 *  @date   09.12.2018
 *  @author GT
 */

#ifndef SIM_SILENT_NXT_H_
#define SIM_SILENT_NXT_H_

// ROS include
#include <ros/ros.h>

// Special ros msgs
#include "offline_nxt/MotorCommand.h"
#include "offline_nxt/SensorData.h"
#include "offline_nxt/FakeRange.h"

class SimSilentNxt {

private:
	enum State {
		DRIVE,
		DELAY,
		STOP
	};

	// Simulation state parameter
	State	state;
	bool	use_default_range;
	double	last_update_time;
	double	time_end_delay;
	double	time_end_range;

	int	current_tick_count_A;
	int	current_tick_count_B;
	int	current_range;

	double	effort_tick_A;
	double	effort_tick_B;
	double	next_effort_tick_A;
	double	next_effort_tick_B;

	// Simulation parameter (A = left, B = right)
	double motor_publish_frequency;
	double tick_A_per_second;
	double tick_B_per_second;
	double tick_A_correction;
	double tick_B_correction;
	double delay_time;
	double default_range;


	// ROS values
	ros::NodeHandle node;

	ros::Subscriber	subMotorCommand;
	ros::Subscriber	subFakeRange;
	ros::Publisher	pubSensorData;

	// Set states functions
	bool isStopMsg(const offline_nxt::MotorCommand& _msg);
	bool isDriveMsg(const offline_nxt::MotorCommand& _msg);

	// State run functions
	double	getRosTimeNow();

	void setDelay();
	bool isDelay(const double& _time_now);

	void updateToNextEffort();

//	void updateStateFlow();

	int getTickByTime(const double& _time);

	void updateSimulation();

	offline_nxt::SensorData getSensorDataMsg();

	// Get simulation parameter from ros node
	void getParam();
public:
	// Constuctor
	SimSilentNxt(ros::NodeHandle _node);
	~SimSilentNxt();

	// Simulation main function
	void run();

	// ros topic subscriber trigger for the motor command msg
	void getMotorCommand(const offline_nxt::MotorCommand& _msg);

	// ros topic subscriber trigger for fake range from ultrasonic
	void getFakeRange(const offline_nxt::FakeRange& _msg);

	// publish the simulated sensor data
//	void publishSensorData(const offline_nxt::SensorDate& _msg);
};

#endif
