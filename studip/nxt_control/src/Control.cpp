/**
 .2 * Control.cpp
 *
 *  Created on: 06.11.2015
 *      Author: rt6
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Range.h"
#include "Control.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include<std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
using namespace std;

Control::Control(ros::NodeHandle n) {
	_n = n;
	_distance = 0;

	//******************************************************** Parameter Server auslesen; wenn leer: Standard Werte setzen */
	if (!_n.getParam("/nxt/tol_rot", _tolRot)) {
		_tolRot = 10.0 / 180.0 * M_PI;
		_n.setParam("/nxt/tol_rot", _tolRot);
	}
	if (!_n.getParam("/nxt/tol_trans", _tolTrans)) {
		_tolTrans = 0.05;
		_n.setParam("/nxt/tol_trans", _tolTrans);
	}
	if (!_n.getParam("/nxt/axis_length", _axis_length)) {
		_axis_length = 0.15;
		_n.setParam("/nxt/axis_length", _axis_length);
	}
	if (!_n.getParam("/nxt/wheel_radius_r", _wheel_radius_r)) {
		_wheel_radius_r = 0.05;
		_n.setParam("/nxt/wheel_radius_r", _wheel_radius_r);
	}
	if (!_n.getParam("/nxt/gear_ratio_right", _gear_ratio_right)) {
		_gear_ratio_right = 1;
		_n.setParam("/nxt/gear_ratio_right", _gear_ratio_right);
	}
	if (!_n.getParam("/nxt/wheel_radius_l", _wheel_radius_l)) {
		_wheel_radius_l = 0.05;
		_n.setParam("/nxt/wheel_radius_l", _wheel_radius_l);
	}
	if (!_n.getParam("/nxt/gear_ratio_left", _gear_ratio_left)) {
		_gear_ratio_left = 1;
		_n.setParam("/nxt/gear_ratio_left", _gear_ratio_left);
	}
	if (!_n.getParam("/nxt/enc_per_turn_right", _enc_per_turn_right)) {
		_enc_per_turn_right = 1000;
		_n.setParam("/nxt/enc_per_turn_right", _enc_per_turn_right);
	}

	if (!_n.getParam("/nxt/enc_per_turn_left", _enc_per_turn_left)) {
		_enc_per_turn_left = 1000;
		_n.setParam("/nxt/enc_per_turn_left", _enc_per_turn_left);
	}

	if (!_n.getParam("/nxt/max_speed", _max_speed)) {
		_max_speed = 100;
		_n.setParam("/nxt/max_speed", _max_speed);
	}

	if (!_n.getParam("/nxt/max_turn_speed", _max_turn_speed)) {
		_max_turn_speed = 100;
		_n.setParam("/nxt/max_turn_speed", _max_turn_speed);
	}

	if (!_n.getParam("min_speed", _min_speed)) {
		_min_speed = 100;
		_n.setParam("/nxt/min_speed", _min_speed);
	}
	if (!_n.getParam("/nxt/turining_adaption", _turning_adaptation)) {
		_turning_adaptation = 1;
		_n.setParam("/nxt/turning_adaption", _turning_adaptation);
	}
	_goal.x = 0;
	_goal.y = 0;
	_goal.th = 0;
	_pose.x = 0;
	_pose.y = 0;
	_pose.th = 0;

	//******************************************************** Publisher & Subscriber initialisieren*/
	pubOdom = _n.advertise < nav_msgs::Odometry > ("odom", 10); //Topic = "odom", msg_type=Position (x,y,z)+ quaternion orientation(x,y,z) +covariance, Wartschlange = 10
	pubEffort = _n.advertise < nxt_control::MotorCommand
			> ("motor_command", 10); //Topic = "motor_command" msg_type = MotorCommand
	subCommand = _n.subscribe("cmd_vel", 10, &Control::getCommand, this);
	subSensors = _n.subscribe("sensor_data", 10, &Control::getSensorData, this);
	pubNextcmd = _n.advertise < std_msgs::Bool > ("nextcmd", 1000);
	pubUltrasonic = _n.advertise < sensor_msgs::Range > ("ultrasonic", 1000);
	_goal_reached = true;
}

void Control::getCommand(const geometry_msgs::Twist& msg) {
	double th_speed = _max_speed;

	//******************************************************** Drehung = 0: lineare Bewegung berechnen */
	if (msg.angular.z == 0) {
		_goal.x = _pose.x + msg.linear.x * cos(_pose.th);
		_goal.y = _pose.y + msg.linear.x * sin(_pose.th);
		_goal.th = _pose.th;

		ROS_INFO("MSG FROM BRAIN: x: %f ", msg.linear.x);

	}

	//******************************************************** sonst: Rotation berechnen */
	else {
		th_speed = _max_turn_speed;
		double newTheta = fmod(_pose.th + msg.angular.z + 3 * M_PI, 2 * M_PI)
				- M_PI;
		double radius = msg.linear.x / msg.angular.z;

		double cx = _pose.x - (radius) * sin(_pose.th);
		double cy = _pose.y - (radius) * -cos(_pose.th);

		_goal.x = cx + (radius) * sin(newTheta); //goal == new position of robot after cmd got executed
		_goal.y = cy + (radius) * -cos(newTheta);
		_goal.th = newTheta;
	}
	_goal_reached = false;
	//******************************************************** leistung der Motoren berechnen und publishen */
	nxt_control::MotorCommand cmd = publishEffort(msg, th_speed); //Nehme geometry_msgs/Twist msg und berechne l_effort und r_effort der Motoren
	ROS_INFO("MotorCommand: r effort before publishing: %f l effort: %f",
			cmd.r_effort, cmd.l_effort);
	pubEffort.publish(cmd);

	ros::spinOnce();
	//Leistung auf das zugehörige Topic publishen

}

nxt_control::MotorCommand Control::publishEffort(
		const geometry_msgs::Twist& msg, double th_speed) {

	//******************************************************** effort der Motoren berechnen */
	double v_l_soll = msg.linear.x + _axis_length * 0.5 * msg.angular.z;
	double v_r_soll = msg.linear.x - _axis_length * 0.5 * msg.angular.z;
	double v_left = (v_l_soll / (2.0 * M_PI * _wheel_radius_l) * 60.0
			* _gear_ratio_left);
	double v_right = (v_r_soll / (2.0 * M_PI * _wheel_radius_r) * 60.0
			* _gear_ratio_right);
	double relation = 0;
	if (v_right != 0 && v_left != 0) {

		normSpeedTo(v_left, v_right, th_speed);
	}
	nxt_control::MotorCommand cmd;
	cmd.name = "drive";
	cmd.r_effort = v_left;
	cmd.l_effort = v_right;

	return cmd;
}

sensor_msgs::Range Control::publishUltrasonicRange(
		const nxt_control::SensorData& msg) {
	sensor_msgs::Range mr;
	mr.field_of_view = 1;
	mr.header.frame_id = "ultrasonic";
	mr.header.stamp = ros::Time::now();
	mr.max_range = 2.55;
	mr.min_range = 0;
	mr.radiation_type = 1;
	mr.range = static_cast<float>(msg.distance) / 100;
	return mr;
}

void Control::getSensorData(const nxt_control::SensorData& msg) {
	updatePosition(msg);
	if (goal_reached(_pose.x, _pose.y, _pose.th)) {
		pubEffort.publish(pubStop());
		std_msgs::Bool m;
		m.data=true;
		pubNextcmd.publish(m);
	}
	pubUltrasonic.publish(publishUltrasonicRange(msg));
	pubOdom.publish(publishOdom());

}

void Control::updatePosition(const nxt_control::SensorData& msg) {
	_distance = msg.distance;
	double wheel_L = 2.0 * M_PI * _wheel_radius_l * msg.tickCountA
			/ _enc_per_turn_left;
	double wheel_R = (2.0) * M_PI * _wheel_radius_r * msg.tickCountB
			/ _enc_per_turn_right;
	double dtheta = (wheel_R - wheel_L) / _axis_length * _turning_adaptation;
	if (wheel_L * wheel_R >= 0) {
		double wheel = (wheel_L + wheel_R) / 2.0;
		_pose.x = wheel * cos(_pose.th);
		_pose.y = wheel * sin(_pose.th);
		_pose.th=0;
	} else {
		double radius = wheel_L / dtheta;
		double cx = _pose.x - (radius + _axis_length / 2.0) * sin(_pose.th);
		double cy = _pose.y + (radius + _axis_length / 2.0) * cos(_pose.th);
		std::cout << dtheta << std::endl;
		double newTheta = fmod(dtheta + 3 * M_PI, 2 * M_PI) - M_PI;
		std::cout << newTheta << std::endl;
		_pose.x = cx + (radius + _axis_length / 2.0) * sin(newTheta);
		_pose.y = cy - (radius + _axis_length / 2.0) * cos(newTheta);
		_pose.th = newTheta;
	}
}

nav_msgs::Odometry Control::publishOdom() {

	static tf::TransformBroadcaster odom_broadcaster;

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(
			_pose.th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";
	odom_trans.transform.translation.x = _pose.x;
	odom_trans.transform.translation.y = _pose.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);

	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = _pose.x;
	odom.pose.pose.position.y = _pose.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = 0;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = 0;
	return odom;
}

bool Control::goal_reached(double x, double y, double th) {
	ROS_INFO("POSE:%f,%f,%f, GOAL%f,%f,%f", x, y, th, _goal.x, _goal.y,
			_goal.th);

	return (fabs(x - _goal.x) < _tolTrans && fabs(y - _goal.y) < _tolTrans
			&& fabs(fmod(th - _goal.th + 3 * M_PI, 2 * M_PI) - M_PI) < _tolRot); //different method because of angle
}
nxt_control::MotorCommand Control::pubStop() {
	nxt_control::MotorCommand cmd;
	cmd.name = "stop";
	cmd.r_effort = 0;
	cmd.l_effort = 0;
	return cmd;
}

/**
 * Subscribe to the SensorData of the nxt-Brick, Updates the distance and position of the robot.
 * Send a stop command to the robot if the goal position is reached.
 * @param msg nxt_control/SensorData TickcountA, TickCountB and distance (Port4)
 */
double Control::getDistance() {
	return _distance;
}

void Control::normSpeedTo(double& speed1, double& speed2, double maxspeed) {
	double l=speed1;
	double r=speed2;
	double s = maxspeed / std::max(fabs(l), fabs(r));
	speed1 *= s;
	speed2 *= s;
}

Control::~Control() {
} //texttext

int main(int argc, char** argv) {
	ros::init(argc, argv, "nxtControler");
	ros::NodeHandle node;
	Control x = Control(node);
	ros::spin();				//check for incoming msg

	return 0;
}

