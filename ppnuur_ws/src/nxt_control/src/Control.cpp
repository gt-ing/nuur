/**
 * Control.cpp
 *
 *  Created on: 06.11.2015
 *      Author: rt6
 *
 * Update on: 10.12.2018
 *	Author: GT
 */

#include "Control.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

Control::Control(ros::NodeHandle _node) {
	this->node = _node;
	this->distance = 0;

	// Parameter Server auslesen; wenn leer: Standard Werte setzen
    
	if (!this->node.getParam("/nxt/tol_rot",
                             this->tolRot)) {
		this->tolRot = 10.0 / 180.0 * M_PI;
		this->node.setParam("/nxt/tol_rot",
                            this->tolRot);
	}
    
	if (!this->node.getParam("/nxt/tol_trans",
                             this->tolTrans)) {
		this->tolTrans = 0.05;
		this->node.setParam("/nxt/tol_trans",
                            this->tolTrans);
	}
    
	if (!this->node.getParam("/nxt/axis_length",
                             this->axis_length)) {
		this->axis_length = 0.15;
		this->node.setParam("/nxt/axis_length",
                            this->axis_length);
	}
    
	if (!this->node.getParam("/nxt/wheel_radius_r",
                             this->wheel_radius_r)) {
		this->wheel_radius_r = 0.05;
		this->node.setParam("/nxt/wheel_radius_r",
                            this->wheel_radius_r);
	}
    
	if (!this->node.getParam("/nxt/gear_ratio_right",
                             this->gear_ratio_right)) {
		this->gear_ratio_right = 1;
		this->node.setParam("/nxt/gear_ratio_right",
                            this->gear_ratio_right);
	}
    
	if (!this->node.getParam("/nxt/wheel_radius_l",
                             this->wheel_radius_l)) {
		this->wheel_radius_l = 0.05;
		this->node.setParam("/nxt/wheel_radius_l",
                            this->wheel_radius_l);
	}
    
	if (!this->node.getParam("/nxt/gear_ratio_left",
                             this->gear_ratio_left)) {
		this->gear_ratio_left = 1;
		this->node.setParam("/nxt/gear_ratio_left",
                            this->gear_ratio_left);
	}
    
	if (!this->node.getParam("/nxt/enc_per_turn_right",
                             this->enc_per_turn_right)) {
		this->enc_per_turn_right = 1000;
		this->node.setParam("/nxt/enc_per_turn_right",
                            this->enc_per_turn_right);
	}

	if (!this->node.getParam("/nxt/enc_per_turn_left",
                             this->enc_per_turn_left)) {
		this->enc_per_turn_left = 1000;
		this->node.setParam("/nxt/enc_per_turn_left",
                            this->enc_per_turn_left);
	}

	if (!this->node.getParam("/nxt/max_speed",
                             this->max_speed)) {
		this->max_speed = 100;
		this->node.setParam("/nxt/max_speed",
                            this->max_speed);
	}

	if (!this->node.getParam("/nxt/max_turn_speed",
                             this->max_turn_speed)) {
		this->max_turn_speed = 100;
		this->node.setParam("/nxt/max_turn_speed",
                            this->max_turn_speed);
	}

	if (!this->node.getParam("/nxt/min_speed",
                             this->min_speed)) {
		this->min_speed = 100;
		this->node.setParam("/nxt/min_speed",
                            this->min_speed);
	}
    
	if (!this->node.getParam("/nxt/turining_adaption",
                             this->turning_adaptation)) {
		this->turning_adaptation = 1;
		this->node.setParam("/nxt/turning_adaption",
                            this->turning_adaptation);
	}
    
    // Inital Position und Ziel setzten
    this->pose.x = 0;
    this->pose.y = 0;
    this->pose.th = 0;
	this->goal.x = 0;
	this->goal.y = 0;
	this->goal.th = 0;
    this->goal_reached_flag = true;

	// Publisher & Subscriber initialisieren
	this->pubOdom   = this->node.advertise<nav_msgs::Odometry>(
                                                    "odom", 10);
	this->pubEffort = this->node.advertise<nxt_control::MotorCommand>(
                                                    "motor_command", 10);
    this->pubNextcmd    = this->node.advertise<std_msgs::Bool>(
                                                    "nextcmd", 1000);
    this->pubUltrasonic = this->node.advertise<sensor_msgs::Range>(
                                                    "ultrasonic", 1000);
    
	this->subMotorCommand   = this->node.subscribe("cmd_vel", 10,
                                      &Control::getCommand, this);
	this->subSensorData     = this->node.subscribe("sensor_data", 10,
                                      &Control::getSensorData, this);

}

void Control::getCommand(const geometry_msgs::Twist& msg) {
	double th_speed = this->max_speed;

	// Drehung = 0: lineare Bewegung berechnen
	if (msg.angular.z == 0) {
		this->goal.x    = this->pose.x + msg.linear.x * cos(this->pose.th);
		this->goal.y    = this->pose.y + msg.linear.x * sin(this->pose.th);
		this->goal.th   = this->pose.th;

		ROS_INFO("MSG FROM BRAIN: x: %f ", msg.linear.x);
	}
    // sonst: Rotation berechnen */
	else {
		th_speed = this->max_turn_speed;
		double newTheta = fmod(this->pose.th + msg.angular.z
                               + 3 * M_PI, 2 * M_PI) - M_PI;
        
		double radius = msg.linear.x / msg.angular.z;

		double cx = this->pose.x - (radius) * sin(this->pose.th);
		double cy = this->pose.y - (radius) * -cos(this->pose.th);
        
		this->goal.x = cx + (radius) * sin(newTheta);
		this->goal.y = cy + (radius) * -cos(newTheta);
		this->goal.th = newTheta;
        
        ROS_INFO("MSG FROM BRAIN: th: %f ", msg.angular.z);
	}
    
	this->goal_reached_flag = false;
    
	// Leistung der Motoren berechnen und publishen
	nxt_control::MotorCommand cmd = publishEffort(msg, th_speed);
	ROS_INFO("MotorCommand: r_effort: %f, l_effort: %f",
             cmd.r_effort, cmd.l_effort);
	pubEffort.publish(cmd);

    //Leistung auf das zugehörige Topic publishen
	ros::spinOnce();
}

nxt_control::MotorCommand Control::publishEffort(
                        const geometry_msgs::Twist& msg,
                        double th_speed) {

	//effort der Motoren berechnen
	double v_l_soll = msg.linear.x
                    + this->axis_length * 0.5 * msg.angular.z;
	double v_r_soll = msg.linear.x
                    - this->axis_length * 0.5 * msg.angular.z;
    
	double v_left  = v_l_soll / (2.0 * M_PI * this->wheel_radius_l)
                     * 60.0 * this->gear_ratio_left;
	double v_right = v_r_soll / (2.0 * M_PI * this->wheel_radius_r)
                     * 60.0 * this->gear_ratio_right;
    
	double relation = 0;
    
	if (v_right != 0 && v_left != 0) {
		normSpeedTo(v_left, v_right, th_speed);
	}
    
    // create msg
	nxt_control::MotorCommand cmd;
	cmd.name = "drive";
	cmd.r_effort = v_left;
	cmd.l_effort = v_right;

	return cmd;
}

sensor_msgs::Range Control::publishUltrasonicRange(
                        const nxt_control::SensorData& msg) {
    
    // Broadcasting the ultrasonic transformation
    static tf::TransformBroadcaster ultrasonic_broadcaster;
    
    geometry_msgs::TransformStamped ultrasonic_trans;
    
    ultrasonic_trans.header.stamp   = ros::Time::now();
    ultrasonic_trans.header.frame_id    = "ultrasonic";
    ultrasonic_trans.child_frame_id     = "robot";
    ultrasonic_trans.transform.translation.x = 0.085;
    ultrasonic_trans.transform.translation.y = 0.000;
    ultrasonic_trans.transform.translation.z = 0.050;
    ultrasonic_trans.transform.rotation      = tf::createQuaternionMsgFromYaw(0.0);
    
    ultrasonic_broadcaster.sendTransform( ultrasonic_trans );
    
    // Ros range msg
	sensor_msgs::Range mr;
	mr.field_of_view    = 1;
	mr.header.frame_id  = "ultrasonic";
	mr.header.stamp     = ros::Time::now();
	mr.max_range        = 2.55;
	mr.min_range        = 0;
	mr.radiation_type   = 1;  // 0 for Ultrasonic will be correct
    
	mr.range = static_cast<float>(msg.distance) / 100; // to meters
    
	return mr;
}

void Control::getSensorData(const nxt_control::SensorData& msg) {
   
    // Update the positon
	updatePosition(msg);
    
    // Update range
    this->distance = msg.distance;
    
    // check the result is reached, if true: stop motor
	if (goal_reached(this->pose.x, this->pose.y, this->pose.th)) {
        
        // Stop the motor
		pubEffort.publish(pubStop());
        
        // Send message: Command completed
		std_msgs::Bool  m;
		m.data = true;
		pubNextcmd.publish(m);
        
        // Reset goal tracker
        this->goal_reached_flag = true;
	}
    
    // Republish sonic range in ros format with transformation
	pubUltrasonic.publish(publishUltrasonicRange(msg));
    
    // Publish the updated pose als odometry transformation/nav_msg
	pubOdom.publish(publishOdom());
}

// ONLY WORKS WITH NEGATIVE TICKS FOR BAKCWARD DRIVE
void Control::updatePosition(const nxt_control::SensorData& msg) {
    // Transform motor ticks into distance and rotation
	double wheel_L = 2.0 * M_PI * this->wheel_radius_l
                    * msg.tickCountA / this->enc_per_turn_left;
	double wheel_R = 2.0 * M_PI * this->wheel_radius_r
                    * msg.tickCountB / this->enc_per_turn_right;
	double dtheta = (wheel_R - wheel_L) / this->axis_length
                    * this->turning_adaptation;
    
	if (wheel_L * wheel_R >= 0) {      // If linear (fabs(dtheta) < 0.005)
		double wheel    = (wheel_L + wheel_R) / 2.0;
		this->pose.x    += wheel * cos(this->pose.th);
		this->pose.y    += wheel * sin(this->pose.th);
		this->pose.th   += 0;
	} else {                           // If rotation (forward * backward)
		double radius = wheel_L / dtheta;
		double cx = this->pose.x - (radius + this->axis_length / 2.0)
                    * sin(this->pose.th);
		double cy = this->pose.y + (radius + this->axis_length / 2.0)
                    * cos(this->pose.th);
        
		std::cout << dtheta << std::endl;
		double newTheta = fmod(dtheta + 3 * M_PI, 2 * M_PI) - M_PI;
		std::cout << newTheta << std::endl;
        
		this->pose.x = cx + (radius + this->axis_length / 2.0)
                    * sin(newTheta);
		this->pose.y = cy - (radius + this->axis_length / 2.0)
                    * cos(newTheta);
		this->pose.th = newTheta;
	}
}

nav_msgs::Odometry Control::publishOdom() {

	static tf::TransformBroadcaster odom_broadcaster;

	geometry_msgs::Quaternion odom_quat =
                tf::createQuaternionMsgFromYaw(this->pose.th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp     = ros::Time::now();
	odom_trans.header.frame_id  = "odom";
	odom_trans.child_frame_id   = "base_link";
	odom_trans.transform.translation.x  = this->pose.x;
	odom_trans.transform.translation.y  = this->pose.y;
	odom_trans.transform.translation.z  = 0.0;
	odom_trans.transform.rotation       = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);

    // seond, we'll publish the odom nav_msg
	nav_msgs::Odometry odom;
	odom.header.stamp           = ros::Time::now();
	odom.header.frame_id        = "odom";
    odom.child_frame_id         = "base_link";
    // set the pose
	odom.pose.pose.position.x   = this->pose.x;
	odom.pose.pose.position.y   = this->pose.y;
	odom.pose.pose.position.z   = 0.0;
	odom.pose.pose.orientation  = odom_quat;
	//set the velocity
	odom.twist.twist.linear.x   = 0;
	odom.twist.twist.linear.y   = 0;
	odom.twist.twist.angular.z  = 0;
    
	return odom;
}

bool Control::goal_reached(double x, double y, double th) {
	ROS_INFO("POSE:%f,%f,%f, GOAL%f,%f,%f",
             x, y, th,
             this->goal.x, this->goal.y, this->goal.th);

	return (fabs(x - this->goal.x) < this->tolTrans
            && fabs(y - this->goal.y) < this->tolTrans
			&& fabs(fmod(th - this->goal.th + 3 * M_PI, 2 * M_PI) - M_PI)
                < this->tolRot);  //different method because of angle
}


nxt_control::MotorCommand Control::pubStop() {
	nxt_control::MotorCommand cmd;
    
	cmd.name        = "stop";
	cmd.r_effort    = 0;
	cmd.l_effort    = 0;
    
	return cmd;
}

double Control::getDistance() {
	return this->distance;
}

void Control::normSpeedTo(double& speed1,
                          double& speed2,
                          double maxspeed) {
	double l    = speed1;
	double r    = speed2;
	double s    = maxspeed / std::max(fabs(l), fabs(r));
	speed1      *= s;
	speed2      *= s;
}

Control::~Control() {} //texttext

int main(int argc, char** argv) {
	ros::init(argc, argv, "nxtControler");
	ros::NodeHandle node;
	Control x = Control(node);
	ros::spin();				//check for incoming msg

	return 0;
}

