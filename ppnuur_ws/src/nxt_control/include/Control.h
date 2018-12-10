/*
 * Control.h
 *
 *  Created on: 06.11.2015
 *      Author: rt6
 */

#ifndef CONTROL_H_
#define CONTROL_H_
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nxt_control/MotorCommand.h>
#include <nxt_control/SensorData.h>

/**
 * The Control class takes care about the communication with a Nxt-Brick (two motors A&B and one ultrasonic port 4)
 * The Controler concept is the translation from ros commmon commands to nxt-commands.
 * The Controler calculates the MotorCommands messages corresponding to a geometry_msgs/Twist message.
 * The Controler calculates the pose from the ticks of the two Motors and publish a geometry_msgs/PoseWithCovarianceStamped at /odom topic.
 * The Controller publish the ultrasonic measurement as range message in the frame "ultrasonic"
 */

class Control {
    
    // Structure of 2D pose
    struct Pose {
        double x;
        double y;
        double th;
    };
    
    // Values
    Pose pose;
    Pose goal;
    
    // ROS values
    ros::NodeHandle node;
    
    ros::Publisher  pubOdom;
    ros::Publisher  pubUltrasonic;
    ros::Publisher  pubEffort;
    ros::Publisher  pubNextcmd;
    
    ros::Subscriber subMotorCommand;
    ros::Subscriber subSensorData;
    
    //! Distance between the wheels
    double axis_length;
    //! Radius of the robot wheel
    double wheel_radius_r;
    //! Ration between command and right motor effort
    double gear_ratio_right;
    //! Radius of the robot wheel
    double wheel_radius_l;
    //! Ration between command and right motor effort
    double gear_ratio_left;
    //! encounter per turn left wheel
    double enc_per_turn_left;
    //! encounter per turn right wheel
    double enc_per_turn_right;
    //! Adjustment for difference between left and right turning
    double turning_adaptation;
    //! Tolerance for the position
    double tolTrans;
    //! Tolerance for the heading
    double tolRot;
    
    bool   goal_reached_flag;
    double max_speed;
    double min_speed;
    double max_turn_speed;
    double distance;
    
	nxt_control::MotorCommand publishEffort(
                                const geometry_msgs::Twist& msg,
                                double th_speed);
    
	void updatePosition(const nxt_control::SensorData& msg);
    
	nav_msgs::Odometry publishOdom();
    
	void normSpeedTo(double& speed1, double& speed2, double maxspeed);
    
	nxt_control::MotorCommand pubStop();
    
	/**
	 * goal_reached checks if a position is near enough to the goal position
	 * @param [in] x double, x coordinate of the robot
	 * @param [in] y double, y coordinate of the robot
	 * @param [in] th double, theta the heading of the robot
	 * @return returns true is a position  is reached else false
	 */
	bool goal_reached(double x, double y, double th);
	/**
	 * Set the meta information to a ultrasonic scan: frame id and time stamp
	 */
	sensor_msgs::Range publishUltrasonicRange(
			const nxt_control::SensorData& msg);

public:
	double getDistance();
	/**
	 * The constructor registers all subscriber and publisher at a ros node handle
	 * The parameter axisLength, wheel radius, min/max speed, goal tolerancen gear ration, encounter per tick
	 *  and a turning adaption are requested from the ros param server or set if they don't exist.
	 */
	Control(ros::NodeHandle n);
	~Control();

	/**
	 * getCommand subscribs a geometry_msgs/Twist to publish the MotorCommands.
	 * The x translation and the z angular ot the twist messageare interpreted as a radian measure
	 * and are convert in motor efforts.
	 * This efforts are normalized to an mximum speed, for turn only the speed is adapted.
	 *
	 */
	void getCommand(const geometry_msgs::Twist& msg);
	/**
	 * getSensorData subscribs a nxt_control/Sensordata message to publish position and range message
	 */
	void getSensorData(const nxt_control::SensorData& msg);
	/**
	 * publishOdometry create a odometry message from the pose variable
	 * @return geometry_msgs::PoseWithCovarianceStamped at map frame
	 */
	nav_msgs::Odometry publishOdometry();
	/**
	 * setInitialPose sets the pose of the robot
	 * @param[in]  const geometry_msgs::PoseWithCovariance message enables pose setting via rviz
	 */
	void setInitialPose(const geometry_msgs::PoseWithCovariance& msg);
};

#endif /* CONTROL_H_ */
