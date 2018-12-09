/*
 * adpated by Jens-Andre Paffenholz@ikg, 012013
 * 20140113, added parameter to define direction of drive, since rt3 and rt6 act differently due to motor installation
 * @author: Thorsten Linder
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class cmd_vel_joystick {
public:
	cmd_vel_joystick();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	ros::NodeHandle nodeHandle;
	ros::Publisher cmd_vel_pub;
	ros::Subscriber joy_sub;

	geometry_msgs::Twist cmd_vel;

    int robot_drive_dir;
};

cmd_vel_joystick::cmd_vel_joystick() {
	joy_sub = nodeHandle.subscribe<sensor_msgs::Joy> ("/joy", 1,
			&cmd_vel_joystick::joyCallback, this);

	cmd_vel_pub = ros::Publisher(nodeHandle.advertise<geometry_msgs::Twist> (
                        "/cmd_vel", 2));

    ros::NodeHandle nh_ns("~");

    nh_ns.param("robot_drive_dir", robot_drive_dir, 1);
    ROS_INFO("drive direction is %d", robot_drive_dir);

	ROS_INFO("rt_commandByJoystick ready and waiting for input");
}
int sgn(double d){ 
    return d<0?-1:d>0;
}
void cmd_vel_joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	double maxSpeedX = 1;
	double maxSpeedAngularZ = -1;

    // if((true == joy->buttons[8])) {
    //     // turbo
    //     maxSpeedX = +0.75;
    //     maxSpeedAngularZ = +0.75;
    // }
    // else {
    //     maxSpeedX = +0.25;
    //     maxSpeedAngularZ = +0.25;
    // }
    
	if ((true == joy->buttons[10])) { //&& (true == joy->buttons[1])
		ROS_DEBUG("speedX=%f speedZ=%f", joy->axes[1], joy->axes[0] );
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;
		//speed
		if(fabs(joy->axes[1])>fabs(joy->axes[0])&&(fabs(joy->axes[1])>0.1||fabs(joy->axes[0])>0.1)){
		cmd_vel.linear.x = maxSpeedX * robot_drive_dir*sgn(joy->axes[1]);
		cmd_vel.angular.z=0.0;
		}
		else if(fabs(joy->axes[0])>fabs(joy->axes[1])&&(fabs(joy->axes[1])>0.1||fabs(joy->axes[0])>0.1)){
		  cmd_vel.linear.x=0.0;
		  cmd_vel.angular.z = maxSpeedAngularZ*sgn(joy->axes[0]);
		}
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
	
	}
	else {
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.linear.z = 0.0;
		cmd_vel.angular.x = 0.0;
		cmd_vel.angular.y = 0.0;
		cmd_vel.angular.z = 0.0;
	}
	cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "rt_commandByJoystick");
	cmd_vel_joystick rt_commandByJoystick;

	ros::spin();
}
