/*********************************************************************
* Author: Jewel James 
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <ar_track_alvar/AlvarMarkers.h>

#include "miniQ.h"

#define WHEEL_SEPARATION 8.0

miniQ robot = miniQ();

void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    robot.setVelocities(cmd_vel->linear.x, cmd_vel->angular.z);
 }



int main(int argc, char** argv)
{
	ros::init(argc, argv, "miniq_node");

	ROS_INFO("miniQ for ROS - Single robot version.");
	float left_pwm, right_pwm;
   	ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
    std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
    int baudrate;
	pn.param("baudrate", baudrate, 9600); //set the baud for sociobots network
		
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	
	ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::Twist>("/sociobots/cmd_vel", 10, cmdVelReceived);
	
    
	if(!miniQ::openPort((char*)port.c_str(), baudrate))
	{
		ROS_FATAL("miniQ -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		ROS_BREAK();
	}
	ROS_INFO("miniQ -- Successfully connected to sociobots minion!");
    // ros::Duration(0.5).sleep();
	ros::Time current_time;
	ros::Rate r(1);
	while(n.ok())
	{
		ros::spinOnce();

		nav_msgs::Odometry odom;
		
		odom.twist.twist.linear.x = robot.getLinearVelocity();
		odom.twist.twist.angular.z = robot.getAngularVelocity();
		
		ROS_INFO("Received  %f and %f",odom.twist.twist.linear.x,odom.twist.twist.angular.z);		

		left_pwm = (odom.twist.twist.linear.x - odom.twist.twist.angular.z)*(WHEEL_SEPARATION/2) *10 ;
		right_pwm = (odom.twist.twist.linear.x + odom.twist.twist.angular.z)*(WHEEL_SEPARATION/2)*10;

		ROS_INFO("Sending %f and %f",left_pwm,right_pwm);
		
		if(!robot.setPWM(left_pwm,left_pwm/(abs(left_pwm)),right_pwm,right_pwm/(abs(right_pwm))))

			{ROS_WARN("didnt recieve response");
				  ros::Duration(0.5).sleep();
				// ros::shutdown();
				}
		else{
			ROS_INFO("Rebins command recieved");
		}

		r.sleep();
	}
	return 0;
}