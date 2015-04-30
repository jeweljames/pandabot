/*********************************************************************
* Author: Jewel James 
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <ar_track_alvar/AlvarMarkers.h>

#include "miniQ.h"



miniQ robot = miniQ();


void visualOdomCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& marker_msg)
{
  for (unsigned int i = 0; i < marker_msg->markers.size(); i ++)
  {
    //check if the marker detected is the calibration marker
    if (marker_msg->markers[i].id == 0)
    {
     robot.getPositionFromCamera(marker_msg->markers[i].pose.pose.position.x,marker_msg->markers[i].pose.pose.position.y,
     							marker_msg->markers[i].pose.pose.position.z, marker_msg->markers[i].pose.pose.orientation) ;
    }
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_updater");

	ROS_INFO("miniQ for ROS - Single robot version.");

	float left_pwm, right_pwm;

	

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
   	tf::TransformBroadcaster odom_broadcaster;
	
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
	ros::Subscriber visual_odom = n.subscribe<ar_track_alvar::AlvarMarkers>("/ar_pose_marker",10, visualOdomCallback);

	
    ROS_INFO("miniQ -- Successfully connected to sociobots minion!");
    
        
	ros::Time current_time;
	
	ros::Rate r(2);
	while(n.ok())
	{
		ros::spinOnce();
        current_time = ros::Time::now();
        
        
			double odom_x = robot.getX();
	       	double odom_y = robot.getY();
	       	
        	double yaw_angle = tf::getYaw(robot.getRotation());

        	geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(yaw_angle);

        	
		// ROS_INFO("Publishing data... %lf %lf %lf", odom_x, odom_y, odom_yaw);
		
		// Since all odometry is 6DOF we'll need a quaternion created from yaw
		// geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);
		
		// First, we'll publish the transform over tf

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "camera1";
		odom_trans.child_frame_id = "footprint";
		
		odom_trans.transform.translation.x = odom_x;
		odom_trans.transform.translation.y = odom_y;
		odom_trans.transform.translation.z = 1.55;
		odom_trans.transform.rotation = odom_quaternion;
		
		// Send the transform
		odom_broadcaster.sendTransform(odom_trans);
		
		// Next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "camera1";
		
		// Set the position
		odom.pose.pose.position.x = odom_x;
		odom.pose.pose.position.y = odom_y;
		odom.pose.pose.orientation= odom_quaternion;
		
		// Set the velocity
		odom.child_frame_id = "footprint";
		odom.twist.twist.linear.x = robot.getLinearVelocity();
		odom.twist.twist.angular.z = robot.getAngularVelocity();
		// Publish the message
		odom_pub.publish(odom);
		r.sleep();
	}
	return 0;
}