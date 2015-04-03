/*********************************************************************
* Author: Jewel James
*********************************************************************/


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel


int main(int argc, char** argv)
{
	ros::init(argc, argv, "eagle_node");

	ROS_INFO("Node that acts as Visual Odometry");

    ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
	ros::Time current_time;
	
	ros::Rate r(MINIQ_RATE);
	while(n.ok())
	{
        current_time = ros::Time::now();
        
        if(!robot.update()) ROS_WARN("Sociobots minion -- Failed to update the data!!!");
	else
	{
        	

		
	}

		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
