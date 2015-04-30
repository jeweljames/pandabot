/**********************************************************************
 Author: Gonçalo Cabrita on 03/09/2012
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>			// odom
#include <geometry_msgs/Twist.h>		// cmd_vel
#include <ar_track_alvar/AlvarMarkers.h>

#include "miniQ.h"

// Robot data structure
class Robot
{
    public:
    Robot(int id) : robot(), odom_pub(), cmd_vel_sub(), prefix()
    {
        robot.setId(id);

        prefix = "/robot_";
        prefix.append<int>(1, 48+id);
        ROS_WARN("Created robot %s",prefix.c_str());
    };
    
    Robot(const Robot& r) : robot(), odom_pub(), cmd_vel_sub(), prefix()	
    {
        robot = r.robot;

	odom_pub = r.odom_pub;
	cmd_vel_sub = r.cmd_vel_sub;
	prefix = r.prefix;

    };
    
    // Robot
    miniQ robot;
    
    ros::Publisher odom_pub;
    ros::Subscriber cmd_vel_sub;
	ros::Subscriber visual_odom_sub ;



    std::string prefix;
};

// The group of miniQs!!!
std::vector<Robot> miniqs;


void cmdVelReceived(int index, const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    miniqs[index].robot.setVelocities(cmd_vel->linear.x, cmd_vel->angular.z);
}

void visualOdomCallback(int index , const ar_track_alvar::AlvarMarkers::ConstPtr& marker_msg)
{
    
     for (unsigned int i = 0; i < marker_msg->markers.size(); i ++)
  {
    //check if the marker detected is the calibration marker
    if (marker_msg->markers[i].id == index)
    {
     miniqs[index].robot.getPositionFromCamera(marker_msg->markers[i].pose.pose.position.x,marker_msg->markers[i].pose.pose.position.y,
     							marker_msg->markers[i].pose.pose.position.z, marker_msg->markers[i].pose.pose.orientation) ;
    }
  }
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "miniq_node");

	ROS_INFO("miniQ for ROS - Multi robot version.");

    	ros::NodeHandle n;
	ros::NodeHandle pn("~");
    
    	std::string port;
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
    	int baudrate;
	pn.param("baudrate", baudrate, 9600);

    	if(!miniQ::openPort((char*)port.c_str(), baudrate))
	{
		ROS_FATAL("miniQ -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		
	}
	ROS_INFO("miniQ -- Successfully connected to the miniQ!");

    	std::vector<int> ids;
	
        // Lets load the list of robot ids...
    	XmlRpc::XmlRpcValue list_of_ids;
    	if( n.getParam("/list_of_ids", list_of_ids) )
    	{   
        	ROS_ASSERT(list_of_ids.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<list_of_ids.size() ; ++i) 
		{
		    ROS_ASSERT(list_of_ids[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    ids.push_back(static_cast<int>(list_of_ids[i]));
		}
    	}
    	else
    	// If a list of ids is not defined scan for robots...
    	{
		ROS_FATAL("miniQ -- A list of IDs was not provided, scanning for robots...");
        
    	}
	if(ids.size() == 0)
	{
		ROS_FATAL("miniQ -- Could not find any miniQs!!!");
		ROS_BREAK();
	}
	ROS_INFO("miniQ -- Finished scanning for robots!");
	    
	for(int i=0 ; i<ids.size() ; ++i) 
	{
		miniqs.push_back(Robot(ids[i]));

		std::string odom_topic = miniqs[i].prefix;
		odom_topic.append("/odom");
		miniqs[i].odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 50);

		std::string cmd_vel_topic = miniqs[i].prefix;
		cmd_vel_topic.append("/cmd_vel");
		miniqs[i].cmd_vel_sub = n.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 10, boost::bind(cmdVelReceived, i, _1) );
		
		std::string visual_odom_topic = "/ar_pose_marker";
		
		miniqs[i].visual_odom_sub = n.subscribe<ar_track_alvar::AlvarMarkers>(visual_odom_topic,10, boost::bind(visualOdomCallback, i, _1));

	
	}

	tf::TransformBroadcaster odom_broadcaster;
    
	ros::Time current_time;

	std::string frame_id;
	
	ros::Rate r(2);
	while(n.ok())
	{
			ros::spinOnce();
        	current_time = ros::Time::now();
        
		for(int i=0 ; i<miniqs.size() ; i++)
       	 	{
						
			double odom_x = miniqs[i].robot.getX();
			double odom_y = miniqs[i].robot.getY();
        	double yaw_angle = tf::getYaw(miniqs[i].robot.getRotation());
			//ROS_INFO("Publishing data... %lf %lf %lf", odom_x, odom_y, odom_yaw);
	
			// Since all odometry is 6DOF we'll need a quaternion created from yaw
			geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(yaw_angle);
	
			// First, we'll publish the transform over tf
			//jewel added a 1.55 z-offset to see the robot not at the camera frame but on the arena
		
			tf::Transform new_tf(tf::createQuaternionFromYaw(yaw_angle), tf::Vector3(odom_x, odom_y, 1.55));



			ros::Time transform_expiration = current_time + ros::Duration(1/(MINIQ_RATE)*2.0);
			std::string odom_frame_id = "/camera1";
			
			std::string base_frame_id = miniqs[i].prefix;
			base_frame_id.append("/footprint");
			tf::StampedTransform odom_trans(new_tf, transform_expiration, odom_frame_id, base_frame_id);
	
			// Send the transform
			odom_broadcaster.sendTransform(odom_trans);
	
			// ros::Duration((1/MINIQ_RATE)/(double)(miniqs.size())-elapsed_time.toSec()*2.0).sleep();
		}
		r.sleep();
	}
	return 0;
}

// EOF
