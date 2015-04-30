/*********************************************************************
* Author: Jewel James
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <angles/angles.h>
#include <tf/transform_listener.h>

// Simple Navigation States
typedef enum _SimpleNavigationState {
    
    SN_STOPPED = 1,
    SN_MOVING = 2,
    SN_ROTATING = 3,
    SN_MOVING_AS = 4,
    SN_ROTATING_AS = 5
    
} SimpleNavigationState;

// Simple Navigation State
SimpleNavigationState state;

// Global frame_id
std::string global_frame_id;

// Target position
geometry_msgs::PoseStamped real_goal, goal;


// Robot odometry
nav_msgs::Odometry odom;

bool rotate_in_place;

void goalReceived(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    real_goal = *msg;

    if(rotate_in_place) state = SN_ROTATING;
    else state = SN_MOVING;
}

void odomReceived(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_base_simple");
    
    ROS_INFO("Move Base Simple for ROS");
    
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    
    tf::TransformListener listener;
    
    // Parameters
    double rate;
    double in_place_angular_velocity;
    double max_linear_velocity;
    double min_linear_velocity;
    double alpha;
    double attraction_coefficient;
    double repulsion_coefficient;
    double goal_tolerance;
    double angular_threshold;

    bool visualization;

    pn.param("rate", rate, 1.0);
    pn.param("in_place_angular_velocity", in_place_angular_velocity, 3.0);
    pn.param("max_linear_velocity", max_linear_velocity, 0.2);
    pn.param("min_linear_velocity", min_linear_velocity, 0.05);
    pn.param("alpha", alpha, 0.5);
    pn.param("attraction_coefficient", attraction_coefficient, 0.5);
    pn.param("goal_tolerance", goal_tolerance, 3.80);
    pn.param("angular_threshold", angular_threshold, 0.4);
    pn.param("visualization", visualization, false);

    if(angular_threshold == 0.0)
    {
    rotate_in_place = false;
    ROS_INFO("MoveBase Simple -- Not using in-place rotations.");
    }
    else
    {
        rotate_in_place = true;
    ROS_INFO("MoveBase Simple -- Using in-place rotations.");
    }

    pn.param<std::string>("global_frame_id", global_frame_id, "/footprint");
    
    ROS_INFO("MoveBase Simple -- Using %s as the global frame.", global_frame_id.c_str());
    
    // Making all the publishing and subscriptions...
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/sociobots/cmd_vel", 10);
    ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odom", 20, odomReceived);
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, goalReceived);
    
    state = SN_STOPPED;
    
    // Main Loop
    ros::Rate r(rate);
    while(n.ok())
    {
        ros::spinOnce();
    
        //jewel did this because the goal being returned through rviz was w.r.t to the map frame. the odom was 
        //published between camera1 and footprint. 
        //so the goal is transformed to the camera1 frame before using it for calculation

        try{  listener.transformPose("camera1",ros::Time(0),real_goal,"map",goal); 
            }
        catch( tf::TransformException ex)
        {
          ROS_ERROR("transfrom exception : %s",ex.what());
        }
        
        double linear_velocity = 0.0;
        double angular_velocity = 0.0;
        
        double current_orientation = tf::getYaw(odom.pose.pose.orientation);
        double current_x = odom.pose.pose.position.x;
        double current_y = odom.pose.pose.position.y;
        
        float distance_to_be_covered = sqrt(pow(current_x-goal.pose.position.x,2)+pow(current_y-goal.pose.position.y,2));
        ROS_FATAL("Distance _left = %f", distance_to_be_covered);

        // If we reached our target position 
        if((state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS) && (distance_to_be_covered) < goal_tolerance)
        {   
            state = SN_STOPPED;
            linear_velocity = 0.0;
            angular_velocity = 0.0;
        }

        // If we are moving...
        if(state == SN_MOVING || state == SN_MOVING_AS || state == SN_ROTATING || state == SN_ROTATING_AS)
        {
            double G_attr_x = -attraction_coefficient*(current_x-goal.pose.position.x);
            double G_attr_y = -attraction_coefficient*(current_y-goal.pose.position.y);
            
            double target_orientation = atan2(G_attr_y, G_attr_x);
            
            linear_velocity = sqrt(G_attr_x*G_attr_x + G_attr_y*G_attr_y);
            if(fabs(linear_velocity) > max_linear_velocity) linear_velocity = (linear_velocity > 0 ? max_linear_velocity : -max_linear_velocity);
            if(fabs(linear_velocity) < min_linear_velocity) linear_velocity = (linear_velocity > 0 ? min_linear_velocity : -min_linear_velocity);

        angular_velocity = -alpha*(angles::shortest_angular_distance(target_orientation, current_orientation));

        // If we intend to rotate before moving forward...
        if(state == SN_ROTATING || state == SN_ROTATING_AS)
        {
            linear_velocity = 0.0;
        angular_velocity = (angular_velocity < 0 ? -in_place_angular_velocity : in_place_angular_velocity);
    
            if(fabs(angles::shortest_angular_distance(current_orientation, target_orientation)) < angular_threshold)
            {
                    angular_velocity = 0.0;
                if(state == SN_ROTATING) state = SN_MOVING;
                else if(state == SN_ROTATING_AS) state = SN_MOVING_AS;
            }
        }
        }
        
        // Send the new velocities to the robot...
        geometry_msgs::Twist cmd_vel;
        
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        
        cmd_vel_pub.publish(cmd_vel);
                
	r.sleep();
    }

  	return(0);
}

// EOF

