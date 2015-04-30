/*********************************************************************
* Author: Jewel James
*********************************************************************/

#include <ros/ros.h>
#include "miniQ.h"
#include <sstream>
#include <iostream>

cereal::CerealPort miniQ::serial_port;


miniQ::miniQ()
{
    id_ = 0;
    linear_velocity_ = 0.0;
    angular_velocity_ = 0.0;
    cam_x = 0;
    cam_y = 0;
    cam_z = 0;
    cam_quat.x=0;
    cam_quat.y=0;
    cam_quat.z=0;
    cam_quat.w=1;

}

miniQ::~miniQ()
{
    
}

bool miniQ::openPort(char * port, int baudrate)
{
    try{ serial_port.open(port, baudrate); }
    catch(cereal::Exception& e)
    {
        return false;
    }
    return true;
}

void miniQ::setVelocities(double linear_velocity, double angular_velocity)
{
    linear_velocity_= linear_velocity;
    angular_velocity_ = angular_velocity;
}

bool miniQ::setPWM(int left_pwm, int left_dir, int right_pwm, int right_dir)
{   

    ROS_INFO("DIRECTIONS %d %d",left_pwm,right_pwm);
    int left_ascii = '\x01',right_ascii= '\x01',left_dir_ascii='p',right_dir_ascii='m'; 


if((left_dir/right_dir)==1)
    {
    if(abs(left_pwm)>0) left_ascii = '\x0F';
    else left_ascii = '\x01';

    if(abs(right_pwm)>0) right_ascii = '\x0F';
    else right_ascii = '\x01';
    }

if((left_dir/right_dir)==-1)
    {
    if(abs(left_pwm)>0) left_ascii = '\x14';
    else left_ascii = '\x01';

    if(abs(right_pwm)>0) right_ascii = '\x14';
    else right_ascii = '\x01';
    }

    ROS_INFO("DIRECTIONS %c %c",left_ascii,right_ascii);

    if ((left_pwm == 0)&&(right_pwm == 0))
    {
        char stop[MSG_LENGTH];
        sprintf(stop, "%c%c%c",'\x05','j','j');
        serial_port.write(stop);
    
        sprintf(stop, "%c%c%c",'\x05','k','k');
        serial_port.write(stop);
        
        return true;
    }

    if(left_dir==1)
        {left_dir_ascii= 'p';
        }
    else if (left_dir==-1)
        {left_dir_ascii= 'q';
        }

    if(right_dir==1)
        {right_dir_ascii= 'm';
        }
    else if (right_dir==-1)
        {right_dir_ascii= 'n';
        }

    char msg[MSG_LENGTH];

    sprintf(msg, "%c%c%c%c%c%c%c%c%c%c%c%c",'\x05',left_dir_ascii,left_dir_ascii,'\x05',right_dir_ascii,right_dir_ascii,'\x05','l',left_ascii,'\x05', 'r',right_ascii); 


    ROS_INFO("Xbee ing %s",msg);
    serial_port.write(msg);
    
    
    std::string reply;
    try{ serial_port.readBetween(&reply,'l','r', 600); 

         // sscanf(reply.c_str(), "l%x%xr",&left_ascii,&right_ascii);  
         // ROS_INFO("left= %d right= %d",left_ascii,right_ascii);
     }
    catch(cereal::TimeoutException& e)
    {
        return false;
    }
    
    return true;

}


bool miniQ::update()
{
    // int linear_velocity_int = (int)(linear_velocity_*1000);
    // int angular_velocity_int = (int)(angular_velocity_*1000);
    
    char msg[MSG_LENGTH];
    sprintf(msg, "%c%c%c%c%c%c", '\x02','l','\x00','\x02', 'r','\x00');
    


    serial_port.write(msg);
    
    // std::string reply;
    // try{ serial_port.readBetween(&reply, '@', 'e', 50); }
    // catch(cereal::TimeoutException& e)
    // {
    //     return false;
    // }
    
    // int id, action, x, y, yaw, gas, gas_raw;
    // sscanf(reply.c_str(), "@%d,%d,%d,%d,%de", &id, &action, &x, &y, &yaw);
    
    // x_ = x/1000.0;
    // y_ = y/1000.0;
    // yaw_ = yaw/1000.0;
    
    return true;
}
 

void miniQ::getPositionFromCamera(float x,float y,float z,geometry_msgs::Quaternion quat)
{
    cam_x = x;
    cam_y = y;
    cam_z = z;
    
    cam_quat = quat;

}

bool miniQ::updateVelocities()
{
    int linear_velocity_int = (int)(linear_velocity_*1000);
    int angular_velocity_int = (int)(angular_velocity_*1000);
    
    char msg[MSG_LENGTH];
    sprintf(msg, "@%d,%d,%d,%d,%de", id_, MQ_ACTION_DRIVE, linear_velocity_int, angular_velocity_int, 0);
    
    //ROS_INFO("Drive: %s", msg);
    
    serial_port.write(msg);
    
    return true;
}

void miniQ::setId(int id)
{
    id_ = id;
}