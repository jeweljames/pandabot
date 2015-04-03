/*********************************************************************
* Author: Gonçalo Cabrita on 20/08/2012
*********************************************************************/
#include <ros/ros.h>
#include "CerealPort.h"
#include <geometry_msgs/Quaternion.h>

#define MSG_LENGTH 64

#define MINIQ_RATE 3.0

class miniQ
{
public:
    
    //! miniQ actions
    typedef enum _miniQaction {
        
        // Get version
    	MQ_ACTION_GET_VERSION = 1,
    	// Actuate the robot
    	MQ_ACTION_DRIVE = 2,
    	MQ_ACTION_DRIVE_DIRECT = 3,
    	// Get robot sensors
    	MQ_ACTION_GET_ODOMETRY = 4,
        MQ_ACTION_GET_ENCODER_PULSES = 5,
        MQ_ACTION_GET_WHEEL_VELOCITIES = 6,
        MQ_ACTION_GET_GAS_SENSOR = 7,
    	MQ_ACTION_GET_IR_BUMPER = 8,
    	MQ_ACTION_GET_LINE_SENSOR = 9,
    	MQ_ACTION_GET_BATTERY = 10,
    	// Debug mode
    	MQ_ACTION_GET_DEBUG = 11,
    	MQ_ACTION_SET_DEBUG = 12,
    	// Configuration
    	MQ_ACTION_SET_PID_GAINS = 13,
    	MQ_ACTION_GET_PID_GAINS = 14,
    	MQ_ACTION_SET_ODOMETRY_CALIBRATION = 15,
    	MQ_ACTION_GET_ODOMETRY_CALIBRATION = 16,
    	MQ_ACTION_SET_ID = 17,
    	MQ_ACTION_GET_ID = 18,
        MQ_ACTION_SET_MODE = 19,
    	MQ_ACTION_GET_MODE = 20,
    	MQ_ACTION_SET_GAS_CALIBRATION = 21,
        MQ_ACTION_GET_GAS_CALIBRATION = 22,
    	MQ_ACTION_SET_BATTERY_TYPE = 23,
    	MQ_ACTION_GET_BATTERY_TYPE = 24,
        MQ_ACTION_SET_TIMEOUT = 25,
    	MQ_ACTION_GET_TIMEOUT = 26,
    	// Group messages
    	// Group 1 - Odometry, gas sensor
    	MQ_ACTION_GET_GROUP_1 = 27,
    	// Group 2 - Odometry, gas sensor, IR bumper
    	MQ_ACTION_GET_GROUP_2 = 28,
    	MQ_ACTION_COUNT = 29
        
    } miniQaction;

    typedef enum _miniQPID
    {
	LeftStartingPID = 0,
	LeftRunningPID = 1,
	RightStartingPID = 2,	
	RightRunningPID = 3

    } miniQPID;
    
    miniQ();
    ~miniQ();
    
    /*!
	 *
	 * \fn bool openPort(char * port, int baudrate)
	 * \brief This function opens the virtual comm to the miniQ robot.
     * \param port Serial port name.
     * \param baudrate The name is self explanatory.
     * \return True if port is open, false otherwise.
	 *
	 */
    static bool openPort(char * port, int baudrate);
    
    /*!
	 *
	 * \fn bool checkVersion()
	 * \brief This function asks the firmware version from the miniQ robot.
     * \return True if the version is compatible, false otherwise.
	 *
	 */
    bool checkVersion();
    
    /*!
	 *
	 * \fn void setVelocities(double linear_velocity, double angular_velocity)
	 * \brief This function allows to send velocity commands to the velocity controllers.
     * \param linear_velocity Linear velocity to the robot in m/s.
     * \param angular_velocity Angular velocity to the robot in m/s.
	 *
	 */
    void setVelocities(double linear_velocity, double angular_velocity);
    
    /*!
	 *
	 * \fn bool update()
	 * \brief This function communicates with the robot to update the odometry and gas sensor data.
     * \return True if update was successful, false otherwise.
	 *
	 */
    bool update();
    
    /*!
	 *
	 * \fn bool updateVelocities()
	 * \brief This function pushes the velocities in memory to the robot.
     * \return True if update was successful, false otherwise.
	 *
	 */


    bool getPositionFromCamera(float x,float y,float z, geometry_msgs::Quaternion quat );


    bool updateVelocities();
    
    /*!
	 *
	 * \fn double getPositionX()
	 * \brief Getter for the robot x position.
     * \return The robot x position.
	 *
	 */
    float getX(){ return cam_x;
             }
    /*!
	 *
	 * \fn float getPositionY()
	 * \brief Getter for the robot y position.
     * \return The robot y position.
	 *
	 */
    float getY(){ return cam_y; }

    float getZ(){ return cam_z; }

    
    // fn to return the Quaternion from the marker detected
    geometry_msgs::Quaternion getRotation(){return cam_quat;}
    float getQuatX(){return cam_qx;}
    float getQuatY(){return cam_qy;}
    float getQuatZ(){return cam_qz;}
    float getQuatW(){return cam_qw;}




    /*!
     *
     * \fn float getYay()
     * \brief Getter for the robot yaw.
     * \return The robot yaw.
	 *
	 */


    double getYaw(){ return yaw_; }
    /*!
	 *
	 * \fn double getLinearVelocity()
	 * \brief Getter for the robot linear velocity.
     * \return The robot linear velocity.
	 *
	 */
    double getLinearVelocity(){ return linear_velocity_; }
    /*!
	 *
	 * \fn double getAngularVelocity()
	 * \brief Getter for the robot angular velocity.
     * \return The robot angular velocity.
	 *
	 */
    double getAngularVelocity(){ return angular_velocity_; }
    /*!
	 *
	 * \fn double getLeftWheelVelocity()
	 * \brief Getter for the left wheel velocity.
     * \return The left wheel velocity.
	 *
	 */
    double getLeftWheelVelocity(){ return left_wheel_velocity_; }
    /*!
	 *
	 * \fn double getRightWheelVelocity()
	 * \brief Getter for the right wheel velocity.
     * \return The right wheel velocity.
	 *
	 */
    double getRightWheelVelocity(){ return right_wheel_velocity_; }
    /*!
	 *
	 * \fn double getGas()
	 * \brief Getter for the gas sensor reading.
     * \return The gas sensor reading in ppm.
	 *
	 */
    double getGas(){ return gas_; }
    /*!
	 *
	 * \fn double getRawGas()
	 * \brief Getter for the gas sensor reading.
     * \return The gas sensor reading in raw form.
	 *
	 */
    double getRawGas(){ return gas_raw_; }

    static int scanForId(int id);

    void setId(int id);
    int getID(){ return id_; };

    bool setMode(int mode);
    bool setBatteryType(int battery_type);
    bool setPIDGains(int kp, int ki, int kd, miniQPID pid);
    bool setOdometryCallibration(double odometry_d, double odometry_yaw);
    bool setGasSensorCallibration(double a, double b);
    bool setTimeout(double timeout);

    bool setPWM(int left_pwm, int left_dir, int right_pwm, int right_dir);

    bool updateWheelVelocities();
    
private:
    
    // RobChair variables
    //! Position in x
    double x_;
    //! Position in y
    double y_;
    //! Yaw
    double yaw_;
    //! Velocity in x
    double linear_velocity_;
    //! Angular velocity
    double angular_velocity_;
    //! Left wheel velocity
    double left_wheel_velocity_;
    //! Right wheel velocity
    double right_wheel_velocity_;

    float cam_x;

    float cam_y;
    float cam_z;
    float cam_qx;
    float cam_qy;
    float cam_qz;
    float cam_qw;

    geometry_msgs::Quaternion cam_quat;

    //! Gas
    double gas_;
    //! Raw gas
    double gas_raw_;

    //! Robot id
    int id_;
    
    static cereal::CerealPort serial_port;    
};

// EOF
