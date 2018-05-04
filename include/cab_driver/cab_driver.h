/**
 *      @file  cab_driver.h
 *      @brief  Driver for the Cab Front Cleaning Robot Prototype
 *
 *
 *     @author  João Moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  05-Feb-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 *   Copyright  Copyright (c) 2017, João Moura
 *
 * ===============================================================
 */

// print to terminal
#include <iostream>
// standard string
//#include <string>
// ROS header
#include <ros/ros.h>
// Messages types
#include <sensor_msgs/JointState.h>
// Kinematics library
#include <serial_arm_lib/serialArmKin.h>

class CABDriver
{
    public:
        CABDriver(); // This is the constructor declaration
        ~CABDriver(); // This is the destructor declaration
    private:
        /**
         * @brief Function called with a fixed frequency
         **/
        void mainCallback_(const ros::TimerEvent& e);

        // ROS variables:
        ros::Timer timerMain; // ROS timer to call the main cycle of the program with a specified frequency
        ros::Publisher pubJointsState_; // ROS publisher for joint state
        ros::NodeHandle nh_; // ROS node handle
        // Other Variables:
        std::vector<std::string> jointNames_; // Joint names
		//-----------------------------------------
		// Kinematic related variables
		//-----------------------------------------
		SerialArmKin kin_;
};
