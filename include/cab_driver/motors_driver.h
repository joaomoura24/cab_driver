/**
 *      @file  motors_driver.cpp
 *      @brief  Driver for the Cab Front Cleaning Robot Prototype
 *
 *
 *     @author  William McColl, wm70@hw.ac.uk
 *     @author  João Moura, Joao.Moura@ed.ac.uk
 *
 *   @internal
 *     Created  02-Mar-2017
 *    Revision  ---
 *    Compiler  gcc/g++
 *     Company  Edinburgh Centre for Robotics
 *   Copyright  Copyright (c) 2017, João Moura
 *
 * ===============================================================
 */
#ifndef _MOTOR_DRIVER_
#define _MOTOR_DRIVER_

#include <iostream>
#include <stdlib.h>
#include <sstream>
// C library for time
#include <ctime>
// C++ strings
#include <string>
// Library for vectors, matrices, and algebra operations
#include <Eigen/Dense>
// ROS header
#include <ros/ros.h>
#include <serial/serial.h>
// Type of message to be published:
#include <sensor_msgs/JointState.h>
// Kinematics library
#include <serial_arm_lib/serialArmKin.h>

#define DriverMsgError(msg) (std::string("Lowleveldriver: ") + (msg) + "\nLine: " + std::to_string(__LINE__) + "; Function " + __PRETTY_FUNCTION__ + "; File " + __FILE__)
#define SERIAL_MSG_SIZE 12
#define CMD_MSG_SIZE 20
#define MAX_MOTOR_ROT 3000

class Lowleveldriver
{
    public:
        Lowleveldriver(); // constructor declaration
        ~Lowleveldriver(); // destructor declaration
        // Other public functions
        void commandVel_(const sensor_msgs::JointState::ConstPtr& msg);
        // ROS publisher message:
    private:
		// Kinematic related variables
		SerialArmKin kin_;
        // Serial comm Variables
        serial::Serial serialComm_; // serial communication variable
        std::string arduino_output_;
		// ROS variables
		ros::NodeHandle nh_;
        ros::Publisher pub_joint_pos_;
		ros::Timer cycleTimer_;
		ros::Timer firstTimer_;
		ros::Subscriber sub_;
		sensor_msgs::JointState pubMsg_;
		// Methods
        void serial_communication_(const ros::TimerEvent& e);
		// sensor readings
		Eigen::Matrix<int, 6, 1> jointLimits_;
		Eigen::Matrix<int, 5, 1> motorPosInt_;
		Eigen::Matrix<double, 5, 1> motorPos_;
		Eigen::Matrix<int, 5, 1> prevMotorPosInt_;
		Eigen::Matrix<int, 5, 1> deltaMotorPosInt_;
		Eigen::Matrix<int, 5, 1> zeroMotorPosInt_;
		Eigen::Matrix<int, 5, 1> fullRotationCounter_;
		std::string prevCommand_; 
		// measurements conversion constants
		Eigen::Matrix<double, 5, 1> ticks2rot_;
		Eigen::Matrix<double, 5, 1> rot2SI_;
		Eigen::Matrix<double, 5, 1> maxRPM;
		double maxTicks;
		double sec2min;
		// Variable toi flag calibrarion execution
		bool calibrated_ = false;
		// Command motor velocity variables
		Eigen::Matrix<double, 5, 1> cmdVel_;
		Eigen::Matrix<int, 5, 1> cmdVelInt_;
		Eigen::Matrix<int, 5, 1> cmdVelIntAbs_;
		std::string ref_string = "000";
};

#endif // _MOTOR_DRIVER_
