/**
 *      @file  cab_driver.cpp
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

#include<cab_driver/cab_driver.h>

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// CABDriver class methods
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

CABDriver::CABDriver()
{
	// Initialize kinematics class from the ROS robot_description parameter
	kin_.initFromParam("robot_description","contact_point");
	// Publisher constructor
	pubJointsState_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
    // Set state machine in a loop with frequency 100 Hz
	timerMain = nh_.createTimer(ros::Duration(1.0/100.0), &CABDriver::mainCallback_, this);
	//jointNames_ = {"cab_arm_0_joint","cab_arm_1_joint","cab_arm_2_joint","cab_arm_3_joint","cab_arm_4_joint","cab_arm_5_joint","cab_arm_6_joint"};
}

CABDriver::~CABDriver()
{
}

/**
 * @brief Function called with a fixed frequency
 **/
void CABDriver::mainCallback_(const ros::TimerEvent& e)
{
    //---------------------------------------------------------
    std::vector<double> initVector(kin_.nrJoints, 0.0);
	//initVector = {0.0, -0.7, 0.0, 1.0, 0.0, -1.4, 0.0};
    //---------------------------------------------------------
	sensor_msgs::JointState jnt;
    jnt.name = kin_.jointNames;
	jnt.position.resize(kin_.nrJoints);
	jnt.velocity.resize(kin_.nrJoints);
	jnt.effort.resize(kin_.nrJoints);
	jnt.header.stamp = ros::Time::now();

	jnt.position=std::vector<double>(initVector.begin(),initVector.end());
	jnt.velocity=std::vector<double>(initVector.begin(),initVector.end());
	jnt.effort=std::vector<double>(initVector.begin(),initVector.end());
	pubJointsState_.publish(jnt);
}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
// MAIN
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
/**
 * @brief  Main function.
 *
 * @param  argc - Number of inputs.
 * @param  argv - Array with the inputs as strings.
 * @return 0 - The function has terminated correctly.
 * @return 1 - The function has terminated with error.
 */
int main(int argc, char **argv) 
{
    // Initialize the ROS system and become a node, registering with the master as SweepingRobot
    ros::init(argc, argv, "cab_driver", ros::init_options::AnonymousName);
    // Initializing robot class (it has to be after init_node)
    CABDriver driver;
    // Let ROS take over.
    ros::spin();
    // Cleaning and returning
    return 0;
}
