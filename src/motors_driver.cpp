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
#include <cab_driver/motors_driver.h>

// Member functions definitions (including constructor and destructor)
Lowleveldriver::Lowleveldriver(void)
{
	// get kinematics for toolLink
	std::string toolLink; // End-effector/tool link name
	ros::param::param<std::string>("~toolLink",toolLink,"contact_point");
	// Initialize kinematics class from the ROS robot_description parameter
	kin_.initFromParam("robot_description",toolLink);
	// fill in pubMsg_ with joint names
	pubMsg_.name = kin_.jointNames;
	// Initialize variables
	jointLimits_.setZero();
	motorPos_.setZero();
	zeroMotorPosInt_.setZero();
	motorPosInt_.setZero(); prevMotorPosInt_.setZero(); deltaMotorPosInt_.setZero();
	prevCommand_.resize(CMD_MSG_SIZE);
	fullRotationCounter_.setZero();
	// Set constants
	ticks2rot_ << (1.0/4096.0), (1.0/4096.0), (1.0/4096.0), (1.0/4096.0), (1.0/4096.0); // 
	rot2SI_ << 0.072, 0.072, 0.055, 2.0*3.14159, 2.0*3.14159; // 
	maxRPM << 122.5, 125.0, 125.0, 120.0, 110.0;
	maxTicks = 999;
	sec2min = 60.0;
    try
    {
		// Stablishing Serial communication connection
		// set serial port
		std::string port;
		ros::param::param<std::string>("~port",port,"/dev/ttyACM1");
		serialComm_.setPort(port);
		// set baudrate:
		int baudrate; ros::param::param<int>("~baudrate", baudrate, 230400);
		serialComm_.setBaudrate(baudrate);

		// Apparently makes the write function work
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		serialComm_.setTimeout(to);

		// open communication
		ROS_INFO_STREAM("I passed here");
		serialComm_.open();
		ROS_INFO_STREAM("Lowleveldriver: Serial Port communication stablished");
		// Initialize communication
		ROS_INFO_STREAM("I did not pass here");
		if(serialComm_.isOpen()) ROS_INFO_STREAM("Lowleveldriver: Serial Port initialized");
		else ROS_ERROR_STREAM("Lowleveldriver: Unable to initialize connection");
    }
    catch (serial::IOException& e)
    {
		ROS_ERROR_STREAM(e.what());
		throw std::runtime_error(DriverMsgError("Unable to open port"));
    }
    // Publisher to publish joint positions
    pub_joint_pos_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
	// get cycle frequency
	double freq; ros::param::param<double>("~freq", freq, 50.0);
    // Set communication in a loop with frequency freq Hz
    cycleTimer_ = nh_.createTimer(ros::Duration(1.0/freq), &Lowleveldriver::serial_communication_, this, false);
    // Set subcriber to receive joint velocities
    sub_ = nh_.subscribe<sensor_msgs::JointState>("/driver/command",10,&Lowleveldriver::commandVel_, this);
}

Lowleveldriver::~Lowleveldriver(void)
{
	ROS_INFO_STREAM("Lowleveldriver: Terminating class");
}

/**
 * @brief Function called with a fixed frequency to manage serial communication. It reads the serial message from arduino with motors positions and joint limits. It sends the motor velocities to the motors. It sends position or/and velocities to the end-effector.
**/
void Lowleveldriver::serial_communication_(const ros::TimerEvent& e)
{
	static bool firstDelta = true;
	//ROS_INFO_STREAM("here");
	if(serialComm_.available()){
		// Read serial as string
		arduino_output_ = serialComm_.readline(57);
		serialComm_.flushInput();
		// stream to split message
		std::vector<std::string> sensors;
		std::string sensor;
		for(std::stringstream s(arduino_output_); s >> sensor;) sensors.push_back(sensor);
		int input_size = sensors.size();
		if(input_size == SERIAL_MSG_SIZE){ // message size
			//ROS_INFO_STREAM(arduino_output_);
			// Read sensor/motor values
			for(int idx=0; idx<6; idx++) jointLimits_(idx) = atoi(sensors[idx].c_str());
			for(int idx=6; idx<11; idx++) motorPosInt_(idx-6) = atoi(sensors[idx].c_str());
			// Read previous velocity command
			prevCommand_ = sensors[11];
			if((!(motorPosInt_.array()==9999).any())){ // if 2st 3 joint limits active
				if(calibrated_){
					// compute position delta
					if(firstDelta) firstDelta = false;
					else deltaMotorPosInt_ = motorPosInt_ - prevMotorPosInt_ ;
					prevMotorPosInt_ = motorPosInt_;
					// count number of total rotations for first 3 motors
					for(int idx=0; idx<3; idx++){
						if(deltaMotorPosInt_(idx) < -MAX_MOTOR_ROT) fullRotationCounter_(idx)++;
						else if(deltaMotorPosInt_(idx) > MAX_MOTOR_ROT) fullRotationCounter_(idx)--;
					}
					// Convert motor positions (from ticks) - first 3 to meters and last 2 to radians
					motorPos_ = ((motorPosInt_ -  zeroMotorPosInt_).cast<double>()).cwiseProduct(ticks2rot_).cwiseProduct(rot2SI_) + (fullRotationCounter_.cast<double>().cwiseProduct(rot2SI_));
					// change sign of 2nd and 3rd joints
					motorPos_(1) *= -1; motorPos_(2) *= -1;
					// Publish messages:
					pubMsg_.position.clear();
					pubMsg_.header.stamp = ros::Time::now();
					for(int idx=0; idx<5; idx++) pubMsg_.position.push_back(motorPos_(idx));
					pubMsg_.position.push_back(0.0); // add extra degree of freedom
					pub_joint_pos_.publish(pubMsg_);
				}
				else{
					serialComm_.write("100n100n100n000n000n\n"); // move negative direction
					if((jointLimits_.head(3).array()==1).all()){ // if 2st 3 joint limits active
						//ROS_INFO_STREAM("got here!");
						serialComm_.write("000n000n000n000n000n\n"); // stop motors
						if(prevCommand_.compare("000n000n000n000n000n")){
							zeroMotorPosInt_ = motorPosInt_; // record zero position for 1st 3 motors
							zeroMotorPosInt_(3) = 2048;
							zeroMotorPosInt_(4) = 2048;
							calibrated_ = true;
						}
					}
				}
			}
			//else ROS_ERROR_STREAM("9999");
		}
	}
}

void Lowleveldriver::commandVel_(const sensor_msgs::JointState::ConstPtr& msg)
{
	// save commanded values
	for(int idx=0; idx<5; idx++) cmdVel_(idx) = msg->velocity[idx]; std::cout << std::endl;
	// Convert from velocities from m/s and rad/s to ticks fraction
	cmdVelInt_ = ((cmdVel_.cwiseQuotient(rot2SI_) * sec2min).cwiseQuotient(maxRPM) * maxTicks).cast<int>();
	// get absolute value
	cmdVelIntAbs_ = cmdVelInt_.cwiseAbs();
	// convert to string
	std::string output = "";
	std::string vel;
	for(int idx=0; idx<5; idx++){
		vel = std::to_string((cmdVelIntAbs_(idx)<999)?(cmdVelIntAbs_(idx)):(999));
		output += (ref_string.substr(0,3-vel.length()) + vel);
		if(cmdVelInt_(idx)>0) output += 'p';
		else output += 'n';
	}
	output += '\n';
	// send commanded vel through serial
	serialComm_.write(output);
	ROS_INFO_STREAM("---------------------------------------------");
	ROS_INFO_STREAM(motorPos_.transpose());
	ROS_INFO_STREAM(cmdVel_.transpose());
	ROS_INFO_STREAM(output);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "motors_driver"); // Initialize driver for the motors with
	ros::NodeHandle dummy_handle; // because of clash between throw exception and ROS handle
	try{
		// Initializing Lowleveldriver class
		Lowleveldriver driver;
		// Let ROS take over.
		ros::spin();
	}
	//catch(int msg){
	catch(std::exception &e){
		ROS_ERROR_STREAM(e.what());
		ROS_ERROR_STREAM("Motions: Terminating node!");
		return 1;
	}
    // Cleaning and returning
    return 0;
}

