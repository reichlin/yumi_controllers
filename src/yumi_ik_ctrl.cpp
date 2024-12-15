/* doing IK control;
 i.e. given a goal pose, a current joint & gripper configuration
 -- find desired joint velocities & gripper commands */

#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <signal.h>
//#include <boost/lexical_cast.hpp>

#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>

class YumiArmController {
public:

	void gripper_state_callback(const sensor_msgs::JointState & msg) {

		left_gripper_current = msg.position[0];
		right_gripper_current = msg.position[1];
		return;
	}

	void joint_state_callback(const sensor_msgs::JointState & msg) {
		if (arm_id_ == 1) {
			int arm_indices[7] = {0,2,12,4,6,8,10};
			for(int i = 0; i < 7; i++) {
				q_current(i) = msg.position[arm_indices[i]];
				dq_current[i] = msg.velocity[arm_indices[i]];
			}
		} else {
			int arm_indices[7] = {1,3,13,5,7,9,11};
			for(int i = 0; i < 7; i++) {
				q_current(i) = msg.position[arm_indices[i]];
				dq_current[i] = msg.velocity[arm_indices[i]];
			}
		}

		robot_ready = 1;
	}

	void desired_pose_callback(const geometry_msgs::PoseStamped &msg)
	{
		desired_geometry_msg = msg;
		return;
	}

//	double lowPassFilter(double current, double previous, double alpha) {
//    	return alpha * current + (1.0 - alpha) * previous;
//	}

	double limit_joint_velcmd(double cmd, int joint) {
		double limited_cmd = cmd;
		double joint_vel = arm_joint_velocity[joint];
		if((cmd - joint_vel) > JOINT_VELOCITY_LIMIT)
			limited_cmd = JOINT_VELOCITY_LIMIT;
		else if((cmd - joint_vel) < (-JOINT_VELOCITY_LIMIT))
			limited_cmd = -JOINT_VELOCITY_LIMIT;
		return limited_cmd;
	}

    // Constructor
	YumiArmController(ros::NodeHandle& nh, int arm_id) : nh_(nh), arm_id_(arm_id) {

		robot_ready = 0;

		q_current.resize(7);
		dq_current.resize(7);
		dq_cmd.resize(7);
		dq_pub.resize(7);

		joint_subscriber = nh_.subscribe("/yumi/joint_states", 1, &YumiArmController::joint_state_callback, this);
		gripper_subscriber = nh_.subscribe("/yumi/gripper_states",1, &YumiArmController::gripper_state_callback, this);


		// set joints velocity commands and gripper topics
		int urdf_order[7] = {1,2,7,3,4,5,6};
		std::string gripper_effort_topic_name;
		if (arm_id_ == 1) { // left arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_l/command";
				dq_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effort_topic_name = "/yumi/gripper_l_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effort_topic_name, 2);
		} else { // right arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_r/command";
				dq_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effort_topic_name = "/yumi/gripper_r_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effort_topic_name, 2);
		}

		nh_.getParam("JOINT_VELOCITY_LIMIT", JOINT_VELOCITY_LIMIT);
		nh_.getParam("VELOCITY_CONST", VELOCITY_CONST);
		nh_.getParam("ROTATION_CONST", ROTATION_CONST);
		nh_.getParam("MAX_POS_ERR", MAX_POS_ERR);
		nh_.getParam("MAX_ROT_ERR", MAX_ROT_ERR);
		nh_.getParam("ALPHA", ALPHA);

		// get wanted initial configuration of the joints
		std::vector<double> vect;
		if (arm_id_ == 1) {
			nh_.getParam("/initial_joint_position/left_arm", vect);
			init_joint_position.push_back(vect);
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
				ROS_ERROR("Error initializing right_arm_kdl_wrapper");

			arm_kdl_wrapper_sim.init("yumi_body", "yumi_link_7_l");

		} else {
			nh_.getParam("/initial_joint_position/right_arm", vect);
			init_joint_position.push_back(vect);
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
				ROS_ERROR("Error initializing right_arm_kdl_wrapper");

			arm_kdl_wrapper_sim.init("yumi_body", "yumi_link_7_r");

		}

//		arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);
//		arm_kdl_wrapper_sim.ik_solver_vel->setLambda(0.4);
//
//		std::cout << "yumi_node started" << std::endl;
//
//		// reset position arm
//		while(ros::ok()) {
//			ros::spinOnce();
//			if (robot_ready == 1) {
//				reset_arm();
//				arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
//				init_rotation = current_pose.M;
//				std::cout << "current_pose xyz: " << current_pose.p(0)<< " " <<current_pose.p(1)<< " "<<current_pose.p(2)<< " "<< std::endl;
//				std::cout << "arm resetted" << std::endl;
//
//				return;
//			}
//			ros::Duration(0.1).sleep();
//		}


    }