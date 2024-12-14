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
#include <boost/lexical_cast.hpp>
#include "planning/PickPlaceRequest.h"
#include "quest2ros/OVR2ROSInputs.h"
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <Eigen/Dense>
#include <cstdlib>
#include <ctime>

// global variables
/*
#define JOINT_VELOCITY_LIMIT 0.05
#define VELOCITY_CONST 2.0 //1.0
#define ROTATION_CONST 2.0 //1.0
#define MAX_POS_ERR 0.05
#define MAX_ROT_ERR 0.05
#define ALPHA 0.7
*/

// functions declarations


class YumiArmController {
public:

	void gripper_state_callback(const sensor_msgs::JointState & msg) {

		left_gripper = msg.position[0];
		right_gripper = msg.position[1];
		return;
	}

	void joint_state_callback(const sensor_msgs::JointState & msg) {
		if (arm_id_ == 1) {
			int arm_indecis[7] = {0,2,12,4,6,8,10};
			for(int i = 0; i < 7; i++) {
				arm_joint_positions(i) = msg.position[arm_indecis[i]];
				arm_joint_velocity[i] = msg.velocity[arm_indecis[i]];
			}
		} else {
			int arm_indecis[7] = {1,3,13,5,7,9,11};
			for(int i = 0; i < 7; i++) {
				arm_joint_positions(i) = msg.position[arm_indecis[i]];
				arm_joint_velocity[i] = msg.velocity[arm_indecis[i]];
			}
		}

		robot_ready = 1;
	}

	void tele_button_callback(const quest2ros::OVR2ROSInputs & msg)
	{
		press_middle = msg.press_middle;
		press_index = msg.press_index;
		button_x = msg.button_lower;
		button_y = msg.button_upper;
	}

	void desired_pose_callback(const geometry_msgs::PoseStamped &msg)
	{
		desired_geometry_msg = msg;
		return;
	}

	double lowPassFilter(double current, double previous, double alpha) {
    	return alpha * current + (1.0 - alpha) * previous;
	}

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

		flag_teleop=false;

		arm_joint_positions.resize(7);
		arm_joint_velocity.resize(7);
		arm_joint_velcmd.resize(7);
		velocity_command_pub.resize(7);

		joint_subscriber = nh_.subscribe("/yumi/joint_states", 1, &YumiArmController::joint_state_callback, this);
		gripper_subscriber = nh_.subscribe("/yumi/gripper_states",1, &YumiArmController::gripper_state_callback, this);
		desired_pose_subscriber = nh_.subscribe("/arms/pick_pose",1, &YumiArmController::desired_pose_callback, this);
		
		
		// set joints velocity commands and gripper topics
		int urdf_order[7] = {1,2,7,3,4,5,6};
		std::string gripper_effor_topic_name;
		if (arm_id_ == 1) { // left arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_l/command";
				velocity_command_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effor_topic_name = "/yumi/gripper_l_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effor_topic_name, 2);
			// teleop_subscriber = nh_.subscribe("/q2r_left_hand_twist",1, &YumiArmController::teleoperation_callback, this);
			// teleop_button_subscriber = nh_.subscribe("/q2r_left_hand_inputs",1, &YumiArmController::tele_button_callback, this);
			teleop_subscriber = nh_.subscribe("/q2r_right_hand_twist",1, &YumiArmController::teleoperation_callback, this);
			teleop_button_subscriber = nh_.subscribe("/q2r_right_hand_inputs",1, &YumiArmController::tele_button_callback, this);
		} else { // right arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_r/command";
				velocity_command_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effor_topic_name = "/yumi/gripper_r_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effor_topic_name, 2);
			// teleop_subscriber = nh_.subscribe("/q2r_right_hand_twist",1, &YumiArmController::teleoperation_callback, this);
			// // teleop_subscriber = nh_.subscribe("/q2r_left_hand_twist",1, &YumiArmController::teleoperation_callback, this);
			// teleop_button_subscriber = nh_.subscribe("/q2r_right_hand_inputs",1, &YumiArmController::tele_button_callback, this);

			teleop_subscriber = nh_.subscribe("/q2r_left_hand_twist",1, &YumiArmController::teleoperation_callback, this);
			teleop_button_subscriber = nh_.subscribe("/q2r_left_hand_inputs",1, &YumiArmController::tele_button_callback, this);

		}

		// std::string("left");

		pick_srv = nh_.advertiseService(std::string("/yumi_pick_place_srv")+std::to_string(arm_id_), &YumiArmController::pick_object_callback, this);
		// place_srv = nh_.advertiseService("/yumi_place_srv"+std::to_string(arm_id_), &YumiArmController::place_object_callback, this);
		teleop_srv = nh_.advertiseService("/yumi_teleop_srv"+std::to_string(arm_id_), &YumiArmController::teleop_srv_callback, this);
		// rotate_srv = nh_.advertiseService("/yumi_rotate_srv"+std::to_string(arm_id_), &YumiArmController::rotate_srv_callback, this);

		nh_.getParam("JOINT_VELOCITY_LIMIT", JOINT_VELOCITY_LIMIT);
		nh_.getParam("VELOCITY_CONST", VELOCITY_CONST);
		nh_.getParam("ROTATION_CONST", ROTATION_CONST);
		nh_.getParam("TELE_VELOCITY_CONST", TELE_VELOCITY_CONST);
		nh_.getParam("TELE_ROTATION_CONST", TELE_ROTATION_CONST);
		nh_.getParam("MAX_POS_ERR", MAX_POS_ERR);
		nh_.getParam("MAX_ROT_ERR", MAX_ROT_ERR);
		nh_.getParam("ALPHA", ALPHA);
		// get wanted initial configuration of the joints
		std::vector<double> vect;
		std::vector<double> vect2;
		if (arm_id_ == 1) {
			nh_.getParam("/initial_joint_position/left_arm", vect);
			//nh_.getParam("/place_joint_position/left_arm", vect);
			init_joint_position.push_back(vect);
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
				ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");

			arm_kdl_wrapper_sim.init("yumi_body", "yumi_link_7_l");
			nh_.getParam("/place_joint_position/left_arm", vect2);
			place_joint_position.push_back(vect2);

		} else {
			nh_.getParam("/initial_joint_position/right_arm", vect);
			//nh_.getParam("/place_joint_position/right_arm", vect);
			init_joint_position.push_back(vect);
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
				ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");

			arm_kdl_wrapper_sim.init("yumi_body", "yumi_link_7_r");
			nh_.getParam("/place_joint_position/right_arm", vect2);
			place_joint_position.push_back(vect2);

		}

		arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);
		arm_kdl_wrapper_sim.ik_solver_vel->setLambda(0.4);
		
		std::cout << "yumi_node started" << std::endl;

		// reset position arm
		while(ros::ok()) {
			ros::spinOnce();
			if (robot_ready == 1) {
				reset_arm();
				arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
				init_rotation = current_pose.M;
				std::cout << "current_pose xyz: " << current_pose.p(0)<< " " <<current_pose.p(1)<< " "<<current_pose.p(2)<< " "<< std::endl;
				std::cout << "arm resetted" << std::endl;
    
				return;
			}
			ros::Duration(0.1).sleep();
		}

		
    }

    // Destructor
    ~YumiArmController() {
        // reset_arm();
		cmd.data = 0;
		for (int i = 0; i < 7; i++)
			velocity_command_pub[i].publish(cmd);
    }

	void main_loop() {

		for (int i = 0; i < 5; i++) 
		{
			ros::spinOnce();
			ros::Duration(0.1).sleep();
		}

		/*
		int flag = 0;		
		while(ros::ok()) {
			ros::spinOnce();

			arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);

			double angle1 = 0.0;
			double angle2 = 0.0;
			double angle3 = 0.0;
			current_pose.M.GetRPY(angle1, angle2, angle3);

			std::cout << angle1 << " " << angle2 << " " << angle3 << std::endl;


			if (flag == 0) {
				for (int i = 0; i < 20; i++) {

					arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);

					double angle1 = 0.0;
					double angle2 = 0.0;
					double angle3 = 0.0;
					current_pose.M.GetRPY(angle1, angle2, angle3);

					ros::spinOnce();
					KDL::Twist desired_vel;
					desired_vel.vel = KDL::Vector(0.0, 0.0, 0.0);
					desired_vel.rot = KDL::Vector(0.0, 0.0, 0.1);
					arm_kdl_wrapper.ik_solver_vel->CartToJnt(arm_joint_positions, desired_vel, arm_joint_velcmd);

					//double prevCmd = 0.0;
					std::vector<double> prevCmd(7, 0.0);
					for(int i = 0; i < 7; i++)
					{
						cmd.data = VELOCITY_CONST * arm_joint_velcmd(i);
						cmd.data = limit_joint_velcmd(cmd.data, i);
						cmd.data = lowPassFilter(cmd.data, prevCmd[i], ALPHA);
						velocity_command_pub[i].publish(cmd);
						prevCmd[i] = cmd.data;
					}
					std::cout << angle1 << " " << angle2 << " " << angle3 << std::endl;
					ros::Duration(0.1).sleep();
				}
				flag = 1;
			}

			cmd.data = 0;
			for (int i = 0; i < 7; i++)
				velocity_command_pub[i].publish(cmd);


			ros::Duration(1.0).sleep();
		}
		*/

		//for (int i = 0; i < 5; i++) {
		
		// ros::spinOnce();
		// KDL::Frame desired_pose;
		// arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// // desired_pose.p = current_pose.p + KDL::Vector(0.25, 0.35, -0.1); //KDL::Vector(0.0, 0.0, 0.0);
		// double z = current_pose.p(2);
		// desired_pose.p = KDL::Vector(0.475, -0.160, z-0.05);
		// desired_pose.M = current_pose.M; //KDL::Rotation();
		// // go_to_position_waypoints(desired_pose, VELOCITY_CONST);
		// go_to_position_waypoints(desired_pose, VELOCITY_CONST);
		// pick();
		// close_gripper();
		// get_up();
		// open_gripper();
		//}

		ros::spin();

		// cmd.data = 0;
		// for (int i = 0; i < 7; i++)
		// 	velocity_command_pub[i].publish(cmd);
		

		
		
		// ros::spinOnce();
		// KDL::Frame desired_pose;
		// arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// desired_pose.p = current_pose.p + KDL::Vector(0.2, 0.4, 0.0); //KDL::Vector(0.0, 0.0, 0.0);
		// desired_pose.M = current_pose.M; //KDL::Rotation();
		// go_to_position(desired_pose, VELOCITY_CONST);
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// pick();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// close_gripper();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// get_up();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// desired_pose.p = current_pose.p + KDL::Vector(-0.1, 0.0, 0.0); //KDL::Vector(0.0, 0.0, 0.0);
		// desired_pose.M = current_pose.M; //KDL::Rotation();
		// go_to_position(desired_pose, VELOCITY_CONST);
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// open_gripper();
		// ros::Duration(1.0).sleep();
		// ros::spinOnce();
		// reset_arm();
		// ros::Duration(1.0).sleep();
	}
		
		

	void reset_arm() {
		
		// go to intial joint position
		bool all_fine = false;
		while(all_fine == false) {
			ros::spinOnce();
			all_fine = true;
			for (int i = 0; i < 7; i++) {
				cmd.data = 0.3*(init_joint_position[0][i]-arm_joint_positions(i));
				cmd.data = limit_joint_velcmd(cmd.data, i);
				velocity_command_pub[i].publish(cmd);
				if(std::abs(init_joint_position[0][i]-arm_joint_positions(i))>0.02)
					all_fine = false;
			}
			ros::Duration(0.1).sleep();
		}

		// reset 0 velocity to all joints
		cmd.data = 0;
		for (int i = 0; i < 7; i++)
			velocity_command_pub[i].publish(cmd);

		// open gripper (for the right arm this is a position not an effort)
		open_gripper();
		//gripper_cmd.data = 20.0;
		//gripper_command_pub.publish(gripper_cmd);
		//ros::Duration(0.1).sleep();

	}

	void open_gripper() {
		if (arm_id_ == 1) {  // left arm
			// for the left arm this is an effort: -20 max opening force, 20 max closing force
			gripper_cmd.data = -10.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
			gripper_cmd.data = 0.0;
			gripper_command_pub.publish(gripper_cmd);
		} else { // right arm
			// for the right arm this is a position: 0 closed, 20 open
			gripper_cmd.data = 20.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
		}
	}

	void close_gripper() {
		if (arm_id_ == 1) {  // left arm
			// for the left arm this is an effort: -20 max opening force, 20 max closing force
			gripper_cmd.data = 15.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
			// gripper_cmd.data = 0.0;
			// gripper_command_pub.publish(gripper_cmd);
		} else { // right arm
			// for the right arm this is a position: 0 closed, 20 open
			// gripper_cmd.data = 0.0;
			// gripper_command_pub.publish(gripper_cmd);
			// ros::Duration(1.0).sleep();
			bool closed = false;
			double current_gripper = right_gripper;
			gripper_cmd.data = 20.;
			while(!closed)
			{	
				current_gripper = right_gripper;
				gripper_cmd.data -= 4.;
				gripper_command_pub.publish(gripper_cmd);
				ros::Duration(1.).sleep();
				ros::spinOnce();

				std::cout << "closing gripper?" << current_gripper - right_gripper << std::endl;

				if(current_gripper - right_gripper<0.0003 || gripper_cmd.data<=4.)
				{
					closed=true;
				}

			}
			ros::Duration(1.).sleep();
		}
	}

	bool go_to_position(KDL::Frame desired_pose, double vel_constant, double max_pos_error=0.01, bool safety_check=false) {
		// ros::spinOnce();
		// arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// std::vector<KDL::Vector> waypoints = interpolateWaypoints(current_pose.p, desired_pose.p, 0.03);

		bool reached = false;
		int iter = 0;
		
		while (reached == false) {

			ros::spinOnce();

			arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
			

			KDL::Twist desired_vel;
			desired_vel.vel = desired_pose.p - current_pose.p;
			double angle1_des = 0.0;
			double angle2_des = 0.0;
			double angle3_des = 0.0;
			double angle1_cur = 0.0;
			double angle2_cur = 0.0;
			double angle3_cur = 0.0;
			desired_pose.M.GetRPY(angle1_des, angle2_des, angle3_des);
			current_pose.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);

			double angle1_diff = angle1_des - angle1_cur;
			double angle2_diff = angle2_des - angle2_cur;
			double angle3_diff = angle3_des - angle3_cur;
			angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
			angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
			angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;

			desired_vel.rot = ROTATION_CONST * KDL::Vector(angle1_diff, angle2_diff, angle3_diff);
			arm_kdl_wrapper_sim.ik_solver_vel->CartToJnt(arm_joint_positions, desired_vel, arm_joint_velcmd);

			//double prevCmd = 0.0;
			std::vector<double> prevCmd(7, 0.0);
			for(int i = 0; i < 7; i++)
			{
				cmd.data = vel_constant * arm_joint_velcmd(i);
				cmd.data = limit_joint_velcmd(cmd.data, i);
				cmd.data = lowPassFilter(cmd.data, prevCmd[i], ALPHA);
				velocity_command_pub[i].publish(cmd);
				prevCmd[i] = cmd.data;
			}

			// std::cout << "pos: " << current_pose.p(0) << " " << current_pose.p(1) << " " << current_pose.p(2) << std::endl;
			// std::cout << "err pos: " << std::abs(current_pose.p(0) - desired_pose.p(0)) << " " << std::abs(current_pose.p(1) - desired_pose.p(1)) << " " << std::abs(current_pose.p(2) - desired_pose.p(2)) << std::endl;
			// std::cout << "err rot: " << angle1_diff << " " << angle2_diff << " " << angle3_diff << std::endl;

			if (std::abs(current_pose.p(0) - desired_pose.p(0)) < max_pos_error &&
				std::abs(current_pose.p(1) - desired_pose.p(1)) < max_pos_error &&
				std::abs(current_pose.p(2) - desired_pose.p(2)) < max_pos_error &&
				angle1_diff < MAX_ROT_ERR && angle2_diff < MAX_ROT_ERR && angle3_diff < MAX_ROT_ERR) {
				cmd.data = 0;
				for (int i = 0; i < 7; i++)
					velocity_command_pub[i].publish(cmd);
				reached = true;
			}

			iter++;

			if (iter > 2000 && safety_check) {
				std::cout << "cannot reach next waypoint!" << std::endl;
				reset_arm();
				return false;
			}


			ros::Duration(0.01).sleep();

		}

		return true;
	}

	
	std::vector<KDL::Vector> interpolateWaypoints(KDL::Vector& start, KDL::Vector& end, double min_step_size) 
	{
		std::vector<KDL::Vector> waypoints;

		// Calculate the distance between start and end positions
		double distance = (end - start).Normalize(1e-5);

		// Calculate the number of waypoints based on the minimum step size
		int numWaypoints = static_cast<int>(distance / min_step_size) + 1;

		//std::cout<<"numWaypoints: "<<numWaypoints<<std::endl;

		// Generate waypoints
		for (int i = 0; i < numWaypoints; ++i) {
			double ratio = static_cast<double>(i) / (numWaypoints - 1); // Interpolation ratio from 0 to 1
			KDL::Vector interpolatedPoint = start + ratio * (end - start);
			waypoints.push_back(interpolatedPoint);

			// std::cout<<"ratio: "<<ratio<<std::endl;
			// std::cout<<"x: "<<interpolatedPoint.x()<<" y: "<<interpolatedPoint.y()<<" z: "<<interpolatedPoint.z()<<std::endl;
		}
	
		return waypoints;
	}
	
	bool go_to_position_waypoints(KDL::Frame desired_pose, double vel_constant, double min_step_size=0.1) 
	{

		ros::spinOnce();
		int success = 0;

		arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);

		std::vector<KDL::Vector> waypoints = interpolateWaypoints(current_pose.p, desired_pose.p, min_step_size);

		int i=0;
		for (const auto& wp : waypoints)
		{
			//std::cout<<"x: "<<wp.x()<<" y: "<<wp.y()<<" z: "<<wp.z()<<std::endl;

			if (i == waypoints.size() - 1) {
				success = go_to_position(KDL::Frame(current_pose.M , wp),VELOCITY_CONST, MAX_POS_ERR, true);
				//success = go_to_position(KDL::Frame(desired_pose.M , wp),VELOCITY_CONST, MAX_POS_ERR, true);
				auto err = current_pose.p - desired_pose.p;
				//std::cout<<"final err x: "<<err(0)<<" y: "<<err(1)<<" z: "<<err(2)<<std::endl;
			}
			else{
				success = go_to_position(KDL::Frame(current_pose.M , wp),VELOCITY_CONST, 0.01, true);
				//success = go_to_position(KDL::Frame(current_pose.M , wp),VELOCITY_CONST, 0.01, true);
			}

			if (!success) // failure in going to the next waypoint
				return false;

			
			++i;
		}

		return true;
	}

	void pick() {

		KDL::Frame desired_pose;
		desired_pose.p = current_pose.p;
		desired_pose.p(2) = 0.202;
		desired_pose.M = current_pose.M;

		go_to_position(desired_pose, 0.5, 0.01, false);

		return;
	}

	void get_up() {

		KDL::Frame desired_pose;
		desired_pose.p = current_pose.p;
		desired_pose.p(2) = 0.42;
		desired_pose.M = current_pose.M;

		go_to_position(desired_pose, 0.5, 0.01, false);

		return;
	}

	bool pick_object(KDL::Frame& desired_pose)
	{
	
		// desired_pose.p = KDL::Vector(desired_pose.p(0), desired_pose.p(1), z-0.05);
		// desired_pose.M = current_pose.M; //KDL::Rotation();
		// go_to_position_waypoints(desired_pose, VELOCITY_CONST);
		bool success;
		success = go_to_position_waypoints(desired_pose, VELOCITY_CONST);
		if (success)
		{
			pick();
			std::cout << "got here" << std::endl;
			close_gripper();
			get_up();
		}
		return success;

	}

	void place_object()
	{
		ros::spinOnce();
		arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		
		bool all_fine = false;
		while(all_fine == false) {
			ros::spinOnce();
			all_fine = true;
			for (int i = 0; i < 7; i++) {
				cmd.data = 0.3*(place_joint_position[0][i]-arm_joint_positions(i));
				cmd.data = limit_joint_velcmd(cmd.data, i);
				velocity_command_pub[i].publish(cmd);
				if(std::abs(place_joint_position[0][i]-arm_joint_positions(i))>0.02)
					all_fine = false;
			}
			ros::Duration(0.1).sleep();
		}

		// reset 0 velocity to all joints
		cmd.data = 0;
		for (int i = 0; i < 7; i++)
			velocity_command_pub[i].publish(cmd);

		open_gripper();
		ros::Duration(0.5).sleep();
		reset_arm();

	}

	// Katharina code ##################################################################

	 double calc_cond_for_q(KDL::JntArray current_q) {

        KDL::Jacobian jac;
		jac.resize(current_q.rows());

        arm_kdl_wrapper.jnt_jac_solver->JntToJac(current_q, jac);

        // svd process
        Eigen::MatrixXd jac_eigen(6, jac.columns()); // confirm kdl jac is 6 rows
        for (unsigned int i = 0; i < jac.rows(); ++i) {
            for (unsigned int j = 0; j < jac.columns(); ++j) {
                jac_eigen(i, j) = jac(i, j);
            }
        }


        // Calculate the condition number using the singular value decomposition (SVD)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac_eigen);

 
        double epsilon = std::numeric_limits<double>::epsilon(); 
        double sigma_min = svd.singularValues()(svd.singularValues().size() - 1);
        double cond = std::numeric_limits<double>::max();

        if (std::abs(sigma_min) > epsilon) {
             cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1); 
        }

 
        return cond;
    }
 
    // function needs: parametrized discretized curve (array of waypoints) and current pose
    // but trajectory is like std::vector<KDL::Vector> waypoints;
    double get_cond_from_rollout(KDL::JntArray q_initial, std::vector<KDL::Frame> trajectory) { // first val traj = t+1 C

		int max_iter = 10000;
		int iter=0;
        KDL::Frame pose_t; //pose_t is current pose, obtained from fk(q_t)
        KDL::JntArray q_t;
        KDL::JntArray dq_t;
 
        double max_pos_error = 0.01;
        double stepsize = 0.01; //1e-2;
        KDL::Frame p_target;
        KDL::Twist vel_des;
 
        double max_cond = 1.0;
 
        q_t = q_initial;
 
        arm_kdl_wrapper.fk_solver_pos->JntToCart(q_t, pose_t, -1); 
 
        for (int i = 0; i < trajectory.size(); i++) {
 
            p_target = trajectory[i];
            
            bool pose_reached = false;
			iter = 0;
            while(pose_reached == false && iter<max_iter) {
 
 
                double angle1_des = 0.0;
                double angle2_des = 0.0;
                double angle3_des = 0.0;
                double angle1_cur = 0.0;
                double angle2_cur = 0.0;
                double angle3_cur = 0.0;
                p_target.M.GetRPY(angle1_des, angle2_des, angle3_des);
                pose_t.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);
        
                double angle1_diff = angle1_des - angle1_cur;
                double angle2_diff = angle2_des - angle2_cur;
                double angle3_diff = angle3_des - angle3_cur;
                angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
                angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
                angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;

 
                if (std::abs(pose_t.p(0) - p_target.p(0)) < max_pos_error &&
                    std::abs(pose_t.p(1) - p_target.p(1)) < max_pos_error &&
                    std::abs(pose_t.p(2) - p_target.p(2)) < max_pos_error &&
                    angle1_diff < MAX_ROT_ERR && angle2_diff < MAX_ROT_ERR && angle3_diff < MAX_ROT_ERR) {
    
                    pose_reached = true;
					// std::cout << "broke out bc waypoint was reached at iter" << iter << std::endl;
                    break;
                }
                else {
                    vel_des.vel = p_target.p - pose_t.p;
                    vel_des.rot = ROTATION_CONST * KDL::Vector(angle1_diff, angle2_diff, angle3_diff);
                    arm_kdl_wrapper.ik_solver_vel->CartToJnt(q_t, vel_des, dq_t);
                }
 
 
                // 1) assuming we start with zero velocity and velocity controllers are ideal 2)low pass?
                for(int i = 0; i < 7; i++)
                {
                    dq_t(i) = limit_joint_velcmd(dq_t(i), i); 
                }
 
                for(int i = 0; i < 7; i++) {
                    q_t(i) += stepsize * dq_t(i); 
                }
                arm_kdl_wrapper.fk_solver_pos->JntToCart(q_t, pose_t, -1);

				// if (iter % 100 == 0){
				// 	std::cout << "x: " << pose_t.p(0) << " y: " << pose_t.p(1) << " z: " << pose_t.p(2) << std::endl;
				// }
				iter++;

					
            }
 
            // get cond(jac) for this point
            double current_cond = calc_cond_for_q(q_t);
 
            // debug conds
            // std::cout << current_cond << std::endl;
 
            if (max_cond < current_cond)
                max_cond = current_cond;
            
        }
 
        return max_cond; 
    }
 
    void test_V() {
 
        // TODO: get from initial joint configuration q0_init, which is double.
        // from std::vector<std::vector<double>> init_joint_position;
 
        KDL::JntArray q0 = arm_joint_positions;
		
 
        KDL::Frame x1, x2, x3;
        arm_kdl_wrapper.fk_solver_pos->JntToCart(q0, x1, -1); 
		std::cout << "x: " << x1.p(0) << " y: " << x1.p(1) << " z: " << x1.p(2) << std::endl;
        x2.p = x1.p + KDL::Vector(0, 0, 0);
        x3.p = x2.p + KDL::Vector(0.0, 0.1, 0.1);
 
 
        // std::vector<KDL::Frame> trajectory = {x1, x2, x3};
		std::cout << "started generating optim traj" << std::endl;
		std::vector<KDL::Frame> trajectory = generate_optim_traj(q0, x3, 0.05, 0);
        double v = get_cond_from_rollout(q0, trajectory);
        // std::cout << v << std::endl;


		// next step of test: test with a generated trajectorz!

 
    }
 
	////////////// UP TILL HERE; DEBUGGED; WE GET THE MAX COND FOR A TRAJ
    /////////////////////////// segment bezier curves crap
 
    int binomialCoefficient(int n, int k) {
        if (k == 0 || k == n)
            return 1;
        return binomialCoefficient(n - 1, k - 1) + binomialCoefficient(n - 1, k);
    }
 
    std::vector<KDL::Vector> generateAndSegmentBezierPath3D(const KDL::Vector start, const KDL::Vector end, int numControlPoints, int numWaypoints, const KDL::Vector workspaceMin, const KDL::Vector workspaceMax) {
		std::vector<KDL::Vector> controlPoints(numControlPoints);
		srand(time(0));
		for (int i = 0; i < numControlPoints; ++i) {
			controlPoints[i].x(rand() / (RAND_MAX + 1.0) * (workspaceMax.x() - workspaceMin.x()) + workspaceMin.x());
			controlPoints[i].y(rand() / (RAND_MAX + 1.0) * (workspaceMax.y() - workspaceMin.y()) + workspaceMin.y());
			controlPoints[i].z(rand() / (RAND_MAX + 1.0) * (workspaceMax.z() - workspaceMin.z()) + workspaceMin.z());
		}

		int numPoints = numControlPoints + 2;
		std::vector<KDL::Vector> allPoints(numPoints);
		allPoints[0] = start;
		for (int i = 0; i < numControlPoints; ++i)
			allPoints[i + 1] = controlPoints[i];
		allPoints[numControlPoints + 1] = end;

		double totalLength = 0.0;
		for (int i = 1; i < numPoints; ++i)
			totalLength += (allPoints[i] - allPoints[i - 1]).Norm();

		// std::cout << totalLength << std::endl;
		// std::cout << numWaypoints << std::endl;
		double segmentLength = totalLength / (numWaypoints - 1);

		std::vector<KDL::Vector> path(numWaypoints);
		path[0] = start;
		int currentWaypoint = 1;
		double distanceTravelled = 0.0;

		int iter = 0;
		int max_iter = 3;

		for (int i = 1; i < numPoints; ++i) {
			KDL::Vector direction = allPoints[i] - allPoints[i - 1];
			double distanceToNextPoint = direction.Norm();
			while (distanceToNextPoint + distanceTravelled >= segmentLength) {
				double ratio = (segmentLength - distanceTravelled) / distanceToNextPoint; //derivative vector
				// std::cout << "ratio: " << ratio << std::endl;
				std::cout << "before path: " << path[currentWaypoint].y() << std::endl;
				// std::cout << "path0: " << path[0].x() << std::endl;
				// path[currentWaypoint] = allPoints[i - 1] + direction * ratio;
				path[currentWaypoint] = path[currentWaypoint-1] + direction * ratio;
				// std::cout << "after path: " << path[currentWaypoint].y() << std::endl;
				distanceTravelled = 0.0;
				currentWaypoint++;
				// for (int j = 0; j < path.size(); j++)
				// 	std::cout << path[j].y() << std::endl;
				// std::cout << "currentWaypoint: " << currentWaypoint << std::endl;
				direction = allPoints[i] - path[currentWaypoint - 1];
				
				distanceToNextPoint = direction.Norm();
				iter++;
			}
			// std::cout << "got out of while" << std::endl;
			distanceTravelled += distanceToNextPoint;
			// std::cout << path[i].x();
		}
		// std::cout << std::endl;
		path[numWaypoints - 1] = end;
		std::cout << "made it through traj gen" << std::endl;
		return path;
	}
	
    //returns points!
    std::vector<KDL::Vector> create_trajectory_points(KDL::Vector p_start, KDL::Vector p_end, double stepsize, int n_controlpoints) {
        std::vector<KDL::Vector> trajectory;
        int n_waypoints;
 
        double distance = (p_end - p_start).Normalize(1e-5);
        n_waypoints = static_cast<int>(distance / stepsize) + 1;
 
        // TODO: to set and think
        double fixedSegmentNorm3D = 3.0;
        KDL::Vector workspaceLimitsMin(0, 0, 0);
        KDL::Vector workspaceLimitsMax(10, 10, 10);
 
 
        trajectory = generateAndSegmentBezierPath3D(p_start, p_end, n_controlpoints, n_waypoints, workspaceLimitsMin, workspaceLimitsMax);
		std::cout << "made it through traj gen p2" << std::endl;
        return trajectory;
    }
 
    //returns poses!
    std::vector<KDL::Frame>  generate_optim_traj(KDL::JntArray q0, KDL::Frame target_pose, double stepsize=0.01, int n_controlpoints=0) {

        // save optimization strategy for later

        double v_current;
        KDL::Frame pose_current; 
        arm_kdl_wrapper.fk_solver_pos->JntToCart(q0, pose_current, -1); 

		std::cout << "started creating traj points" << std::endl;

        std::vector<KDL::Vector> trajectory_points = create_trajectory_points(pose_current.p, target_pose.p, stepsize, n_controlpoints);
		std::cout<<trajectory_points.size()<<std::endl;

        std::vector<KDL::Frame> trajectory_poses;
		trajectory_poses.resize(trajectory_points.size());

        for (int i=0; i<trajectory_points.size(); i++) {
			// std::cout<<"x: "<<trajectory_points[i].x()<<" y: "<<trajectory_points[i].y()<<" z: "<<trajectory_points[i].z();
            trajectory_poses[i] = KDL::Frame(pose_current.M , KDL::Vector(trajectory_points[i].x(), trajectory_points[i].y(), trajectory_points[i].z()));
        }

		std::cout<<std::endl;
        // put in for loop to compare
        v_current = get_cond_from_rollout(q0, trajectory_poses);
        std::cout << v_current << std::endl;
 
        return trajectory_poses;
 
    }

    // // TODO: ANGLE COMMAND LOGIC!!!!
    // void follow_trajectory(std::vector<KDL::Frame> trajectory, double vel_constant) 
    // {
    //     KDL::Frame desired_pose = trajectory[trajectory.size()-1];
 
    //     ros::spinOnce(); //??
    //     arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
 
    //     // potentially do linear interpolation in between, depending on distance
    //     int i=0;
    //     for (const auto& wp : trajectory)
    //     {
    //         std::cout<<"x: "<<wp.p.x()<<" y: "<<wp.p.y()<<" z: "<<wp.p.z()<<std::endl;
 
    //         if (i == trajectory.size() - 1) {
    //             go_to_position(wp,VELOCITY_CONST, MAX_POS_ERR);
    //             auto err = current_pose.p - desired_pose.p;
    //             std::cout<<"final err x: "<<err(0)<<" y: "<<err(1)<<" z: "<<err(2)<<std::endl;
    //         }
    //         else{
    //             go_to_position(wp,VELOCITY_CONST, 0.01);
    //         }
 
    //         ++i;
    //     }
 
 
    // }

	// #####################################################

	bool pick_object_callback(planning::PickPlaceRequest::Request  &req,
			planning::PickPlaceRequest::Response &res)
	{
		//    res.success = req.a + req.b;
		//    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
		//    ROS_INFO("sending back response: [%ld]", (long int)res.sum);

		// KDL::Frame desired_pose;
		// desired_pose.p = current_pose.p + KDL::Vector(0.25, 0.35, -0.1); //KDL::Vector(0.0, 0.0, 0.0);
		// double z = current_pose.p(2);
		// desired_pose.M = current_pose.M; //KDL::Rotation();
		// desired_pose.M = init_rotation;
		// desired_pose.p = current_pose.p + KDL::Vector(0.11, -0.15, -0.05); //KDL::Vector(0.0, 0.0, 0.0);

		bool success;
		
		ros::spinOnce();
		arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);
		// std::cout<<desired_geometry_msg.pose.position.x<<" "<<desired_geometry_msg.pose.position.y<<" "<<desired_geometry_msg.pose.position.z<<std::endl;



		double z = current_pose.p(2);
		KDL::Vector desired_position(desired_geometry_msg.pose.position.x, desired_geometry_msg.pose.position.y, z-0.03);
		// // KDL::Rotation curr_rotation = current_pose.M;
		KDL::Rotation curr_rotation = init_rotation;
		// curr_rotation.DoRotZ(desired_geometry_msg.pose.orientation.w);

		KDL::Rotation desired_rotation = curr_rotation;
		KDL::Frame desired_pose(desired_rotation, desired_position);

		success = pick_object(desired_pose);
		if (!success) {
			std::cout << "killing myself bro!! bye bye" << std::endl;
			reset_arm();
			res.success=false;
			return true;
		}
		place_object();
		res.success=true;
		return true;
	}

	bool place_object_callback(planning::PickPlaceRequest::Request  &req,
			planning::PickPlaceRequest::Response &res)
	{
		//    res.success = req.a + req.b;
		//    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
		//    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
		
		return true;
	}

	bool teleop_srv_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
	{
		flag_teleop=true;
		while (ros::ok())
		{
			ros::spinOnce();
			if (button_x || button_y)
			{	
				flag_teleop=false;
				res.success = button_x ? true : false;
				reset_arm();
				break;
			}
		}
		return true;
	}
	

	void teleoperation_callback(const geometry_msgs::Twist& vel_user_msg)
	{	
		arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, current_pose, -1);

		if (!flag_teleop)
			return;

		if (press_index<0.8) {
			cmd.data = 0;
			for (int i = 0; i < 7; i++)
				velocity_command_pub[i].publish(cmd);
			return;
		}

		std::cout<<"tele controlling!!"<<std::endl;

		// double vel_constant = 0.25*VELOCITY_CONST;
		KDL::Twist kdl_twist;

		kdl_twist.vel.x(vel_user_msg.linear.x);
		kdl_twist.vel.y(vel_user_msg.linear.y);
		kdl_twist.vel.z(vel_user_msg.linear.z);

		// Keep the rotation constant
		double angle1_des = 0.0;
		double angle2_des = 0.0;
		double angle3_des = 0.0;
		double angle1_cur = 0.0;
		double angle2_cur = 0.0;
		double angle3_cur = 0.0;
		init_rotation.GetRPY(angle1_des, angle2_des, angle3_des);
		current_pose.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);

		// std::cout<<" init rpy: "<<angle1_des<<" "<<angle2_des<<" "<<angle3_des<<" "<< " current rpy: "<<angle1_cur<<" "<<angle2_cur<<" "<<angle3_cur<<std::endl;

		double angle1_diff = angle1_des - angle1_cur;
		double angle2_diff = angle2_des - angle2_cur;
		double angle3_diff = angle3_des - angle3_cur;
		angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
		angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
		angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;
		
		// Set angular components
		kdl_twist.rot.x(TELE_ROTATION_CONST* angle1_diff);
		kdl_twist.rot.y(TELE_ROTATION_CONST* angle2_diff);
		kdl_twist.rot.z(TELE_ROTATION_CONST* angle3_diff);

		arm_kdl_wrapper.ik_solver_vel->CartToJnt(arm_joint_positions, kdl_twist, arm_joint_velcmd);

		std::vector<double> prevCmd(7, 0.0);
		for(int i = 0; i < 7; i++)
		{
			cmd.data = TELE_VELOCITY_CONST * arm_joint_velcmd(i);
			cmd.data = limit_joint_velcmd(cmd.data, i);
			// cmd.data = lowPassFilter(cmd.data, prevCmd[i], ALPHA);
			velocity_command_pub[i].publish(cmd);
			prevCmd[i] = cmd.data;
		}
		
		if (arm_id_==2)
		{
			double command = (1-press_middle)*20;
			gripper_cmd.data = command<4. ? 4. : command;
			gripper_command_pub.publish(gripper_cmd);
		}
		else
		{
			std::cout << "close gripper: " << press_middle << std::endl;
			// gripper_cmd.data = 15.0;
			if (press_middle<0.5)
			{
				if (press_middle<0.01) {gripper_cmd.data=0.;}
				else {gripper_cmd.data=-20;}
			}
			else
			{
				gripper_cmd.data=15;
			}
			gripper_command_pub.publish(gripper_cmd);
		}

		// go_to_position(KDL::Frame(init_rotation, current_pose.p), vel_constant);

		// ros::Duration(0.1).sleep();

		// for (int i = 0; i < 7; i++)
		// 	velocity_command_pub[i].publish(cmd);
		


	}



private:

	ros::NodeHandle nh_;
	int arm_id_;

	KDL::JntArray arm_joint_positions;
	std::vector<double> arm_joint_velocity;
	KDLWrapper arm_kdl_wrapper;
	KDLWrapper arm_kdl_wrapper_sim;
	KDL::Twist arm_cart_velocity;
	KDL::JntArray arm_joint_velcmd;
	KDL::Frame current_pose;
	KDL::Rotation init_rotation;

	ros::Subscriber joint_subscriber;
	ros::Subscriber gripper_subscriber;
	ros::Subscriber teleop_subscriber;
	ros::Subscriber teleop_button_subscriber;
	ros::Subscriber desired_pose_subscriber;
	std::vector<ros::Publisher> velocity_command_pub;
	ros::Publisher gripper_command_pub;

	ros::ServiceServer pick_srv;
	ros::ServiceServer place_srv;
	ros::ServiceServer teleop_srv;

	std_msgs::Float64 cmd;
	std_msgs::Float64 gripper_cmd;
	std::string command_topic;
	int robot_ready;

	geometry_msgs::PoseStamped desired_geometry_msg;

	// constants
	std::vector<std::vector<double>> init_joint_position;
	std::vector<std::vector<double>> place_joint_position;

	double left_gripper;
	double right_gripper;
	double JOINT_VELOCITY_LIMIT;
	double VELOCITY_CONST;
	double ROTATION_CONST;
	double TELE_VELOCITY_CONST;
	double TELE_ROTATION_CONST;
	double MAX_POS_ERR;
	double MAX_ROT_ERR;
	double ALPHA;

	bool flag_teleop=false;
	double press_middle=0;
	double press_index=0;
	bool button_x=false;
	bool button_y=false;

	};

int main(int argc, char** argv) {

	int arm_id = std::stoi(argv[1]); // using the same convention of KTH code. 1: left arm, 2: right arm

    ros::init(argc, argv, std::string("yumi_arm_controller_arm_")+std::string(argv[1]));

    ros::NodeHandle nh;

    YumiArmController arm_controller(nh, arm_id);

    //ros::spin();

	arm_controller.main_loop();

	// arm_controller.test_V();

    return 0;
}






















