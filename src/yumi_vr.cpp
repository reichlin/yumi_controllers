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


class YumiVR {
public:

    // Constructor
	YumiVR(ros::NodeHandle& nh, int arm_id) : nh_(nh), arm_id_(arm_id) {

		
		if (arm_id_ == 1) { // left arm
			teleop_subscriber = nh_.subscribe("/q2r_right_hand_twist",1, &YumiArmController::teleoperation_callback, this);
			teleop_button_subscriber = nh_.subscribe("/q2r_right_hand_inputs",1, &YumiArmController::tele_button_callback, this);
		} else { // right arm
			teleop_subscriber = nh_.subscribe("/q2r_left_hand_twist",1, &YumiArmController::teleoperation_callback, this);
			teleop_button_subscriber = nh_.subscribe("/q2r_left_hand_inputs",1, &YumiArmController::tele_button_callback, this);
		}

		nh_.getParam("JOINT_VELOCITY_LIMIT", JOINT_VELOCITY_LIMIT);
		nh_.getParam("VELOCITY_CONST", VELOCITY_CONST);
		nh_.getParam("ROTATION_CONST", ROTATION_CONST);
		nh_.getParam("TELE_VELOCITY_CONST", TELE_VELOCITY_CONST);
		nh_.getParam("TELE_ROTATION_CONST", TELE_ROTATION_CONST);
		nh_.getParam("MAX_POS_ERR", MAX_POS_ERR);
		nh_.getParam("MAX_ROT_ERR", MAX_ROT_ERR);
		nh_.getParam("ALPHA", ALPHA);
		
		std::cout << "vr_node started" << std::endl;
		
    }

    // Destructor
    ~YumiVR() {
		// TODO: set twist to all zeros?
        return;
    }

	void main_loop() {
		ros::spin();
	}

	void tele_button_callback(const quest2ros::OVR2ROSInputs & msg)
	{
		press_middle = msg.press_middle;
		press_index = msg.press_index;
		button_x = msg.button_lower;
		button_y = msg.button_upper;
	}


	void teleoperation_callback(const geometry_msgs::Twist& vel_user_msg)
	{	
		KDL::Twist kdl_twist;

		if (press_index<0.8) {

			kdl_twist.vel.x(0.0);
			kdl_twist.vel.y(0.0);
			kdl_twist.vel.z(0.0);
			kdl_twist.rot.x(0.0);
			kdl_twist.rot.y(0.0);
			kdl_twist.rot.z(0.0);

			// TODO: publish message

			return;
		}

		kdl_twist.vel.x(vel_user_msg.linear.x);
		kdl_twist.vel.y(vel_user_msg.linear.y);
		kdl_twist.vel.z(vel_user_msg.linear.z);

		
		
		// TODO: rotations?




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


	std_msgs::Float64 cmd;
	std_msgs::Float64 gripper_cmd;



	KDL::JntArray arm_joint_positions;
	std::vector<double> arm_joint_velocity;
	KDLWrapper arm_kdl_wrapper;
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

	std_msgs::Float64 cmd;
	std_msgs::Float64 gripper_cmd;
	std::string command_topic;
	int robot_ready;

	geometry_msgs::PoseStamped desired_geometry_msg;

	// constants
	std::vector<std::vector<double>> init_joint_position;
	std::vector<std::vector<double>> place_joint_position;

	double TELE_VELOCITY_CONST;
	double TELE_ROTATION_CONST;

	bool flag_teleop=false;
	double press_middle=0;
	double press_index=0;
	bool button_x=false;
	bool button_y=false;

	};
	


int main(int argc, char** argv) {

	int arm_id = std::stoi(argv[1]); // using the same convention of KTH code. 1: left arm, 2: right arm

    ros::init(argc, argv, std::string("yumi_vr_arm_")+std::string(argv[1]));

    ros::NodeHandle nh;

    YumiVR yumi_vr(nh, arm_id);

    ros::spin();

	yumi_vr.main_loop();

    return 0;
}






















