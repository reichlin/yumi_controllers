#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
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


class YumiVR {
public:

    // Constructor
	YumiVR(ros::NodeHandle& nh, int arm_id) : nh_(nh), arm_id_(arm_id) {

		
		if (arm_id_ == 1) { // left arm
			teleop_subscriber = nh_.subscribe("/q2r_right_hand_twist",1, &YumiVR::teleoperation_callback, this);
			teleop_button_subscriber = nh_.subscribe("/q2r_right_hand_inputs",1, &YumiVR::tele_button_callback, this);
			des_vel_pub = nh_.advertise<geometry_msgs::Twist>("/yumi/end_effector_vel_cmd_r", 2);
			des_grip_pub = nh_.advertise<std_msgs::Int32>("/yumi/grip_cmd_r", 2);
		} else { // right arm
			teleop_subscriber = nh_.subscribe("/q2r_left_hand_twist",1, &YumiVR::teleoperation_callback, this);
			teleop_button_subscriber = nh_.subscribe("/q2r_left_hand_inputs",1, &YumiVR::tele_button_callback, this);
			des_vel_pub = nh_.advertise<geometry_msgs::Twist>("/yumi/end_effector_vel_cmd_l", 2);
			des_grip_pub = nh_.advertise<std_msgs::Int32>("/yumi/grip_cmd_l", 2);
		}

		nh_.getParam("TELE_VELOCITY_CONST", TELE_VELOCITY_CONST);
		nh_.getParam("TELE_ROTATION_CONST", TELE_ROTATION_CONST);
		
		std::cout << "vr_node started" << std::endl;
		
    }

    // Destructor
    ~YumiVR() {
		set_vel_to_zero();
        return;
    }

	void set_vel_to_zero() {
		desired_vel_msg.linear.x = 0.0;
		desired_vel_msg.linear.y = 0.0;
		desired_vel_msg.linear.z = 0.0;
		desired_vel_msg.angular.x = 0.0;
		desired_vel_msg.angular.y = 0.0;
		desired_vel_msg.angular.z = 0.0;
	}

	void main_loop() {

		ros::Rate loop_rate(50);


		while(ros::ok()) {

			ros::spinOnce();

			des_vel_pub.publish(desired_vel_msg);

			if (press_middle < 0.5)
				desired_gripper_msg.data = 1;
			else
				desired_gripper_msg.data = 0;

			des_grip_pub.publish(desired_gripper_msg);

			loop_rate.sleep();
		}
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
		// KDL::Twist kdl_twist;
		

		if (press_index<0.8) {

			set_vel_to_zero();
			return;
		}

		desired_vel_msg.linear.x = TELE_VELOCITY_CONST * vel_user_msg.linear.x;
		desired_vel_msg.linear.y = TELE_VELOCITY_CONST * vel_user_msg.linear.y;
		desired_vel_msg.linear.z = TELE_VELOCITY_CONST * vel_user_msg.linear.z;
		desired_vel_msg.angular.x = TELE_ROTATION_CONST * vel_user_msg.angular.x;
		desired_vel_msg.angular.y = TELE_ROTATION_CONST * vel_user_msg.angular.y;
		desired_vel_msg.angular.z = TELE_ROTATION_CONST * vel_user_msg.angular.z;

		
		
		// // TODO: rotations?




		// // Keep the rotation constant 
		// double angle1_des = 0.0;
		// double angle2_des = 0.0;
		// double angle3_des = 0.0;
		// double angle1_cur = 0.0;
		// double angle2_cur = 0.0;
		// double angle3_cur = 0.0;
		// init_rotation.GetRPY(angle1_des, angle2_des, angle3_des);
		// current_pose.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);

		// // std::cout<<" init rpy: "<<angle1_des<<" "<<angle2_des<<" "<<angle3_des<<" "<< " current rpy: "<<angle1_cur<<" "<<angle2_cur<<" "<<angle3_cur<<std::endl;

		// double angle1_diff = angle1_des - angle1_cur;
		// double angle2_diff = angle2_des - angle2_cur;
		// double angle3_diff = angle3_des - angle3_cur;
		// angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
		// angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
		// angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;
		
		// // Set angular components
		// kdl_twist.rot.x(TELE_ROTATION_CONST* angle1_diff);
		// kdl_twist.rot.y(TELE_ROTATION_CONST* angle2_diff);
		// kdl_twist.rot.z(TELE_ROTATION_CONST* angle3_diff);

		// arm_kdl_wrapper.ik_solver_vel->CartToJnt(arm_joint_positions, kdl_twist, arm_joint_velcmd);

		// std::vector<double> prevCmd(7, 0.0);
		// for(int i = 0; i < 7; i++)
		// {
		// 	cmd.data = TELE_VELOCITY_CONST * arm_joint_velcmd(i);
		// 	cmd.data = limit_joint_velcmd(cmd.data, i);
		// 	// cmd.data = lowPassFilter(cmd.data, prevCmd[i], ALPHA);
		// 	velocity_command_pub[i].publish(cmd);
		// 	prevCmd[i] = cmd.data;
		// }
		
		// if (arm_id_==2)
		// {
		// 	double command = (1-press_middle)*20;
		// 	gripper_cmd.data = command<4. ? 4. : command;
		// 	gripper_command_pub.publish(gripper_cmd);
		// }
		// else
		// {
		// 	std::cout << "close gripper: " << press_middle << std::endl;
		// 	// gripper_cmd.data = 15.0;
		// 	if (press_middle<0.5)
		// 	{
		// 		if (press_middle<0.01) {gripper_cmd.data=0.;}
		// 		else {gripper_cmd.data=-20;}
		// 	}
		// 	else
		// 	{
		// 		gripper_cmd.data=15;
		// 	}
		// 	gripper_command_pub.publish(gripper_cmd);
		// }

		// // go_to_position(KDL::Frame(init_rotation, current_pose.p), vel_constant);

		// // ros::Duration(0.1).sleep();

		// // for (int i = 0; i < 7; i++)
		// // 	velocity_command_pub[i].publish(cmd);
		


	}



private:

	ros::NodeHandle nh_;
	int arm_id_;

	geometry_msgs::Twist desired_vel_msg;
	std_msgs::Int32 desired_gripper_msg;

	ros::Subscriber teleop_subscriber;
	ros::Subscriber teleop_button_subscriber;
	ros::Publisher des_vel_pub;
	ros::Publisher des_grip_pub;

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

	yumi_vr.main_loop();

    return 0;
}






















