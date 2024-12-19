/* doing IK control;
 i.e. given a goal pose, a current joint & gripper configuration
 -- find desired joint velocities & gripper commands */

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

	void desired_vel_callback(const geometry_msgs::Twist & msg)
	{
		desired_vel.vel.x(msg.linear.x);
		desired_vel.vel.y(msg.linear.y);
		desired_vel.vel.z(msg.linear.z);
		desired_vel.rot.x(msg.angular.x);
		desired_vel.rot.y(msg.angular.y);
		desired_vel.rot.z(msg.angular.z);
	}

	void desired_gripper_callback(const std_msgs::Int32 & msg) {
		desired_gripper = msg.data;
	}

//	double lowPassFilter(double current, double previous, double alpha) {
//    	return alpha * current + (1.0 - alpha) * previous;
//	}

	double limit_joint_delta_velcmd(double cmd, int joint) {
		double limited_cmd = cmd;
		double joint_vel = dq_current[joint];
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
		gripper_subscriber = nh_.subscribe("/yumi/gripper_states", 1, &YumiArmController::gripper_state_callback, this);

		// set joints velocity commands and gripper topics
		int urdf_order[7] = {1,2,7,3,4,5,6};
		std::string gripper_effort_topic_name;
		std::string command_topic;
		if (arm_id_ == 1) { // left arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_l/command";
				dq_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effort_topic_name = "/yumi/gripper_l_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effort_topic_name, 2);
			des_vel_subscriber = nh_.subscribe("/yumi/end_effector_vel_cmd_l", 1, &YumiArmController::desired_vel_callback, this);
			des_grip_subscriber = nh_.subscribe("/yumi/grip_cmd_l", 1, &YumiArmController::desired_gripper_callback, this);
		} else { // right arm
			for(int i = 0; i < 7; i++) {
				command_topic = "yumi/joint_vel_controller_" + std::to_string(urdf_order[i]) + "_r/command";
				dq_pub[i] = nh_.advertise<std_msgs::Float64>(command_topic.c_str(), 10);
			}
			gripper_effort_topic_name = "/yumi/gripper_r_effort_cmd";
			gripper_command_pub = nh_.advertise<std_msgs::Float64>(gripper_effort_topic_name, 2);
			des_vel_subscriber = nh_.subscribe("/yumi/end_effector_vel_cmd_r", 1, &YumiArmController::desired_vel_callback, this);
			des_grip_subscriber = nh_.subscribe("/yumi/grip_cmd_r", 1, &YumiArmController::desired_gripper_callback, this);
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
			if (!nh_.getParam("/initial_joint_position/left_arm", vect))
				vect = {-1.27, -1.84, 0.28, 0.49, 2.08, 1.94, -0.03};
			init_joint_position = vect;
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
				ROS_ERROR("Error initializing right_arm_kdl_wrapper");
		} else {
			if(!nh_.getParam("/initial_joint_position/right_arm", vect))
				vect = {1.262, -1.67, -0.398,0.362,  -2.114,  1.950, 0.129};
			init_joint_position = vect;
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
				ROS_ERROR("Error initializing right_arm_kdl_wrapper");
		}

		arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

		std::cout << "yumi_node started" << std::endl;

		// try to reset position arm when joint state callback is available
		while(ros::ok()) {
			ros::spinOnce();
			if (robot_ready == 1) {
				reset_arm();
				// arm_kdl_wrapper.fk_solver_pos->JntToCart(q_current, current_pose, -1);
				//init_rotation = current_pose.M;
				// std::cout << "current_pose xyz: " << current_pose.p(0)<< " " <<current_pose.p(1)<< " "<<current_pose.p(2)<< " "<< std::endl;
				// std::cout << "arm resetted" << std::endl;
				return;
			}
			ros::Duration(0.1).sleep();
		}


    }

	// Destructor
    ~YumiArmController() {
        // reset_arm();
		std_msgs::Float64 cmd;
		cmd.data = 0;
		for (int i = 0; i < 7; i++)
			dq_pub[i].publish(cmd);
    }

	void main_loop() {

		ros::Rate loop_rate(250);
		std_msgs::Float64 cmd;
		std_msgs::Float64 gripper_cmd;
		double prev_right_gripper = right_gripper_current+1;



		while(ros::ok()) {
			ros::spinOnce();

			arm_kdl_wrapper.ik_solver_vel->CartToJnt(q_current, desired_vel, dq_cmd);

			for(int i = 0; i < 7; i++) // arm
			{
				cmd.data = VELOCITY_CONST * dq_cmd(i);
				cmd.data = limit_joint_delta_velcmd(cmd.data, i);
				// cmd.data = lowPassFilter(cmd.data, prevCmd[i], ALPHA);
				dq_pub[i].publish(cmd);
			}

			// for the left arm this is an effort: -20 max opening force, 20 max closing force
			// for the right arm this is a position: 0 closed, 20 open
			if (arm_id_ == 1) {  // left gripper

				if (desired_gripper == 1) { // close gripper
					gripper_cmd.data = 15.0;
				} else { // open gripper
					if (left_gripper_current < 0.02)
						gripper_cmd.data = -10.0;
					else
						gripper_cmd.data = 0.0;
				}

			} else {  // right gripper

				if (desired_gripper == 1) { // close gripper
					gripper_cmd.data = std::max(1000*right_gripper_current - 4.0, 0.0);
				} else { // open gripper
					gripper_cmd.data = 20.0;
				}
			}
			prev_right_gripper = right_gripper_current;

			
			gripper_command_pub.publish(gripper_cmd);

			loop_rate.sleep();
		}

	}

	void reset_arm() {
		
		// go to intial joint position
		std_msgs::Float64 cmd;
		bool all_fine = false;
		while(all_fine == false) {
			ros::spinOnce();
			all_fine = true;
			for (int i = 0; i < 7; i++) {
				cmd.data = 0.3*(init_joint_position[i]-q_current(i));
				cmd.data = limit_joint_delta_velcmd(cmd.data, i);
				dq_pub[i].publish(cmd);
				if(std::abs(init_joint_position[i]-q_current(i))>0.02)
					all_fine = false;
			}
			ros::Duration(0.1).sleep();
		}

		// reset 0 velocity to all joints
		cmd.data = 0;
		for (int i = 0; i < 7; i++)
			dq_pub[i].publish(cmd);

		// // open gripper (for the right arm this is a position not an effort)
		// open_gripper();
	}

	void open_gripper() {
		std_msgs::Float64 gripper_cmd;
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
		std_msgs::Float64 gripper_cmd;
		if (arm_id_ == 1) {  // left arm
			// for the left arm this is an effort: -20 max opening force, 20 max closing force
			gripper_cmd.data = 15.0;
			gripper_command_pub.publish(gripper_cmd);
			ros::Duration(1.0).sleep();
		} else { // right arm
			// for the right arm this is a position: 0 closed, 20 open
			bool closed = false;
			double current_gripper = right_gripper_current;
			gripper_cmd.data = 20.;
			while(!closed)
			{	
				current_gripper = right_gripper_current;
				gripper_cmd.data -= 4.;
				gripper_command_pub.publish(gripper_cmd);
				ros::Duration(1.).sleep();
				ros::spinOnce();

				if(current_gripper - right_gripper_current<0.0003 || gripper_cmd.data<=4.)
				{
					closed=true;
				}
			}
			ros::Duration(1.).sleep();
		}
	}

private:

	ros::NodeHandle nh_;
	int arm_id_;

	int robot_ready;

	KDL::JntArray q_current;
	std::vector<double> dq_current;
	KDL::JntArray dq_cmd;
	KDLWrapper arm_kdl_wrapper;

	ros::Subscriber joint_subscriber;
	ros::Subscriber gripper_subscriber;
	ros::Subscriber des_vel_subscriber;
	ros::Subscriber des_grip_subscriber;
	std::vector<ros::Publisher> dq_pub;
	ros::Publisher gripper_command_pub;

	// geometry_msgs::PoseStamped desired_geometry_msg;
	KDL::Twist desired_vel;
	int desired_gripper = 0;

	std::vector<double> init_joint_position;

	double left_gripper_current;
	double right_gripper_current;
	double JOINT_VELOCITY_LIMIT = 0.15;
	double VELOCITY_CONST = 3.0;
	double ROTATION_CONST = 3.0;
	double MAX_POS_ERR = 0.005;
	double MAX_ROT_ERR = 0.01;
	double ALPHA = 0.7;



	// KDL::Frame current_pose;

	
	// std_msgs::Float64 cmd;
	// std_msgs::Float64 gripper_cmd;
	
	// KDL::Twist arm_cart_velocity;
	

};

int main(int argc, char** argv) {

	int arm_id = std::stoi(argv[1]); // using the same convention of KTH code. 1: left arm, 2: right arm

    ros::init(argc, argv, std::string("yumi_arm_controller_arm_")+std::string(argv[1]));

    ros::NodeHandle nh;

    YumiArmController arm_controller(nh, arm_id);

	arm_controller.main_loop();

    return 0;
}