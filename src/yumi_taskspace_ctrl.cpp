#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
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



class YumiTaskspaceController {
public:

	void goal_pose_callback(const geometry_msgs::Pose & msg) {

        pose_desired.p = KDL::Vector(msg.position.x, msg.position.y, msg.position.z);
        pose_desired.M = KDL::Rotation::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
        pose_command_ready = 1;

	}

    void robot_state_callback(const sensor_msgs::JointState & msg) {
		if (arm_id_ == 1) { //left arm
			int arm_indices[7] = {0,2,12,4,6,8,10};
			for(int i = 0; i < 7; i++) {
				q_current(i) = msg.position[arm_indices[i]];
				dq_current[i] = msg.velocity[arm_indices[i]];
			}
		} else { //right arm
			int arm_indices[7] = {1,3,13,5,7,9,11};
			for(int i = 0; i < 7; i++) {
				q_current(i) = msg.position[arm_indices[i]];
				dq_current[i] = msg.velocity[arm_indices[i]];
			}
		}
        arm_kdl_wrapper.fk_solver_pos->JntToCart(q_current, pose_current, -1);
		robot_ready = 1;
	}

    // Constructor
	YumiTaskspaceController(ros::NodeHandle& nh, int arm_id) : nh_(nh), arm_id_(arm_id) {

		robot_ready = 0;
        pose_command_ready = 0;

        // set twist command topic & initialize kdl wrapper
		std::string command_topic;
		if (arm_id_ == 1) {
            command_topic = "yumi/end_effector_vel_cmd_l";
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
				ROS_ERROR("Error initializing right_arm_kdl_wrapper");
		} else {
            command_topic = "yumi/end_effector_vel_cmd_r";
			if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
				ROS_ERROR("Error initializing right_arm_kdl_wrapper");
		}
			
        twist_publisher = nh_.advertise<geometry_msgs::Twist>(command_topic.c_str(), 10);

		nh_.getParam("JOINT_VELOCITY_LIMIT", JOINT_VELOCITY_LIMIT);
		nh_.getParam("VELOCITY_CONST", VELOCITY_CONST);
		nh_.getParam("ROTATION_CONST", ROTATION_CONST);
		nh_.getParam("MAX_POS_ERR", MAX_POS_ERR);
		nh_.getParam("MAX_ROT_ERR", MAX_ROT_ERR);
		nh_.getParam("ALPHA", ALPHA);

        // subscribe to desired pose & current joint config
        q_current.resize(7);
		dq_current.resize(7);
        yumi_state_subscriber = nh_.subscribe("/yumi/joint_states", 1, &YumiTaskspaceController::robot_state_callback, this);
		desired_pose_subscriber = nh_.subscribe("/yumi/pose_desired", 1, &YumiTaskspaceController::goal_pose_callback, this);

		// arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

		std::cout << "yumi_taskspace_ctrl_node started" << std::endl;


    }

	// Destructor
    ~YumiTaskspaceController() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0; //or des_twist.linear.x() & angular.x() etc.
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        twist_publisher.publish(cmd);
    }

	void main_loop() {

		ros::Rate loop_rate(50);

		while(ros::ok()) {
			ros::spinOnce();

			// send out a twist message. (also check for pose_command_ready and robot_ready maybe)
            // 2nd level: set out distance to current pose, but on only position-level, tmrw on angular level!. 3rd level: send out desired close-by pose.
            // future: actual waypoint or maniptask control. no self-collision/limit avoidance?
            KDL::Twist desired_vel;
            if (pose_command_ready == 1) {
                desired_vel.vel = pose_desired.p - pose_current.p;
                double angle1_des = 0.0;
                double angle2_des = 0.0;
                double angle3_des = 0.0;
                double angle1_cur = 0.0;
                double angle2_cur = 0.0;
                double angle3_cur = 0.0;
                pose_desired.M.GetRPY(angle1_des, angle2_des, angle3_des);
                pose_current.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);

                double angle1_diff = angle1_des - angle1_cur;
                double angle2_diff = angle2_des - angle2_cur;
                double angle3_diff = angle3_des - angle3_cur;
                angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
                angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
                angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;

                desired_vel.rot = KDL::Vector(angle1_diff, angle2_diff, angle3_diff);
            }
            else {
                desired_vel.vel.x(0.0);
                desired_vel.vel.y(0.0);
                desired_vel.vel.z(0.0);
                
                double angle1_des = 0.0;
                double angle2_des = 0.0;
                double angle3_des = 0.0;
                double angle1_cur = 0.0;
                double angle2_cur = 0.0;
                double angle3_cur = 0.0;
                pose_desired.M.GetRPY(angle1_des, angle2_des, angle3_des);
                pose_current.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);

                double angle1_diff = angle1_des - angle1_cur;
                double angle2_diff = angle2_des - angle2_cur;
                double angle3_diff = angle3_des - angle3_cur;
                angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
                angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
                angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;

                desired_vel.rot = KDL::Vector(angle1_diff, angle2_diff, angle3_diff);

            }
            
            geometry_msgs::Twist cmd;
            cmd.linear.x = desired_vel.vel.x();
            cmd.linear.y = desired_vel.vel.y();
            cmd.linear.z = desired_vel.vel.z();
            cmd.angular.x = desired_vel.rot.x();
            cmd.angular.y = desired_vel.rot.y();
            cmd.angular.z = desired_vel.rot.z();

            twist_publisher.publish(cmd);
			loop_rate.sleep();
		}

	}

private:

	ros::NodeHandle nh_;
	int arm_id_;

	int robot_ready;
    int pose_command_ready;

	KDL::JntArray q_current;
	std::vector<double> dq_current;

    KDL::Frame pose_current;
    KDL::Frame pose_desired;

	KDLWrapper arm_kdl_wrapper;

	ros::Subscriber yumi_state_subscriber;
	ros::Subscriber desired_pose_subscriber;
	ros::Publisher twist_publisher;

	double left_gripper_current;
	double right_gripper_current;
	double JOINT_VELOCITY_LIMIT;
	double VELOCITY_CONST;
	double ROTATION_CONST;
	double MAX_POS_ERR;
	double MAX_ROT_ERR;
	double ALPHA;


	

};

int main(int argc, char** argv) {

	int arm_id = std::stoi(argv[1]); // using the same convention of KTH code. 1: left arm, 2: right arm

    ros::init(argc, argv, std::string("yumi_taskspace_controller_arm_")+std::string(argv[1]));

    ros::NodeHandle nh;

    YumiTaskspaceController pose_controller(nh, arm_id);

	pose_controller.main_loop();

    return 0;
}