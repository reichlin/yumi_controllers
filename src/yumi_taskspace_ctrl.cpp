#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
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

        pose_goal.p = KDL::Vector(msg.position.x, msg.position.y, msg.position.z);
        pose_goal.M = KDL::Rotation::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
        goal_pose_command_ready = 1;

	}

    void ik_ctrl_callback(const std_msgs::Bool & msg) {

        ik_ctrl_ready = msg.data;

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
        goal_pose_command_ready = 0;
        ik_ctrl_ready = false;

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

		nh_.getParam("MAX_POS_ERR", MAX_POS_ERR);
		nh_.getParam("MAX_ROT_ERR", MAX_ROT_ERR);
        nh_.getParam("VELOCITY_CONST", VELOCITY_CONST);
		nh_.getParam("ROTATION_CONST", ROTATION_CONST);


        // subscribe to desired pose & current joint config
        q_current.resize(7);
		dq_current.resize(7);
        yumi_state_subscriber = nh_.subscribe("/yumi/joint_states", 1, &YumiTaskspaceController::robot_state_callback, this);
		goal_pose_subscriber = nh_.subscribe("/yumi/pose_desired", 1, &YumiTaskspaceController::goal_pose_callback, this);
        ik_state_subscriber = nh_.subscribe("/yumi/ik_ctrl_ready", 1, &YumiTaskspaceController::ik_ctrl_callback, this);

		// arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);

		std::cout << "yumi_taskspace_ctrl_node started" << std::endl;


    }

	// Destructor
    ~YumiTaskspaceController() {
        publish_twist_cmd(true);
    }

    void publish_twist_cmd(const bool publish_zero=true) {

        if (publish_zero) {
            twist_cmd.linear.x = 0.0;
            twist_cmd.linear.y = 0.0;
            twist_cmd.linear.z = 0.0;
            twist_cmd.angular.x = 0.0;
            twist_cmd.angular.y = 0.0;
            twist_cmd.angular.z = 0.0;
        }
        else {
            twist_cmd.linear.x = VELOCITY_CONST * vel_desired.vel.x();
            twist_cmd.linear.y = VELOCITY_CONST * vel_desired.vel.y();
            twist_cmd.linear.z = VELOCITY_CONST * vel_desired.vel.z();
            twist_cmd.angular.x = ROTATION_CONST * vel_desired.rot.x();
            twist_cmd.angular.y = ROTATION_CONST * vel_desired.rot.y();
            twist_cmd.angular.z = ROTATION_CONST * vel_desired.rot.z();
        }

        twist_publisher.publish(twist_cmd);
    }

    KDL::Vector get_angle_diff(KDL::Frame p0, KDL::Frame p_des) {

        double angle1_des = 0.0;
        double angle2_des = 0.0;
        double angle3_des = 0.0;
        double angle1_cur = 0.0;
        double angle2_cur = 0.0;
        double angle3_cur = 0.0;
        p_des.M.GetRPY(angle1_des, angle2_des, angle3_des);
        p0.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);

        double angle1_diff = angle1_des - angle1_cur;
        double angle2_diff = angle2_des - angle2_cur;
        double angle3_diff = angle3_des - angle3_cur;
        // normalize to [-pi;pi]
        angle1_diff += (angle1_diff > M_PI) ? -2*M_PI : (angle1_diff < -M_PI) ? 2*M_PI : 0;
        angle2_diff += (angle2_diff > M_PI) ? -2*M_PI : (angle2_diff < -M_PI) ? 2*M_PI : 0;
        angle3_diff += (angle3_diff > M_PI) ? -2*M_PI : (angle3_diff < -M_PI) ? 2*M_PI : 0;

        KDL::Vector delta(angle1_diff, angle2_diff, angle3_diff);
        return delta;
    }

    bool check_pose_distance(KDL::Frame p0, KDL::Frame p_des) {
        // based on pose_is: p0 & p_des: p1, compute the distance & return 1-2-3 for tiny-small-far
        int reached = false;
        KDL::Vector angle_diffs = get_angle_diff(p0, p_des);

        if (std::abs(p0.p(0) - p_des.p(0)) < MAX_POS_ERR &&
            std::abs(p0.p(1) - p_des.p(1)) < MAX_POS_ERR &&
            std::abs(p0.p(2) - p_des.p(2)) < MAX_POS_ERR &&
            angle_diffs[0] < MAX_ROT_ERR && angle_diffs[1] < MAX_ROT_ERR && angle_diffs[2] < MAX_ROT_ERR) {
            reached = true;
        }
        // } else if (std::abs(p0.p(0) - p_des.p(0)) < MAX_POS_THRESH &&
        //     std::abs(p0.p(1) - p_des.p(1)) < MAX_POS_THRESH &&
        //     std::abs(p0.p(2) - p_des.p(2)) < MAX_POS_THRESH &&
        //     angle_diffs[0] < MAX_ROT_THRESH && angle_diffs[1] < MAX_ROT_THRESH && angle_diffs[2] < MAX_ROT_THRESH) {
        //     pose_distance = 2;
        
        return reached;
    }

    void set_desired_vel() {
        vel_desired.vel = pose_desired.p - pose_current.p;
        vel_desired.rot = get_angle_diff(pose_current, pose_desired);
    }

    std::vector<KDL::Frame> get_waypoints(KDL::Frame& start, KDL::Frame& end, double min_step_size = 0.05) {
		
		double distance = (end.p - start.p).Normalize(1e-5);
		int numWaypoints = static_cast<int>(distance / min_step_size) + 1;
        std::vector<KDL::Frame> waypoints;

        double angle1_cur = 0.0;
        double angle2_cur = 0.0;
        double angle3_cur = 0.0;
        start.M.GetRPY(angle1_cur, angle2_cur, angle3_cur);
        KDL::Vector angle_diffs = get_angle_diff(start, end);

		// Generate waypoints
		for (int i = 1; i < numWaypoints; ++i) {
			double ratio = static_cast<double>(i) / (numWaypoints - 1); // interpolation ratio from 0 to 1
			KDL::Vector interpolatedPoint = start.p + ratio * (end.p - start.p);

            double interpAngle1 = angle1_cur + ratio * angle_diffs[0];
            double interpAngle2 = angle2_cur + ratio * angle_diffs[1];
            double interpAngle3 = angle3_cur + ratio * angle_diffs[2];
            KDL::Rotation interpolatedRotation = KDL::Rotation::RPY(interpAngle1, interpAngle2, interpAngle3);
            waypoints.push_back(KDL::Frame(interpolatedRotation, interpolatedPoint));
		}
	
		return waypoints;
	}

	void main_loop() {

		ros::Rate loop_rate(50);
        bool computed_traj = false;

		while(ros::ok()) {

            bool publish_zero = true;
			ros::spinOnce();
            
            pose_goal.p(0) = 0.493208;
            pose_goal.p(1) = -0.157315;
            pose_goal.p(2) =  0.354273;
            pose_goal.M = pose_current.M;
            goal_pose_command_ready = 1;

            if (ik_ctrl_ready && goal_pose_command_ready == 1 && robot_ready == 1) {
                // go to goal pose; strategy will depend on distance!
                bool reached_goal = check_pose_distance(pose_current, pose_goal);
                if (reached_goal) {
                    // then the goal is already reached --- can send out zero_twist.
                    std::cout << "goal pose reached, commanding zero twist" << std::endl;
                    // and reset waypoints and idx!!
                    current_traj_idx = 0;
                    computed_traj = false;
                } else {
                    publish_zero = false;
                    // std::cout << "commanding to goal pose via waypoints." << std::endl;
                    // std::cout << "current diff to goal pose:" << pose_goal.p(0) - pose_current.p(0)<< " " <<pose_goal.p(1) - pose_current.p(1)<< " "<<pose_goal.p(2) - pose_current.p(2)<<  std::endl;
                    if (computed_traj == false) {
                        trajectory = get_waypoints(pose_current, pose_goal);
                        current_traj_idx = 0;
                        pose_desired = trajectory[current_traj_idx];
                        computed_traj = true;
                    } else {
                        bool reached_waypoint = check_pose_distance(pose_current, pose_desired);
                        if (reached_waypoint) {
                            current_traj_idx += 1;
                        }
                        pose_desired = trajectory[current_traj_idx];
                        // std::cout << "commanding to goal pose via waypoints, current idx is" << current_traj_idx <<std::endl;
                        // std::cout << "current diff to waypoint pose:" << pose_desired.p(0) - pose_current.p(0)<< " " <<pose_desired.p(1) - pose_current.p(1)<< " "<<pose_desired.p(2) - pose_current.p(2)<<  std::endl;
                    
                    }
                    set_desired_vel();

                    double lin_vel_len = vel_desired.vel.Norm();
                    // std::cout << "length is " << lin_vel_len <<std::endl;
                    KDL::Vector theoretical_value;
                    if (current_traj_idx < trajectory.size()-1) {
                        for (int i=0; i<3; i++){
                            vel_desired.vel[i] = (0.1 / lin_vel_len) * vel_desired.vel[i];
                        }
                    }
                    // std::cout << "current linvel:" << vel_desired.vel.x()<< " " <<vel_desired.vel.y()<< " "<<vel_desired.vel.z()<<  std::endl;
                    // std::cout << "theoretical linvel:" << theoretical_value[0]<< " " <<theoretical_value[1]<< " "<<theoretical_value[2]<<  std::endl;

                    // // std::cout << "current linvel:" << vel_desired.vel.x()<< " " <<vel_desired.vel.y()<< " "<<vel_desired.vel.z()<<  std::endl;
                    // // std::cout << "current rotvel:" << vel_desired.rot.x()<< " " <<vel_desired.rot.y()<< " "<<vel_desired.rot.z()<<  std::endl;
                    // KDL::Vector total_distance_p = trajectory[trajectory.size()-1].p - pose_current.p;
					// KDL::Vector total_distance_r = get_angle_diff(pose_current, trajectory[trajectory.size()-1]);

					// double vel_p_scaling = total_distance_p.Norm() / vel_desired.vel.Norm();
					// double vel_rot_scaling = total_distance_r.Norm() / vel_desired.rot.Norm();
                    // // std::cout << "total distance lin " << total_distance_p.Norm() << "total distance rot " << total_distance_r.Norm() <<std::endl;
                    // // std::cout << "lin desired norm " << vel_desired.vel.Norm() << "rot desired norm " << vel_desired.rot.Norm() <<std::endl;
                    // std::cout << "lin scaling factor " << vel_p_scaling << "rot scaling factor " << vel_rot_scaling <<std::endl;
                    // for (int i=0; i<3; i++){
                    //     // if (i==1) {
                    //     //     std::cout << "new would linvel:" << vel_desired.vel.x()*vel_p_scaling<< " " <<vel_desired.vel.y()*vel_p_scaling<< " "<<vel_desired.vel.z()*vel_p_scaling<<  std::endl;
                    //     //     std::cout << "new would rotvel:" << vel_desired.rot.x() * vel_rot_scaling<< " " <<vel_desired.rot.y() * vel_rot_scaling<< " "<<vel_desired.rot.z() * vel_rot_scaling<<  std::endl;
                    //     // }
                    //     vel_desired.vel[i] = vel_desired.vel[i] * 4.0;
					// // 	vel_desired.rot[i] = vel_desired.rot[i] * vel_rot_scaling; //;
					// }
                }
                
            }

            publish_twist_cmd(publish_zero);
			loop_rate.sleep();

		}

	}

private:

	ros::NodeHandle nh_;
	int arm_id_;

	int robot_ready;
    int goal_pose_command_ready;
    bool ik_ctrl_ready;

	KDL::JntArray q_current;
	std::vector<double> dq_current;

    KDL::Frame pose_current;
    KDL::Frame pose_goal;
    
    KDL::Frame pose_desired;
    std::vector<KDL::Frame> trajectory;
    int current_traj_idx = 0;

    KDL::Twist vel_desired;
    geometry_msgs::Twist twist_cmd;

	KDLWrapper arm_kdl_wrapper;

	ros::Subscriber yumi_state_subscriber;
	ros::Subscriber goal_pose_subscriber;
    ros::Subscriber ik_state_subscriber;
	ros::Publisher twist_publisher;

	double MAX_POS_ERR = 0.005;
	double MAX_ROT_ERR = 0.01;
    double MAX_POS_THRESH = 0.4;
	double MAX_ROT_THRESH = (10.0 * 180.0 / M_PI);
    double VELOCITY_CONST = 0.3;
	double ROTATION_CONST = 0.3;


	

};

int main(int argc, char** argv) {

	int arm_id = std::stoi(argv[1]); // using the same convention of KTH code. 1: left arm, 2: right arm

    ros::init(argc, argv, std::string("yumi_taskspace_controller_arm_")+std::string(argv[1]));

    ros::NodeHandle nh;

    YumiTaskspaceController pose_controller(nh, arm_id);

	pose_controller.main_loop();

    return 0;
}
// commanding to goal pose via waypoints, current idx is1
// current diff to waypoint pose:0.00976555 0.0543672 0.00699004
// current linvel:0.00976555 0.0543672 0.00699004
// current rotvel:2.73729e-06 -0.000651583 -0.000274793
// total distance lin 0.22244total distance rot 0.000707163
// lin desired norm 0.0556778rot desired norm 0.000707163
// new would linvel:0.0390145 0.217204 0.027926
// new would rotvel:2.73729e-06 -0.000651583 -0.000274793


