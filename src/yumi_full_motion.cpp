#include <ros/ros.h>
#include <kdl_wrapper/kdl_wrapper.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <boost/lexical_cast.hpp>

using namespace std;
#define JOINT_VELOCITY_LIMIT 0.05

// global variables
int arm_idx = 1; // 0: right arm, 1: left arm

vector<double> push_cmd;
double push_cmd_time = -1;
volatile sig_atomic_t flag = 0;
KDL::JntArray arm_joint_positions;
vector<double> arm_joint_velocity;

// function declarations
vector <double> str2vector(string str, int n_data);
double limit_joint_velcmd(double cmd, int joint);
void print_joint_values();

void terminate(int sig){
  flag = 1;
}

void joint_state_callback(const sensor_msgs::JointState & msg)
{

    if (arm_idx == 0) {
      int r_arm_indecis[7] = {1,3,13,5,7,9,11};
      for(int i = 0; i < 7; i++)
      {
          arm_joint_positions(i) = msg.position[r_arm_indecis[i]];
          arm_joint_velocity[i] = msg.velocity[r_arm_indecis[i]];
      }
    } else {
      int l_arm_indecis[7] = {0,2,12,4,6,8,10};
      for(int i = 0; i < 7; i++)
      {
          arm_joint_positions(i) = msg.position[l_arm_indecis[i]];
          arm_joint_velocity[i] = msg.velocity[l_arm_indecis[i]];
      }
    }
    
}

void update_traj_callback(const std_msgs::String & msg)
{
  push_cmd = str2vector(msg.data, 8);
  //cout << push_cmd[0] << ' ' << push_cmd[1] << ' ' << push_cmd[2] << ' ' << push_cmd[3] << ' ' << push_cmd[4] << ' ' << push_cmd[5] << ' ' << push_cmd[6] << ' ' << push_cmd[7] << endl;
  push_cmd_time = ros::Time::now().toSec();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yumi_pushing_node");

  arm_joint_positions.resize(7);
  arm_joint_velocity.resize(7);
  push_cmd.resize(8);
  std_msgs::Float64 cmd;
  std_msgs::Float64 gripper_cmd;
  KDLWrapper arm_kdl_wrapper;
  KDL::Twist arm_cart_velocity;
  KDL::JntArray arm_joint_velcmd(7);
  string command_topic;
  KDL::Frame tool_tip_frame;

  signal(SIGINT, terminate);
  srand (time(NULL));
  std::cout << std::fixed << std::setprecision(2);
  ros::NodeHandle joint_node;
  ros::Subscriber joint_subscriber = joint_node.subscribe("/yumi/joint_states", 1, joint_state_callback);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ros::NodeHandle param_node;
  vector<ros::NodeHandle> velocity_command_node(7);
  vector<ros::Publisher> velocity_command_pub(7);
  
  
  
  int urdf_order[7] = {1,2,7,3,4,5,6};
  string gripper_effor_topic_name;
  if (arm_idx == 0) {
    for(int i = 0; i < 7; i++)
    {
      command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_r/command";
      velocity_command_pub[i] = velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
    }
    gripper_effor_topic_name = "/yumi/gripper_r_effort_cmd";
  } else {
    for(int i = 0; i < 7; i++)
    {
      command_topic = "yumi/joint_vel_controller_" + to_string(urdf_order[i]) + "_l/command";
      velocity_command_pub[i] = velocity_command_node[i].advertise<std_msgs::Float64>(command_topic.c_str(), 10);
    }
    gripper_effor_topic_name = "/yumi/gripper_l_effort_cmd";
  }
  ros::Publisher gripper_command_pub = joint_node.advertise<std_msgs::Float64>(gripper_effor_topic_name, 1);
  
  
  
  
  ros::NodeHandle cmd_node;
  ros::Subscriber cmd_subscriber = cmd_node.subscribe("/socket_string_msg", 10000, update_traj_callback);
  
  
  
  
  //ros::ServiceClient close_gripper_service = joint_node.serviceClient<yumi_hw::YumiGrasp>("do_grasp");
  //ros::ServiceClient open_gripper_service = joint_node.serviceClient<yumi_hw::YumiGrasp>("release_grasp");
  //yumi_hw::YumiGrasp gripper_srv;
  //gripper_srv.gripper_id = 2;
  

  spinner.start();
  cout << "yumi_pushing_node started" << endl;

  // ros parameters
  double cmd_timeout = 0.5;
  param_node.getParam("/command_timeout", cmd_timeout);
  vector<vector<double> > init_joint_position;
  vector<double> vect;
  
  if (arm_idx == 0) {
      param_node.getParam("/initial_joint_position/right_arm1", vect);
      init_joint_position.push_back(vect);
      if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_r"))
        ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
      arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);
    } else {
      param_node.getParam("/initial_joint_position/left_arm1", vect);
      init_joint_position.push_back(vect);
      if(!arm_kdl_wrapper.init("yumi_body", "yumi_link_7_l"))
        ROS_ERROR("Error initiliazing right_arm_kdl_wrapper");
      arm_kdl_wrapper.ik_solver_vel->setLambda(0.3);
    }
  
  
  // param_node.getParam("/initial_joint_position/right_arm0", vect);
  // r_init_joint_position.push_back(vect);
  

  

  usleep(1000000);
  print_joint_values();
  
  /*
  char user_resp;
  cout << "Press any key to continue (ctrl+c to kill)!" << endl;
  cin >> user_resp;
  */
  cmd.data = 0;
  for (int i = 0; i < 7; i++)
    velocity_command_pub[i].publish(cmd);
  if (flag)
    return 0;

  arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, tool_tip_frame, -1);
  double ee_height_setpoint = tool_tip_frame.p(2);


  while(ros::ok())
  {
    // move the arms to the initial position
    bool all_fine = false;
    while(all_fine == false)
    {
      all_fine = true;
      for (int i = 0; i < 7; i++)
      {
        cmd.data = 0.3*(init_joint_position[0][i]-arm_joint_positions(i));
        cmd.data = limit_joint_velcmd(cmd.data, i);
        velocity_command_pub[i].publish(cmd);
        if(abs(init_joint_position[0][i]-arm_joint_positions(i))>0.02)
          all_fine = false;
      }
      usleep(50000);
    }
    
    gripper_cmd.data = -10.0;
    gripper_command_pub.publish(gripper_cmd);
    usleep(2000000);
    gripper_cmd.data = 0.0;
    gripper_command_pub.publish(gripper_cmd);
    
    cmd.data = 0;
    for (int i = 0; i < 7; i++)
      velocity_command_pub[i].publish(cmd);
    if (flag)
      return 0;

    // wait until commands are received
    while(push_cmd_time < 0)
    {
      usleep(1000);
      if (flag)
        return 0;
    }
    
    int gripper_status = 0;
    int gripper_counter = 0;
    int gripper_max_counter = 20;
    int restart = 0;

    // follow the given commands
    while(ros::ok())
    {
      /*
      double time_since_last_cmd = ros::Time::now().toSec() - push_cmd_time;
      cout << time_since_last_cmd << endl;
      if (time_since_last_cmd > cmd_timeout)
      {
        cout << "Commands stopped - restarting!" << endl;
        cmd.data = 0;
        for(int i = 0; i < 7; i++)
          r_velocity_command_pub[i].publish(cmd);
        push_cmd_time = -1;
        break;
      }
      */
      if (flag)
      {
        cmd.data = 0;
        for(int i = 0; i < 7; i++)
          velocity_command_pub[i].publish(cmd);
        return 0;
      }
      
      
      if (push_cmd[7] > 0)
      {
        cout << "Commands stopped - restarting!" << endl;
        cmd.data = 0;
        for(int i = 0; i < 7; i++)
          velocity_command_pub[i].publish(cmd);
        push_cmd_time = -1;
        gripper_cmd.data = 0.0;
        gripper_command_pub.publish(gripper_cmd);
        break;
      }
      
      
      arm_kdl_wrapper.fk_solver_pos->JntToCart(arm_joint_positions, tool_tip_frame, -1);
      double ee_height_error = ee_height_setpoint - tool_tip_frame.p(2);
      
      //cout << "VR commands:" << endl;
      //cout << push_cmd[0] << " - " << push_cmd[1] << " - " << push_cmd[2] << " - " << push_cmd[3] << " - " << push_cmd[4] << " - " << push_cmd[5] << " - " << push_cmd[6] << endl;
      //cout << endl;
      
      arm_cart_velocity.vel = KDL::Vector(push_cmd[0], push_cmd[1], push_cmd[2]);
      arm_cart_velocity.rot = KDL::Vector(push_cmd[3], push_cmd[4], push_cmd[5]);
      
      //right_arm_cart_velocity.vel = KDL::Vector(push_cmd[0], push_cmd[1], ee_height_error*0.1);
      //right_arm_cart_velocity.rot = KDL::Vector(push_cmd[2], 0.0, 0.0);
      
      
      arm_kdl_wrapper.ik_solver_vel->CartToJnt(arm_joint_positions, arm_cart_velocity, arm_joint_velcmd);
      //cout << "joints commands:" << endl;
      for(int i = 0; i < 7; i++)
      {
        cout << arm_joint_velcmd(i) << " - ";
        cmd.data = arm_joint_velcmd(i);
        velocity_command_pub[i].publish(cmd);
      }
      cout << endl;
      //cout << endl;
      
      //
      
      
      
      if(push_cmd[6] > 0.5)// && gripper_status == 0 && gripper_counter < gripper_max_counter) 
      {
        gripper_cmd.data = 10.0;
        gripper_status = 1;
        //gripper_counter += 1;
      }
      else if(push_cmd[6] < 0.5 && gripper_status == 1 && gripper_counter < gripper_max_counter)
      {
        gripper_cmd.data = -10.0;
        gripper_counter += 1;
      }
      else if(push_cmd[6] < 0.5 && gripper_status == 1 && gripper_counter >= gripper_max_counter)
      {
        gripper_cmd.data = 0.0;
        gripper_counter = 0;
        gripper_status = 0;
      }
      else
      {
        gripper_cmd.data = 0.0;
        gripper_counter = 0;
      }
      gripper_command_pub.publish(gripper_cmd);
      
      
      
      usleep(50000); //wait 50 msec
    }
  }
  
  gripper_cmd.data = 0.0;
  gripper_command_pub.publish(gripper_cmd);
  
  cout << "node terminated successfully" << endl;
  return 0;
}

vector <double> str2vector(string str, int n_data)
{
  vector<double> data;
  for (int j = 0; j < n_data; j++)
  {
    std::size_t p = str.find(" ");
    string value = str.substr(0, p);
    str = str.substr(p + 1);
    data.push_back(boost::lexical_cast<double>(value));
  }
  return data;
}

double limit_joint_velcmd(double cmd, int joint)
{
  double limited_cmd = cmd;
  double joint_vel = arm_joint_velocity[joint];
  if((cmd - joint_vel) > JOINT_VELOCITY_LIMIT)
    limited_cmd = JOINT_VELOCITY_LIMIT;
  else if((cmd - joint_vel) < (-JOINT_VELOCITY_LIMIT))
    limited_cmd = -JOINT_VELOCITY_LIMIT;
  return limited_cmd;
}

void print_joint_values()
{
  for(int i = 0; i < 7; i++)
  {
    if (arm_joint_positions(i) >= 0)
      std::cout << " ";
    std::cout << arm_joint_positions(i) << ", ";
  }
  cout << endl;
}
