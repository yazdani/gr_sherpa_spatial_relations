#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <sstream>

void get_model_pose(int argc, char **argv)
{
  ros::init(argc, argv, "get_model_pose_of_pr2");
  ros::NodeHandle n;
  ros::NodeHandle m;
  ros::ServiceClient gmscl, gmscl2 = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceClient smsl = m.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state"); 
  gazebo_msgs::GetModelState getmodelstate, getmodelstate2; 
  gazebo_msgs::SetModelState setmodelstate;

  gazebo_msgs::ModelState modelstate;
  geometry_msgs::Pose start_pose;
  // gazebo_msgs::ModelState modelstate;
  geometry_msgs::Twist start_twist;

  modelstate.model_name = (std::string) "unit_box_1"; //setmodelstate of the model  
  getmodelstate.request.model_name = "pr2";
  getmodelstate2.request.model_name= "unit_box_1";//"quadrotor";
  gmscl.call(getmodelstate);

  //  while(ros::ok())
  // {}
  gmscl2.call(getmodelstate2);
  start_pose.position.x =  getmodelstate.response.pose.position.x + 2;
  start_pose.position.y =  getmodelstate.response.pose.position.y;
  start_pose.position.z =  getmodelstate.response.pose.position.z +1;
  //rt_pose.orientation.x = 0;
  //start_pose.orientation.y =  0;
  //start_pose.orientation.z =  0;
  //start_pose.orientation.w =  0;

  modelstate.pose = start_pose;
  //modelstate.twist = start_twist;
  
  setmodelstate.request.model_state = modelstate;
  smsl.call(setmodelstate);
}

int main(int argc, char **argv)
{
  get_model_pose(argc,argv);
  ROS_INFO("MAIN");
  return 0;
}
