#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "elevator_node/GetControl.h"
#include "elevator_node/GoToFloor.h"
#include "elevator_node/ReleaseControl.h"
#include "elevator_node/SetDoor.h"
#include <cstdlib>
#include <cmath>
#include <sensor_msgs/JointState.h>


ros::Publisher pub_elevatorcab; 
ros::Publisher pub_doorleft; 
ros::Publisher pub_doorright;
ros::Publisher pub_floorextension;

ros::Subscriber sub_elevatorcab;

double height_actual;
double height=0;
double floor_height=40;
  

bool control(elevator_node::GetControl::Request  &req,
         elevator_node::GetControl::Response &res)
{
  ROS_INFO("request: get control");
  ROS_INFO("sending back response: getting control");
  res.done = "yes";
  return true;
}


void heightCallback(const sensor_msgs::JointState& msg)
{
  height_actual=msg.position[2];
}

bool changefloor(elevator_node::GoToFloor::Request  &req,
         elevator_node::GoToFloor::Response &res)
{
  std_msgs::Float64 floorextensionmsg;
  std_msgs::Float64 elevatorcabmsg;
  
  ROS_INFO("request: go to floor %ld", (long int)req.floor);
  ROS_INFO("sending back response: going to floor %ld", (long int)req.floor);
  
  floorextensionmsg.data=-0.2;
  pub_floorextension.publish(floorextensionmsg);
  
  if(height!=0){
	elevatorcabmsg.data=height-0.1;
    pub_elevatorcab.publish(elevatorcabmsg);
    ros::Duration(0.5).sleep(); // sleep for 0.5 seconds
  }
  
  height=req.floor*floor_height;
  elevatorcabmsg.data=height-0.2;
  pub_elevatorcab.publish(elevatorcabmsg);
  while (abs(height-height_actual)>0.4){ros::spinOnce();}  
  
  floorextensionmsg.data=0;
  elevatorcabmsg.data=height;
  if (height!=0){
    elevatorcabmsg.data+=0.1;
    floorextensionmsg.data=0;
  }
  pub_floorextension.publish(floorextensionmsg);
  ros::Duration(0.3).sleep(); // sleep for 0.5 seconds
  pub_elevatorcab.publish(elevatorcabmsg);
  res.done = "yes";
  return true;
}

bool release(elevator_node::ReleaseControl::Request  &req,
         elevator_node::ReleaseControl::Response &res)
{
  ROS_INFO("request: release control");
  ROS_INFO("sending back response: releasing control");
  res.done = "yes";
  return true;
}

bool setdoor(elevator_node::SetDoor::Request  &req,
         elevator_node::SetDoor::Response &res)
{
  ROS_INFO("request: door");
  res.done = "yes";
  std_msgs::Float64 doorleftmsg;
  std_msgs::Float64 doorrightmsg;
  if (req.state=="open"){
	ROS_INFO("request: open the door");
	ROS_INFO("sending back response: opening");
	doorleftmsg.data=0.94375;
	doorrightmsg.data=0.4625;
	pub_doorleft.publish(doorleftmsg);
	pub_doorright.publish(doorrightmsg);
	
  }
  else if (req.state=="close"){
	ROS_INFO("request: open the door");
    ROS_INFO("sending back response: closing");
    doorleftmsg.data=0.0;
	doorrightmsg.data=0.0;
	pub_doorleft.publish(doorleftmsg);
	pub_doorright.publish(doorrightmsg);
  }
  else {
    ROS_INFO("sending back response: argument unknown");
    res.done = "no";
  }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator_node");
  ros::NodeHandle nh;
 
  pub_elevatorcab=nh.advertise<std_msgs::Float64>("/elevator/elevatorcab_joint_position_controller/command",1, true);
  std_msgs::Float64 elevatorcabmsg;
  
  pub_doorleft=nh.advertise<std_msgs::Float64>("/elevator/doorleft_joint_position_controller/command",1, true);
  std_msgs::Float64 doorleftmsg;
  
  pub_doorright=nh.advertise<std_msgs::Float64>("/elevator/doorright_joint_position_controller/command",1, true);
  std_msgs::Float64 doorrightmsg;
  
  pub_floorextension=nh.advertise<std_msgs::Float64>("/elevator/floorextension_joint_position_controller/command",1, true);
  std_msgs::Float64 floorextensionmsg;
  
  ros::ServiceServer service1 = nh.advertiseService("get_control", control);
  ROS_INFO("Ready to be controlled.");
  ros::ServiceServer service2 = nh.advertiseService("go_to_floor", changefloor);
  ROS_INFO("Ready to change of floor.");
  ros::ServiceServer service3 = nh.advertiseService("release_control", release);
  ROS_INFO("Ready to release control.");
  ros::ServiceServer service4 = nh.advertiseService("set_door", setdoor);
  ROS_INFO("Ready to move door.");
  
  ros::Subscriber sub_elevatorcab = nh.subscribe("/elevator/joint_states", 10, heightCallback);
  ros::spin();
  
  return 0;
}

