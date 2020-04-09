#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <cstdlib>
#include <cmath>
#include <sensor_msgs/JointState.h>

#include <robotnik_elevator_component/robotnik_elevator_component.h>



class RobotnikGazeboElevatorNode : public RobotnikElevatorComponent
{
public:
 
public:
  RobotnikGazeboElevatorNode(ros::NodeHandle h)
    : RobotnikElevatorComponent(h)
  {
	height_actual = height=0.0;
	floor_height_=10.0;
	elevator_cab_joint_name_="elevatorcab_joint";
	elevator_goto_step = 0;
  rosReadParams();
  }

  virtual ~RobotnikGazeboElevatorNode()
  {
  }

  /* Rcomponent stuff */
  /*virtual void initState();
  virtual void standbyState();
  virtual void readyState();
  virtual void allState();
  virtual int rosSetup();
  virtual void rosReadParams();
  virtual int setup();
  virtual int stop();
  virtual std::string getStateString();
  virtual void rosPublish();
  
  virtual bool setElevatorControlServiceServerCb(robotnik_elevator_interface_msgs::SetElevatorControl::Request& request, robotnik_elevator_interface_msgs::SetElevatorControl::Response& response);
  virtual bool setDoorStateServiceServerCb(robotnik_elevator_interface_msgs::SetDoorState::Request& request, robotnik_elevator_interface_msgs::SetDoorState::Response& response);
  virtual bool goToFloorServiceServerCb(robotnik_elevator_interface_msgs::GoToFloor::Request& request, robotnik_elevator_interface_msgs::GoToFloor::Response& response);
  */

protected:
	//void heightCallback(const sensor_msgs::JointState& msg);
  
protected:
 
    double height_actual;
	double height;
	double floor_height_;
	
    // PUBLISHERS
	ros::Publisher pub_elevatorcab; 
	ros::Publisher pub_doorleft; 
	ros::Publisher pub_doorright;
	ros::Publisher pub_floorextension;
	
	// SUBSCRIBERS
	ros::Subscriber sub_elevatorcab;
	
	std::string elevator_cab_joint_name_;
	
	int elevator_goto_step;
	ros::Time elevator_goto_step_timer;
  
public:
  
    void rosReadParams(){
		pnh_.param<std::string>("elevator_cab_joint_name", elevator_cab_joint_name_, elevator_cab_joint_name_);
		pnh_.param("floor_height", floor_height_, floor_height_);

		
	}
	
	
    int rosSetup(){
		RobotnikElevatorComponent::rosSetup();
		
		pub_elevatorcab=nh_.advertise<std_msgs::Float64>("elevatorcab_joint_position_controller/command",1, true);
		pub_doorleft=nh_.advertise<std_msgs::Float64>("doorleft_joint_position_controller/command",1, true);
		pub_doorright=nh_.advertise<std_msgs::Float64>("doorright_joint_position_controller/command",1, true);
		pub_floorextension=nh_.advertise<std_msgs::Float64>("floorextension_joint_position_controller/command",1, true);

		sub_elevatorcab = nh_.subscribe("joint_states", 10, &RobotnikGazeboElevatorNode::heightCallback, this);
	}
	
	void standbyState(){
		switchToState(robotnik_msgs::State::READY_STATE);
		switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_IDLE);
		switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_CLOSE);	
	}
  
	int takeElevatorControl(){
		return 0;
	}

	int releaseElevatorControl(){
		return 0;
	}

	int goToFloor(int floor){
		
		elevator_state.target_floor = floor;
  
		return 0;
	}

	int openDoor(){
		
		std_msgs::Float64 doorleftmsg;
		std_msgs::Float64 doorrightmsg;
		
		ROS_INFO("request: open the door");
		ROS_INFO("sending back response: opening");
		doorleftmsg.data=0.94375;
		doorrightmsg.data=0.4625;
		pub_doorleft.publish(doorleftmsg);
		pub_doorright.publish(doorrightmsg);
  
		switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_OPEN);
		return 0;
	}

	int closeDoor(){
		std_msgs::Float64 doorleftmsg;
		std_msgs::Float64 doorrightmsg;

		doorleftmsg.data=0.0;
		doorrightmsg.data=0.0;
		
		pub_doorleft.publish(doorleftmsg);
		pub_doorright.publish(doorrightmsg);
		
		switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_CLOSE);		
		
		return 0;
	}
	
	void readyState(){
		static std_msgs::Float64 floorextensionmsg;
		static std_msgs::Float64 elevatorcabmsg;
		
		if(elevator_state.current_floor != elevator_state.target_floor){
			
			switch(elevator_goto_step){
				case 0:
					switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_MOVING);
					floorextensionmsg.data=-0.2;
					pub_floorextension.publish(floorextensionmsg);
					elevator_goto_step = 1;
					elevator_goto_step_timer == ros::Time::now();
				break;
				
				case 1:
					if((ros::Time::now() - elevator_goto_step_timer).toSec() >= 0.5){
						if(height!=0.0){
							elevatorcabmsg.data=height-0.1;
							pub_elevatorcab.publish(elevatorcabmsg);
						}
						elevator_goto_step = 2;
						elevator_goto_step_timer == ros::Time::now();
					}
				
				break;
				
				case 2:	
					height=elevator_state.target_floor*floor_height_;
					RCOMPONENT_INFO("height=%lf",height);
					elevatorcabmsg.data=height-0.2;
					pub_elevatorcab.publish(elevatorcabmsg);
					elevator_goto_step = 3;
					elevator_goto_step_timer == ros::Time::now();
				break;
				
				case 3:
					if(abs(height-height_actual)<=0.4){
						elevator_goto_step = 4;
						elevator_goto_step_timer == ros::Time::now();
					}else
						RCOMPONENT_INFO_THROTTLE(2,"target = %lf, current = %lf",height, height_actual);
				break;
				
				
				case 4:
					floorextensionmsg.data=0;
					elevatorcabmsg.data=height;
					
					if (height!=0){
						elevatorcabmsg.data+=0.1;
						floorextensionmsg.data=0;
					}
					
					pub_floorextension.publish(floorextensionmsg);
					elevator_goto_step = 5;
					elevator_goto_step_timer == ros::Time::now();
					
				break;
			
				case 5:
					if((ros::Time::now() - elevator_goto_step_timer).toSec() >= 0.5){
						pub_elevatorcab.publish(elevatorcabmsg);
						elevator_state.current_floor = elevator_state.target_floor;
						switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_IDLE);
						elevator_goto_step = 0;
					}
				break;
					
			}
		}
		
	}

protected:
	
	void heightCallback(const sensor_msgs::JointState& msg)
	{
		height_actual=msg.position[2];
	}
  
};


// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_elevator_node");

  ros::NodeHandle n;
  RobotnikGazeboElevatorNode controller(n);

  controller.start();

  return (0);
}
