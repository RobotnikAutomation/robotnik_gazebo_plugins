#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <cstdlib>
#include <cmath>
#include <tf/tf.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/SetModelState.h>

#include <robotnik_elevator_component/robotnik_elevator_component.h>


class RobotnikGazeboFakePhysicsElevatorNode : public RobotnikElevatorComponent
{
public:

public:
  RobotnikGazeboFakePhysicsElevatorNode(ros::NodeHandle h)
    : RobotnikElevatorComponent(h)
  {
    floor_height_ = 10.0;
    set_model_state_client_name_ = "/gazebo/set_model_state";
    x_ = 0.0;
    y_ = 0.0;
    // yaw_ = 0.0;

    rosReadParams();
  }

  virtual ~RobotnikGazeboFakePhysicsElevatorNode()
  {
  }

protected:
  ros::ServiceClient set_model_state_client_;
  std::string set_model_state_client_name_;
  double floor_height_;
  double x_, y_, yaw_;
  gazebo_msgs::SetModelState elevator_state_msg;

public:

  void rosReadParams(){
    pnh_.param<std::string>("set_model_state_client_service_name", set_model_state_client_name_, set_model_state_client_name_);
    pnh_.param("floor_height", floor_height_, floor_height_);
    pnh_.param("x", x_, x_);
    pnh_.param("y", y_, y_);
    pnh_.param("yaw", yaw_, yaw_);

    // double yaw_def = 2.3;
    // ros::param::get("/elevator/elevator_node/yaw", yaw_);
    // readParam(pnh_, "yaw", yaw_, yaw_def);
    // readParam(pnh_, "yaw", yaw_, yaw_);
    ROS_WARN("yaaaaw is %f", yaw_);
  }

  int rosSetup(){
    RobotnikElevatorComponent::rosSetup();
    set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>(set_model_state_client_name_);
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

		// std_msgs::Float64 doorleftmsg;
		// std_msgs::Float64 doorrightmsg;
    //
		// ROS_INFO("request: open the door");
		// ROS_INFO("sending back response: opening");
		// doorleftmsg.data=0.94375;
		// doorrightmsg.data=0.4625;
		// pub_doorleft.publish(doorleftmsg);
		// pub_doorright.publish(doorrightmsg);

		switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_OPEN);
		return 0;
	}

	int closeDoor(){
		// std_msgs::Float64 doorleftmsg;
		// std_msgs::Float64 doorrightmsg;
    //
		// doorleftmsg.data=0.0;
		// doorrightmsg.data=0.0;
    //
		// pub_doorleft.publish(doorleftmsg);
		// pub_doorright.publish(doorrightmsg);

		switchToDoorStatus(robotnik_elevator_interface_msgs::ElevatorState::DOOR_STATUS_CLOSE);

		return 0;
	}

	void readyState(){

    ROS_WARN("yaw is %f", yaw_);

		if(elevator_state.current_floor != elevator_state.target_floor){
      switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_MOVING);
      ros::Duration(0.5).sleep();

      //Call service to set model pose
      elevator_state_msg.request.model_state.model_name = "elevator";
      elevator_state_msg.request.model_state.pose.position.z = elevator_state.target_floor * floor_height_;

      elevator_state_msg.request.model_state.pose.position.x = x_;
      elevator_state_msg.request.model_state.pose.position.y = y_;

      ROS_WARN("yaw is: %f", yaw_);
      tf::Quaternion q;
      q.setRPY(0, 0, yaw_);
      geometry_msgs::Quaternion quaternion;
      tf::quaternionTFToMsg(q, quaternion);
      elevator_state_msg.request.model_state.pose.orientation = quaternion;

      ROS_WARN("q.x = %f, q.y = %f, q.z = %f, q.w = %f", quaternion.x, quaternion.y, quaternion.z, quaternion.w);

      bool result = set_model_state_client_.call(elevator_state_msg);
      if (false == result)
      {
        ROS_ERROR("Moving elevator has failed, couldn't contact service: %s", set_model_state_client_.getService().c_str());
      }
      else if (false == elevator_state_msg.response.success)
      {
        ROS_ERROR("Moving elevator has failed with message: %s", elevator_state_msg.response.status_message.c_str());
      }
      else
      {
        elevator_state.current_floor = elevator_state.target_floor;
        ros::Duration(0.5).sleep();

        switchToElevatorStatus(robotnik_elevator_interface_msgs::ElevatorState::ELEVATOR_STATUS_IDLE);
      }
		}
	}

};

// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_fake_physics_elevator_node");

  ros::NodeHandle n;
  RobotnikGazeboFakePhysicsElevatorNode controller(n);

  controller.start();

  return (0);
}
