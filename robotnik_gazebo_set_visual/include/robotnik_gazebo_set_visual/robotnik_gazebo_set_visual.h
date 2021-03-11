/*
 * Copyright 2020 Robotnik Automation SLL
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef ROBOTNIK_GAZEBO_SET_VISUAL_HH
#define ROBOTNIK_GAZEBO_SET_VISUAL_HH

#include <string>
#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo_msgs/SetLightProperties.h>

namespace gazebo
{
  class RobotnikGazeboSetVisual : public ModelPlugin
  {
    public: 
      RobotnikGazeboSetVisual();                                    //Constructor
      virtual ~RobotnikGazeboSetVisual();                           //Destructor
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);   //Parameters loader

    protected:
      virtual void OnUpdate();
      gazebo::msgs::Visual SetColor();
      bool setColorSrvCallback(gazebo_msgs::SetLightProperties::Request& request, gazebo_msgs::SetLightProperties::Response& response);

    private: 
      std::vector<std::string> visuals_;
      physics::WorldPtr world_;
      physics::ModelPtr model_;
      physics::LinkPtr link_;
      ros::NodeHandle* rosnode_;
      std::string link_name_;
      std::string visual_name_;
      common::Time last_time_;
      std::string robot_namespace_;
      event::ConnectionPtr update_connection_;  // Pointer to the update event connection
      unsigned int seed;
      gazebo::transport::NodePtr node_;
      gazebo::transport::PublisherPtr pub_visual_;
      gazebo::event::ConnectionPtr updateConnection;
      float r,g,b,a;
      ros::NodeHandle nh_;
      ros::ServiceServer set_color_srv_;
      std_msgs::ColorRGBA color;
      std::vector<std::string> scoped_visuals;
      std::string visual_id_2change;
  };
}
#endif
