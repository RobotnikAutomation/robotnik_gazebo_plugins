/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <string>
#include <tf/tf.h>
#include <stdlib.h>

#include "robotnik_gazebo_set_visual/robotnik_gazebo_set_visual.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(RobotnikGazeboSetVisual);

// Constructor
RobotnikGazeboSetVisual::RobotnikGazeboSetVisual()
{
  this->seed = 0;
}

// Destructor
RobotnikGazeboSetVisual::~RobotnikGazeboSetVisual()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->ground_truth_odom_queue_.clear();
  this->ground_truth_odom_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

// Load the controller
void RobotnikGazeboSetVisual::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_WARN("Set Visual Charged!");
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    //ROS_INFO_NAMED("set_visual", "robotnik_gazebo_set_visual robot_namespace: %s", this->robot_namespace_.c_str());
  }
  else
  {
    //ROS_INFO_NAMED("set_visual", "robotnik_gazebo_set_visual has no namespace");
  }

  //Reads and checks the existance of the link specified on the modelName label 
  if (!_sdf->HasElement("modelName"))
  {
    ROS_FATAL_NAMED("set_visual", "robotnik_gazebo_set_visual plugin missing <modelName>, cannot proceed");
    return;
  }
  else
  {
    this->link_name_ = _sdf->GetElement("modelName")->Get<std::string>();
    ROS_INFO_NAMED("set_visual", "robotnik_gazebo_set_visual plugin loaded for %s", this->link_name_.c_str());
  }

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("set_visual", "robotnik_gazebo_set_visual plugin error: modelName: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  //Reads and checks the existance of the link specified on the modelName label 
  if (!_sdf->HasElement("visualName"))
  {
    ROS_FATAL_NAMED("set_visual", "robotnik_gazebo_set_visual plugin missing <visualName>, cannot proceed");
    return;
  }
  else
  {
    this->visual_name_ = _sdf->GetElement("visualName")->Get<std::string>();
    ROS_INFO_NAMED("set_visual", "robotnik_gazebo_set_visual plugin loaded for %s and visual %s", this->link_name_.c_str(), this->visual_name_.c_str());
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("set_visual", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_->Init(world_->Name()); //GetName()
  pub_visual_ = node_->Advertise<gazebo::msgs::Visual>("~/visual");
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RobotnikGazeboSetVisual::OnUpdate, this));
  set_color_srv_ = nh_.advertiseService("/set_visual_raquel", &RobotnikGazeboSetVisual::setColorSrvCallback, this);
}

// Update the controller
void RobotnikGazeboSetVisual::UpdateChild()
{
  if (!this->link_)
    return;

#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  if (cur_time < this->last_time_)
  {
      ROS_WARN_NAMED("set_visual", "Negative update time difference detected.");
      this->last_time_ = cur_time;
  }
      // save last time stamp
      this->last_time_ = cur_time;
}

// Put laser data to the interface
void RobotnikGazeboSetVisual::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->ground_truth_odom_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void RobotnikGazeboSetVisual::OnUpdate()
{
  //ROS_WARN("Updating gazebo_set_visual");
}

gazebo::msgs::Visual RobotnikGazeboSetVisual::SetColor()
{
  physics::LinkPtr link_ = this->model_->GetLink(link_name_);
  std::string visual_link_name = link_->GetScopedName().c_str();
  std::string visual_name = visual_link_name + "::robot_visual";

  gazebo::msgs::Visual visualMsg = link_->GetVisualMessage(visual_name);

  visualMsg.set_name(link_->GetScopedName());
  //ROS_WARN("%s", link_->GetScopedName().c_str());

  visualMsg.set_parent_name(this->model_->GetScopedName());
  //ROS_ERROR("%s", this->model_->GetScopedName().c_str());

  // Initiate material
  if ((!visualMsg.has_material()) || visualMsg.mutable_material() == NULL)
  {
    ROS_ERROR_ONCE_NAMED("set_visual","modelName with scope :: visualName does ot contains material label, it can be due to the combination does not exists");
    gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
    visualMsg.set_allocated_material(materialMsg);
  }
  else
  {
    ROS_ERROR_ONCE_NAMED("set_visual", "modelName with scope :: visualName contains material label");
  }

  
  // Set color
  // gazebo::msgs::Color is deprecated for Gazebo > 7  
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Color newColor(color.r, color.g, color.b, 1);
#else
  gazebo::msgs::Color newColor(r, g, b, a);
#endif

  gazebo::msgs::Color *colorMsg = new gazebo::msgs::Color(gazebo::msgs::Convert(newColor));
  gazebo::msgs::Color *diffuseMsg = new gazebo::msgs::Color(*colorMsg);
  gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
  if (materialMsg->has_ambient())
  {
    materialMsg->clear_ambient();
  }
  materialMsg->set_allocated_ambient(colorMsg);
  if (materialMsg->has_diffuse())
  {
    materialMsg->clear_diffuse();
  }
  materialMsg->set_allocated_diffuse(diffuseMsg);
  //return;
  return visualMsg;  
}

/*bool RobotnikGazeboSetVisual::defineColors()
{
  if (_sdf->HasElement("color"))
  {
    sdf::ElementPtr sdfColor = _sdf->GetElement("color");
    while (sdfColor)
    {
      auto color = std::make_shared<ColorRGBA>();
      // name
      if (sdfColor->HasElement("name"))
      {
        color->name = sdfBlock->Get<string>("name");
      }
      else
      {
        gzerr << "Parameter <name> is missing in a color." << std::endl;
      }
      // code
      if (sdfBlock->HasElement("code"))
      {
        color->code = sdfBlock->Get<ignition::math::Color>("code");
      }
      else
      {
        color->code.Reset();
      }

      this->dataPtr->defined_colors_.push_back(color);
      sdfColor = sdfColor->GetNextElement("color");
    }
  }
  else
  {
    ROS_ERROR("set_visual", "No color tag found. It must be added manually.")
    return false;
  }

  return true;
}*/

bool RobotnikGazeboSetVisual::setColorSrvCallback(gazebo_msgs::SetLightProperties::Request& request,
                                              gazebo_msgs::SetLightProperties::Response& response)
{
  color = request.diffuse;

  pub_visual_->Publish(SetColor());
  response.success = true;
  return true;
}
}
