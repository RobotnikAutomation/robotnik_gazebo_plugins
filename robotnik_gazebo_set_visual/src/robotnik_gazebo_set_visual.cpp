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
  delete this->rosnode_;
}

// Load the controller
void RobotnikGazeboSetVisual::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_INFO_NAMED("Set_visual", "Loading robotnik_gazebo_set_visual");
  
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    ROS_DEBUG_NAMED("set_visual", "RobotnikGazeboSetVisual::Load robotnik_gazebo_set_visual robot_namespace: %s", this->robot_namespace_.c_str());
  }
  else
  {
    ROS_DEBUG_NAMED("set_visual", "RobotnikGazeboSetVisual::Load robotnik_gazebo_set_visual has no namespace");
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
    ROS_INFO_NAMED("set_visual", "RobotnikGazeboSetVisual::Load robotnik_gazebo_set_visual plugin loaded for %s", this->link_name_.c_str());
  }

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("set_visual", "RobotnikGazeboSetVisual::Load robotnik_gazebo_set_visual plugin error: modelName: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  //Reads visual tags, each tag must contain a visualName label 
  //and this label represents the name of the visual inside the link. 
  //It is important to know that the visuals must be inside part of the same link
  int i = 0;
  if (_sdf->HasElement("visual"))
  {
    sdf::ElementPtr sdfVisual = _sdf->GetElement("visual");

    while (sdfVisual)
    {
      std::string visual_sdf;
      // name
      if (sdfVisual->HasElement("visualName"))
      {
        visual_sdf = sdfVisual->Get<std::string>("visualName");
        i++;
        ROS_DEBUG_NAMED("set_visual", "RobotnikGazeboSetVisual::Load %d: %s readed", i, visual_sdf.c_str());
      }
      else
      {
        ROS_ERROR_NAMED("set_visual","RobotnikGazeboSetVisual::Load Parameter 'visualName' is missing in 'visual'.");
      }
      this->visuals_.push_back(visual_sdf);
      sdfVisual = sdfVisual->GetNextElement("visual");
    }
    ROS_DEBUG_NAMED("set_visual", "RobotnikGazeboSetVisual::Load 'visualName' readed succesfully!");
  }
  else
  {
    ROS_ERROR_NAMED("set_visual", "RobotnikGazeboSetVisual::Load No visual tag found");
    //return;
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("set_visual", "RobotnikGazeboSetVisual::Load Gazebo ROS node has not been initialized, unable to load plugin.");
    return;
  }

  //Gets the scope for setting the full name of the visuals
  physics::LinkPtr link_ = this->model_->GetLink(link_name_);
  std::string scoped_name = link_->GetScopedName().c_str();

  for(int i=0; i < visuals_.size(); i++)
  {
    scoped_visuals.push_back(scoped_name + "::" + visuals_[i]);
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node_->Init(world_->Name()); //GetName()
  pub_visual_ = node_->Advertise<gazebo::msgs::Visual>("~/visual");
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RobotnikGazeboSetVisual::OnUpdate, this));
  set_color_srv_ = nh_.advertiseService("/robotnik_gazebo_set_visual/set_visual", &RobotnikGazeboSetVisual::setColorSrvCallback, this);
}

// Update the controller, virtual method
void RobotnikGazeboSetVisual::OnUpdate()
{
  //ROS_WARN_NAMED("set_visual", "Updating gazebo_set_visual");
}

gazebo::msgs::Visual RobotnikGazeboSetVisual::SetColor()
{
  int i = 0;
  //Check the existance of the visual received
  while (visuals_[i] != visual_id_2change && i < visuals_.size())
  {
    i++;
  }
  
  gazebo::msgs::Visual visualMsg;

  if(i!=visuals_.size())
  {
    visualMsg = link_->GetVisualMessage(scoped_visuals[i]);

    visualMsg.set_name(link_->GetScopedName());
    //ROS_WARN("%s", link_->GetScopedName().c_str());
  
    visualMsg.set_parent_name(this->model_->GetScopedName());
    //ROS_ERROR("%s", this->model_->GetScopedName().c_str());
  
    // Initiate material
    if ((!visualMsg.has_material()) || visualMsg.mutable_material() == NULL)
    {
      ROS_ERROR_NAMED("set_visual","RobotnikGazeboSetVisual::SetColor %s does not contain material label, check if it is defined in the urdf.", link_->GetScopedName().c_str());
      gazebo::msgs::Material *materialMsg = new gazebo::msgs::Material;
      visualMsg.set_allocated_material(materialMsg);
    }
    else
    {
      ROS_DEBUG_NAMED("set_visual", "RobotnikGazeboSetVisual::SetColor %s contains material label, creating the visualMsg", link_->GetScopedName().c_str());
  
    // Set color
    // gazebo::msgs::Color is deprecated for Gazebo > 7  
  #if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Color newColor(color.r, color.g, color.b, color.a);
  #else
      gazebo::msgs::Color newColor(color.r, color.g, color.b, color.a); //TODO: No tested, deprecated.
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
    }
    response_ = true;
    status_ = visual_id_2change + " succesfully changed.";
  }
  else
  {
    ROS_ERROR_NAMED("set_visual", "RobotnikGazeboSetVisual::SetColor No matching visual. Did you forget to add %s as a visual in the plugin?", visual_id_2change.c_str());
    response_ = false;
    status_ = "No matching visual, visuals added in the plugin:";

    for(int iter = 0; iter < visuals_.size(); iter++)
    {
      status_ = status_ + ", " + visuals_[iter];
    }
  }
 
  return visualMsg;  
}

bool RobotnikGazeboSetVisual::setColorSrvCallback(gazebo_msgs::SetLightProperties::Request& request,
                                              gazebo_msgs::SetLightProperties::Response& response)
{
  color = request.diffuse;
  visual_id_2change = request.light_name;

  ROS_DEBUG_NAMED("set_visual", "RobotnikGazeboSetVisual::setColorSrvCallback Received a petition to change the color of %s to color.r: %f, color.g: %f, color.b: %f, color.a: %f", visual_id_2change.c_str(), color.r, color.g, color.b, color.a);
  
  pub_visual_->Publish(SetColor());
  response.success = response_;
  response.status_message = status_;
  return true;
}
}
