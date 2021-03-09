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

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <tf/transform_broadcaster.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo_msgs/SetLightProperties.h>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  struct ColorRGBA
  {
    /// \brief The name of the color.
    public: std::string color_name;
    /// \brief The rgba representation of the color.
    public: ignition::math::Color color_code;
  };

  class RobotnikGazeboSetVisual : public ModelPlugin
  {
    /// \brief Constructor
    public: RobotnikGazeboSetVisual();

    /// \brief Destructor
    public: virtual ~RobotnikGazeboSetVisual();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    public: void OnUpdate();

    /// \brief Update the controller  //TODO: Delete this method
    protected: virtual void UpdateChild();

    //protected: bool setColorSrvCallback(ignition::math::Color &color_code);
    
    protected: ignition::math::Color color_;
    protected: gazebo::msgs::Visual SetColor();
  
    protected: bool ReadColors();

    public: std::vector< std::shared_ptr<ColorRGBA> > defined_colors_;

    public: int currentBlockIndex;

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    /// \brief The parent Model
    private: physics::LinkPtr link_;

    /// \brief The body of the frame to display pose, twist
    private: physics::LinkPtr reference_link_;


    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;
    private: boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;


    /// \brief ros message
    private: nav_msgs::Odometry pose_msg_;

    /// \brief store bodyname
    private: std::string link_name_;

    /// \brief store bodyname
    private: std::string visual_name_;

    /// \brief mutex to lock access to fields used in message callbacks
    private: boost::mutex lock;

    // rate control
    private: double update_rate_;

    private: common::Time last_time_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue ground_truth_odom_queue_;
    private: void QueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    private: unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;

    private: gazebo::transport::NodePtr node_;
    private: gazebo::transport::PublisherPtr pub_visual_;
    private: gazebo::event::ConnectionPtr updateConnection;
    private: gazebo::rendering::ScenePtr scene_ptr;
    private: gazebo::rendering::VisualPtr box_ptr ;
    private: size_t iterrations_ = 0;       
    public: float r,g,b,a;

    //private: bool OnServiceCallback(std_srvs::Empty::Request &req,
    // std_srvs::Empty::Response &res)
    private: ros::NodeHandle nh_;
    private: ros::ServiceServer set_color_srv_;
    private: bool setColorSrvCallback(gazebo_msgs::SetLightProperties::Request& request, gazebo_msgs::SetLightProperties::Response& response);
    private: std_msgs::ColorRGBA color;
  };
}
#endif
