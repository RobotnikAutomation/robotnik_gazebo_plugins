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

/*
 * Desc: 3D local velocity for ground truth
 * Author: Marc Bosch-Jorge
 * Date: 13 Feb 2020
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

#ifndef ROBOTNIK_GAZEBO_GROUND_TRUTH_ODOM_HH
#define ROBOTNIK_GAZEBO_GROUND_TRUTH_ODOM_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <tf/transform_broadcaster.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

namespace gazebo
{
  class RobotnikGazeboGroundTruthOdom : public ModelPlugin
  {
    /// \brief Constructor
    public: RobotnikGazeboGroundTruthOdom();

    /// \brief Destructor
    public: virtual ~RobotnikGazeboGroundTruthOdom();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

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

    /// \brief topic name
    private: std::string topic_name_;

    /// \brief frame transform name, should match link name
    /// FIXME: extract link name directly?
    private: std::string frame_name_;
    private: std::string tf_frame_name_;
    /// \brief base world link, should match link name
    private: std::string world_body_name_, world_frame_name_;

    /// \brief if has to broadcast tf
    private: bool broadcastTF_;

    /// \brief allow specifying constant xyz and rpy offsets
    private: ignition::math::Pose3d offset_;

    /// \brief mutex to lock access to fields used in message callbacks
    private: boost::mutex lock;

    /// \brief save last_time
    private: common::Time last_time_;
    private: ignition::math::Vector3d last_vpos_;
    private: ignition::math::Vector3d last_veul_;
    private: ignition::math::Vector3d apos_;
    private: ignition::math::Vector3d aeul_;
    private: ignition::math::Vector3d last_frame_vpos_;
    private: ignition::math::Vector3d last_frame_veul_;
    private: ignition::math::Vector3d frame_apos_;
    private: ignition::math::Vector3d frame_aeul_;

    // rate control
    private: double update_rate_;

    /// \brief Gaussian noise
    private: double gaussian_noise_;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu, double sigma);

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
  };
}
#endif
