/*
 * Copyright 2015 Stefan Kohlbrecher, TU Darmstadt
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
 * Desc: Simple model controller that uses a twist message to exert
 *       forces on a robot, resulting in motion. Prevents slipping of
 *       robot by forcing a pose when no command is received. Based on the
 *       gazebo_ros_force_based_move from Team Hector
 * Author: Marc Bosch-Jorge
 * Date: 06 August 2015
 */

#ifndef ROBOTNIK_FORCE_BASED_MOVE_HH
#define ROBOTNIK_FORCE_BASED_MOVE_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo {

  class RobotnikForceBasedMove : public ModelPlugin {

    public: 
      RobotnikForceBasedMove();
      ~RobotnikForceBasedMove();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChildBegin();
      virtual void UpdateChildEnd();
      virtual void InitChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);

      tf::Transform getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const;

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_begin_;
      event::ConnectionPtr update_connection_end_;

      /// \brief A pointer to the Link, where force is applied
      physics::LinkPtr link_;

      /// \brief The Link this plugin is attached to, and will exert forces on.
      private: std::string link_name_;

      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher odometry_pub_;
      ros::Subscriber vel_sub_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;

      tf::Transform odom_transform_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      double odometry_rate_;
      bool publish_odometry_tf_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // command velocity callback
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double x_;
      double y_;
      double rot_;
      bool alive_;

      common::Time last_odom_publish_time_;
#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Pose3d last_odom_pose_;
      ignition::math::Pose3d last_valid_pose_;
#else
      math::Pose last_odom_pose_;
      math::Pose last_valid_pose_;
#endif
      
      double torque_yaw_velocity_p_gain_;
      double force_x_velocity_p_gain_;
      double force_y_velocity_p_gain_;

      ros::Time command_last_stamp_;
      ros::Duration command_watchdog_;
      bool command_watchdog_shown_;
  };

}

#endif /* end of include guard: ROBOTNIK_PLANAR_MOVE_HH */
