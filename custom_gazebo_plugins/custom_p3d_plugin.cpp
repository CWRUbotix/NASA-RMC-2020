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

#include "custom_gazebo_ros_p3d.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(CustomGazeboRosP3D);

////////////////////////////////////////////////////////////////////////////////
// Constructor
CustomGazeboRosP3D::CustomGazeboRosP3D()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
CustomGazeboRosP3D::~CustomGazeboRosP3D()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->p3d_queue_.clear();
  this->p3d_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void CustomGazeboRosP3D::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("p3d", "p3d plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("p3d", "gazebo_ros_p3d plugin error: bodyName: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("p3d", "p3d plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("frameName"))
  {
    ROS_DEBUG_NAMED("p3d", "p3d plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  if (!_sdf->HasElement("xyzOffset"))
  {
    ROS_DEBUG_NAMED("p3d", "p3d plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
  }
  else
    this->offset_.Pos() = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();

  if (!_sdf->HasElement("rpyOffset"))
  {
    ROS_DEBUG_NAMED("p3d", "p3d plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.Rot() = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));
  }
  else
    this->offset_.Rot() = ignition::math::Quaterniond(_sdf->GetElement("rpyOffset")->Get<ignition::math::Vector3d>());

  if (!_sdf->HasElement("gaussianNoise"))
  {
    ROS_DEBUG_NAMED("p3d", "p3d plugin missing <gaussianNoise>, defaults to 0.0");
    this->gaussian_noise_ = 0;
  }
  else
    this->gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_DEBUG_NAMED("p3d", "p3d plugin missing <updateRate>, defaults to 0.0"
             " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("p3d", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->tf_frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<geometry_msgs::PoseWithCovarianceStamped>();
    this->pub_ =
      this->rosnode_->advertise<geometry_msgs::PoseWithCovarianceStamped>(this->topic_name_, 1);
  }

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif

  // if frameName specified is "/world", "world", "/map" or "map" report
  // back inertial values in the gazebo world
  if (this->frame_name_ != "/world" &&
      this->frame_name_ != "world" &&
      this->frame_name_ != "/map" &&
      this->frame_name_ != "map")
  {
    this->reference_link_ = this->model_->GetLink(this->frame_name_);
    if (!this->reference_link_)
    {
      ROS_ERROR_NAMED("p3d", "gazebo_ros_p3d plugin: frameName: %s does not exist, will"
                " not publish pose\n", this->frame_name_.c_str());
      return;
    }
  }

  // init reference frame state
  if (this->reference_link_)
  {
    ROS_DEBUG_NAMED("p3d", "got body %s", this->reference_link_->GetName().c_str());
  }


  // start custom queue for p3d
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&CustomGazeboRosP3D::P3DQueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&CustomGazeboRosP3D::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void CustomGazeboRosP3D::UpdateChild()
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
      ROS_WARN_NAMED("p3d", "Negative update time difference detected.");
      this->last_time_ = cur_time;
  }

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  if (this->pub_.getNumSubscribers() > 0)
  {
    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0)
    {
      this->lock.lock();

      if (this->topic_name_ != "")
      {
        // copy data into pose message
        this->pose_msg_.header.frame_id = this->tf_frame_name_;
        this->pose_msg_.header.stamp.sec = cur_time.sec;
        this->pose_msg_.header.stamp.nsec = cur_time.nsec;

        ignition::math::Pose3d pose, frame_pose;

        // get inertial Rates
        // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
        pose = this->link_->WorldPose();
#else
        pose = this->link_->GetWorldPose().Ign();
#endif

        // Apply Reference Frame
        if (this->reference_link_)
        {
          // convert to relative pose, rates
#if GAZEBO_MAJOR_VERSION >= 8
          frame_pose = this->reference_link_->WorldPose();
#else
          frame_pose = this->reference_link_->GetWorldPose().Ign();
#endif
          pose.Pos() = pose.Pos() - frame_pose.Pos();
          pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
          pose.Rot() *= frame_pose.Rot().Inverse();
        }

        // Apply Constant Offsets
        // apply xyz offsets and get position and rotation components
        pose.Pos() = pose.Pos() + this->offset_.Pos();
        // apply rpy offsets
        pose.Rot() = this->offset_.Rot()*pose.Rot();
        pose.Rot().Normalize();

        // Fill out messages
        this->pose_msg_.pose.pose.position.x    = pose.Pos().X();
        this->pose_msg_.pose.pose.position.y    = pose.Pos().Y();
        this->pose_msg_.pose.pose.position.z    = pose.Pos().Z();

        this->pose_msg_.pose.pose.orientation.x = pose.Rot().X();
        this->pose_msg_.pose.pose.orientation.y = pose.Rot().Y();
        this->pose_msg_.pose.pose.orientation.z = pose.Rot().Z();
        this->pose_msg_.pose.pose.orientation.w = pose.Rot().W();

        // fill in covariance matrix
        /// @todo: let user set separate linear and angular covariance values.
        double gn2 = this->gaussian_noise_*this->gaussian_noise_;
        this->pose_msg_.pose.covariance[0] = gn2;
        this->pose_msg_.pose.covariance[7] = gn2;
        this->pose_msg_.pose.covariance[14] = gn2;
        this->pose_msg_.pose.covariance[21] = gn2;
        this->pose_msg_.pose.covariance[28] = gn2;
        this->pose_msg_.pose.covariance[35] = gn2;

        // publish to ros
        this->pub_Queue->push(this->pose_msg_, this->pub_);
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double CustomGazeboRosP3D::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void CustomGazeboRosP3D::P3DQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->p3d_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}