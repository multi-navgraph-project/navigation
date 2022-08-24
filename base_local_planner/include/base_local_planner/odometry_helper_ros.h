/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/

#ifndef ODOMETRY_HELPER_ROS2_H_
#define ODOMETRY_HELPER_ROS2_H_

#include <nav_msgs/Odometry.h>
#include "robin_bridge_generated/EncoderData.h"
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>


namespace base_local_planner {

class OdometryHelperRos {
public:

  /** @brief Constructor.
   * @param odom_topic The topic on which to subscribe to Odometry
   *        messages.  If the empty string is given (the default), no
   *        subscription is done. */
  OdometryHelperRos(std::string odom_topic = "", std::string steering_topic = "");
  ~OdometryHelperRos() {}

  /**
   * @brief  Callback for receiving odometry data
   * @param msg An Odometry message
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void getOdom(nav_msgs::Odometry& base_odom);

  void SteerCallback(const robin_bridge_generated::EncoderData::ConstPtr& msg);
  void getSteeringAngle(std_msgs::Float64& steer_ang);
  
  

  void getRobotVel(geometry_msgs::PoseStamped& robot_vel);

  /** @brief Set the odometry topic.  This overrides what was set in the constructor, if anything.
   *
   * This unsubscribes from the old topic (if any) and subscribes to the new one (if any).
   *
   * If odom_topic is the empty string, this just unsubscribes from the previous topic. */
  void setOdomTopic(std::string odom_topic);
  void setSteeringTopic(std::string steering_topic);

  /** @brief Return the current odometry topic. */
  std::string getOdomTopic() const { return odom_topic_; }

private:
  //odom topic
  std::string odom_topic_;
  std::string steering_topic_;
  boost::mutex steering_mutex_;
  // we listen on odometry on the odom topic
  std_msgs::Float64 steering_;
  ros::Subscriber odom_sub_;
  ros::Subscriber steering_sub_;
  nav_msgs::Odometry base_odom_;
  boost::mutex odom_mutex_;
  
  // global tf frame id
  std::string frame_id_; ///< The frame_id associated this data
};

} /* namespace base_local_planner */
#define CHUNKY 1
#endif /* ODOMETRY_HELPER_ROS2_H_ */
