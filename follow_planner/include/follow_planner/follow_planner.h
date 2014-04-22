/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *                      2014, PAL Robotics, S.L.
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Paul Mathieu
 *********************************************************************/

#ifndef FOLLOW_PLANNER_H
#define FOLLOW_PLANNER_H

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <vector>

// forward declarations
namespace costmap_2d
{
  class Costmap2D;
  class Costmap2DROS;
}

namespace follow_planner
{

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the follow_planner planner which runs a
 *        very dumb goal follower.
 */

class FollowPlanner : public nav_core::BaseGlobalPlanner
{
public:

  /**
   * @brief  Default constructor for the PlannerCore object
   */
  FollowPlanner();

  /**
   * @brief  Constructor for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap A pointer to the costmap to use (ignored)
   * @param  frame_id Frame of the costmap
   */
  FollowPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

  /**
   * @brief  Initialization function for the PlannerCore object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   *                     for planning (ignored)
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose (ignored)
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose (ignored)
   * @param tolerance The tolerance on the goal point for the planner
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief  Publish a path for visualization purposes
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

  ~FollowPlanner()
  {}

protected:

  std::string frame_id_;
  ros::Publisher plan_pub_;
  bool initialized_, allow_unknown_, visualize_potential_;

private:

  void followCb(const geometry_msgs::PoseStamped& pose);

  double default_tolerance_;
  std::string tf_prefix_;
  boost::mutex mutex_;

  ros::Subscriber follow_sub_; /// will listen to a PoseStamped and follow it
  geometry_msgs::PoseStamped last_follow_pose_;
};

} //end namespace follow_planner

#endif
