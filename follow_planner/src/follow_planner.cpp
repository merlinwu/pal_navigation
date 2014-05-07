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

#include "follow_planner/follow_planner.h"

#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseFollowPlanner plugin
PLUGINLIB_EXPORT_CLASS(follow_planner::FollowPlanner, nav_core::BaseGlobalPlanner)

namespace
{

void makeStraightPath(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan)
{
    // TODO: check timestamp of the goal

    // simplest thing ever
    plan.clear();
    plan.push_back(start);
    plan.push_back(goal);
}

} // unnamed namnespace

namespace follow_planner
{

FollowPlanner::FollowPlanner()
  : initialized_(false)
  , allow_unknown_(true)
{}

FollowPlanner::FollowPlanner(std::string name,
                             costmap_2d::Costmap2D* costmap,
                             std::string frame_id)
  : initialized_(false)
  , allow_unknown_(true)
{
    //initialize the planner
    initialize(name, costmap, frame_id);
}

void FollowPlanner::initialize(std::string name,
                               costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void FollowPlanner::initialize(std::string name,
                               costmap_2d::Costmap2D* /* unused */,
                               std::string frame_id)
{
  if (!initialized_)
  {
    ros::NodeHandle private_nh("~/" + name);
    frame_id_ = frame_id;

    ros::NodeHandle move_base_nh("~");
    follow_sub_ = move_base_nh.subscribe("follow_goal", 100, &FollowPlanner::followCb, this);

    private_nh.param("default_tolerance", default_tolerance_, 0.0);


    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 10);

    //get the tf prefix
    ros::NodeHandle prefix_nh;
    tf_prefix_ = tf::getPrefixParam(prefix_nh);

    initialized_ = true;
  }
  else
  {
    ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }
}

void FollowPlanner::followCb(const geometry_msgs::PoseStamped& pose)
{
  last_follow_pose_ = pose;
}

bool FollowPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
    return makePlan(start, goal, default_tolerance_, plan);
}

bool FollowPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                             const geometry_msgs::PoseStamped& goal,
                             double tolerance,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_)
    {
      ROS_ERROR("This planner has not been initialized yet, but it is being "
                "used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame))
    {
        ROS_ERROR("The start pose passed to this planner must be in the "
                  "%s frame.  It is instead in the %s frame.",
                  tf::resolve(tf_prefix_, global_frame).c_str(),
                  tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    if (last_follow_pose_.header.frame_id == "")
    {
      ROS_ERROR("Follow planner never received a goal to follow, or its frame_id was empty");
      return false;
    }

    //make plan from start and goal
    makeStraightPath(start, last_follow_pose_, plan);

    //publish the plan for visualization purposes
    publishPlan(plan);
    return !plan.empty();
}

void FollowPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being "
              "used, please call initialize() before use");
    return;
  }

  //create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  if (!path.empty())
  {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
  }

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < path.size(); i++)
  {
    gui_path.poses[i] = path[i];
  }

  plan_pub_.publish(gui_path);
}

} //end namespace follow_planner

