#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_srvs/SetBool.h>

// Include local planner
#include <teb_local_planner/teb_local_planner_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>

// Include global planner
#include <navfn/navfn_ros.h>
#include <carrot_planner/carrot_planner.h>

#include <costmap_2d/costmap_2d_ros.h>

// #include "table_docking_planner/dock_goal_updater.hpp"
#include "custom_msgs/DockingAction.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class Planner
{
public:
    Planner(std::string action_name_);
    // void sendCmd();

    ros::NodeHandle nh, nh_dockUpdater;

protected:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::string referenceFrame = "map";

    costmap_2d::Costmap2DROS* underTableCostmap;
    costmap_2d::Costmap2DROS* localCostmap;
    costmap_2d::Costmap2DROS* globalCostmap;
    teb_local_planner::TebLocalPlannerROS tebPlannerROS;
    dwa_local_planner::DWAPlannerROS dwaPlannerROS;
    navfn::NavfnROS navFnROS;
    carrot_planner::CarrotPlanner carrotPlannerROS;

    // set during each goalCB
    custom_msgs::DockingGoal currentGoal;
    geometry_msgs::PoseStamped currentGoalPose;   /**< USE THIS FOR Path Planning NOT currentGoal*/
    geometry_msgs::PoseStamped basePose;
    std::vector<geometry_msgs::PoseStamped> currentGlobalPlan;
    // bool moveFlag = false;
};

#endif