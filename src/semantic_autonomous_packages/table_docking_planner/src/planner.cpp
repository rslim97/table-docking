#include "table_docking_planner/planner.hpp"

Planner::Planner(std::string action_name):
tfListener(tfBuffer)
{
    // underTableCostmap = new costmap_2d::Costmap2DROS("under_table_costmap", tfBuffer);
    localCostmap = new costmap_2d::Costmap2DROS("local_costmap", tfBuffer);
    globalCostmap = new costmap_2d::Costmap2DROS("global_costmap", tfBuffer);

    // Might be better to change costmap for TebPlanner to local_costmap, 
    // if using carrot planner is able to plan a path close to goal
    // Initialize local planner
    // tebPlannerROS.initialize("localTebPlanner", &tfBuffer, underTableCostmap);
    dwaPlannerROS.initialize("localDwaPlanner", &tfBuffer, localCostmap);

    // Initialize global planner
    carrotPlannerROS.initialize("globalCarrotPlanner", globalCostmap);
    navFnROS.initialize("globalNavfnROS", globalCostmap);

    ROS_INFO("Planner initialized");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");
    ROS_INFO("planner_node is now running ...");

    Planner Planner("/custom_move_base");
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
