#ifndef PERCEPTION_PERCEPTION_NODE
#define PERCEPTION_PERCEPTION_NODE

#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"perception_node");
    ros::param::set("/perception_node/num_worker_threads",2);

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    // std::string nodelet_name = ros::this_node::getName();
    nodelet.load("perception","perception/PerceptionNodelet",remap,nargv);
    // nodelet.load("tablegrid","perception/TableGridNodelet",remap,nargv);
    ros::spin();
    return 0;
}

#endif