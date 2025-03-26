#ifndef PERCEPTION_VISUALIZER
#define PERCEPTION_VISUALIZER

#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

class CloudVisualizer{
    public:
        CloudVisualizer(ros::NodeHandle nh)
            : nh_(nh) {
                cloud_sub_ = nh_.subscribe("/perception/visualize_cloud", 10, &CloudVisualizer::cloud_cb, this);

                PCL_viewer_ = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer("PCL Viewer"));
                PCL_viewer_-> setBackgroundColor(0.0,0.1,0.2);
                PCL_viewer_->addCoordinateSystem(1.0,"reference",0);
                display_count = 0;
            }

            void cloud_cb(const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
                ROS_INFO("Cloud received in visualizer... Visualizing cloud %d", display_count++);
                pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*cloud_msg,*input_cloud);

                PCL_viewer_->removeAllPointClouds();
                // add test data
                // // input_cloud->push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
                // // input_cloud->push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
                // // input_cloud->push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
                // // input_cloud->push_back (pcl::PointXYZ (rand (), rand (), rand ())); 
                std::cout<<"visualizer input_cloud->points.size()"<<input_cloud->points.size()<<std::endl;
                PCL_viewer_->addPointCloud<pcl::PointXYZ>(input_cloud, "input_cloud");
                // PCL_viewer_->resetCamera();
                PCL_viewer_->spinOnce(5000);

                return;
            }
    private:
        ros::NodeHandle nh_;
        ros::Subscriber cloud_sub_;
        int display_count;
        
        pcl::visualization::PCLVisualizer::Ptr PCL_viewer_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualizer_node");
    ros::NodeHandle nh;
    CloudVisualizer visualizer(nh);

    ros::Rate loop_rate(5);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#endif