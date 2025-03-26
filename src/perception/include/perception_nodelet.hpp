#ifndef PERCEPTION_NODELET_H
#define PERCEPTION_NODELET_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/Vertices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16.h>
#include <actionlib/server/simple_action_server.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include<nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>

#include "custom_msgs/Bboxes.h"
#include "custom_msgs/Bbox.h"
#include "custom_msgs/Detect.h"
#include <custom_msgs/DetectionAction.h>

#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <unistd.h>

// #include "tablegrid_nodelet.hpp"
#include "camera_detail.hpp"

namespace perception{
    class PerceptionNodelet: public nodelet::Nodelet
    {

        public:
            PerceptionNodelet() {};
            ~PerceptionNodelet() {};

            void detection_cb(const custom_msgs::DetectionGoalConstPtr &goal);

            bool filter_point_cloud_by_point(
                const std::uint16_t point_x_input,
                const std::uint16_t point_y_input,
                // const cv::Mat& depth_img, 
                const sensor_msgs::ImageConstPtr& depth_input,
                const sensor_msgs::PointCloud2::ConstPtr& voxel_input,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& nan_filtered_point_cloud,
                custom_msgs::DetectionFeedback& as_feedback
            );

            bool get_table_docking_pose(
                std::uint16_t, 
                std::uint16_t, 
                // const cv::Mat&, 
                const sensor_msgs::ImageConstPtr&,
                const sensor_msgs::ImageConstPtr&, 
                const sensor_msgs::PointCloud2::ConstPtr&,
                bool, 
                // std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr >&
                custom_msgs::BboxesPtr&,
                pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                custom_msgs::DetectionFeedback&
            );

            bool filter_point_cloud_by_bbox(
                custom_msgs::Bboxes&, 
                // const cv::Mat&, 
                const sensor_msgs::ImageConstPtr&, 
                const sensor_msgs::PointCloud2::ConstPtr&,
                bool, 
                std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr >&
            );
            /* 
                Convert all bbox 2D points to 3D points
            */
            void deproject(int, int , float z, pcl::PointXYZ&);
            /*
                deproject 2D points x,y and into pcl::PointXYZ.
            */

            void publishPointMarker(geometry_msgs::Point& p, float r, float g, float b, float scale, int marker_id);
            void publishArrowMarker(geometry_msgs::Pose& p, float r, float g, float b, int marker_id);
            void deleteAllMarker();

            bool checkClsList(const std::string&, std::vector<std::string>&);

            bool get_table_clouds_combined(const sensor_msgs::ImageConstPtr& rgb_input, 
                               const sensor_msgs::ImageConstPtr& depth_input, 
                                const sensor_msgs::PointCloud2::ConstPtr& voxel_input,
                                custom_msgs::BboxesPtr& table_bboxes,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr table_clouds_combined);
            
            float getMedianOfBox(const cv::Mat& depthImage, int x, int y, int m);
            float getMeanOfBox(const cv::Mat& depthImage, int x, int y, int m);
            
            void projectToPlane(
                pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_projected,
                float table_height
            );           

            void getConvexHull(
                pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_convex_hull,
                std::vector<pcl::Vertices>& polygons
            );

            bool process_laser_img(
                const sensor_msgs::Image&, 
                const sensor_msgs::Image&, 
                const sensor_msgs::LaserScan&
            );

            /*
                Process the image saved from callbacks
            */


        private:
            virtual void onInit();

            void tableCostmapCB(
                const nav_msgs::OccupancyGrid::ConstPtr& tableCostmapMsg
            );

            bool update_cam2map_transform(const std_msgs::Header&);
            bool update_wheelchair2map_transform();

            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            ros::Subscriber info_sub;
            ros::Subscriber voxel_point_cloud_sub;
            ros::ServiceClient FasterRCNN_client;
            std::unique_ptr<actionlib::SimpleActionServer<custom_msgs::DetectionAction>> as_;
            ros::Time srv_startT; 
            ros::Publisher marker_array_pub_;
            ros::Publisher marker_pub_;
            std::shared_ptr<visualization_msgs::MarkerArray> marker_array_ptr_ = nullptr;
            ros::Publisher cloud_pub_;
            ros::Publisher table_bboxes_pub_;
            ros::Publisher table_docking_pose_pub_;
            laser_geometry::LaserProjection projector_;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_=nullptr;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
            std::string laser_topic_    = "/scan";
            geometry_msgs::PoseStamped actual_pose;
            geometry_msgs::TransformStamped cam2map_, pc2map_, wheelchair2map_, laser2map_;
            float fx  = 614.1568603515625;
            float fy  = 614.7573852539062;
            float ppx = 311.96697998046875;
            float ppy = 248.37994384765625;
            // camera intrinsic parameters
            std::string reference_frame_id_ = "map";
            std::vector<std::string> door_class_list; // = {"door", "garage door", "doorway", "shower door", "doors", "door frame", "glass door"};
            std::vector<std::string> table_class_list; //= {"table"};
            bool map_flag_;
            bool lidar_flag_;
            float camera_height_;
            bool segmentation_flag_;
            //rosparam
            std::unique_ptr<CameraDetail> rightCam;
            std::unique_ptr<CameraDetail> leftCam;

    };
}

PLUGINLIB_EXPORT_CLASS(perception::PerceptionNodelet, nodelet::Nodelet);

#endif
