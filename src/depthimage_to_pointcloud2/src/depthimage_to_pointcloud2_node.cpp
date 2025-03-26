#include <ros/ros.h>
#include <iostream>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Transform.h>
// #include <tf2/LinearMath/Vector3.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <depthimage_to_pointcloud2/depth_conversions.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <condition_variable>

#include <limits>
#include <memory>
#include <string>
// #include <Eigen/Dense>
#include <thread>
// #include <XmlRpcValue.h>

// This program publishes TF frame transformations with nonzero timestamps (dynamic).
// adapted from Orbbec SDK.

class DepthImagetoPointCloud2{
    public:
        DepthImagetoPointCloud2(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::string camera_name):
        nh_(nh),nh_private_(nh_private),camera_name_(camera_name),
        camera_info_topic_param_("/"+camera_name+"/depth/camera_info"),
        depth_image_topic_param_("/"+camera_name+"/depth/image_raw"),
        pointcloud2_topic_param_("/"+camera_name+"/pointcloud2")
        {
            init();
        }

        private:

          void init(){
            is_running_ = true;
            camerainfo_subscriber = nh_.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_param_, 1, &DepthImagetoPointCloud2::infoCb,this);
            depthimage_subscriber = nh_.subscribe<sensor_msgs::Image>(depth_image_topic_param_, 1, &DepthImagetoPointCloud2::depthCb,this);
            pointcloud2_publisher = nh_.advertise<sensor_msgs::PointCloud2>(pointcloud2_topic_param_,1);
            is_initialized_ = true;
          }
          
        bool isInitialized() const { return is_initialized_; }

        void depthCb(const sensor_msgs::ImageConstPtr& depth_image)
        {
            // ROS_INFO("depthCb");
            if (camera_info_ == nullptr) {
                ROS_WARN("No [%s] camera info, skipping point cloud conversion",camera_name_.c_str());
                return;
            }

            // sensor_msgs::PointCloud2ConstPtr cloud_msg = std::make_shared<sensor_msgs::PointCloud2>(); //? already allocated memory
            // sensor_msgs::PointCloud2ConstPtr cloud_msg()

            // boost::shared_ptr<::sensor_msgs::PointCloud2 > cloud_msg(new sensor_msgs::PointCloud2);
            boost::shared_ptr<::sensor_msgs::PointCloud2 > cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();


            cloud_msg->header = depth_image->header;
            cloud_msg->height = depth_image->height;
            cloud_msg->width = depth_image->width;
            cloud_msg->is_dense = false;
            cloud_msg->is_bigendian = false;
            cloud_msg->fields.clear();
            cloud_msg->fields.reserve(2);

            sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
            pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

            image_geometry::PinholeCameraModel model;
            model.fromCameraInfo(camera_info_);

            if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                depth_image_proc::convert<uint16_t>(depth_image,cloud_msg,model);
            } else if (depth_image->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                depth_image_proc::convert<float>(depth_image,cloud_msg,model);
            } else {
                ROS_WARN("Depth image has unsupported encoding [%s]", depth_image->encoding.c_str());
                return;
            }

            // tf_thread_ = std::make_shared<std::thread>([this,&cloud_msg]() { publishPointCloud2(cloud_msg); });  //publish point cloud

            pointcloud2_publisher.publish(*cloud_msg);

        }

        void infoCb(const sensor_msgs::CameraInfoConstPtr& info) {
            camera_info_=info;
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        bool publish_tf_ = true;
        ros::Subscriber camerainfo_subscriber;
        ros::Subscriber depthimage_subscriber;
        ros::Publisher pointcloud2_publisher;
        double tf_publish_rate_ = 10.0;
        bool is_initialized_ = false;
        std::shared_ptr<std::thread> tf_thread_ = nullptr;
        std::condition_variable tf_cv_;
        std::atomic_bool is_running_{false};
        float range_max_;
        bool use_quiet_nan_;
        std::string camera_info_topic_param_;
        std::string depth_image_topic_param_;
        std::string pointcloud2_topic_param_;
        sensor_msgs::CameraInfo::ConstPtr camera_info_;
        std::string camera_name_;


};

int main(int argc, char** argv){
    ros::init(argc,argv, "depthimage_to_pointcloud2_node");
    std::string camera_name="";
    if (argc>1)
        camera_name = argv[1];
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    DepthImagetoPointCloud2 depthimage_to_pointcloud2_node(nh, nh_private,camera_name);
    ros::spin();
    ros::shutdown();
    return 0;
}
