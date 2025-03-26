#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H

#include <string>
#include <sensor_msgs/CameraInfo.h>

namespace perception{
    class CameraDetail
    {
    public:
        std::string cam_name = "/camera";

        CameraDetail();
        // ~CameraDetail();
        // CameraDetail(const std::string& cameraName);

        void camInfoCB(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg);
        std::string getRGBTopic();
        std::string getDepthTopic();
        // std::string getPCTopic();
        // std::string getCamInfoTopic();
        // std::string getCamName();

        // void setDepthTopic(const std::string& depthTopic);

        float fx  = 614.1568603515625;
        float fy  = 614.7573852539062;
        float ppx = 311.96697998046875;
        float ppy = 248.37994384765625;

    protected:
        // orbbec
        // std::string rgb_topic = "/color/image_raw";
        // std::string depth_topic = "/aligned_depth_to_color/image_raw";
        // std::string pc_topic = "/depth/color/points";
        // std::string cam_info_topic = "/color/camera_info";
        // std::string cam_name = "/camera";

        std::string rgb_topic = "/rgb/image_raw";
        std::string depth_topic = "/depth/image_raw";
        // std::string pc_topic = "/depth/color/points";
        std::string cam_info_topic = "/rgb/camera_info";

    };
}
#endif