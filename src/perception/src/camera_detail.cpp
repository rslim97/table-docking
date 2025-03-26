#include "camera_detail.hpp"

namespace perception{
    CameraDetail::CameraDetail()
    {
    }

    // CameraDetail::~CameraDetail()
    // {
    // }

    // CameraDetail::CameraDetail(const std::string& cameraName)
    // {
    //     this->cam_name = cameraName;
    //     // return;
    // }

    std::string CameraDetail::getRGBTopic()
    {
        std::string output= this->cam_name + this->rgb_topic;
        return output;
    }

    std::string CameraDetail::getDepthTopic()
    {
        std::string output = this->cam_name + this->depth_topic;
        return output;
    }

    // std::string CameraDetail::getPCTopic()
    // {
    //     std::string output = this->cam_name + this->pc_topic;
    //     return output;
    // }

    // std::string CameraDetail::getCamInfoTopic()
    // {
    //     std::string output = this->cam_name + this->cam_info_topic;
    //     return output;
    // }

    // std::string CameraDetail::getCamName()
    // {
    //     std::string output = this->cam_name;
    //     return output;
    // }



    // void CameraDetail::setDepthTopic(const std::string& depthTopic)
    // {
    //     this->depth_topic = depthTopic;
    // }

    void CameraDetail::camInfoCB(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg)
    {
        if(cam_info_msg == nullptr)
            return;

        this->fx = cam_info_msg->K[0];
        this->fy = cam_info_msg->K[4];
        this->ppx = cam_info_msg->K[2];
        this->ppy = cam_info_msg->K[5];
        
        return;
    }
}