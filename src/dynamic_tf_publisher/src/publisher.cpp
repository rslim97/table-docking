#include <ros/ros.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <Eigen/Dense>
#include <thread>
#include <XmlRpcValue.h>

// This program publishes TF frame transformations with nonzero timestamps (dynamic).
// adapted from Orbbec SDK.

class TransformPublisherNode{
    public:
        TransformPublisherNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private, std::string reference_frame_name, std::string frame_name):
        nh_(nh),nh_private_(nh_private),reference_frame_name_(reference_frame_name),frame_name_(frame_name)
        {
            init();
        }
        // CameraTransformPublisherNode(const CameraTransformPublisherNode&) = delete;
        // CameraTransformPublisherNode& operator=(const CameraTransformPublisherNode&) = delete;
        // CameraTransformPublisherNode(CameraTransformPublisherNode&&) = delete;
        // CameraTransformPublisherNode& operator=(CameraTransformPublisherNode&&) = delete;
        // ~CameraTransformPublisherNode();
        // bool isInitialized() const;
        

    private:

          void init(){
            is_running_ = true;
            initParams();
            setupTopics();
            is_initialized_ = true;
          }
          
        bool isInitialized() const { return is_initialized_; }

        void setupTopics() {
            // setupPublishers();
            publishStaticTransforms();
        }

        void initParams(){
            // XmlRpc::XmlRpcValue rgb_frame_rot;
            // XmlRpc::XmlRpcValue rgb_frame_trans;
            XmlRpc::XmlRpcValue frame_rot;
            XmlRpc::XmlRpcValue frame_trans;
            // XmlRpc::XmlRpcValue base_footprint_frame_rot;
            // XmlRpc::XmlRpcValue base_footprint_frame_trans;

            // std::string rgb_frame_rotation_param("/rgb/frame/rot");
            // std::string rgb_frame_translation_param("/rgb/frame/trans");
            std::string frame_rotation_param("/"+frame_name_+"/frame/rot");
            std::string frame_translation_param("/"+frame_name_+"/frame/trans");
            // std::string base_footprint_frame_rotation_param("/odom/frame/rot");
            // std::string base_footprint_frame_translation_param("/odom/frame/trans");

            // if (!nh_private_.getParam(rgb_frame_rotation_param,rgb_frame_rot)){
            //     ROS_ERROR("Could not find %s parameter.",rgb_frame_rotation_param.c_str());
            // }
            // if (!nh_private_.getParam(rgb_frame_translation_param,rgb_frame_trans)){
            //     ROS_ERROR("Could not find %s parameter.",rgb_frame_translation_param.c_str());
            // }
            if (!nh_private_.getParam(frame_rotation_param,frame_rot)){
                ROS_ERROR("Could not find %s parameter.",frame_rotation_param.c_str());
            }
            if (!nh_private_.getParam(frame_translation_param,frame_trans)){
                ROS_ERROR("Could not find %s parameter.",frame_translation_param.c_str());
            }
            for (int i=0;i<frame_rot.size();i++){
                frame_rot_.push_back(double(frame_rot[i]));
            }
            for (int i=0;i<frame_trans.size();i++){
                frame_trans_.push_back(double(frame_trans[i]));
            }
            // ROS_INFO("frame_trans[0]: %f",frame_trans_[0]);
            // ROS_INFO("frame_trans[1]: %f",frame_trans_[1]);
            // ROS_INFO("frame_trans[2]: %f",frame_trans_[2]);

        }

        void publishStaticTF(const ros::Time& t, const tf2::Vector3& trans, const tf2::Quaternion& q,
            const std::string& from, const std::string& to)
        {
            geometry_msgs::TransformStamped msg;
            msg.header.stamp = t;
            msg.header.frame_id = from;
            msg.child_frame_id = to;
            msg.transform.translation.x = trans[0] ;
            msg.transform.translation.y = trans[1];
            msg.transform.translation.z = trans[2];
            msg.transform.rotation.x = q.getX();
            msg.transform.rotation.y = q.getY();
            msg.transform.rotation.z = q.getZ();
            msg.transform.rotation.w = q.getW();
            static_tf_msgs_.push_back(msg);
        }
    
        void calcAndPublishStaticTransform() {
            tf2::Quaternion quaternion, zero_rot, Q;
            zero_rot.setRPY(0.0, 0.0, 0.0);
            // quaternion.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
            quaternion.setRPY(0.0, 0.0, 0.0);
            tf2::Vector3 zero_trans(0.0, 0.0, 0.0);
            tf2::Vector3 trans(0.0, 0.0, 0.0);
            Q.setRPY(frame_rot_[0],
                     frame_rot_[1],
                     frame_rot_[2]);
            Q = quaternion * Q * quaternion.inverse();
            for (int i = 0; i < 3; i++) {
                trans[i] = frame_trans_[i];
            }
            auto tf_timestamp = ros::Time::now();
            
            // ROS_INFO("trans[0]: %f",trans[0]);
            // ROS_INFO("trans[1]: %f",trans[1]);
            // ROS_INFO("trans[2]: %f",trans[2]);

            // tf topics for camera
            // std::string reference_frame_ = ;
            // std::string camera_color_frame = camera_name_+"_rgb_frame";
            // std::string camera_color_optical_frame = camera_name_+"_rgb_optical_frame";

            publishStaticTF(tf_timestamp, trans, Q, reference_frame_name_, frame_name_);
            // publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, camera_color_frame, camera_color_optical_frame);

        }

        void publishDynamicTransforms() {
            ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", tf_publish_rate_);
            static std::mutex mu;
            std::unique_lock<std::mutex> lock(mu);
            while (ros::ok() && is_running_) {
                tf_cv_.wait_for(lock, std::chrono::milliseconds((int)(1000.0 / tf_publish_rate_)),
                                [this] { return (!(is_running_)); });
                {
                ros::Time t = ros::Time::now();
                for (auto& msg : static_tf_msgs_) {
                    msg.header.stamp = t;
                }
                // CHECK_NOTNULL(dynamic_tf_broadcaster_.get());
                dynamic_tf_broadcaster_->sendTransform(static_tf_msgs_);
                }
            }
        }

        void publishStaticTransforms() {
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
            dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
            calcAndPublishStaticTransform();
            if (tf_publish_rate_ > 0) {
                tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
            } else {
                // CHECK_NOTNULL(static_tf_broadcaster_.get());
                static_tf_broadcaster_->sendTransform(static_tf_msgs_);
            }
        }

        tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) {
            Eigen::Matrix3f m;
            // We need to be careful about the order, as RS2 rotation matrix is
            // column-major, while Eigen::Matrix3f expects row-major.
            m << rotation[0], rotation[3], rotation[6], rotation[1], rotation[4], rotation[7], rotation[2],
                rotation[5], rotation[8];
            Eigen::Quaternionf q(m);
            return {q.x(), q.y(), q.z(), q.w()};
        }


    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        bool publish_tf_ = true;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
        std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ = nullptr;
        std::vector<geometry_msgs::TransformStamped> static_tf_msgs_;
        double tf_publish_rate_ = 10.0;
        bool is_initialized_ = false;
        std::shared_ptr<std::thread> tf_thread_ = nullptr;
        std::condition_variable tf_cv_;
        std::atomic_bool is_running_{false};
        // std::vector<float> rgb_optical_frame_rot_;
        // std::vector<float> rgb_optical_frame_trans_;
        std::vector<float> frame_rot_;
        std::vector<float> frame_trans_;
        // std::vector<float> base_footprint_frame_rot_;
        // std::vector<float> base_footprint_frame_trans_;
        std::string reference_frame_name_;
        std::string frame_name_;

};

int main(int argc, char** argv){
    ros::init(argc,argv, "dynamic_tf_publisher_node");
    std::string reference_frame_name="";
    std::string frame_name="";
    if (argc>1)
        reference_frame_name = argv[1];
        frame_name = argv[2];
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    TransformPublisherNode dynamic_tf_publisher_node(nh, nh_private,reference_frame_name,frame_name);
    ros::spin();
    ros::shutdown();
    return 0;
}
