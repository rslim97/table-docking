#include "perception_nodelet.hpp"
#include <chrono>

namespace perception {
    void PerceptionNodelet::onInit()
    {
        ros::NodeHandle& nh_ = getMTNodeHandle();
        ros::NodeHandle& private_nh_ = getMTPrivateNodeHandle();

        // Initialize Subscriber
        ros::Subscriber costmap_sub = nh_.subscribe("/docking_planner/table_costmap/costmap", 10, &PerceptionNodelet::tableCostmapCB, this);
        
        // Initialize Service Client
        FasterRCNN_client = nh_.serviceClient<custom_msgs::Detect>(
            "faster_rcnn_srv"
        );
        // Initialize Publisher
        cloud_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/perception/visualize_cloud",10);
        table_bboxes_pub_ = nh_.advertise<custom_msgs::Bboxes>("/perception/table_bboxes",1);
        table_docking_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
        marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/perception/visualization_marker",5);
        marker_array_ptr_ = std::make_shared<visualization_msgs::MarkerArray>();

        rightCam = std::unique_ptr<CameraDetail>(new CameraDetail);
        leftCam = std::unique_ptr<CameraDetail>(new CameraDetail);

        rightCam->cam_name = "camera_right";
        leftCam->cam_name = "camera_left";

        // std::string sim_flag_param("sim_flag");
        std::string map_flag_param("map_flag");
        std::string lidar_flag_param("lidar_flag");
        std::string segmentation_flag_param("segmentation_flag");
        // std::string camera_height_param("wc_param/camera_height");

        // if (!private_nh_.getParam(sim_flag_param, ))
        if (!private_nh_.getParam(map_flag_param, map_flag_)) {
            NODELET_ERROR("Could not find %s parameter.", map_flag_param.c_str());
        }
        if (!private_nh_.getParam(lidar_flag_param, lidar_flag_)) {
            NODELET_ERROR("Could not find %s parameter.", lidar_flag_param.c_str());
        }
        if (!private_nh_.getParam(segmentation_flag_param, segmentation_flag_)) {
            NODELET_ERROR("Could not find %s parameter.", segmentation_flag_param.c_str());
        }
        reference_frame_id_=(map_flag_)?"map":"base_link";

        as_ = std::unique_ptr<actionlib::SimpleActionServer<custom_msgs::DetectionAction>> (new actionlib::SimpleActionServer<custom_msgs::DetectionAction>
                (private_nh_,"detection_as", boost::bind(&PerceptionNodelet::detection_cb, this, _1), false));

        door_class_list={"door", "garage door", "doorway", "shower door", "doors", "door frame", "glass door"};
        table_class_list= {"table"};

        NODELET_INFO("before getting_cam info");

        sensor_msgs::CameraInfo::ConstPtr rightCameraInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            "/camera_right/rgb/camera_info",
            nh_,
            ros::Duration(3.0)
        );
        rightCam->camInfoCB(rightCameraInfoPtr);

        sensor_msgs::CameraInfo::ConstPtr leftCameraInfoPtr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
            "/camera_left/rgb/camera_info",
            nh_,
            ros::Duration(3.0)
        );
        leftCam->camInfoCB(leftCameraInfoPtr);
        
        NODELET_INFO("after getting_cam info");

        as_->start();

        tf_buffer_ = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        NODELET_INFO("detection running");
    }



    void PerceptionNodelet::tableCostmapCB(
        const nav_msgs::OccupancyGrid::ConstPtr& tableCostmapMsg
    )
    {
        update_wheelchair2map_transform();
    }

    bool PerceptionNodelet::filter_point_cloud_by_point(
        const std::uint16_t point_x_input,
        const std::uint16_t point_y_input,
        // const cv::Mat& depth_img, 
        const sensor_msgs::ImageConstPtr& depth_input,
        const sensor_msgs::PointCloud2::ConstPtr& voxel_input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& nan_filtered_point_cloud,
        custom_msgs::DetectionFeedback& as_feedback
    )
    {
        // convert ros msg to cv
        cv_bridge::CvImagePtr depth_image_ptr;
        try
        {
            depth_image_ptr = cv_bridge::toCvCopy(
                depth_input, 
                sensor_msgs::image_encodings::TYPE_16UC1
            );
        }
        catch (cv_bridge::Exception& e)
        {
            NODELET_ERROR("cv_bridge exception 1: %s", e.what());
            as_feedback.status = "Depth image conversion error.";
            return false;
        }
        cv::Mat depth_img = depth_image_ptr->image;

        // uncomment to discard the use of bounding box to filter out point cloud and change table_clouds_combined for subsequent kdTree into voxel_pcl
        // convert voxel grid from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_pcl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*voxel_input,*voxel_pcl);

        // visualize voxel pcl
        // sensor_msgs::PointCloud2 cloud_out_msg;
        // pcl::toROSMsg(*voxel_pcl, cloud_out_msg);
        // cloud_out_msg.header.stamp = ros::Time::now();
        // cloud_out_msg.header.frame_id = reference_frame_id_;
        // cloud_pub_.publish(cloud_out_msg);

        std::cout<<"Create KdTree."<<std::endl;
        // create a KD-tree for cloud
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_1;
        if (voxel_pcl->points.size()>0)
            kdtree_1.setInputCloud(voxel_pcl);
        else
        {
            as_feedback.status = "Error KdTree is empty.";
            return false;
        }

        float nearest_neighbor_distance_threshold=1.5;  // 2.5 m

        pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ point{};

        /**
         * In Pyqt as well as Opencv, the (x,y) coordinates are given by
            -------------> x
            |
            |
            |
            | y
            v
         */
        std::cout<<"Get input point."<<std::endl;
        int x=point_x_input;
        int y=point_y_input;
        // std::cout<<"x: "<<x<<"y: "<<y<<std::endl;

        // std::cout<<"depth_img.cols "<<depth_img.cols<<", depth_img.rows"<<depth_img.rows<<std::endl;
        // remove depth values in invalid regions
        if((x <0 || y < 0) || (x >= depth_img.cols || y >= depth_img.rows ))
        {
            as_feedback.status = "Invalid depth image region.";
            return false;
        }
        // deal with depth image with zero value at clicked point
        // if (depth_img.at<uint16_t>(y,x) == 0)
        //     return false;
        // remove depth values more than some determined depth value
        std::cout<<"depth value at (y,x) :"<<depth_img.at<uint16_t>(y,x)<<std::endl;

        // access element in cv::Mat using (row,col) indexing
        if (depth_img.at<uint16_t>(y, x) * 0.001 > 5.0)
        {
            as_feedback.status = "Invalid depth value.";
            return false;
        }
        int m_median = 60;
        float depth_median = getMedianOfBox(depth_img,x,y,m_median);
        int m_mean = 120;
        float depth_mean = getMeanOfBox(depth_img,x,y,m_mean);

        if (depth_median==-1)
        {
            // reset table_bboxes
            as_feedback.status = "Invalid median depth value.";
            return false;
        }
        if (depth_mean==-1)
        {
            // reset table_bboxes
            as_feedback.status = "Invalid mean depth value.";
            return false;
        }

        float depth_val = 0.7*depth_median+0.3*depth_mean;
        std::cout<<"depth_val :"<<depth_val<<std::endl;
        if (depth_val>10000){
            as_feedback.status = "Invalid depth value.";
            return false;
        }
        // deproject a single depth point clicked by user
        deproject(x, y, depth_val*0.001, point);

        (*input_point_cloud).push_back(point);
        // use obtained cloud from depth img to filter voxel input
        update_cam2map_transform(depth_input->header);

        /// transform depth point cloud from camera frame to map frame
        tf::Transform cam2mapTF;
        tf::transformMsgToTF(cam2map_.transform, cam2mapTF);
        pcl_ros::transformPointCloud(
            *input_point_cloud, 
            *input_point_cloud,  // overwrite
            cam2mapTF
        );

        // placeholder variable for filtered voxel result
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxels_near_input_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        // for (size_t i=0; i<input_point_cloud->points.size(); ++i){
        if (input_point_cloud->points.size()==0)
        {
            as_feedback.status = "Input cloud is empty.";
            return false;
        }
        // deprojected input point using depth
        pcl::PointXYZ searchPoint = input_point_cloud->points[0];

        // publish deprojected input point as marker in Rviz
        geometry_msgs::Point input_point_marker;
        input_point_marker.x = searchPoint.x;
        input_point_marker.y = searchPoint.y;
        input_point_marker.z = searchPoint.z;
        publishPointMarker(input_point_marker,  0.0f, 1.0f, 0.0f, 0.2, 0);
        

        // search for the nearest neighbor
        int K=1000000;
        std::vector<int> nearestIndices(K);
        std::vector<float> nearestDistances(K);
        if (kdtree_1.nearestKSearch(searchPoint,K,nearestIndices,nearestDistances)>0){
            for (std::size_t j=0;j<nearestIndices.size();++j)
            {
                if (nearestDistances[j]<nearest_neighbor_distance_threshold){                                    
                    voxels_near_input_point_cloud->points.push_back(voxel_pcl->points[nearestIndices[j]]);
                }  
            }                          
            
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_voxels_near_input_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*voxels_near_input_point_cloud,*nan_filtered_voxels_near_input_point_cloud,indices);
        std::cout<<"nan_filtered_voxels_near_input_point_cloud.points.size() :"<<nan_filtered_voxels_near_input_point_cloud->points.size()<<std::endl;
        
        // no points nearby to input point cloud
        if (nan_filtered_voxels_near_input_point_cloud->points.size()==0){
            as_feedback.status = "No points found near input point cloud.";
            return false;
        }

        // // visualize voxel pcl
        // sensor_msgs::PointCloud2 cloud_out_msg;
        // pcl::toROSMsg(*nan_filtered_voxels_near_input_point_cloud, cloud_out_msg);
        // cloud_out_msg.header.stamp = ros::Time::now();
        // cloud_out_msg.header.frame_id = reference_frame_id_;
        // cloud_pub_.publish(cloud_out_msg);

        pcl::copyPointCloud(*nan_filtered_voxels_near_input_point_cloud, *nan_filtered_point_cloud);
        return true;
    }

    bool PerceptionNodelet::get_table_docking_pose(
        const std::uint16_t point_x_input,
        const std::uint16_t point_y_input,
        // const cv::Mat& depth_img, 
        const sensor_msgs::ImageConstPtr& rgb_input,
        const sensor_msgs::ImageConstPtr& depth_input,
        const sensor_msgs::PointCloud2::ConstPtr& voxel_input,
        bool wall_mean, 
        custom_msgs::BboxesPtr& table_bboxes,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_point_cloud,
        custom_msgs::DetectionFeedback& as_feedback
    )
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr table_clouds_combined(new pcl::PointCloud<pcl::PointXYZ>);

        // detect_objects
        bool is_detect_success;
        is_detect_success = get_table_clouds_combined(rgb_input, depth_input, voxel_input, table_bboxes, table_clouds_combined);

        if (!is_detect_success)
        {
            NODELET_WARN("No tables detected by FasterRCNN.");
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_voxels_near_input_point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        bool is_filter_success = filter_point_cloud_by_point(point_x_input, point_y_input, 
                                                            depth_input, voxel_input, 
                                                            nan_filtered_voxels_near_input_point_cloud, as_feedback);
        if (!is_filter_success)
        {
            return false;
        }

        // // visualize voxel pcl
        // sensor_msgs::PointCloud2 cloud_out_msg;
        // pcl::toROSMsg(*nan_filtered_voxels_near_input_point_cloud, cloud_out_msg);
        // cloud_out_msg.header.stamp = ros::Time::now();
        // cloud_out_msg.header.frame_id = reference_frame_id_;
        // cloud_pub_.publish(cloud_out_msg);

        // project point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        float table_height = 0.5;
        projectToPlane(nan_filtered_voxels_near_input_point_cloud, plane_cloud, table_height);

        // // visualize voxel pcl
        // sensor_msgs::PointCloud2 cloud_out_msg;
        // pcl::toROSMsg(*plane_cloud, cloud_out_msg);
        // cloud_out_msg.header.stamp = ros::Time::now();
        // cloud_out_msg.header.frame_id = reference_frame_id_;
        // cloud_pub_.publish(cloud_out_msg);

        // apply euclidean clustering to projected point cloud
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(plane_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.1);  // 10cm
        ec.setMinClusterSize(20);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(plane_cloud);
        ec.extract(cluster_indices);
        std::cout<<"cluster_indices.size() "<<cluster_indices.size()<<std::endl;
        int i=0;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster_vector;

        for (const auto& cluster: cluster_indices)
        {
            std::cout<<std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx: cluster.indices){
                cloud_cluster->push_back((*plane_cloud)[idx]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            std::cout<<"cloud_cluster :"<<i<<" size :"<<cloud_cluster->size()<<std::endl;
            cloud_cluster_vector.push_back(cloud_cluster);

            // // compute cloud cluster centroid
            // Eigen::Vector4f centroid;
            // pcl::compute3DCentroid(*cloud_cluster, centroid);
            // point_cloud_centroid_vector.push_back(centroid);
            // std::cout<<"centroid.x :"<<centroid[0]<<"centroid.y :"<<centroid[1]<<"centroid.z :"<<centroid[2]<<std::endl;

            // // compute distance from cluster centroid to input point
            // // std::cout<<"searchPoint.x :"<<searchPoint.x<<std::endl;
            // float distance_x = centroid[0] - searchPoint.x;
            // float distance_y = centroid[1] - searchPoint.y;
            // float distance_z = centroid[2] - searchPoint.z;
            // float centroid2inputpointcloud_distance = pow(distance_x,2) + pow(distance_y,2) + pow(distance_z,2);
            // std::cout<<"centroid2inputpointcloud_distance :"<<centroid2inputpointcloud_distance<<std::endl;
            // // if (centroid2inputpointcloud_distance<clustering_distance_threshold){
            // centroid2inputpointcloud_distance_vector.push_back(centroid2inputpointcloud_distance);
            // // }
            // i++;
        }
        if (cloud_cluster_vector.size()==0){
            as_feedback.status="No cloud clusters detected.";
            return false;
        }

        // visualize voxel pcl
        sensor_msgs::PointCloud2 cloud_out_msg;
        pcl::toROSMsg(*cloud_cluster_vector[0], cloud_out_msg);
        cloud_out_msg.header.stamp = ros::Time::now();
        cloud_out_msg.header.frame_id = reference_frame_id_;
        cloud_pub_.publish(cloud_out_msg);

        // get cloud inliers in a convex hull
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_convex_hull(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<pcl::Vertices> polygons;
        getConvexHull(cloud_cluster_vector[0], cloud_convex_hull, polygons);

        // for (const auto& point : cloud_convex_hull->points) {
        //     std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;
        // }
        
        // // visualize voxel pcl
        // sensor_msgs::PointCloud2 cloud_out_msg;
        // pcl::toROSMsg(*cloud_convex_hull, cloud_out_msg);
        // cloud_out_msg.header.stamp = ros::Time::now();
        // cloud_out_msg.header.frame_id = reference_frame_id_;
        // cloud_pub_.publish(cloud_out_msg);

        std::cout << " pause "<< std::endl;
    
        for (const auto& polygon: polygons){
            for (const auto& index: polygon.vertices){
                const auto& point = cloud_convex_hull->points[index];
                std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;
            }
        }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_convex_hull, centroid);
        geometry_msgs::Point centroid_point;
        centroid_point.x = centroid[0];
        centroid_point.y = centroid[1];
        centroid_point.z = centroid[2];
        publishPointMarker(centroid_point,  1.0f, 0.0f, 0.0f, 0.5, 1);

        // Compute arrows from convex hull points the centroid
        geometry_msgs::PoseArray docking_poses;
        docking_poses.header.stamp = ros::Time::now();
        docking_poses.header.frame_id = reference_frame_id_;
        for (const auto& point : cloud_convex_hull->points) {
            // std::cout<<"centroid.x-x"<<centroid_point.x-point.x<<"centroid.y-point.y"<<centroid_point.y-point.y<<"centroid.z-point.z"<<centroid_point.z-point.z<<std::endl;
            geometry_msgs::Pose pose;
            float ka = 2.5;
            pose.position.x = centroid_point.x + ka*(point.x - centroid_point.x);
            pose.position.y = centroid_point.y + ka*(point.y - centroid_point.y);
            pose.position.z = centroid_point.z + ka*(point.z - centroid_point.z);
            float direction_x = centroid_point.x - pose.position.x;
            float direction_y = centroid_point.y - pose.position.y;
            float theta = std::atan2(direction_y,direction_x);
            // euler to quaternion
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = std::sin(theta/2);
            pose.orientation.w = std::cos(theta/2);
            docking_poses.poses.push_back(pose);
        }
        
        // // sort points in lexicograhic order
        // std::sort(docking_poses.poses.begin(), docking_poses.poses.end() , [](const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2){
        //     if (p1.position.x != p2.position.x)
        //         return p1.position.x < p2.position.x; // compare by x first
        //     return p1.position.y < p2.position.y; // then compare by y
        // });

        // sort points in lexicograhic order
        std::sort(docking_poses.poses.begin(), docking_poses.poses.end() , [](const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2){
            if (p1.orientation.z != p2.orientation.z)
                return p1.orientation.z < p2.orientation.z; // compare by x first
            return p1.orientation.w < p2.orientation.w; // then compare by y
        });

        geometry_msgs::Pose median_pose;
        // find the median point
        size_t docking_poses_size = docking_poses.poses.size();
        if (docking_poses_size%2==1){
            median_pose = docking_poses.poses[docking_poses_size/2];
        } else {
            median_pose = docking_poses.poses[docking_poses_size/2-1];
        }

        // compute angle between median_pose and wheelchair pose to rule out infeasible wheelchair pose
        // https://stackoverflow.com/questions/57063595/how-to-obtain-the-angle-between-two-quaternions
        // https://math.stackexchange.com/questions/1375194/multiplication-of-quaternion-vectors
        update_wheelchair2map_transform();
        // q wheelchair - q median_pose
        float a1, b1, c1, d1;
        float a2, b2, c2, d2;
        float a3, b3, c3, d3;
        a1 = median_pose.orientation.w;
        b1 = median_pose.orientation.x;
        c1 = median_pose.orientation.y;
        d1 = median_pose.orientation.z;
        a2 = wheelchair2map_.transform.rotation.w;
        b2 = -wheelchair2map_.transform.rotation.x;
        c2 = -wheelchair2map_.transform.rotation.y;
        d2 = -wheelchair2map_.transform.rotation.z;
        a3 = a1*a2 - b1*b2 - c1*c2 - d1*d2;
        b3 = a1*b2 + b1*a2 + c1*d2 - d1*c2;
        c3 = a1*c2 - b1*d2 + c1*a2 + d1*b2;
        d3 = a1*d2 + b1*c2 - c1*b2 + d1*a2;
        // quaternion to euler
        float phi = atan2( 2*(a3*b3 + c3*d3), 1-2*(pow(b3,2)+pow(c3,2)) );  // pitch
        float theta = asin( 2*(a3*b3 - d3*b3) );  // roll
        float psi = atan2( 2*(a3*d3 + b3*c3), 1-2*(pow(b3,2)+pow(d3,2)) );  // yaw
        std::cout<<"phi :"<<phi<<", theta :"<<theta<<", psi :"<<psi<<std::endl;

        // get the inverse transform of transformation fo base_link wrt world coordinates
        tf2::Transform tf_arrow2map;
        // convert from geometry_msgs to tf2 compatible msg
        tf2::fromMsg(median_pose, tf_arrow2map);
        tf2::Transform tf_wheelchair2map;
        // convert from geometry_msgs to tf2 compatible msg
        tf2::fromMsg(wheelchair2map_.transform, tf_wheelchair2map);
        tf2::Transform arrow2wheelchair = tf_wheelchair2map.inverse()*tf_arrow2map;
        tf2::Vector3 arrow_origin2wheelchair = arrow2wheelchair.getOrigin();
        std::cout<<arrow_origin2wheelchair[0]<<","<<arrow_origin2wheelchair[1]<<","<<arrow_origin2wheelchair[2]<<std::endl;
        
        auto modifyPsi = [](float &psi, float arrow_origin2wheelchair_y) {
            // arrow is to the left of the wheelchair
            if (arrow_origin2wheelchair_y>0){
                // psi is in first quadrant
                if (-3.14<=psi && psi<-3.14/2){
                    psi+=3.14;
                }
                // psi is in second quadrant
                else if (-3.14/2<=psi && psi<0){
                    // reflect angle wrt y-axis
                    psi+=2*std::fabs(psi);
                }
                // psi is in third quadrant
                else if (0<psi && psi<3.14/2){
                }
                else{
                    psi-=2*(std::fabs(psi)-3.14/2);
                }
            }
            else{
                // psi is in first quadrant
                if (-3.14<=psi && psi<-3.14/2){
                    psi+=2*(std::fabs(psi)-3.14/2);
                }
                // psi is in second quadrant
                else if (-3.14/2<=psi && psi<0){
                }
                // psi is in third quadrant
                else if (0<psi && psi<3.14/2){
                    // reflect angle wrt y-axis
                    psi-=2*std::fabs(psi);
                }
                else{
                    psi-=3.14;
                }
            }
        };

        // arrow is behind the wheelchair
        if (arrow_origin2wheelchair[0]<0){
            as_feedback.status = "Invalid table docking pose. Arrow pose is behind the wheelchair.";
            return false;
        }
        else {
            modifyPsi(psi,arrow_origin2wheelchair[1]);
        }

        


        std::cout<<"phi :"<<phi<<", theta :"<<theta<<", psi :"<<psi<<std::endl;
        
        // keep psi between -pi and pi radians
        psi-=2*3.14*std::floor((psi+3.14)/(2*3.14));



        // get corrected arrow pose origin by flipping the direction of psi,
        // and extrude the (x,y,z) point of arrow pose origin from table centroid.
        float alpha;
        // flip psi direction to get alpha
        if (psi>0){
            alpha = psi-3.14;
        }
        else{
            alpha = psi+3.14;
        }
        // keep alpha between -pi and pi radians
        alpha-=2*3.14*std::floor((alpha+3.14)/(2*3.14));

        // extrude arrow pose origin from table centroid
        // geometry_msgs::Point new_point;
        float kd = 2.0;
        // new_point.x = centroid_point.x + kd*std::cos(alpha);
        // new_point.y = centroid_point.y + kd*std::sin(alpha);
        // new_point.z = centroid_point.z;

        // // update median pose
        // median_pose.position.x = new_point.x;
        // median_pose.position.y = new_point.y;
        // median_pose.orientation.z = std::sin(psi/2);
        // median_pose.orientation.w = std::cos(psi/2);

        // publish table docking pose
        geometry_msgs::PoseStamped table_docking_pose_msg;

        table_docking_pose_msg.header.frame_id = reference_frame_id_;
        table_docking_pose_msg.header.stamp = ros::Time::now();

        table_docking_pose_msg.pose.position.x = centroid_point.x + kd*std::cos(alpha);
        table_docking_pose_msg.pose.position.y = centroid_point.y + kd*std::sin(alpha);
        table_docking_pose_msg.pose.position.z = 0.0;

        table_docking_pose_msg.pose.orientation.x = 0.0;
        table_docking_pose_msg.pose.orientation.y = 0.0;
        table_docking_pose_msg.pose.orientation.z = std::sin(psi/2);
        table_docking_pose_msg.pose.orientation.w = std::cos(psi/2);
        
        NODELET_INFO("Publishing table docking pose...");
        table_docking_pose_pub_.publish(table_docking_pose_msg);

        // tf2::Transform tf_arrow(wheelchair2map_.transform);

        publishArrowMarker(table_docking_pose_msg.pose, 0.0f, 0.0f, 1.0f, 2);

        pcl::copyPointCloud(*cloud_convex_hull, *filtered_point_cloud);

        return true;
    }

    // return point clouds of objects detected by faster rcnn
    bool PerceptionNodelet::get_table_clouds_combined(
        // const cv::Mat& rgb_input, 
        const sensor_msgs::ImageConstPtr& rgb_input,
        const sensor_msgs::ImageConstPtr& depth_input,
        const sensor_msgs::PointCloud2::ConstPtr& voxel_input,
        custom_msgs::BboxesPtr& table_bboxes,
        pcl::PointCloud<pcl::PointXYZ>::Ptr table_clouds_combined
    )
    {
        custom_msgs::Detect faster_rcnn_srv;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_point_cloud_vector;

        faster_rcnn_srv.request.im_req = *(rgb_input);  //request
        if(!FasterRCNN_client.call(faster_rcnn_srv))
        {
            NODELET_INFO("Failed to run detection");
            return false;
        };
        NODELET_INFO("faster rcnn detection successful");

        filter_point_cloud_by_bbox(faster_rcnn_srv.response.bboxes, depth_input, voxel_input, true, filtered_point_cloud_vector);

        for(int i = 0; i < faster_rcnn_srv.response.bboxes.bboxes.size(); i++)
        {
            // bboxes.bboxes.push_back(faster_rcnn_srv.response.bboxes.bboxes[i]);
            (*table_bboxes).bboxes.push_back(faster_rcnn_srv.response.bboxes.bboxes[i]);
        }
        std::cout<<"filtered_point_cloud_vector.size() "<<filtered_point_cloud_vector.size()<<std::endl;

        for(int i = 0; i<filtered_point_cloud_vector.size(); i++)
        {
            // pcl_ros::transformPointCloud(
            //     *(table_clouds[i]), 
            //     *(table_clouds[i]),  // overwrite
            //     cam2mapTF
            // );
            std::cout<<"filtered_point_cloud_vector[i]->points.size() "<<filtered_point_cloud_vector[i]->points.size()<<std::endl;
            if(i == 0)
                pcl::copyPointCloud(*(filtered_point_cloud_vector[i]), *table_clouds_combined);
            else
                *table_clouds_combined += *(filtered_point_cloud_vector[i]);
        }
        // sensor_msgs::PointCloud2 output_cloud;
        // pcl::toROSMsg(combined_table_cloud, output_cloud);
        // output_cloud.header = depth_input.header;
        // this->table_pub.publish(output_cloud);
        if (filtered_point_cloud_vector.size()<=0) {
            std::cout<<"filtered_point_cloud_vector.size() is zero."<<std::endl;
            return false;
        }
        return true;
    }

    // Function to compute the median of a depth image region
    float PerceptionNodelet::getMedianOfBox(const cv::Mat& depthImage, int x, int y, int m) {
        // Ensure the image is of a valid type (e.g., CV_32F, CV_16U, or CV_8U)
        CV_Assert(depthImage.type() == CV_32F || depthImage.type() == CV_16U || depthImage.type() == CV_8U);

        // Compute the bounding box around (x, y)
        int halfSize = m / 2;
        int startX = std::max(x - halfSize, 0);
        int startY = std::max(y - halfSize, 0);
        int endX = std::min(x + halfSize, depthImage.cols - 1);
        int endY = std::min(y + halfSize, depthImage.rows - 1);

        // Collect all valid pixels in the region
        std::vector<float> values;
        for (int row = startY; row <= endY; ++row) {
            for (int col = startX; col <= endX; ++col) {
                float value = depthImage.at<uint16_t>(row, col); // Adjust for type
                if (!std::isnan(value) && value > 0) { // Exclude invalid or zero depth
                    values.push_back(value);
                }
            }
        }

        // Check if there are valid values
        if (values.empty()) {
            std::cerr << "No valid depth values found in the region." << std::endl;
            return -1; // Return a sentinel value for no data
        }

        // Sort the values
        std::sort(values.begin(), values.end());

        // Compute the median
        size_t n = values.size();
        if (n % 2 == 0) {
            // For even-sized vector, take the average of the two middle values
            return (values[n / 2 - 1] + values[n / 2]) / 2.0f;
        } else {
            // For odd-sized vector, return the middle value
            return values[n / 2];
        }
        
        // return -1;
    }

    // Function to compute the mean of a depth image region
    float PerceptionNodelet::getMeanOfBox(const cv::Mat& depthImage, int x, int y, int m) {
        // Ensure the image is of a valid type (e.g., CV_32F, CV_16U, or CV_8U)
        CV_Assert(depthImage.type() == CV_32F || depthImage.type() == CV_16U || depthImage.type() == CV_8U);

        // Compute the bounding box around (x, y)
        int halfSize = m / 2;
        int startX = std::max(x - halfSize, 0);
        int startY = std::max(y - halfSize, 0);
        int endX = std::min(x + halfSize, depthImage.cols - 1);
        int endY = std::min(y + halfSize, depthImage.rows - 1);

        // Accumulate sum of valid values
        float sum = 0.0f;
        int validCount = 0;

        for (int row = startY; row <= endY; ++row) {
            for (int col = startX; col <= endX; ++col) {
                float value = depthImage.at<float>(row, col); // Adjust for type
                if (!std::isnan(value) && value > 0) { // Exclude invalid or zero depth
                    sum += value;
                    validCount++;
                }
            }
        }

        // Check if there are valid values
        if (validCount == 0) {
            std::cerr << "No valid depth values found in the region." << std::endl;
            return -1; // Return a sentinel value for no data
        }

        // Compute the mean
        return sum / validCount;
    }

    void PerceptionNodelet::projectToPlane(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_projected,
        float table_height
    )
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        coefficients->values.resize(4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = -1.0;
        coefficients->values[3] = table_height;
        std::cout<<"projectToPlane halfway"<<std::endl;
        std::cout<<"input_cloud->points.size() :"<<input_cloud->points.size()<<std::endl;

        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (input_cloud);
        proj.setModelCoefficients (coefficients);
        proj.filter (*cloud_projected);
        std::cout<<"cloud_projected->points.size() :"<<cloud_projected->points.size()<<std::endl;
    }

    void PerceptionNodelet::getConvexHull(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_convex_hull,
        std::vector<pcl::Vertices>& polygons
    )
    {
        pcl::ConvexHull<pcl::PointXYZ> convex_hull;
        convex_hull.setDimension(2);
        convex_hull.setInputCloud(plane_cloud);
        convex_hull.reconstruct(*cloud_convex_hull, polygons);
    }

    // convert points in bounding box to point cloud
    bool PerceptionNodelet::filter_point_cloud_by_bbox(
        custom_msgs::Bboxes& res_bbox,
        // const cv::Mat& depth_img, 
        const sensor_msgs::ImageConstPtr& depth_input,
        const sensor_msgs::PointCloud2::ConstPtr& voxel_input,
        bool wall_mean, 
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& filtered_point_cloud_vector
    )
    {
        std::cout<<"start filter_point_cloud_by_bbox"<<std::endl;
        // convert ros msg to cv
        cv_bridge::CvImagePtr depth_image_ptr;
        try
        {
            depth_image_ptr = cv_bridge::toCvCopy(
                depth_input, 
                sensor_msgs::image_encodings::TYPE_16UC1
            );
            // rgb_image_ptr = cv_bridge::toCvCopy(
            //     rgb_input, 
            //     sensor_msgs::image_encodings::RGB8
            // );
        }
        catch (cv_bridge::Exception& e)
        {
            NODELET_ERROR("cv_bridge exception 1: %s", e.what());
            return false;
        }
        cv::Mat depth_img = depth_image_ptr->image;

        // convert voxel grid sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_pcl(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*voxel_input,*voxel_pcl);

        // create a KD-tree for cloud
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_1;
        kdtree_1.setInputCloud(voxel_pcl);

        float distance_threshold=1.0;  // 2.5 m

        for(int i = 0; i < res_bbox.bboxes.size(); i++)
        {
            // for each bbox
            ROS_INFO("bbox_size: %ld", res_bbox.bboxes.size());
            ROS_INFO("i : %d", i);

            if(checkClsList(res_bbox.bboxes[i].cls, this->door_class_list))  // detected door
            {
                continue;
            }
            else if(checkClsList(res_bbox.bboxes[i].cls, this->table_class_list))  // detected table
            {
                // cloud for each detected table
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (int x = res_bbox.bboxes[i].top_left.x; x< res_bbox.bboxes[i].btm_right.x+1 ; x++)
                {
                    for (int y = res_bbox.bboxes[i].top_left.y; y < res_bbox.bboxes[i].btm_right.y+1 ; y++)
                    {
                        pcl::PointXYZ point;
                        point.x = x;
                        point.y = y;

                        // remove depth values in invalid regions
                        if((x <0 || y < 0) || (x >= depth_img.cols || y >= depth_img.rows ))
                            continue;
                        // remove depth values more than some determined depth value
                        if(depth_img.at<uint16_t>(y, x) * 0.001 > 5.0)
                            continue;

                        // int m = 30;
                        // float depth_val = getMedianOfBox(depth_img,x,y,m);

                        // std::cout<<"depth_val :"<<depth_val<<std::endl;

                        // if (depth_val==-1)
                        //     // reset table_bboxes
                        //     return false;

                        deproject(x, y, depth_img.at<uint16_t>(y, x)*0.001, point);
                        (*cloud).push_back(point);
                    }
                }
                // use obtained cloud from depth img to filter voxel input
                update_cam2map_transform(depth_input->header);

                /// transform depth point cloud from camera frame to map frame
                tf::Transform cam2mapTF;
                tf::transformMsgToTF(cam2map_.transform, cam2mapTF);
                pcl_ros::transformPointCloud(
                    *cloud, 
                    *cloud,  // overwrite
                    cam2mapTF
                );

                // placeholder variable for filtered voxel result
                pcl::PointCloud<pcl::PointXYZ>::Ptr voxels_inside_table_bounding_box(new pcl::PointCloud<pcl::PointXYZ>());

                // iterate over points in depth cloud and find the nearest points in voxel grid
                for (size_t i=0; i<cloud->points.size(); ++i){
                    pcl::PointXYZ searchPoint = cloud->points[i];

                    // search for the nearest neighbor
                    int K=1000;
                    std::vector<int> nearestIndices(K);
                    std::vector<float> nearestDistances(K);
                    if (kdtree_1.nearestKSearch(searchPoint,K,nearestIndices,nearestDistances)>0){
                        for (std::size_t j=0;j<nearestIndices.size();++j)
                        {
                            if (nearestDistances[j]<distance_threshold){                                    
                                voxels_inside_table_bounding_box->points.push_back(voxel_pcl->points[nearestIndices[j]]);
                            }  
                        }                          
                        
                    }
                }
                std::cout<<"voxels_inside_table_bounding_box.points.size() :"<<voxels_inside_table_bounding_box->points.size()<<std::endl;
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr nan_filtered_voxels_inside_table_bounding_box(new pcl::PointCloud<pcl::PointXYZ>());
                std::vector<int> indices;
                pcl::removeNaNFromPointCloud(*voxels_inside_table_bounding_box,*nan_filtered_voxels_inside_table_bounding_box,indices);
                std::cout<<"nan_filtered_voxels_inside_table_bounding_box.points.size() :"<<nan_filtered_voxels_inside_table_bounding_box->points.size()<<std::endl;
                filtered_point_cloud_vector.emplace_back(nan_filtered_voxels_inside_table_bounding_box);
            }
        }
        std::cout<<"end filter_point_cloud_by_bbox"<<std::endl;
        std::cout<<std::endl;
        return true;
    }

    void PerceptionNodelet::deproject(int x, int y, float z, pcl::PointXYZ& point_temp)
    {
        point_temp.x = (float(x) - this->ppx) / (this->fx) * z;
        point_temp.y = (float(y) - this->ppy) / (this->fy) * z;
        point_temp.z = z;
    }

    void PerceptionNodelet::publishPointMarker(geometry_msgs::Point& p,  float r, float g, float b, float scale, int marker_id)
    {
        std::cout<<p.x<<","<<p.y<<","<<p.z<<","<<std::endl;
        visualization_msgs::Marker marker;
        marker.header.frame_id = reference_frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "spheres";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // set the pose
        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = p.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // set the scale
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;

        // set the color
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;
        
        // publish marker
        // marker_pub_.publish(marker);
        marker_array_ptr_->markers.push_back(marker);
    }

    void PerceptionNodelet::publishArrowMarker(geometry_msgs::Pose& p,  float r, float g, float b, int marker_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = reference_frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "arrows";
        marker.id = marker_id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // set the pose
        marker.pose.position.x = p.position.x;
        marker.pose.position.y = p.position.y;
        marker.pose.position.z = p.position.z;
        marker.pose.orientation.x = p.orientation.x;
        marker.pose.orientation.y = p.orientation.y;
        marker.pose.orientation.z = p.orientation.z;
        marker.pose.orientation.w = p.orientation.w;
        std::cout<<"arrow_position_x :"<<p.position.x<<"arrow_position_y :"<<p.position.y<<"arrow_position_z :"<<p.position.z<<std::endl;
        std::cout<<"arrow_orientation_x :"<<p.orientation.x<<"arrow_orientation_y :"<<p.orientation.y<<"arrow_orientation_z :"<<p.orientation.z<<"arrow_orientation_w :"<<p.orientation.w<<std::endl;

        // set the scale
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // set the color
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0f;
        
        // publish marker
        // marker_pub_.publish(marker);
        marker_array_ptr_->markers.push_back(marker);

        // reset marker_array_ptr_
    }

    // void PerceptionNodelet::deleteAllMarker()
    // {
    //     visualization_msgs::Marker marker;
    //     marker.header.frame_id = reference_frame_id_;
    //     marker.header.stamp = ros::Time::now();
    //     marker.action = visualization_msgs::Marker::DELETEALL;
    //     marker_pub_.publish(marker);
    // }

    bool PerceptionNodelet::checkClsList(const std::string& input_cls, std::vector<std::string>& cls_list)
    {
        for(int i = 0; i < cls_list.size(); i++)
        {
            if(input_cls.compare(cls_list[i]) ==  0)
            {
                return true;
            }
        }
        return false;
    }

    bool PerceptionNodelet::update_cam2map_transform(const std_msgs::Header& camHeader)
    {
        try{
            if (tf_buffer_->canTransform(
                reference_frame_id_,
                camHeader.frame_id,
                ros::Time(0),
                ros::Duration(10)))
            {
                cam2map_ = tf_buffer_->lookupTransform(
                    reference_frame_id_, 
                    camHeader.frame_id,
                    ros::Time(0));
                NODELET_INFO("canTransform: TRUE");
                NODELET_INFO("Transform: [%.2f, %.2f, %.2f]",
                    cam2map_.transform.translation.x,
                    cam2map_.transform.translation.y,
                    cam2map_.transform.translation.z);
            }
            else
            {
                NODELET_INFO("canTransform: FALSE");
            }
        }
        catch (tf2::TransformException &ex) 
        {
            NODELET_WARN("%s",ex.what());
            return false;
        }
        
        return true;
    }

    bool PerceptionNodelet::update_wheelchair2map_transform()
    {
        // get transform of wheelchair relative to map
        try{
            if (tf_buffer_->canTransform(
                reference_frame_id_,
                "base_link",
                ros::Time(0),
                ros::Duration(10)))
            {
                wheelchair2map_ = tf_buffer_->lookupTransform(
                    reference_frame_id_, 
                    "base_link",
                    ros::Time(0));
                NODELET_INFO("canTransform: TRUE");
                NODELET_INFO("Transform: [%.2f, %.2f, %.2f]",
                    wheelchair2map_.transform.translation.x,
                    wheelchair2map_.transform.translation.y,
                    wheelchair2map_.transform.translation.z);
            }
            else
            {
                NODELET_INFO("canTransform: FALSE");
            }
        }
        catch (tf2::TransformException &ex) 
        {
            NODELET_WARN("%s",ex.what());
            return false;
        }
        
        return true;
    }

    void PerceptionNodelet::detection_cb(const custom_msgs::DetectionGoalConstPtr& as_goal){
        custom_msgs::DetectionFeedback as_feedback;
        custom_msgs::DetectionResult as_result;

        sensor_msgs::ImageConstPtr right_rgb_image_msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(rightCam->getRGBTopic());
        if (right_rgb_image_msg_ptr == NULL) {
            NODELET_ERROR("Error receiving color image!");
            as_result.succeeded = false;
            as_result.information = "Error capturing right color image. Please try again.";
            as_->setAborted(as_result);
            return;
        }

        sensor_msgs::ImageConstPtr left_rgb_image_msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(leftCam->getRGBTopic());
        if (left_rgb_image_msg_ptr == NULL) {
            NODELET_ERROR("Error receiving color image!");
            as_result.succeeded = false;
            as_result.information = "Error capturing left color image. Please try again.";
            as_->setAborted(as_result);
            return;
        }
        
        sensor_msgs::ImageConstPtr right_depth_image_msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(rightCam->getDepthTopic());
        if (right_depth_image_msg_ptr == NULL) {
            NODELET_ERROR("Error receiving depth image!");
            as_result.succeeded = false;
            as_result.information = "Error capturing right depth image. Please try again.";
            as_->setAborted(as_result);
            return;
        }

        sensor_msgs::ImageConstPtr left_depth_image_msg_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(leftCam->getDepthTopic());
        if (left_depth_image_msg_ptr == NULL) {
            NODELET_ERROR("Error receiving depth image!");
            as_result.succeeded = false;
            as_result.information = "Error capturing left depth image. Please try again.";
            as_->setAborted(as_result);
            return;
        }

        sensor_msgs::PointCloud2ConstPtr voxel_point_cloud_msg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/planner/global_costmap/voxel_obstacle_layer/voxel_grid", ros::Duration(3.0));
        if (voxel_point_cloud_msg_ptr == NULL) {
            NODELET_ERROR("Error receiving voxel point cloud!");
            as_result.succeeded = false;
            as_result.information = "Error capturing voxel point cloud. Please try again.";
            as_->setAborted(as_result);
            return;
        }

        // returned point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        // to publish table bounding boxes message to gui
        custom_msgs::BboxesPtr table_bboxes(new custom_msgs::Bboxes);
        // reset bool is_success
        bool is_success = false;
        // reset markers array
        // marker_array_ptr_->markers.clear();

        if (as_goal->detection_request == "left_rgb_image_click"){
            is_success = get_table_docking_pose(as_goal->point_x, as_goal->point_y,
                                        left_rgb_image_msg_ptr, left_depth_image_msg_ptr, voxel_point_cloud_msg_ptr, 
                                        true, table_bboxes, point_cloud_xyz, as_feedback);
            std::cout<<"is_success :"<<is_success<<std::endl;
            std::cout<<as_feedback.status<<std::endl;
        }
        else if (as_goal->detection_request == "right_rgb_image_click")
        {
            is_success = get_table_docking_pose(as_goal->point_x, as_goal->point_y,
                                        right_rgb_image_msg_ptr, right_depth_image_msg_ptr, voxel_point_cloud_msg_ptr, 
                                        true, table_bboxes, point_cloud_xyz, as_feedback);
            std::cout<<"is_success :"<<is_success<<std::endl;
            std::cout<<as_feedback.status<<std::endl;
        }
        
        else if (as_goal->detection_request == "detection_button_click"){
            // process image if detection button click
            get_table_clouds_combined(left_rgb_image_msg_ptr, left_depth_image_msg_ptr, voxel_point_cloud_msg_ptr, table_bboxes, point_cloud_xyz);
            get_table_clouds_combined(right_rgb_image_msg_ptr, right_depth_image_msg_ptr, voxel_point_cloud_msg_ptr, table_bboxes, point_cloud_xyz);
            // process_image(left_rgb_image_msg_ptr, left_depth_image_msg_ptr, voxel_point_cloud_msg_ptr, point_cloud_xyz);
        }

        // for (int i=0; i<*table_bboxes.bboxes.size(); i++){

        // }

        // process_image(right_rgb_image_msg_ptr, right_depth_image_msg_ptr, point_cloud_xyz);
        // process_image(left_rgb_image_msg_ptr, left_depth_image_msg_ptr, point_cloud_xyz);
        else {
            NODELET_ERROR("Error receiving depth image!");
            as_result.succeeded = false;
            as_result.information = "Invalid detection request.";
            as_->setAborted(as_result);
            return;
        }

        table_bboxes_pub_.publish(*table_bboxes);

        if (is_success){
            // table_bboxes_pub_.publish(*table_bboxes);
            NODELET_INFO("Table point cloud detection success!");
            as_result.succeeded = true;
            as_result.information = "Table point cloud detection success!";
            marker_array_pub_.publish(*marker_array_ptr_);
            
            as_->setSucceeded(as_result);
        }

        else{
            NODELET_ERROR("Table point cloud not detected!");
            NODELET_ERROR("%s",as_feedback.status.c_str());
            as_result.succeeded = false;
            // as_result.information = "Point cloud detection failed.";
            as_result.information = as_feedback.status;
            as_->setAborted(as_result);
            return;
        }
        return;
    }

}