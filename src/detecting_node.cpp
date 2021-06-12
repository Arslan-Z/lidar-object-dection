#include <math.h>

#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>  
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/feature_histogram.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h> 
#include <geometry_msgs/PoseStamped.h>  
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include <string.h>
#include <iostream> 
#include <string>
#include <vector>   
#include <math.h>

typedef pcl::PointXYZI  PointType;

class AdapIntegraDetector{

private:
    
    ros::NodeHandle nh ;
    
    ros::Subscriber subLaserCloud;

    ros::Publisher  pubMapCloud; 
    ros::Publisher  pubClusters;
    ros::Publisher  pubLaserCloud2;
    ros::Publisher  pubLaserCloudFilter1;
    ros::Publisher  pubLaserCloudFilter2;
    ros::Publisher  pubObjCloud;  
    ros::Publisher  pubObjPose;
    ros::Publisher  pubObjbbox; 
      
    pcl::PointCloud<PointType>::Ptr ig_Cloud_1      ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr ig_Cloud_2      ; // Integrating cloud
    pcl::PointCloud<PointType>::Ptr tmpCloud        ;
    pcl::PointCloud<PointType>::Ptr interated_Cloud ; 
    pcl::PointCloud<PointType>::Ptr latest_Cloud    ; 
    pcl::PointCloud<PointType>::Ptr dynamic_Cloud   ;
    pcl::PointCloud<PointType>::Ptr obj_Cloud       ;
    pcl::PointCloud<PointType>::Ptr map_cloud       ;

    geometry_msgs::PoseStamped      obj_pose        ; 

    int     ig_cnt = 0;
    bool    map_built = false;

public:
    AdapIntegraDetector():nh("~") { 
        ig_Cloud_1.reset(       new pcl::PointCloud<PointType>());
        ig_Cloud_2.reset(       new pcl::PointCloud<PointType>()); 
        tmpCloud.reset(         new pcl::PointCloud<PointType>()); 
        latest_Cloud.reset(     new pcl::PointCloud<PointType>()); 
        interated_Cloud.reset(  new pcl::PointCloud<PointType>()); 
        dynamic_Cloud.reset(    new pcl::PointCloud<PointType>()); 
        obj_Cloud.reset(        new pcl::PointCloud<PointType>()); 
        map_cloud.reset(        new pcl::PointCloud<PointType>()); 

        interated_Cloud->header.frame_id    = "/livox_frame";
        latest_Cloud->header.frame_id       = "/livox_frame";
        tmpCloud->header.frame_id           = "/livox_frame";
        ig_Cloud_1->header.frame_id         = "/livox_frame";
        ig_Cloud_2->header.frame_id         = "/livox_frame"; 
        dynamic_Cloud->header.frame_id      = "/livox_frame";  
        obj_Cloud->header.frame_id          = "/livox_frame";  
        map_cloud->header.frame_id          = "/livox_frame"; 

        map_built = false;
 
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &AdapIntegraDetector::laserCloudHandler, this); 
        
        pubMapCloud     = nh.advertise<pcl::PointCloud<PointType>>("/livox/map", 2);
        pubClusters     = nh.advertise<pcl::PointCloud<PointType>>("/livox/cluster", 2);
        pubLaserCloud2  = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_cloud_2", 2);
        pubLaserCloudFilter1 = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_cloud_1_filter", 2);
        pubLaserCloudFilter2 = nh.advertise<pcl::PointCloud<PointType>>("/livox/ig_cloud_2_filter", 2);
        pubObjCloud     = nh.advertise<pcl::PointCloud<PointType>>("/livox/obj_cloud", 2);   
        pubObjPose      = nh.advertise<geometry_msgs::PoseStamped>("/livox/obj_pose", 2); 

        pubObjbbox       = nh.advertise<visualization_msgs::Marker>("/livox/obj_bbox", 2);
    }
 

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        // timestamp = laserCloudMsg->header.stamp.toSec();
        // last_time = laserCloudMsg->header.stamp; 
        // frame_seq = laserCloudMsg->header.seq;
        tmpCloud->clear();
        pcl::fromROSMsg(*laserCloudMsg, *tmpCloud); 

        // ROS_INFO_STREAM(  " MAP BUILT? " << map_built );
        if(map_built){
            if (ig_cnt < 5){ 
                ig_cnt++;
                *interated_Cloud = *interated_Cloud + *tmpCloud;
            }
            else{
                latest_Cloud->clear(); 
                *latest_Cloud = *latest_Cloud + *tmpCloud;
                obj_inmap_detecting(interated_Cloud);
                interated_Cloud->clear();
                ig_cnt = 0;
            }
            return;
        }

        int itg_num = 80;
        // ROS_INFO_STREAM("Map initing ... ");
        if(ig_cnt < itg_num ){
            *interated_Cloud = *interated_Cloud + *tmpCloud;
            ig_cnt++; 
            if(ig_cnt % 10 == 0){
                ROS_INFO_STREAM("Map initing - Cloud 1 is inegrating ... " << ig_cnt << " / " << itg_num);
            }

        }else if(ig_cnt == itg_num ){
            ROS_INFO_STREAM("Cloud 1 is ready ! ");
            *ig_Cloud_1 = *ig_Cloud_1 + *interated_Cloud;
            simple_map_building(ig_Cloud_1);
            // pubLaserCloud1.publish(ig_Cloud_1);
            interated_Cloud->clear();
            ig_cnt++;

        }else if(ig_cnt > itg_num + 30 && ig_cnt < 2 * itg_num + 30){
            *interated_Cloud = *interated_Cloud + *tmpCloud;
            ig_cnt++; 
            if(ig_cnt % 10 == 0){
                ROS_INFO_STREAM("Map initing - Cloud 2 is inegrating ... " << ig_cnt << "/ " << itg_num);
            }
        }else if(ig_cnt == 2 * itg_num + 30){
            latest_Cloud->clear();
            *ig_Cloud_2 = *ig_Cloud_2 + *interated_Cloud; 
            *latest_Cloud = *latest_Cloud + *tmpCloud;
            // pubLaserCloud2.publish(ig_Cloud_2);
            ig_cnt++;
            ROS_INFO_STREAM("Cloud 2 is ready ! "); 
            map_building(ig_Cloud_1, ig_Cloud_2); 
            interated_Cloud->clear(); 
        }else{
            if(!map_built)
                ig_cnt++;
            else
                ig_cnt = 0;
        }
    }

    // Take current clouds, substract points belong to static objects, extract the objects points
    void obj_inmap_detecting(pcl::PointCloud<PointType>::Ptr cloud){
        
        pcl::KdTreeFLANN<PointType>::Ptr kdtreePts(new pcl::KdTreeFLANN<PointType>());
        kdtreePts->setInputCloud(map_cloud); 
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        
        dynamic_Cloud->points.clear();
        float radius = 0.1;
        for(int nIdex = 0; nIdex < cloud->points.size(); nIdex++){ 
            if ( kdtreePts->radiusSearch(cloud->points[nIdex], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            { 
            }else{
                dynamic_Cloud->points.push_back(cloud->points[nIdex]);
            }
        }

         
        if(dynamic_Cloud->points.size() == 0){
            ROS_INFO_STREAM("No dynamic points detected ! ");
            return;
        }else{
            ROS_INFO_STREAM("Found dynamics points "<< dynamic_Cloud->points.size());
        } 
 
        // Cluster the object 
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud(dynamic_Cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance (0.1); // 5cm
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (1000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (dynamic_Cloud);
        ec.extract (cluster_indices);

        ROS_INFO_STREAM("Found PopintCloud Cluster " <<  cluster_indices.size());

        pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            // pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>); 
            cloud_cluster->header.frame_id = "/livox_frame";
            for (const auto& idx : it->indices)
                cloud_cluster->push_back ((*dynamic_Cloud)[idx]); 
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            ROS_INFO_STREAM("PointCloud Trajectory Clustering: "  << cloud_cluster->size () << " data points.");
            pubClusters.publish(*cloud_cluster);
        
 
            if(cloud_cluster->size() > 0 ){
                // Find the object pose from latest frame
                pointIdxRadiusSearch.resize(0);
                pointRadiusSquaredDistance.resize(0);
                kdtreePts->setInputCloud(cloud_cluster);  

                radius = 0.2;
                obj_Cloud->clear();
                for(int nIdex = 0; nIdex < latest_Cloud->points.size(); nIdex++){ 
                    if ( kdtreePts->radiusSearch(latest_Cloud->points[nIdex], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
                    { 
                        obj_Cloud->points.push_back(latest_Cloud->points[nIdex]);
                    }
                }
                ROS_INFO_STREAM("Publishing Object PointCloud: " << obj_Cloud->size () << " data points.");
            }


            // Caculate the pose and the covariance of the pose
            Eigen::Vector4f centroid;
            Eigen::Matrix3f covariance;
            pcl::compute3DCentroid(*obj_Cloud, centroid);
            pcl::computeCovarianceMatrix(*obj_Cloud, centroid, covariance);


            if(covariance(0,0) < 0.2 && covariance(1,1) < 0.2 && covariance(2,2) < 0.2){
                geometry_msgs::PoseStamped  obj_pose;
                obj_pose.header.frame_id = "/livox_frame";
                obj_pose.pose.position.x = centroid(0,0);
                obj_pose.pose.position.y = centroid(1,0);
                obj_pose.pose.position.z = centroid(2,0); 

                //Bounding box
                visualization_msgs::Marker marker;
                marker.header.frame_id = "livox_frame";
                // marker.text = bbox2d->boundingBoxes[i].Class;
                marker.text = "car";
                marker.header.stamp = ros::Time::now();
                marker.id = 0;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.lifetime = ros::Duration(1);
                marker.pose.position.x = centroid(0,0);
                marker.pose.position.y = centroid(1,0);
                marker.pose.position.z = centroid(2,0);
                marker.pose.orientation.x = 0;
                marker.pose.orientation.y = 0;
                marker.pose.orientation.z = 0;
                marker.pose.orientation.w = 1;
                marker.scale.x = covariance(0,0) * 2;
                marker.scale.y = covariance(1,1) * 2;
                marker.scale.z = covariance(2,2) * 2;
                //  marker.scale.x = 3;
                //  marker.scale.y = 2;
                //  marker.scale.z = 2;
                marker.color.a = 0.5;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0; 

                pubObjbbox.publish(marker);
                pubObjCloud.publish(*obj_Cloud);
                pubObjPose.publish(obj_pose);
            }else{
                ROS_WARN_STREAM("Invalid UAV cluster -> covariance \n" <<  covariance);
            }
        }

    }

    //Received the object_poses, current obj_covariance, publish the most probablity object pose
    void obj_tracking(){
     
    }

    //Take a trajectory which is composed of serveral pointcloud. Then check the covariance of each
    // pointcloud to verify it's ability  
    void traj_verify(){

    }

    void simple_map_building(pcl::PointCloud<PointType>::Ptr cloud1){ 
   
        pcl::PCLPointCloud2::Ptr point_cloud1(new pcl::PCLPointCloud2());  
          
        pcl::toPCLPointCloud2(*cloud1, *point_cloud1); 

        pcl::PCLPointCloud2::Ptr cloud_grided_1(new pcl::PCLPointCloud2()); 
        cloud_grided_1->header.frame_id = "/livox_frame"; 
        
        // Cloud 1
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(point_cloud1);
        sor.setLeafSize (0.05f, 0.05f, 0.05f);
        sor.filter (*cloud_grided_1);
        pubLaserCloudFilter1.publish(*cloud_grided_1); 

        pcl::PointCloud<PointType>::Ptr ccloud1 (new pcl::PointCloud<PointType>()); 
        pcl::fromPCLPointCloud2(*cloud_grided_1, *ccloud1);  

        *map_cloud = *map_cloud + *ccloud1;

        ROS_INFO_STREAM("Pub map points "<< map_cloud->points.size());
        pubMapCloud.publish(*map_cloud);

        if(map_cloud->points.size() > 0){
            map_built = true;
            ROS_INFO_STREAM("A map has been build with points "<< map_cloud->points.size());
        }
 
    } 

    void map_building(pcl::PointCloud<PointType>::Ptr cloud1, pcl::PointCloud<PointType>::Ptr cloud2){ 
   
        pcl::PCLPointCloud2::Ptr point_cloud1(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr point_cloud2(new pcl::PCLPointCloud2()); 
          
        pcl::toPCLPointCloud2(*cloud1, *point_cloud1);
        pcl::toPCLPointCloud2(*cloud2, *point_cloud2);

        pcl::PCLPointCloud2::Ptr cloud_grided_1(new pcl::PCLPointCloud2());
        pcl::PCLPointCloud2::Ptr cloud_grided_2(new pcl::PCLPointCloud2());
        cloud_grided_1->header.frame_id = "/livox_frame";
        cloud_grided_2->header.frame_id = "/livox_frame";
        
        // Cloud 1
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(point_cloud1);
        sor.setLeafSize (0.05f, 0.05f, 0.05f);
        sor.filter (*cloud_grided_1);
        pubLaserCloudFilter1.publish(*cloud_grided_1);

        // Cloud 2
        sor.setInputCloud(point_cloud2);
        sor.setLeafSize (0.05f, 0.05f, 0.05f);
        sor.filter (*cloud_grided_2);
        pubLaserCloudFilter2.publish(*cloud_grided_2); 

        pcl::PointCloud<PointType>::Ptr ccloud1 (new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr ccloud2 (new pcl::PointCloud<PointType>());
        pcl::fromPCLPointCloud2(*cloud_grided_1, *ccloud1);
        pcl::fromPCLPointCloud2(*cloud_grided_2, *ccloud2);
        pcl::PointCloud<PointType>::Ptr diffcloud (new pcl::PointCloud<PointType>());
        diffcloud->header.frame_id = "/livox_frame";

        pcl::KdTreeFLANN<PointType>::Ptr kdtreePts(new pcl::KdTreeFLANN<PointType>());
        kdtreePts->setInputCloud(ccloud1); 
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        float radius = 0.1;
        for(int nIdex = 0; nIdex < ccloud2->points.size(); nIdex++){ 
            if ( kdtreePts->radiusSearch(ccloud2->points[nIdex], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
            {
                map_cloud->points.push_back(ccloud2->points[nIdex]); // Common area are treated as map(static objects)
            }else{
                diffcloud->points.push_back(ccloud2->points[nIdex]);
            }
        }

        ROS_INFO_STREAM("Pub map points "<< map_cloud->points.size());
        pubMapCloud.publish(*map_cloud);

        if(map_cloud->points.size() > 0){
            map_built = true;
            ROS_INFO_STREAM("A map has been build with points "<< map_cloud->points.size());
        }
         
 
    } 
}; 


int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_drone_detector"); 
 
    AdapIntegraDetector DD; 
    // ros::MultiThreadedSpinner spinner(15);
    // spinner.spin();
    ros::spin();
 
    return 0;
}

