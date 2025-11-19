/********************************************************
 *  停障代码
 *
 *  相关话题
 *  订阅：
 *      
 *
 *  发布：
 * 
 *
 ******************************************************/
////////////////////////////////////////////////////////////
#include <cmath>
#include <thread>
#include <iomanip>
#include <iostream>
#include <string.h>

#include <ros/time.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/duration.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <controller/command.h>

#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <Eigen/Eigen>
#include <eigen3/Eigen/Dense> 
#include <Eigen/Core>

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/asio.hpp>
#include <string>

#include <cv_bridge/cv_bridge.h>

#include <yhs_can_msgs/steering_ctrl_fb.h>
#include <yhs_can_msgs/ctrl_cmd.h>
#include <ackermann_msgs/AckermannDrive.h>

#include "planner/SendPath.h"
#include "Utility.h"

using namespace std;

enum ACC_DEC_mode{
    ACC,
    DEC,
    KEEP
};
// 运动状态，决定用哪颗雷达检测障碍物
enum MoveDirection{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOP
};
bool NO_received_mission_FLAG_ = true;
bool lidar_stop_obstacle_FLAG_ = false;

bool detect_FLAG_ = false;
bool stop_FLAG_ = false;
bool arrive_mission_FLAG = false;
bool forward_mode_FLAG_ = false;
bool mission_mode_change_FLAG_ = false;
bool ready_arrive_end_FLAG_ = false;

class ObstacleDetect
{
private:
    ros::NodeHandle nh_;
    ros::CallbackQueue self_queue_;
    ros::AsyncSpinner as_;
    std::thread* thread_run = nullptr;

    ros::Subscriber hesai_pc_front_sub_;
    ros::Publisher collision_pc_pub_;

    int dep_count_thres_;
    double dep_value_thres_;

    pcl::CropBox<pcl::PointXYZ> front_box_filter_;
    pcl::CropBox<pcl::PointXYZ> back_box_filter_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr front_local_pc_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr back_local_pc_;

    //camera stop
    cv::Mat back_depth_pic_;
    cv::Mat front_depth_pic_;
    sensor_msgs::PointCloud2 pointcloud_data_pub_;

    //坐标转换相关变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;  // 目标坐标系（车辆坐标系）
    std::string source_frame_;  // 源坐标系（雷达坐标系）

private:
    void   livoxFrontCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg);

public:
    ObstacleDetect();
    ~ObstacleDetect(){};
    //速度回调，根据速度判断前进还是横移
    void velCallback(cosnt geometry_msgs::Twist::ConstPtr& msg);
};

ObstacleDetect::ObstacleDetect():
as_(ros::AsyncSpinner(2, &self_queue_))
{
    nh_.setCallbackQueue(&self_queue_);
    hesai_pc_front_sub_ = nh_.subscribe("/livox/lidar", 10, &ObstacleDetect::livoxFrontCallBack, this);  //注意雷达话题
    collision_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_filiter", 1);
    
    //init
    //camera stop
    dep_count_thres_ = 1000;
    dep_value_thres_ = 1.5;

    //lidar stop
    front_local_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    front_box_filter_.setMin(Eigen::Vector4f( 0.5  ,-1.3 ,-0.2  ,1.0));
    front_box_filter_.setMax(Eigen::Vector4f( 1.5  ,1.3  , 0.3  ,1.0));
    front_box_filter_.setNegative(false);
    
    as_.start();
}

void ObstacleDetect::livoxFrontCallBack(const sensor_msgs::PointCloud2::ConstPtr& msg){
    // if(NO_received_mission_FLAG_) return;
    
    if(forward_mode_FLAG_){
        pcl::fromROSMsg(*msg , *front_local_pc_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pc(new pcl::PointCloud<pcl::PointXYZ>());
        front_box_filter_.setInputCloud(front_local_pc_);
        front_box_filter_.filter(*collision_pc);
        geometry_msgs::Point tempstopcmd;
        sensor_msgs::PointCloud2 collision_pc_msg;
        pcl::toROSMsg(*collision_pc, collision_pc_msg);
        collision_pc_msg.header = msg->header; // 保持与原始点云消息的头部信息一致
        collision_pc_pub_.publish(collision_pc_msg);
        if(collision_pc->width > 20){
            lidar_stop_obstacle_FLAG_ = true;
        }
        else{
            lidar_stop_obstacle_FLAG_ = false;
        }
    }
}
