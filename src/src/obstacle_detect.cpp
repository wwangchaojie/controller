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

#include <yhs_can_msgs/steering_ctrl_fb.h>
#include <yhs_can_msgs/ctrl_cmd.h>
#include <mutex>

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
    STOPPED
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

    ros::Subscriber livox_pc_front_sub_1;
    ros::Subscriber livox_pc_front_sub_2;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher collision_pc_pub_;

    int dep_count_thres_;
    double dep_value_thres_;

    pcl::CropBox<pcl::PointXYZ> front_box_filter_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processing_pc_;

    sensor_msgs::PointCloud2 pointcloud_data_pub_;

    MoveDirection curDirection;     //当前运动方向
    yhs_can_msgs::ctrl_cmd current_cmd_vel_;  // 当前速度命令
    std::mutex vel_mutex_;  // 速度数据互斥锁

    // 检测区域参数
    double square_size_;
    double front_start_;
    double height_min_, height_max_;
    int current_detect_lidar;     //当前用于检测障碍物的雷达
    int obstacle_threshold_;  // 障碍物阈值（点云数量阈值）

    //坐标转换相关变量
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string target_frame_;  // 目标坐标系（车辆坐标系）
    std::string source_frame_;  // 源坐标系（雷达坐标系）



private:
    void livoxFrontCallBack1(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void livoxFrontCallBack2(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void processLidarData(const sensor_msgs::PointCloud2::ConstPtr& msg, int lidar_id);


public:
    ObstacleDetect();
    ~ObstacleDetect(){};
    //速度回调，根据速度判断前进还是横移
    void cmdvelCallback(const yhs_can_msgs::ctrl_cmd msg_);
    // 判断运动方向
    MoveDirection getCurrentDirection();
    // 根据运动方向更新检测框
    void updateDirectionBox(MoveDirection direction_);
};

ObstacleDetect::ObstacleDetect():
as_(ros::AsyncSpinner(2, &self_queue_)),
tf_listener_(tf_buffer_),  // 添加TF监听器初始化
current_detect_lidar(1)   //默认用雷达1
{
    nh_.setCallbackQueue(&self_queue_);
    livox_pc_front_sub_1 = nh_.subscribe("/livox/lidar1", 10, &ObstacleDetect::livoxFrontCallBack1, this);  //注意雷达话题
    livox_pc_front_sub_2 = nh_.subscribe("/livox/lidar2", 10, &ObstacleDetect::livoxFrontCallBack2, this);  //注意雷达话题
    cmd_vel_sub_ = nh_.subscribe("/ctrl_cmd",10, &ObstacleDetect::cmdvelCallback,this);
    collision_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_filiter", 1);
    
    // 检测参数
    nh_.param("square_size", square_size_, 2.0);
    nh_.param("front_start", front_start_, 0.5);
    nh_.param("obstacle_threshold", obstacle_threshold_, 20);
    // TODO: 检测高度范围
    nh_.param("height_min", height_min_, -1.0);
    nh_.param("height_max", height_max_, -0.3);

    // 初始化点云
    processing_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    // //init
    // //camera stop
    // dep_count_thres_ = 1000;
    // dep_value_thres_ = 1.5;

    // //lidar stop

    // front_box_filter_.setMin(Eigen::Vector4f( 0.5  ,-1.3 ,-0.2  ,1.0));
    // front_box_filter_.setMax(Eigen::Vector4f( 1.5  ,1.3  , 0.3  ,1.0));
    // front_box_filter_.setNegative(false);
    updateDirectionBox(FORWARD);    //初始化为向前
    
    as_.start();
    ROS_INFO("Dual LiDAR obstacle detection initialized");
    ROS_INFO("LiDAR1: Front-Left, LiDAR2: Back-Right");
}

void ObstacleDetect::livoxFrontCallBack1(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if (current_detect_lidar == 1) {
        processLidarData(msg, 1);
    }
}
void ObstacleDetect::livoxFrontCallBack2(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if (current_detect_lidar == 2) {
        processLidarData(msg, 2);
    }
}
void ObstacleDetect::processLidarData(const sensor_msgs::PointCloud2::ConstPtr& msg, int lidar_id)
{
    // if(!forward_mode_FLAG_) return;
    
    try {
        // 获取当前运动方向
        MoveDirection direction = getCurrentDirection();
        
        // 将点云从雷达坐标系变换到车辆坐标系
        sensor_msgs::PointCloud2 transformed_cloud;
        geometry_msgs::TransformStamped transform = 
            tf_buffer_.lookupTransform("base_link", msg->header.frame_id, 
                                     ros::Time(0), ros::Duration(0.1));
        
        tf2::doTransform(*msg, transformed_cloud, transform);
        
        // 转换到PCL格式
        pcl::fromROSMsg(transformed_cloud, *processing_pc_);
        
        // 使用当前方向的检测区域
        pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pc(new pcl::PointCloud<pcl::PointXYZ>());
        front_box_filter_.setInputCloud(processing_pc_);
        front_box_filter_.filter(*collision_pc);
        
        // 发布检测结果
        sensor_msgs::PointCloud2 collision_pc_msg;
        pcl::toROSMsg(*collision_pc, collision_pc_msg);
        collision_pc_msg.header.frame_id = "base_link";
        collision_pc_msg.header.stamp = ros::Time::now();
        collision_pc_pub_.publish(collision_pc_msg);
        
        // 障碍物判断逻辑
        if(collision_pc->width > obstacle_threshold_){
            lidar_stop_obstacle_FLAG_ = true;
            ROS_WARN_THROTTLE(1.0, "LiDAR%d Obstacle detected in direction %d! Points: %d", 
                             lidar_id, direction, collision_pc->width);
        } else {
            lidar_stop_obstacle_FLAG_ = false;
        }
        
        // 调试信息
        ROS_DEBUG_THROTTLE(2.0, "LiDAR%d processing: %d points, collision points: %d", 
                          lidar_id, processing_pc_->width, collision_pc->width);
        
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF transformation failed for LiDAR%d: %s", lidar_id, ex.what());
        return;
    }
}

void ObstacleDetect::cmdvelCallback(const yhs_can_msgs::ctrl_cmd msg_){
    std::lock_guard<std::mutex> lock(vel_mutex_);
    current_cmd_vel_ = msg_;

    //速度判断阈值
    double velocity_threshold = 0.2;
    // 更新运动方向和雷达序号
    MoveDirection newDirection = STOPPED;
    int new_detect_lidar = current_detect_lidar;

    if(fabs(msg_.ctrl_cmd_x_linear) > velocity_threshold){
        if(msg_.ctrl_cmd_x_linear > 0){
            newDirection = FORWARD;
            new_detect_lidar = 1;   //向前运动，使用雷达1
        }
        else{
            newDirection = BACKWARD;
            new_detect_lidar = 2;   //向后运动，使用雷达2
        }
    }
    else if(fabs(msg_.ctrl_cmd_y_linear) > velocity_threshold){
        if(msg_.ctrl_cmd_y_linear > 0){
            newDirection = LEFT;
            new_detect_lidar = 1;   //向左运动，使用雷达1
        }
        else{
            newDirection = RIGHT;
            new_detect_lidar = 2;   //向右运动，使用雷达2
        }
    }
    else{
        newDirection = STOPPED;
    }

    updateDirectionBox(newDirection);
    //更新正在检测的雷达
    if(new_detect_lidar != current_detect_lidar){
        current_detect_lidar = new_detect_lidar;
        ROS_INFO("Switched to LiDAR%d for %s movement", 
                current_detect_lidar, 
                (newDirection == FORWARD) ? "FORWARD" :
                (newDirection == BACKWARD) ? "BACKWARD" :
                (newDirection == LEFT) ? "LEFT" :
                (newDirection == RIGHT) ? "RIGHT" : "STOPPED");
    }
}

void ObstacleDetect::updateDirectionBox(MoveDirection direction_){
    switch(direction_){
        case FORWARD:   // X方向前进
            front_box_filter_.setMin(Eigen::Vector4f(front_start_, -square_size_/2, height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(front_start_ + square_size_, square_size_/2, height_max_, 1.0));
            break;
            
        case BACKWARD:  // X方向后退
            front_box_filter_.setMin(Eigen::Vector4f(-front_start_ - square_size_, -square_size_/2, height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(-front_start_, square_size_/2, height_max_, 1.0));
            break;
            
        case LEFT:  // Y方向左移（Y正方向为左）
            front_box_filter_.setMin(Eigen::Vector4f(-square_size_/2, front_start_, height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(square_size_/2, front_start_ + square_size_, height_max_, 1.0));
            break;
            
        case RIGHT:  // Y方向右移（Y负方向为右）
            front_box_filter_.setMin(Eigen::Vector4f(-square_size_/2, -front_start_ - square_size_, height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(square_size_/2, -front_start_, height_max_, 1.0));
            break;
            
        case STOPPED:  // 停止时使用向前检测作为默认
        default:
            front_box_filter_.setMin(Eigen::Vector4f(front_start_, -square_size_/2, height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(front_start_ + square_size_, square_size_/2, height_max_, 1.0));
            break;
    }
    front_box_filter_.setNegative(false);
}

MoveDirection ObstacleDetect::getCurrentDirection(){
    std::lock_guard<std::mutex> lock(vel_mutex_);
    double linear_threshold = 0.2;
    
    if (fabs(current_cmd_vel_.ctrl_cmd_x_linear) > linear_threshold) {
        return (current_cmd_vel_.ctrl_cmd_x_linear > 0) ? FORWARD : BACKWARD;
    } 
    else if (fabs(current_cmd_vel_.ctrl_cmd_y_linear) > linear_threshold) {
        return (current_cmd_vel_.ctrl_cmd_y_linear > 0) ? LEFT : RIGHT;
    }
    else {
        return STOPPED;
    }
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_detect");
    
    try {
        ObstacleDetect obstacle_detect;
        ROS_INFO("Obstacle Detect node started successfully");
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize Obstacle Detect node: %s", e.what());
        return -1;
    }
    
    return 0;
}
