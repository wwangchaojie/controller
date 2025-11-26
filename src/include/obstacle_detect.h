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
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

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


enum MoveDirection{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    STOPPED
};

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
    ros::Publisher stop_flag_pub_;
    ros::Publisher detection_box_pub_;

    int dep_count_thres_;
    double dep_value_thres_;

    pcl::CropBox<pcl::PointXYZ> front_box_filter_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr processing_pc_;

    sensor_msgs::PointCloud2 pointcloud_data_pub_;

    MoveDirection curDirection;     //当前运动方向
    yhs_can_msgs::ctrl_cmd current_cmd_vel_;  // 当前速度命令
    std::mutex vel_mutex_;  // 速度数据互斥锁

    // 检测区域参数
    double vehicle_length,vehicle_width,safety_margin;
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

    //flag
    bool lidar_stop_obstacle_FLAG_ = false;
    std_msgs::Bool stop_flag;


private:
    void livoxFrontCallBack1(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void livoxFrontCallBack2(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void processLidarData(const sensor_msgs::PointCloud2::ConstPtr& msg, int lidar_id);
    // Rviz调试用的函数 
    void publishDetectionBox(MoveDirection direction);

public:
    ObstacleDetect();
    ~ObstacleDetect(){};
    //速度回调，根据速度判断前进还是横移
    void cmdvelCallback(const yhs_can_msgs::ctrl_cmd msg_);
    // 判断运动方向
    MoveDirection getCurrentDirection();
    // 根据运动方向更新检测框
    void updateDirectionBoxDoubleLidars(MoveDirection direction_);
    void updateDirectionBoxSingleLidar(MoveDirection direction_);


};