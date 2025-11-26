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
#include "obstacle_detect.h"

using namespace std;


ObstacleDetect::ObstacleDetect():
as_(ros::AsyncSpinner(2, &self_queue_)),
tf_listener_(tf_buffer_),  // 添加TF监听器初始化
current_detect_lidar(1)   //默认用雷达1
{
    nh_.setCallbackQueue(&self_queue_);
    livox_pc_front_sub_1 = nh_.subscribe("/lio/robo/cloud_uav", 10, &ObstacleDetect::livoxFrontCallBack1, this);  //注意雷达话题
    livox_pc_front_sub_2 = nh_.subscribe("/livox/lidar2", 10, &ObstacleDetect::livoxFrontCallBack2, this);  //注意雷达话题
    cmd_vel_sub_ = nh_.subscribe("/ctrl_cmd",10, &ObstacleDetect::cmdvelCallback,this);
    collision_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/collision_pc", 1);
    stop_flag_pub_ = nh_.advertise<std_msgs::Bool>("/stop_flag",1);
    detection_box_pub_ = nh_.advertise<visualization_msgs::Marker>("/detection_box_debug", 1);  // 调试发布器
    
    ros::NodeHandle private_nh("~");
    // 检测参数
    private_nh.param("square_size", square_size_, 1.0);
    private_nh.param("vehicle_length", vehicle_length, 0.8);
    private_nh.param("vehicle_width", vehicle_width, 0.5);
    private_nh.param("safety_margin", safety_margin, 0.2);
    private_nh.param("obstacle_threshold", obstacle_threshold_, 20);
    // TODO: 检测高度范围
    private_nh.param("height_min", height_min_, -1.0);
    private_nh.param("height_max", height_max_, -0.3);

    // 初始化点云
    processing_pc_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    updateDirectionBoxSingleLidar(FORWARD);    //初始化为向前
    
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
        // sensor_msgs::PointCloud2 transformed_cloud;
        // geometry_msgs::TransformStamped transform = 
        //     tf_buffer_.lookupTransform("base_link", msg->header.frame_id, 
        //                              ros::Time(0), ros::Duration(0.1));
        
        // tf2::doTransform(*msg, transformed_cloud, transform);
        
        // 转换到PCL格式
        pcl::fromROSMsg(*msg, *processing_pc_);
        
        // 使用当前方向的检测区域
        pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pc(new pcl::PointCloud<pcl::PointXYZ>());
        front_box_filter_.setInputCloud(processing_pc_);
        front_box_filter_.filter(*collision_pc);
        
        // 发布检测结果
        sensor_msgs::PointCloud2 collision_pc_msg;
        pcl::toROSMsg(*collision_pc, collision_pc_msg);
        collision_pc_msg.header.frame_id = "uav";
        collision_pc_msg.header.stamp = ros::Time::now();
        collision_pc_pub_.publish(collision_pc_msg);
        
        static std::deque<int> collision_history;
        collision_history.push_back(collision_pc->width);

        // 保持历史数据长度为10帧
        if (collision_history.size() > 20) {
            collision_history.pop_front();
        }
        bool has_obstacle = false;
        for (int count : collision_history) {
            if (count > obstacle_threshold_) {
                has_obstacle = true;
                break;  // 只要有一帧满足条件就跳出循环
            }
        }
        // 障碍物判断逻辑
        if(has_obstacle){
            stop_flag.data = true;
            ROS_WARN_THROTTLE(1.0, "LiDAR%d Obstacle detected in direction %d! Points: %d", 
                             lidar_id, direction, collision_pc->width);
            stop_flag_pub_.publish(stop_flag);
            ros::Rate rate(1);
            rate.sleep();

        } else {
            stop_flag.data = false;
            stop_flag_pub_.publish(stop_flag);
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

    if((fabs(msg_.ctrl_cmd_x_linear) > velocity_threshold) && fabs(msg_.ctrl_cmd_y_linear < velocity_threshold)){
        if(msg_.ctrl_cmd_x_linear > 0){
            newDirection = FORWARD;
            new_detect_lidar = 1;   //向前运动，使用雷达1
        }
        else{
            newDirection = BACKWARD;
            new_detect_lidar = 2;   //向后运动，使用雷达2
        }
    }
    else if((fabs(msg_.ctrl_cmd_y_linear) > velocity_threshold) && fabs(msg_.ctrl_cmd_x_linear < velocity_threshold)){
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

    // updateDirectionBoxDoubleLidars(newDirection);
    updateDirectionBoxSingleLidar(newDirection);
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

void ObstacleDetect::updateDirectionBoxSingleLidar(MoveDirection direction_){
    switch(direction_){
        case FORWARD:   // X正方向前进
            // 检测区域：车头前方
            front_box_filter_.setMin(Eigen::Vector4f(
                vehicle_length/2 + safety_margin,  // x_min: 车头前方开始
                -vehicle_width/2,                  // y_min: 车体左侧
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2 + safety_margin + square_size_,  // x_max: 向前延伸
                vehicle_width/2,                   // y_max: 车体右侧
                height_max_, 1.0));
            break;
            
        case BACKWARD:  // X负方向后退
            // 检测区域：车尾后方
            front_box_filter_.setMin(Eigen::Vector4f(
                -vehicle_length/2 - safety_margin - square_size_, // x_min: 向后延伸
                -vehicle_width/2,                  // y_min: 车体左侧
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                -vehicle_length/2 - safety_margin, // x_max: 车尾后方开始
                vehicle_width/2,                   // y_max: 车体右侧
                height_max_, 1.0));
            break;
            
        case LEFT:  // Y正方向左移（Y正方向确实是左侧）
            // 检测区域：车体左侧
            front_box_filter_.setMin(Eigen::Vector4f(
                -vehicle_length/2,                  // x_min: 车体前部
                vehicle_width/2 + safety_margin,    // y_min: 车体左侧开始
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2,                   // x_max: 车体后部
                vehicle_width/2 + safety_margin + square_size_, // y_max: 向左延伸
                height_max_, 1.0));
            break;
            
        case RIGHT:  // Y负方向右移（Y负方向是右侧）
            // 检测区域：车体右侧
            front_box_filter_.setMin(Eigen::Vector4f(
                -vehicle_length/2,                  // x_min: 车体前部
                -vehicle_width/2 - safety_margin - square_size_, // y_min: 向右延伸
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2,                   // x_max: 车体后部
                -vehicle_width/2 - safety_margin,  // y_max: 车体右侧开始
                height_max_, 1.0));
            break;
            
        case STOPPED:  // 停止时检测所有方向
        default:
            // 停止时可以设置一个较小的全向检测区域
            front_box_filter_.setMin(Eigen::Vector4f(
                -vehicle_length/2 - safety_margin, 
                -vehicle_width/2 - safety_margin, 
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2 + safety_margin, 
                vehicle_width/2 + safety_margin, 
                height_max_, 1.0));
            break;
    }
    front_box_filter_.setNegative(false);
    // 发布调试可视化
    publishDetectionBox(direction_);
}
void ObstacleDetect::updateDirectionBoxDoubleLidars(MoveDirection direction_){
    switch(direction_){
        case FORWARD:   // X正方向前进
            // 检测区域：车头前方
            front_box_filter_.setMin(Eigen::Vector4f(
                vehicle_length/2 + safety_margin,  // x_min: 车头前方开始
                -vehicle_width/2,                  // y_min: 车体左侧
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2 + safety_margin + square_size_,  // x_max: 向前延伸
                vehicle_width/2,                   // y_max: 车体右侧
                height_max_, 1.0));
            break;
            
        case BACKWARD:  // X负方向后退
            // 检测区域：车尾后方
            front_box_filter_.setMin(Eigen::Vector4f(
                vehicle_length/2 + safety_margin,  // x_min: 车头前方开始
                -vehicle_width/2,                  // y_min: 车体左侧
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2 + safety_margin + square_size_,  // x_max: 向前延伸
                vehicle_width/2,                   // y_max: 车体右侧
                height_max_, 1.0));
            break;
            
        case LEFT:  // Y正方向左移（Y正方向确实是左侧）
            // 检测区域：车体左侧
            front_box_filter_.setMin(Eigen::Vector4f(
                -vehicle_length/2,                  // x_min: 车体前部
                vehicle_width/2 + safety_margin,    // y_min: 车体左侧开始
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2,                   // x_max: 车体后部
                vehicle_width/2 + safety_margin + square_size_, // y_max: 向左延伸
                height_max_, 1.0));
            break;
            
        case RIGHT:  // Y负方向右移（Y负方向是右侧）
            // 检测区域：车体右侧
            front_box_filter_.setMin(Eigen::Vector4f(
                -vehicle_length/2,                  // x_min: 车体前部
                vehicle_width/2 + safety_margin,    // y_min: 车体左侧开始
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2,                   // x_max: 车体后部
                vehicle_width/2 + safety_margin + square_size_, // y_max: 向左延伸
                height_max_, 1.0));
            break;
            
        case STOPPED:  // 停止时检测所有方向或默认向前
        default:
            // 停止时可以设置一个较小的全向检测区域
            front_box_filter_.setMin(Eigen::Vector4f(
                -vehicle_length/2 - safety_margin, 
                -vehicle_width/2 - safety_margin, 
                height_min_, 1.0));
            front_box_filter_.setMax(Eigen::Vector4f(
                vehicle_length/2 + safety_margin, 
                vehicle_width/2 + safety_margin, 
                height_max_, 1.0));
            break;
    }
    front_box_filter_.setNegative(false);
    // 发布调试可视化
    publishDetectionBox(direction_);
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

void ObstacleDetect::publishDetectionBox(MoveDirection direction) {
    visualization_msgs::Marker marker;
    
    // 基础设置
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "detection_box";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
                
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;  // 蓝色
    marker.color.a = 0.3;

    double center_x = 0.0, center_y = 0.0, center_z = 0.0;
    double size_x = 0.0, size_y = 0.0, size_z = 0.0;
    
    // 计算高度中心
    center_z = (height_min_ + height_max_) / 2.0;
    size_z = height_max_ - height_min_;
    
    switch(direction) {
        case FORWARD:  // X方向前进
            center_x = (vehicle_length + square_size_ )/ 2.0 + safety_margin;
            center_y = 0.0;
            size_x = square_size_;
            size_y = square_size_;
            break;
            
        case BACKWARD:  // X方向后退
            center_x = -(vehicle_length + square_size_ )/ 2.0 - safety_margin;
            center_y = 0.0;
            size_x = square_size_;
            size_y = square_size_;
            break;
            
        case LEFT:  // Y方向左移
            center_x = 0.0;
            center_y = (vehicle_width + square_size_ )/ 2.0 + safety_margin;
            size_x = square_size_;
            size_y = square_size_;
            break;
            
        case RIGHT:  // Y方向右移
            center_x = 0.0;
            center_y = -(vehicle_width + square_size_ )/ 2.0 - safety_margin;
            size_x = square_size_;
            size_y = square_size_;
            break;
            
        case STOPPED:  // 停止
        default:
            center_x = 0.0;
            center_y = 0.0;
            size_x = vehicle_length / 2.0 + safety_margin;
            size_y = vehicle_width / 2.0 + safety_margin;
            break;
    }
    
    // 设置位置和尺寸
    marker.pose.position.x = center_x;
    marker.pose.position.y = center_y;
    marker.pose.position.z = center_z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = size_x;
    marker.scale.y = size_y;
    marker.scale.z = size_z;
    
    // 设置生命周期
    marker.lifetime = ros::Duration(0.2);
    
    // 发布标记
    detection_box_pub_.publish(marker);
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
