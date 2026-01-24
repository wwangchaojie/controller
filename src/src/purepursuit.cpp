#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/steering_ctrl_cmd.h"

#include <std_msgs/Bool.h>



// #include "purepursuit.h"
#define M_PI 3.14159265358979323846
using namespace std;



class PurePursuit
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_odom_;
    ros::Subscriber sub_newodom_;
    ros::Subscriber sub_endtarget_;
    ros::Subscriber sub_path_;
    ros::Subscriber sub_stop_flag_;
    ros::Subscriber sub_path_changed_;
    ros::Subscriber sub_yaw_need_correct_;
    ros::Subscriber sub_yaw_correct_complete_;
    ros::Subscriber sub_safe_;
    ros::Subscriber sub_start_;
    ros::Subscriber sub_double_car_;


    ros::Publisher pub_cmd_;
    ros::Publisher pub_yhs_cmd_;
    ros::Publisher pub_index_;
    ros::Publisher pub_targetpos_;
    ros::Publisher pub_endpos_;
    ros::Publisher pub_yaw_need_correct_;
    ros::Publisher pub_yaw_complete_correct_;
    ros::Publisher pub_start_;
    ros::Publisher pub_arrive_;
    ros::Publisher pub_arrive_double_;




    std::string odom_topic_;
    std::string path_topic_;
    std::string safe_topic_;
    std::string cmd_vel_topic_;
    std::string steering_ctrl_cmd_vel_topic_;
    std::string stop_flag_topic_;
    std::string map_topic_;

    Eigen::Vector2d vehiclePos_;
    Eigen::Quaterniond vehicleQuat_;
    double vehicleYaw_;
    geometry_msgs::Twist cmd_vel;

    //yhs底盘指令
    yhs_can_msgs::ctrl_cmd yhs_cmd_vel_;
    yhs_can_msgs::steering_ctrl_cmd yhs_cmd_vel2_;


    bool change_path_ = false;
    double lookahead_distance_ = 0.0;
    bool safe_ = true;
    double integral_error_ = 0.0;
    double pos_error_last_ = 0.0;
    double integral_yaw_error_ = 0.0;
    double yaw_error_last_ = 0.0;
    ros::Time last_time_ = ros::Time::now();
    Eigen::Vector2d pos_error_xy_last_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d integral_xy_error_ = Eigen::Vector2d::Zero();
    double vehicle_max_speed_ = 0.3;
    double vehicle_min_speed_ = 0.01;
    double vehicle_avg_speed_ = 0.0;
    double dis_end_to_me = 0.0;
    int index = 1;
    int count = 0;
    int traget_count_ = 0;
    double total_time_ = 0.0;
    ros::Time start_time_ = ros::Time::now();
    ros::Time last_odom_time_ = ros::Time::now();
    ros::Time guaidian_timer_ = ros::Time::now();

    bool start_ = false;
    // lfhTODO:
    bool odom_fail = false;


    bool is_need_stop_ = false;
    bool odom_received_ = false;
    bool is_stopping_ = false;
    bool is_adjusting_ = false;

    double yaw_error_integral_ = 0.0;     //yaw角误差积分项
    double yaw_previous_error_ = 0.0;
    double yaw_error_max_threshold_ = 0.0;
    double yaw_error_min_threshold_ = 0.0;
    double max_angular_z_ = 0.0;
    double kp_yaw_ = 0.0;
    double ki_yaw_ = 0.0;
    double kd_yaw_ = 0.0;
    ros::Time yaw_previous_time_ = ros::Time::now();
    //发布停止命令时的持续时间
    ros::Time stop_start_time_ = ros::Time::now();
    double stop_duration_time_ = 0.0;

    bool is_currently_adjusting_ = false;    //调整yaw角时pause
    bool is_currently_waiting_ = false;
    ros::Time current_adjust_start_time_ = ros::Time::now();
    ros::Time current_wait_start_time_ = ros::Time::now();
    double total_adjustment_time_ = 0.0;     // 累计暂停时间
    bool other_car_need_adjust_yaw_ = false;
    bool other_car_complete_adjust_yaw_ = false;
    double last_yaw_ = 0.0;
    int adjust_condition_count_ = 0;    //记录yaw角超出阈值的次数，满足一定次数再开始调整yaw，避免临界值附近跳变
    bool guaidian_ = false;
    bool next_path_ = false;
    double atan0_ = 0.0;
    int car2_path_index_ = 0;
    int last_index_ = 0;
    Eigen::Vector2d vehicleendPos_;
    double vehicleendYaw_;
    Eigen::Vector2d targetPos_;
    double targetYaw_;
    bool first_yaw_ = false;
    bool first_pos_ = false;
    double vehicle_max_yaw_vel_ = 15;
    double vehicle_min_yaw_vel_ = 1;

    double integral_end_yaw_error_ = 0.0;
    double end_yaw_error_last_ = 0.0;
    Eigen::Vector2d integral_end_xy_error_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d end_pos_error_xy_last_ = Eigen::Vector2d::Zero();



    double end_yaw_kp_ = 0.0;
    double end_yaw_ki_ = 0.0;
    double end_yaw_kd_ = 0.0;
    double end_pos_kp_ = 0.0;
    double end_pos_ki_ = 0.0;
    double end_pos_kd_ = 0.0;

    std::vector<Eigen::Vector2d> tar_pos_vec;
    std::vector<double> tar_yaw_vec;
    double chelength = 1.8;



    void param_init(ros::NodeHandle nh_);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void rob_odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void targetPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    double yaw_PID(double yaw_error);
    Eigen::Vector2d pos_PID(Eigen::Vector2d pos_error);
    void AdjustYaw(double yaw_error);
    void AdjustPos(Eigen::Vector2d pos_error);



    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void stopFlagCallBack(const std_msgs::Bool::ConstPtr& msg);
    void safeCallback(const std_msgs::Bool::ConstPtr& msg);
    void pathChangedCallback(const std_msgs::Bool::ConstPtr& msg);
    void otherCarYaw_correctCallback(const std_msgs::Bool::ConstPtr& msg);
    void otherCarYaw_completeCallback(const std_msgs::Bool::ConstPtr& msg);
    
    double PositionPID_V(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos);
    Eigen::Vector2d PositionPID_VxVy(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos, double kp, double kd, double ki, double dt);
    // 原来的阿克曼模式下调整yaw的函数，现在用不到了
    double YawPID(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos, double vehicleYaw);
    double getTheta(double ref_theta, double vehicleYaw);
    double getBackTheta(double ref_theta, double vehicleYaw, int &vehicle_inv_vel_);
    void publishStopCommand();
    void startCallback(const std_msgs::Bool::ConstPtr& msg);
    void doubleCarCallback(const std_msgs::Bool::ConstPtr& msg);







public:
    void PurePursuitbyVel();
    PurePursuit(ros::NodeHandle nh, ros::NodeHandle nhPrivate);
    ~PurePursuit(){};
    void goalyaw();
    double normalizeAngle(double angle);
    double getYawFromQuaternion(const geometry_msgs::Quaternion& quat);
    void adjustYawAngle(double yaw_error);
    double PID_adjustYaw(double error);
    void FSM();
    bool double_car_ = false;
    bool end_goal_pose_ = false;
    bool get_target_ = false;

    int end_cout_ = 0;
    nav_msgs::Path path_;
    

};


PurePursuit::PurePursuit(ros::NodeHandle nh, ros::NodeHandle nhPrivate) : nh_(nh)
{
    param_init(nh_);
    // sub_odom_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &PurePursuit::odomCallback, this);
    if(double_car_){
        sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic_, 1, &PurePursuit::pathCallback, this);
    }else{
        sub_path_ = nh_.subscribe<nav_msgs::Path>("/car1_astar_path_o", 1, &PurePursuit::pathCallback, this);
    }
    sub_stop_flag_ = nh_.subscribe<std_msgs::Bool>(stop_flag_topic_, 1, &PurePursuit::stopFlagCallBack, this);
    sub_safe_ = nh_.subscribe<std_msgs::Bool>(safe_topic_, 1, &PurePursuit::safeCallback, this);
    sub_path_changed_ = nh_.subscribe<std_msgs::Bool>("/path_change", 1, &PurePursuit::pathChangedCallback, this);
    sub_start_ = nh_.subscribe<std_msgs::Bool>("/start", 1, &PurePursuit::startCallback, this);
    sub_double_car_ = nh_.subscribe<std_msgs::Bool>("/double_car", 1, &PurePursuit::doubleCarCallback, this);
    sub_yaw_need_correct_ = nh_.subscribe<std_msgs::Bool>("/car2/yaw_need_correct", 1, &PurePursuit::otherCarYaw_correctCallback, this);
    sub_yaw_correct_complete_ = nh_.subscribe<std_msgs::Bool>("/car2/yaw_correct_complete", 1, &PurePursuit::otherCarYaw_completeCallback, this);
    sub_newodom_ = nh_.subscribe<nav_msgs::Odometry>(odom_topic_, 1, &PurePursuit::rob_odomCallback, this);
    sub_endtarget_ = nh_.subscribe<geometry_msgs::PoseArray>("/target_poses", 1, &PurePursuit::targetPoseCallback, this);


    pub_cmd_ = nh_.advertise<yhs_can_msgs::ctrl_cmd>(cmd_vel_topic_, 1);
    pub_yhs_cmd_ = nh_.advertise<yhs_can_msgs::steering_ctrl_cmd>(steering_ctrl_cmd_vel_topic_, 1);

    pub_index_ = nh_.advertise<std_msgs::Int32>("/car2/car2_path_index_pub", 1);   
    pub_targetpos_ = nh_.advertise<geometry_msgs::PoseStamped>("path_index_pos", 1);    

    pub_yaw_need_correct_ = nh_.advertise<std_msgs::Bool>("/car1/yaw_need_correct" ,1);
    pub_yaw_complete_correct_ = nh_.advertise<std_msgs::Bool>("/car1/yaw_complete_correct", 1);

    pub_start_ = nh_.advertise<std_msgs::Bool>("/start" ,1);
    pub_arrive_ = nh_.advertise<std_msgs::Bool>("/AGV005/tag_arrived" ,1);
    pub_arrive_double_ = nh_.advertise<std_msgs::Bool>("/point_arrived" ,1);
    pub_endpos_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_pos", 1, true); 

  
    
    
    
}


void PurePursuit::param_init(ros::NodeHandle nh_)
{
    // 话题
    nh_.param("purepursuit_node/safe_topic", safe_topic_, std::string("/safe"));
    nh_.param("purepursuit_node/odom_topic", odom_topic_, std::string("/lio/robot/odom"));
    nh_.param("purepursuit_node/path_topic", path_topic_, std::string("/astar_path_o"));
    nh_.param("purepursuit_node/cmd_vel_topic", cmd_vel_topic_, std::string("/ctrl_cmd"));
    nh_.param("purepursuit_node/steering_ctrl_cmd_vel_topic", steering_ctrl_cmd_vel_topic_, std::string("/steering_ctrl_cmd"));

    nh_.param("purepursuit_node/stop_flag_topic", stop_flag_topic_, std::string("/stop_flag"));
    // nh_.param("purepursuit_node/yaw_need_correct_topic", stop_flag_topic_, std::string("/car1/yaw_need_correct"));

    // 参数
    nh_.param("purepursuit_node/double_car", double_car_, false);
    nh_.param("purepursuit_node/lookahead_distance", lookahead_distance_, 0.1);
    nh_.param("purepursuit_node/vehicle_max_speed", vehicle_max_speed_, 0.3);
    nh_.param("purepursuit_node/vehicle_min_speed", vehicle_min_speed_, 0.01);
    nh_.param("purepursuit_node/yaw_error_max_threshold", yaw_error_max_threshold_, 10.0);
    nh_.param("purepursuit_node/yaw_error_min_threshold", yaw_error_min_threshold_, 3.0);
    nh_.param("purepursuit_node/stop_duration_time_", stop_duration_time_, 1.5);
    nh_.param("purepursuit_node/max_angular_z_", max_angular_z_, 0.3);
    nh_.param("purepursuit_node/kp_yaw_", kp_yaw_, 0.5);
    nh_.param("purepursuit_node/ki_yaw_", ki_yaw_, 0.01);
    nh_.param("purepursuit_node/kd_yaw_", kd_yaw_, 0.1);
    nh_.param("purepursuit_node/vehicle_avg_speed", vehicle_avg_speed_, 0.5);
    nh_.param("purepursuit_node/end_yaw_kp_", end_yaw_kp_, 0.5);
    nh_.param("purepursuit_node/end_yaw_ki_", end_yaw_ki_, 0.01);
    nh_.param("purepursuit_node/end_yaw_kd_", end_yaw_kd_, 0.1);
    nh_.param("purepursuit_node/end_pos_kp_", end_pos_kp_, 0.68);
    nh_.param("purepursuit_node/end_pos_ki_", end_pos_ki_, 0.03);
    nh_.param("purepursuit_node/end_pos_kd_", end_pos_kd_, 0.8);
    nh_.param("purepursuit_node/vehicle_max_yaw_vel_", vehicle_max_yaw_vel_, 0.2);
    nh_.param("purepursuit_node/vehicle_min_yaw_vel_", vehicle_min_yaw_vel_, 0.01);
    is_adjusting_ = false;
    is_stopping_ = false;
    is_currently_waiting_ = false;
    other_car_need_adjust_yaw_ = false;
    other_car_complete_adjust_yaw_ = true;
    yaw_error_integral_ = 0.0;
    yaw_previous_error_ = 0.0;
    yaw_previous_time_ = ros::Time::now();
    last_time_ = ros::Time::now();
    adjust_condition_count_ = 0;
}


void PurePursuit::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    vehiclePos_ << msg->pose.pose.position.x, 
                    msg->pose.pose.position.y;
    // vehicleYaw_ = getYawFromQuaternion(msg->pose.pose.orientation);
    vehicleQuat_.w() = msg->pose.pose.orientation.w;
    vehicleQuat_.x() = msg->pose.pose.orientation.x;
    vehicleQuat_.y() = msg->pose.pose.orientation.y;
    vehicleQuat_.z() = msg->pose.pose.orientation.z;

    vehicleYaw_ = tf2::getYaw(msg->pose.pose.orientation);

    std::cout<<"odom_update"<<std::endl;
    ros::Time odom_time = ros::Time::now();
    double odom_dt = (odom_time - last_odom_time_).toSec();
    if(odom_dt < 0.2){
        odom_fail = false;
    }
}


void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_.poses.clear();
    path_ = *msg;
    if (!path_.poses.empty()) {
        // 获取起点坐标
        geometry_msgs::Point start_point = path_.poses.front().pose.position;
        // 获取终点坐标
        geometry_msgs::Point end_point = path_.poses.back().pose.position;
        
        // 计算两点之间的欧几里得距离
        double distance = abs(end_point.x - start_point.x) + abs(end_point.y - start_point.y);
        total_time_ = distance / vehicle_avg_speed_;
        ROS_INFO("Distance from start to end of path: %.3f meters", distance);
    } else {
        ROS_WARN("Path is empty, cannot calculate distance");
    }
    index = 1;
    change_path_ = false;   
    start_ = false;
    guaidian_ = false;
    next_path_ = false;
    end_goal_pose_ = false;
    pos_error_xy_last_ = Eigen::Vector2d::Zero();
    integral_xy_error_ = Eigen::Vector2d::Zero();
    integral_end_yaw_error_ = 0.0;
    end_yaw_error_last_ = 0.0;
    integral_end_xy_error_ = Eigen::Vector2d::Zero();
    end_pos_error_xy_last_ = Eigen::Vector2d::Zero();
    integral_yaw_error_ = 0.0;
    yaw_error_last_ = 0.0;
    last_time_ = ros::Time::now();
    first_yaw_ = true;
    first_pos_ = true;
    traget_count_ = 0;
    get_target_ = false;
    end_cout_ = 0;
    yaw_error_integral_ = 0.0;


    // 接受到新路径后，总调整时间重置
    total_adjustment_time_ = 0.0;
    guaidian_timer_ = ros::Time::now();

    ROS_INFO("path size: %ld", path_.poses.size());
}

void PurePursuit::stopFlagCallBack(const std_msgs::Bool::ConstPtr& msg)
{
    is_need_stop_ = msg->data;
}
void PurePursuit::safeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    safe_ = msg->data;
}
void PurePursuit::pathChangedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    change_path_ = msg->data;
}

void PurePursuit::otherCarYaw_correctCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // 只有双车的时候需要这个回调
    if(double_car_ && msg->data){
        ROS_WARN("收到对方需要调整yaw的消息!!");
        other_car_need_adjust_yaw_ = true;
        other_car_complete_adjust_yaw_ = false;
    }
}
void PurePursuit::otherCarYaw_completeCallback(const std_msgs::Bool::ConstPtr& msg)
{
    // 只有双车的时候需要这个回调
    if(double_car_ && msg->data){
        ROS_WARN("收到对方yaw调整已经完成的消息!!");
        other_car_need_adjust_yaw_ = false;
        other_car_complete_adjust_yaw_ = true;
    }
}


// double PurePursuit::PositionPID_V(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos)
// {
//     double k_ = 1.0;
//     double kp_ = 0.5;
//     double kd_ = 0.1;
//     double ki_ = 0.01;
//     double dt_ = 0.1;
//     // P
//     double pos_error_ = (target_pos - current_pos).norm();
//     // I
//     integral_error_ += pos_error_;
//     // D
//     double pos_error_dot_ = (pos_error_ - pos_error_last_) / dt_;
//     pos_error_last_ = pos_error_;
//     // total
//     double pos_vel_ = k_ * (kp_ * pos_error_ + ki_ * integral_error_ + kd_ * pos_error_dot_);

//     return pos_vel_;
// }

void PurePursuit::startCallback(const std_msgs::Bool::ConstPtr& msg)
{
    start_ = msg->data;
    start_time_ = ros::Time::now();
}
void PurePursuit::doubleCarCallback(const std_msgs::Bool::ConstPtr& msg)
{
    bool double_car_new = msg->data;
    if(double_car_ == double_car_new) return;

    double_car_ = double_car_new;

    // shutdown old subscibers:
    sub_path_.shutdown();

    // generate new topics:
    std::string path_topic_new = double_car_ ? "/car1_astar_path_o" : "/car1_astar_path_o";
    // subscribe new topics:
    sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic_new, 1, &PurePursuit::pathCallback, this);
}




// bug: 在横移时，yaw角已经没有了参考意义
double PurePursuit::YawPID(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos, double vehicleYaw)
{
    double k_ = 1.0;
    double kp_ = 0.5;
    double kd_ = 0.1;
    double ki_ = 0.01;
    double dt_ = 0.1;
    // P
    double yaw_error_ = atan2(target_pos(1) - current_pos(1), target_pos(0) - current_pos(0)) - vehicleYaw;
    
    while(yaw_error_ > M_PI) yaw_error_ -= 2 * M_PI;
    while(yaw_error_ <= -M_PI) yaw_error_ += 2 * M_PI;
    // I
    integral_yaw_error_ += yaw_error_;
    // D`
    double yaw_error_dot_ = (yaw_error_ - yaw_error_last_) / dt_;
    yaw_error_last_ = yaw_error_;
    // total
    double yaw_vel = k_ * (kp_ * yaw_error_ + ki_ * integral_yaw_error_ + kd_ * yaw_error_dot_);

    return yaw_vel;
}

Eigen::Vector2d PurePursuit::PositionPID_VxVy(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos, double k_ = 2.0, 
                                                                                                        double kp_ = 0.8, 
                                                                                                        double ki_ = 0.001,
                                                                                                        double kd_ = 0.8
                                                                                                        )
{
    // P
    Eigen::Vector2d pos_error_ = target_pos - current_pos;
    // I
    integral_xy_error_ += pos_error_;
    // 添加积分限幅防止windup
    Eigen::Vector2d integral_max(0.5, 0.5); // XY积分上限
    integral_xy_error_ = integral_xy_error_.cwiseMin(integral_max).cwiseMax(-integral_max);
    // D
    ros::Time current_time = ros::Time::now();
    double dt_ = (current_time - last_time_).toSec();
    last_time_ = current_time;
    // if(dt_>0.2) return Eigen::Vector2d(0.0,0.0);
    Eigen::Vector2d pos_error_dot_ = (pos_error_ - pos_error_xy_last_) / dt_;
    pos_error_xy_last_ = pos_error_;

    // total
    Eigen::Vector2d pos_vel_ = k_ * (kp_ * pos_error_ + ki_ * integral_xy_error_ + kd_ * pos_error_dot_);
    std::cout << "pos_vel_: " << pos_vel_(0) << " " << pos_vel_(1) << std::endl;
    return pos_vel_;
}

double PurePursuit::getBackTheta(double ref_theta, double vehicleYaw, int &vehicle_inv_vel_)
{
    double theta = ref_theta - vehicleYaw;
    while(theta > M_PI/2){
        theta -= M_PI;
        vehicle_inv_vel_ *= -1;
        std::cout << "theta--: " << theta << std::endl;
    } 
    while(theta <= -M_PI/2){
        theta += M_PI;
        vehicle_inv_vel_ *= -1;
        std::cout << "theta++: " << theta << std::endl;
    }
    return theta;
}

double PurePursuit::getTheta(double ref_theta, double vehicleYaw)
{
    double theta = ref_theta - vehicleYaw;
    while(theta > M_PI){
        theta -= 2 * M_PI;
        std::cout << "2 PI theta--: " << theta << std::endl;
    } 
    while(theta <= -M_PI){
        theta += 2 * M_PI;
        std::cout << "2 PI theta++: " << theta << std::endl;
    }
    return theta;
}

double PurePursuit::getYawFromQuaternion(const geometry_msgs::Quaternion& quat){
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quat, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

double PurePursuit::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void PurePursuit::adjustYawAngle(double yaw_error)
{
    ROS_ERROR(" I AM FUCKED!!!!!!!!!!!!!!!!!!!!!!");
    yhs_can_msgs::ctrl_cmd yhs_cmd_vel;

    yhs_cmd_vel.ctrl_cmd_gear = 6;
    yhs_cmd_vel.ctrl_cmd_z_angular = 0.0;
    yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;

    if(!is_adjusting_ && !is_stopping_)     //yaw超过了阈值，但还没停也没调整时
    {
        double yaw_error_deg = yaw_error * 180 / M_PI;
        ROS_INFO("yaw error: %.1f , exceeds max threshold %.1f , initiating stop and adjust",
                            yaw_error_deg,      yaw_error_max_threshold_);
        
        is_stopping_ = true;
        stop_start_time_ = ros::Time::now();

        publishStopCommand();
        return;
    }

    if(is_stopping_)
    {
        double stop_elapsed_time = (ros::Time::now() - stop_start_time_).toSec();

        if(stop_elapsed_time > stop_duration_time_)
        {
            //超过停止时间，开始调整yaw角
            is_stopping_ = false;
            is_adjusting_ = true;

            // 重置pid积分项，避免历史误差
            yaw_error_integral_ = 0.0;
            yaw_previous_error_ = yaw_error;
            yaw_previous_time_ = ros::Time::now();
        }
        else
        {
            //还没到停止时间，继续停止
            publishStopCommand();
            ROS_INFO("stopping...");
            return;
        }
    }
    if(is_adjusting_){
        // 调整yaw的代码
        double angular_z = PID_adjustYaw(yaw_error);

        yhs_cmd_vel.ctrl_cmd_gear = 6;
        yhs_cmd_vel.ctrl_cmd_z_angular = -angular_z;
        yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
        ROS_INFO_THROTTLE(1, "Adjusting yaw : error=%.1f°, angular_z=%.3f degree/s", 
                            yaw_error * 180.0 / M_PI, angular_z);
    }
    pub_cmd_.publish(yhs_cmd_vel);

}

double PurePursuit::PID_adjustYaw(double error)
{
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - yaw_previous_time_).toSec();

    if(dt <= 0.0 || dt > 1.0)
    {
        dt = 0.01;  //10ms
    }

    //i
    yaw_error_integral_ += error * dt;
    //积分限幅，防止积分饱和
    double yaw_error_integral_max = (max_angular_z_ * M_PI / 180) / ki_yaw_;
    if(yaw_error_integral_ > yaw_error_integral_max) yaw_error_integral_ = yaw_error_integral_max;
    if(yaw_error_integral_ < -yaw_error_integral_max) yaw_error_integral_ = -yaw_error_integral_max;

    //d
    double derivative = (error - yaw_previous_error_) / dt;

    //pid
    double output = kp_yaw_ * error + ki_yaw_ * yaw_error_integral_ + kd_yaw_ * derivative;
    double output_deg = output * 180 / M_PI;
    ROS_INFO("output_degree = %.3f", output_deg);

    //输出限幅
    if(output_deg > max_angular_z_) output_deg = max_angular_z_;
    if(output_deg < -max_angular_z_) output_deg = -max_angular_z_;

    //update state
    yaw_previous_error_ = error;
    yaw_previous_time_ = current_time;

    return output_deg;
}

void PurePursuit::publishStopCommand()
{
    ROS_ERROR("I AM IN PUBLISH STOP FUNC~~~~~~~~~~~~~~~~~~~~");
    yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
    // yhs_cmd_vel.ctrl_cmd_gear = 8;
    // yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
    // yhs_cmd_vel.ctrl_cmd_y_linear = 0.0;
    // pub_cmd_.publish(yhs_cmd_vel);
    yhs_cmd_vel2_.ctrl_cmd_gear = 7;
    yhs_cmd_vel2_.steering_ctrl_cmd_velocity = 0.0;
    yhs_cmd_vel2_.steering_ctrl_cmd_steering = 0.0;
    pub_yhs_cmd_.publish(yhs_cmd_vel2_);

    ROS_DEBUG_THROTTLE(2, "Publishing stop command");
}

void PurePursuit::FSM()
{
    last_odom_time_ = ros::Time::now();
    ros::Rate detect_stop_rate(10);
    // 1. 先处理障碍物停止逻辑
    if(is_need_stop_) 
    {
        ROS_INFO("need to stop!!!");
        while(is_need_stop_ && ros::ok())
        {
            // 停止1秒内，保持速度为0
            publishStopCommand();
            detect_stop_rate.sleep();
            // return;  // 直接返回，不执行路径跟踪
            ros::spinOnce();    // 确保ctrl+C可以被执行
        }
    } 

    // 2. 检查是否需要处理其他车辆的yaw调整请求
    if (other_car_need_adjust_yaw_ && !other_car_complete_adjust_yaw_) {
        // ROS_INFO("the other car needs to adjust yaw, I stop and wait ~ ~ ~ ~");
        ROS_INFO("另一辆车正在调整yaw角,我停车等待 ~ ~ ~ ~ ~");
        if(!is_currently_waiting_){
            is_currently_waiting_ = true;
            // 开始计时
            current_wait_start_time_ = ros::Time::now();
        }
        // 发停止命令
        publishStopCommand();
        return;
    }else{
        if(is_currently_waiting_){
            // 对方已经完成yaw角的修正，开始计算总用时：
            
            ros::Duration(2.0).sleep();
            double adjust_duration = (ros::Time::now() - current_wait_start_time_).toSec();
            total_adjustment_time_ += adjust_duration;
            // ROS_INFO("Other vehicle yaw adjustment complete, resuming...");
            ROS_INFO("另一辆车调整yaw角校正已结束,恢复正常运动,并添加调整时间 %.2f 至总调整时间: %.2f秒",
                        adjust_duration, total_adjustment_time_);
            is_currently_waiting_ = false;
        }
    }

    // 3.  计算yaw的误差，再和阈值作比较
    double yaw_error = normalizeAngle(vehicleYaw_);
    double yaw_error_deg = yaw_error * 180 / M_PI;
    ROS_INFO("car_yaw : %.3f , error : %.3f", vehicleYaw_ * 180 / M_PI, yaw_error_deg);


    // 4.  检查是否需要开始yaw调整
    if(fabs(yaw_error_deg) > yaw_error_max_threshold_ && !is_adjusting_ && !is_stopping_)
    {
        adjust_condition_count_++;

        // 开始yaw调整，记录开始时间
        if(adjust_condition_count_ > 5 && !is_currently_adjusting_) {
            is_currently_adjusting_ = true;
            is_adjusting_ = true;

            std_msgs::Bool request_msg;
            request_msg.data = true;
            pub_yaw_need_correct_.publish(request_msg);

            current_adjust_start_time_ = ros::Time::now();
            // ROS_INFO("Starting yaw adjustment, recording start time");
            ROS_WARN("HAVE PUBLISHED TO OTHER CAR!!!!!!!!!");
            ROS_INFO("自车开始校正yaw角,记录时间中...");
        }
    }else{
        adjust_condition_count_ = 0;
    }

    // 5.  检查yaw调整是否完成
    if(is_adjusting_ && fabs(yaw_error_deg) < yaw_error_min_threshold_)
    {
        // yaw调整完成，累计调整时间
        if(is_currently_adjusting_) {
            is_currently_adjusting_ = false;
            // is_adjusting_ = false;
            is_stopping_ = false;

            std_msgs::Bool complete_msg;
            complete_msg.data = true;
            pub_yaw_complete_correct_.publish(complete_msg);
            ROS_WARN("HAVE PUBLISHED TO OTHER CAR ~~~~ `~~~~~ ` `~  ~");
            ros::Duration(2.0).sleep();

            double adjust_duration = (ros::Time::now() - current_adjust_start_time_).toSec();
            total_adjustment_time_ += adjust_duration;
            // ROS_INFO("Yaw adjustment completed. Added %.2fs to total adjustment time (now: %.2fs)", 
            //             adjust_duration, total_adjustment_time_);
            ROS_INFO("yaw角校正完成. 添加 %.2f 秒至总调整时间 (当前总调整时间: %.2f秒)", 
                        adjust_duration, total_adjustment_time_);
        }
    }
    // std::cout<<"yaw_error_deg: "<<yaw_error_deg <<std::endl;
    // std::cout<<"yaw_error_max_threshold_: "<<yaw_error_max_threshold_ <<std::endl;
    // std::cout<<"adjust_condition_count_: "<<adjust_condition_count_ <<std::endl;
    // std::cout<<"is_adjusting_: "<<is_adjusting_ <<std::endl;
    // std::cout<<"yaw_error_min_threshold_: "<<yaw_error_min_threshold_ <<std::endl;




    // 6.  校正yaw角
    if(fabs(yaw_error_deg) > yaw_error_max_threshold_ && adjust_condition_count_ > 5)
    {
        // 超过误差阈值，开始校正yaw角
        ROS_INFO("now we are in condition 111111111111111, need to adjust yaw!!!");
        adjustYawAngle(yaw_error);
    }else if(is_adjusting_ && fabs(yaw_error_deg) < yaw_error_min_threshold_){
        //校正完成
        is_adjusting_ = false;
        is_stopping_ = false;
        adjust_condition_count_ = 0;
        ROS_INFO("now we are in condition 222222222222222, yaw correction completed!!!!!!!");
        // 继续纯跟踪
        PurePursuitbyVel();
    }else if(is_adjusting_){
        ROS_INFO("now we are in condition 333333333333333, keep adjusting yaw angle!!!");
        adjustYawAngle(yaw_error);
    }else{
        ROS_INFO("now we are in condition 444444444444444, normal pure pursuit process");
        PurePursuitbyVel();
    }
}

void PurePursuit::PurePursuitbyVel(){
    if(!safe_ || change_path_)
    {
        // ROS_WARN(safe_);
        std::cout<<"safe_"<<safe_<<std::endl;
        std::cout<<"change_path_"<<change_path_<<std::endl;
        // yhs_cmd_vel_.ctrl_cmd_gear = 8;   // 4T4D:6  横移：8
        // yhs_cmd_vel_.ctrl_cmd_x_linear = 0.0;
        // yhs_cmd_vel_.ctrl_cmd_y_linear = 0.0;
        // pub_cmd_.publish(yhs_cmd_vel_);
        yhs_cmd_vel2_.ctrl_cmd_gear = 7;
        yhs_cmd_vel2_.steering_ctrl_cmd_velocity = 0.0;
        yhs_cmd_vel2_.steering_ctrl_cmd_steering = 0.0;
        pub_yhs_cmd_.publish(yhs_cmd_vel2_);
        ROS_WARN("not safe PurePursuit");
        ROS_ERROR("velocity is 00000000000000000000000");
        return;
    }
    if(odom_fail){
        // yhs_cmd_vel_.ctrl_cmd_gear = 8;   // 4T4D:6  横移：8
        // yhs_cmd_vel_.ctrl_cmd_x_linear = 0.0;
        // yhs_cmd_vel_.ctrl_cmd_y_linear = 0.0;
        // pub_cmd_.publish(yhs_cmd_vel_);
        yhs_cmd_vel2_.ctrl_cmd_gear = 7;
        yhs_cmd_vel2_.steering_ctrl_cmd_velocity = 0.0;
        yhs_cmd_vel2_.steering_ctrl_cmd_steering = 0.0;
        pub_yhs_cmd_.publish(yhs_cmd_vel2_);
        ROS_ERROR("odom  fail  !!");
        ROS_ERROR("velocity is 00000000000000000000000");
        return;
    }
    // odom_fail = true;
    if(path_.poses.empty())
    {
        ROS_WARN("PurePursuit is empty");
        return;
    }
    if(!start_ && double_car_)
    {
        ROS_WARN("PurePursuit not start");
        // std_msgs::Bool double_start_msg;
        // double_start_msg.data = true;
        // pub_start_.publish(double_start_msg);
        return;
    }


    ROS_INFO("find PurePursuit");

    dis_end_to_me = (vehiclePos_ - Eigen::Vector2d(path_.poses.back().pose.position.x, path_.poses.back().pose.position.y)).norm();
    // if(dis_end_to_me < 0.03)
    if(dis_end_to_me < 0.01)
    {
        ROS_INFO("arrive at end");
        if(++count >= 1){
            yhs_cmd_vel2_.ctrl_cmd_gear = 7;
            yhs_cmd_vel2_.steering_ctrl_cmd_velocity = 0.0;
            yhs_cmd_vel2_.steering_ctrl_cmd_steering = 0.0;
            pub_yhs_cmd_.publish(yhs_cmd_vel2_);
            std_msgs::Bool arrive_msg;
            arrive_msg.data = true;
            if(!double_car_){
                pub_arrive_.publish(arrive_msg);
            }else{
                pub_arrive_double_.publish(arrive_msg);
            }
            end_goal_pose_ = true;
            ROS_ERROR("arrive");
            start_ = false;
            return;
        }
        // return;
    }
    else
    {
        count = 0;
        // 双车模式，时间同步
        if(double_car_){
            // lfhTODO;zhuchedaima
            for(int i = index; i < path_.poses.size(); i++)
            {
                if(index == path_.poses.size() - 1) break;
                double dis_goal_to_me = (vehiclePos_ - Eigen::Vector2d(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y)).norm();
                double atan1 = atan2(path_.poses[i + 1].pose.position.y - path_.poses[i].pose.position.y, 
                                path_.poses[i + 1].pose.position.x - path_.poses[i].pose.position.x);
                atan0_  = atan2(path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y, 
                        path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x); 
                if(abs(atan1 - atan0_) > M_PI/3 && !next_path_){
                    index = i;
                    guaidian_ = true;
                    break;

                } else if(abs(atan1 - atan0_) > M_PI/3 && next_path_){
                    next_path_ = false;
                    continue;
                }      
                if(dis_goal_to_me > lookahead_distance_)
                {
                    index = i;
                    break;
                }
            }
            std_msgs::Int32 index_msg;
            index_msg.data = index;
            pub_index_.publish(index_msg);
        }
        // 单车模式，不用时间同步
        else{
            for(int i = index; i < path_.poses.size(); i++)
            {
                if(i == path_.poses.size() - 1) break;
                double dis_goal_to_me = (vehiclePos_ - Eigen::Vector2d(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y)).norm();
                double atan1 = atan2(path_.poses[i + 1].pose.position.y - path_.poses[i].pose.position.y, 
                                path_.poses[i + 1].pose.position.x - path_.poses[i].pose.position.x);
                atan0_  = atan2(path_.poses[i].pose.position.y - path_.poses[i - 1].pose.position.y, 
                        path_.poses[i].pose.position.x - path_.poses[i - 1].pose.position.x); 
                if(abs(atan1 - atan0_) > M_PI/3 && !next_path_){
                    index = i;
                    guaidian_ = true;
                    break;
                }
                else if(abs(atan1 - atan0_) > M_PI/3 && next_path_){
                    next_path_ = false;
                    continue;
                }      
                if(dis_goal_to_me > lookahead_distance_)
                {
                    index = i;
                    break;
                }
            }
        }      
    }
    {
        geometry_msgs::PoseStamped index_msg;
        index_msg.header.stamp = ros::Time::now();
        index_msg.header.frame_id = "map";
        index_msg.pose.position.x = path_.poses[index].pose.position.x;
        index_msg.pose.position.y = path_.poses[index].pose.position.y;
        pub_targetpos_.publish(index_msg);
    }

    Eigen::Vector2d targetPos_(path_.poses[index].pose.position.x, path_.poses[index].pose.position.y);
    Eigen::Vector2d xy_pos_vel_;

    if(index >= path_.poses.size() - 5)
    {   
        index = path_.poses.size() - 1;
        std_msgs::Int32 index_msg;
        index_msg.data = index;
        pub_index_.publish(index_msg);
        // 根据距离动态调整增益，实现渐进减速
        xy_pos_vel_ = PositionPID_VxVy(vehiclePos_, targetPos_, 1.0 , 0.68, 0.006, 0.1);
        ROS_WARN("dis_end_to_me : %f",dis_end_to_me);
    }
    else{
        if((vehiclePos_ - targetPos_).norm() < 0.05 && guaidian_){
            yhs_cmd_vel2_.ctrl_cmd_gear = 7;
            yhs_cmd_vel2_.steering_ctrl_cmd_velocity = 0.0;
            yhs_cmd_vel2_.steering_ctrl_cmd_steering = 0.0;
            pub_yhs_cmd_.publish(yhs_cmd_vel2_);
            guaidian_ = false;
            next_path_ = true;

            ros::Rate rate(5);
            rate.sleep();
            if(index < path_.poses.size() - 1){
                index ++;
            }
            guaidian_timer_ = ros::Time::now();
            return;
        }

        xy_pos_vel_ = PositionPID_VxVy(vehiclePos_, targetPos_, 1.6, 1.0, 0, 0.05);
    }
    
    double ref_vehicle_vel_ = xy_pos_vel_.norm();
    double ref_theta = atan2(xy_pos_vel_.y(), xy_pos_vel_.x());
    double yaw_vel;
    int vehicle_inv_vel_ = 1;
    if(atan0_ < - M_PI * 175 / 180 || atan0_ > M_PI * 175 / 180){
        yaw_vel = getBackTheta(ref_theta, vehicleYaw_, vehicle_inv_vel_);
    }else{
        yaw_vel = getTheta(ref_theta, vehicleYaw_);
    }
    if(ref_vehicle_vel_ > vehicle_max_speed_){
        ref_vehicle_vel_ = vehicle_max_speed_;
    }else if(ref_vehicle_vel_ < vehicle_min_speed_){
        ref_vehicle_vel_ = 0.0;
    }

    yhs_cmd_vel2_.ctrl_cmd_gear = 7;
    yhs_cmd_vel2_.steering_ctrl_cmd_velocity = ref_vehicle_vel_ * vehicle_inv_vel_;
    yhs_cmd_vel2_.steering_ctrl_cmd_steering = yaw_vel * 180 /M_PI;

    if(ros::Time::now().toSec() - guaidian_timer_.toSec() < 1.0){
        yhs_cmd_vel2_.steering_ctrl_cmd_velocity  = 0.0;
    }

    pub_yhs_cmd_.publish(yhs_cmd_vel2_);
    // last_yaw_ = yaw_vel;
    std::cout << "target_pos: " << targetPos_(0)<<","<< targetPos_(1) << std::endl;
    std::cout << "vehiclePos_: " << vehiclePos_(0)<<","<<vehiclePos_(1)  << std::endl;


    std::cout << "v: " << ref_vehicle_vel_ * vehicle_inv_vel_ << std::endl;
    std::cout << "yaw_vel: " << yaw_vel * 180 /M_PI  << std::endl;
    
    std::cout << "Vx: " << ref_vehicle_vel_ * vehicle_inv_vel_ * cos(yaw_vel)  << std::endl;
    std::cout << "Vy: " << ref_vehicle_vel_ * vehicle_inv_vel_ * sin(yaw_vel) << std::endl;

}


// 终点调节位姿的

void PurePursuit::rob_odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    vehicleendPos_ << msg->pose.pose.position.x, 
                    msg->pose.pose.position.y;
    vehicleendYaw_ = tf2::getYaw(msg->pose.pose.orientation);

    vehiclePos_ << msg->pose.pose.position.x, 
                    msg->pose.pose.position.y;

    vehicleYaw_ = tf2::getYaw(msg->pose.pose.orientation);
    ROS_WARN("rob_odom Update");
}

void PurePursuit::targetPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // lfhTODO:这里需要判断一下取数组中的哪个点
    if(end_goal_pose_  && !get_target_){
        if(msg->poses.size() == 0) return;
        if(path_.poses.empty()) return;
        int end_index = -1;

        float min_dis = 5.0;

        for(int i = 0; i < msg->poses.size(); i++){
            Eigen::Vector2d pos(msg->poses[i].position.x, msg->poses[i].position.y);
            Eigen::Vector2d end_pos(path_.poses.back().pose.position.x, path_.poses.back().pose.position.y);    
            double dis = (end_pos - pos).norm();
            if(dis < 4.0){
                if(dis < min_dis){
                    min_dis = dis;
                    end_index = i;
                }
            }
        }

        if(end_index != -1){
            if(++traget_count_ == 10){
                if(end_index < 3){
                    targetPos_ << msg->poses[end_index].position.x - chelength * cos(tf2::getYaw(msg->poses[end_index].orientation)), 
                                msg->poses[end_index].position.y - chelength * sin(tf2::getYaw(msg->poses[end_index].orientation));
                }else if(end_index == 3 || end_index == 4 ){
                    targetPos_ << msg->poses[end_index].position.x + chelength * cos(tf2::getYaw(msg->poses[end_index].orientation)), 
                                msg->poses[end_index].position.y + chelength * sin(tf2::getYaw(msg->poses[end_index].orientation));
                } 
                geometry_msgs::PoseStamped target_pos;
                target_pos.header.stamp = ros::Time::now();
                target_pos.header.frame_id = "map";
                target_pos.pose.position.x = targetPos_(0);
                target_pos.pose.position.y = targetPos_(1);
                pub_endpos_.publish(target_pos);

                targetYaw_ = tf2::getYaw(msg->poses[end_index].orientation);
                get_target_ = true;
            }
            ROS_ERROR("traget_count_: %ld", traget_count_);
        } 
    }
}



void PurePursuit::goalyaw(){
    if (!get_target_) return;  // 没锁定目标就不调
    double end_yaw_error = targetYaw_ - vehicleendYaw_;
    double end_pos_error = (targetPos_ - vehicleendPos_).norm();
    // if(abs(end_yaw_error) > M_PI / 180){
    //     AdjustYaw(end_yaw_error);
    // }else if(abs(end_yaw_error) < M_PI / 180 && end_pos_error > 0.03){
    //     end_cout_ = 0;
    //     AdjustPos(targetPos_ - vehicleendPos_);
    // }else if(abs(end_yaw_error) < M_PI / 180 && end_pos_error < 0.03 && ++end_cout > 3){
    //     end_goal_pose_ = false;
    //     get_target_ = false;
    //     yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
    //     yhs_cmd_vel.ctrl_cmd_gear = 8;
    //     yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
    //     yhs_cmd_vel.ctrl_cmd_y_linear = 0.0;
    //     pub_cmd_.publish(yhs_cmd_vel);
    // }

    while(end_pos_error > 0.01 && end_cout_ <= 4){
        ros::spinOnce();
        AdjustPos(targetPos_ - vehicleendPos_);
    }
    ROS_INFO("end_cout_1: %ld",end_cout_);
    if(abs(end_yaw_error) > 2 * M_PI / 180 && end_cout_ > 3){
        ros::spinOnce();
        AdjustYaw(end_yaw_error);
        ROS_INFO("end_yaw_error: %f",end_yaw_error);
    }else if(abs(end_yaw_error) < 2 * M_PI / 180 && end_cout_ > 3){
        end_goal_pose_ = false;
        get_target_ = false;
        yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
        yhs_cmd_vel.ctrl_cmd_gear = 8;
        yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
        yhs_cmd_vel.ctrl_cmd_y_linear = 0.0;
        pub_cmd_.publish(yhs_cmd_vel);
        end_cout_ = 0;
    }


}

void PurePursuit::AdjustYaw(double end_yaw_error){
    if(first_yaw_){
        first_yaw_ = false;
        first_pos_ = true;
        yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
        yhs_cmd_vel.ctrl_cmd_gear = 6;
        yhs_cmd_vel.ctrl_cmd_z_angular = 0.0;
        yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
        pub_cmd_.publish(yhs_cmd_vel);
    }
    double yaw_vel = yaw_PID(end_yaw_error);
    if(yaw_vel * 180 / M_PI > vehicle_max_yaw_vel_){
        yaw_vel = vehicle_max_yaw_vel_;
    }else if(yaw_vel * 180 / M_PI < -vehicle_max_yaw_vel_){
        yaw_vel = -vehicle_max_yaw_vel_;
    }else if(abs(yaw_vel * 180 / M_PI) < vehicle_min_yaw_vel_){
        yaw_vel = 0.0;
    }
    yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
    yhs_cmd_vel.ctrl_cmd_gear = 6;
    yhs_cmd_vel.ctrl_cmd_z_angular = yaw_vel * 180 / M_PI;
    yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
    pub_cmd_.publish(yhs_cmd_vel);

}



void PurePursuit::AdjustPos(Eigen::Vector2d end_pos_error){
    if(first_pos_){
        first_yaw_ = true;
        first_pos_ = false;
        yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
        yhs_cmd_vel.ctrl_cmd_gear = 8;
        yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
        yhs_cmd_vel.ctrl_cmd_y_linear = 0.0;
        pub_cmd_.publish(yhs_cmd_vel);
        ros::Rate rate(10);
        return;
    }
    if(end_pos_error.norm()<0.03){
        end_cout_++;
    }else{
        end_cout_ = 0;
        Eigen::Vector2d vx_vy = pos_PID(end_pos_error);

        if(vx_vy(0) > vehicle_max_speed_){
            vx_vy(0) = vehicle_max_speed_;
        }else if(vx_vy(0) < -vehicle_max_speed_){
            vx_vy(0) = -vehicle_max_speed_;
        }else if(abs(vx_vy(0)) < vehicle_min_speed_){
            vx_vy(0) = 0.0;
        }

        if(vx_vy(1) > vehicle_max_speed_){
            vx_vy(1) = vehicle_max_speed_;
        }else if(vx_vy(1) < -vehicle_max_speed_){
            vx_vy(1) = -vehicle_max_speed_;
        }else if(abs(vx_vy(1)) < vehicle_min_speed_){
            vx_vy(1) = 0.0;
        }
        yhs_can_msgs::ctrl_cmd yhs_cmd_vel;
        yhs_cmd_vel.ctrl_cmd_gear = 8;
        yhs_cmd_vel.ctrl_cmd_x_linear = vx_vy(0);
        yhs_cmd_vel.ctrl_cmd_y_linear = vx_vy(1);
        pub_cmd_.publish(yhs_cmd_vel);

    }


}

double PurePursuit::yaw_PID(double yaw_error)
{
    // P
    double yaw_error_ = yaw_error;

    // I
    integral_end_yaw_error_ += yaw_error_;
    // 添加积分限幅防止windup
    double integral_max = 0.5; // XY积分上限
    integral_end_yaw_error_ = std::max(std::min(integral_end_yaw_error_, integral_max), -integral_max);

    // D
    ros::Time current_time = ros::Time::now();
    double dt_ = (current_time - last_time_).toSec();
    last_time_ = current_time;
    double end_yaw_error_dot_ = (yaw_error_ - end_yaw_error_last_) / dt_;
    end_yaw_error_last_ = yaw_error_;

    // total
    double yaw_vel = end_yaw_kp_ * yaw_error_ + end_yaw_ki_ * integral_end_yaw_error_ + end_yaw_kd_ * end_yaw_error_dot_;
    std::cout << "yaw_vel: " << yaw_vel << std::endl;
    return yaw_vel;
}


Eigen::Vector2d PurePursuit::pos_PID(Eigen::Vector2d pos_error)
{
    // P
    Eigen::Vector2d pos_error_ = pos_error;

    // I
    integral_end_xy_error_ += pos_error_;
    // 添加积分限幅防止windup
    Eigen::Vector2d integral_max(0.5, 0.5); // XY积分上限
    integral_end_xy_error_ = integral_end_xy_error_.cwiseMin(integral_max).cwiseMax(-integral_max);

    // D
    ros::Time current_time = ros::Time::now();
    double dt_ = (current_time - last_time_).toSec();
    last_time_ = current_time;
    // if(dt_>0.2) return Eigen::Vector2d(0.0,0.0);
    Eigen::Vector2d pos_error_dot_ = (pos_error_ - end_pos_error_xy_last_) / dt_;
    end_pos_error_xy_last_ = pos_error_;
    
    // total
    Eigen::Vector2d pos_vel_ = end_pos_kp_ * pos_error_ + end_pos_ki_ * integral_end_xy_error_ + end_pos_kd_ * pos_error_dot_;
    // std::cout << "pos_vel_: " << pos_vel_(0) << " " << pos_vel_(1) << std::endl;
    return pos_vel_;
}


int main(int argc, char **argv)
{
    setlocale(LC_CTYPE,"zh_CN.utf8");
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    PurePursuit pure_pursuit(nh, nhPrivate);
    ros::Rate looprate(10);
    ros::Publisher pub_double_car_state = nh.advertise<std_msgs::Bool>("/double_car_state",1);
    while (ros::ok())
    {
        ros::spinOnce();
        std_msgs::Bool double_car_state_msg;
        double_car_state_msg.data = pure_pursuit.double_car_;
        pub_double_car_state.publish(double_car_state_msg);
        // pure_pursuit.PurePursuitbyVel();
        if(pure_pursuit.end_goal_pose_ && pure_pursuit.get_target_){
            ROS_WARN("end_adjust");
            pure_pursuit.goalyaw();
            pure_pursuit.path_.poses.clear();

        }else if(!pure_pursuit.end_goal_pose_){
            pure_pursuit.FSM();
        }
        looprate.sleep();
    }
    return 0;
}