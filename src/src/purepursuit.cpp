#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include "yhs_can_msgs/ctrl_cmd.h"



// #include "purepursuit.h"
#define M_PI 3.14159265358979323846
using namespace std;



class PurePursuit
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_path_;
    ros::Publisher pub_cmd_;

    std::string odom_topic_;
    std::string path_topic_;
    std::string cmd_vel_topic_;

    Eigen::Vector2d vehiclePos_;
    Eigen::Quaterniond vehicleQuat_;
    double vehicleYaw_;
    nav_msgs::Path path_;
    geometry_msgs::Twist cmd_vel;

    //yhs底盘指令
    yhs_can_msgs::ctrl_cmd yhs_cmd_vel;



    bool change_path_;
    double lookahead_distance_;
    double integral_error_;
    double pos_error_last_;
    double integral_yaw_error_;
    double yaw_error_last_;
    ros::Time last_time_;
    Eigen::Vector2d pos_error_xy_last_;
    Eigen::Vector2d integral_xy_error_;
    double vehicle_max_speed_;
    double dis_end_to_me;
    int index;
    int count;





    void param_init(ros::NodeHandle nh_);
    void odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    double PositionPID_V(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos);
    Eigen::Vector2d PositionPID_VxVy(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos, double kp, double kd, double ki, double dt);
    double YawPID(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos, double vehicleYaw);
    double getTheta(double ref_theta, double vehicleYaw, int &vehicle_inv_vel_);



    



public:
    void PurePursuitbyVel();
    PurePursuit(ros::NodeHandle nh, ros::NodeHandle nhPrivate);
    ~PurePursuit(){};
};


PurePursuit::PurePursuit(ros::NodeHandle nh, ros::NodeHandle nhPrivate) : nh_(nh)
{
    param_init(nh_);
    sub_odom_ = nh_.subscribe<geometry_msgs::PoseStamped>(odom_topic_, 1, &PurePursuit::odomCallback, this);
    sub_path_ = nh_.subscribe<nav_msgs::Path>(path_topic_, 1, &PurePursuit::pathCallback, this);
    //TODO: 消息类型
    pub_cmd_ = nh_.advertise<yhs_can_msgs::ctrl_cmd>(cmd_vel_topic_, 1);
    
}


void PurePursuit::param_init(ros::NodeHandle nh_)
{
    nh_.param("purepursuit_node/lookahead_distance", lookahead_distance_, 0.1);
    nh_.param("purepursuit_node/odom_topic", odom_topic_, std::string("/mavros/vision_pose/pose"));
    nh_.param("purepursuit_node/path_topic", path_topic_, std::string("/astar_path_o"));
    // nh_.param("purepursuit_node/cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nh_.param("purepursuit_node/cmd_vel_topic", cmd_vel_topic_, std::string("/ctrl_cmd"));
    nh_.param("purepursuit_node/vehicle_max_speed", vehicle_max_speed_, 1.0);
    last_time_ = ros::Time::now();
}


void PurePursuit::odomCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    vehiclePos_ << msg->pose.position.x, 
                    msg->pose.position.y;
    vehicleQuat_.w() = msg->pose.orientation.w;
    vehicleQuat_.x() = msg->pose.orientation.x;
    vehicleQuat_.y() = msg->pose.orientation.y;
    vehicleQuat_.z() = msg->pose.orientation.z;

    vehicleYaw_ = tf2::getYaw(msg->pose.orientation);
    std::cout<<"odom_update"<<std::endl;
}


void PurePursuit::pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    change_path_ = true;
    path_.poses.clear();
    path_ = *msg;
    index = 0;
    ROS_INFO("path size: %ld", path_.poses.size());
}

double PurePursuit::PositionPID_V(Eigen::Vector2d current_pos, Eigen::Vector2d target_pos)
{
    double k_ = 1.0;
    double kp_ = 0.5;
    double kd_ = 0.1;
    double ki_ = 0.01;
    double dt_ = 0.1;
    // P
    double pos_error_ = (target_pos - current_pos).norm();
    // I
    integral_error_ += pos_error_;
    // D
    double pos_error_dot_ = (pos_error_ - pos_error_last_) / dt_;
    pos_error_last_ = pos_error_;
    // total
    double pos_vel_ = k_ * (kp_ * pos_error_ + ki_ * integral_error_ + kd_ * pos_error_dot_);

    return pos_vel_;
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
    double yaw_vel_ = k_ * (kp_ * yaw_error_ + ki_ * integral_yaw_error_ + kd_ * yaw_error_dot_);

    return yaw_vel_;
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
    // D
    ros::Time current_time = ros::Time::now();
    double dt_ = (current_time - last_time_).toSec();
    last_time_ = current_time;
    pos_error_xy_last_ = pos_error_;
    if(dt_>0.2) return Eigen::Vector2d(0.0,0.0);
    Eigen::Vector2d pos_error_dot_ = (pos_error_ - pos_error_xy_last_) / dt_;
    // total
    Eigen::Vector2d pos_vel_ = k_ * (kp_ * pos_error_ + ki_ * integral_xy_error_ + kd_ * pos_error_dot_);
    std::cout << "pos_vel_: " << pos_vel_(0) << " " << pos_vel_(1) << std::endl;
    return pos_vel_;
}

double PurePursuit::getTheta(double ref_theta, double vehicleYaw, int &vehicle_inv_vel_)
{
    double theta = ref_theta - vehicleYaw;
    std::cout << "ref_theta: " << ref_theta << std::endl;
    std::cout << "vehicleYaw: " << vehicleYaw << std::endl;
    std::cout << "theta: " << theta << std::endl;
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



void PurePursuit::PurePursuitbyVel(){
    change_path_ = false;
    if(path_.poses.empty())
    {
        ROS_WARN("path is empty");
        return;
    }
    ROS_INFO("find path");



    if(change_path_)
    {
        ROS_WARN("change path");
        return;
    }
    dis_end_to_me = (vehiclePos_ - Eigen::Vector2d(path_.poses.back().pose.position.x, path_.poses.back().pose.position.y)).norm();
    if(dis_end_to_me < 0.1)
    {
        index = path_.poses.size() - 1;
        yhs_cmd_vel.ctrl_cmd_gear = 8;   // 4T4D:6  横移：8
        yhs_cmd_vel.ctrl_cmd_x_linear = 0.0;
        yhs_cmd_vel.ctrl_cmd_y_linear = 0.0;
        pub_cmd_.publish(yhs_cmd_vel);
        // cmd_vel.linear.x = 0.0;
        // cmd_vel.angular.z = 0.0;
        ROS_INFO("arrive at end");
        // pub_cmd_.publish(cmd_vel);
        if(count++ > 10) path_.poses.clear();
        return;
    }
    else
    {
        count = 0;
        for(int i = index; i < path_.poses.size(); i++)
        {
            dis_end_to_me = (vehiclePos_ - Eigen::Vector2d(path_.poses[i].pose.position.x, path_.poses[i].pose.position.y)).norm();
            if(dis_end_to_me > lookahead_distance_)
            {
                index = i;
                break;
            }
        }
    }
    Eigen::Vector2d targetPos_(path_.poses[index].pose.position.x, path_.poses[index].pose.position.y);
    std::cout << "targetPos_: " << targetPos_.transpose() << std::endl;
    Eigen::Vector2d xy_pos_vel_;
    if(index >= path_.poses.size() - 5)
    {
        index = path_.poses.size() - 1;
        xy_pos_vel_ = PositionPID_VxVy(vehiclePos_, targetPos_, 0.5, 0.8, 0, 0.1);
    }else{
        xy_pos_vel_ = PositionPID_VxVy(vehiclePos_, targetPos_);
    }

    double ref_vehicle_vel_ = xy_pos_vel_.norm();
    double ref_theta = atan2(xy_pos_vel_.y(), xy_pos_vel_.x());
    int vehicle_inv_vel_ = 1;
    double yaw_vel_ = getTheta(ref_theta, vehicleYaw_, vehicle_inv_vel_);
    std::cout << "ref_vehicle_vel_: " << ref_vehicle_vel_ << std::endl;
    double vehicle_vel_ = ref_vehicle_vel_ * vehicle_inv_vel_;

    if(vehicle_vel_ > vehicle_max_speed_)
    {
        ROS_WARN("vehicle_vel+__ out of range");
        vehicle_vel_ = vehicle_max_speed_;
    }else if(vehicle_vel_ < -vehicle_max_speed_)
    {
        ROS_WARN("vehicle_vel-_ out of range");
        vehicle_vel_ = -vehicle_max_speed_;
    }
    std::cout << "index: " << index << std::endl;
    std::cout << "vehicle_vel_: " << vehicle_vel_ << std::endl;
    std::cout << "yaw_vel_: " << yaw_vel_ << std::endl;
    std::cout << "vehiclePos_: " << vehiclePos_[0] << " " << vehiclePos_[1] << std::endl;



    yhs_cmd_vel.ctrl_cmd_gear = 8;   // 4T4D:6  横移：8
    yhs_cmd_vel.ctrl_cmd_x_linear = vehicle_vel_ * cos(yaw_vel_);
    yhs_cmd_vel.ctrl_cmd_y_linear = vehicle_vel_ * sin(yaw_vel_);
    pub_cmd_.publish(yhs_cmd_vel);

    // cmd_vel.linear.x = vehicle_vel_;
    // cmd_vel.angular.z = yaw_vel_;
    // pub_cmd_.publish(cmd_vel);


}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    PurePursuit pure_pursuit(nh, nhPrivate);
    ros::Rate looprate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        pure_pursuit.PurePursuitbyVel();
        looprate.sleep();
    }
    return 0;
}