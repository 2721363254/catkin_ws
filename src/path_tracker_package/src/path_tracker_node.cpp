#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

class DronePathTracker
{
public:
    DronePathTracker(ros::NodeHandle& nh)
    {
        path_sub_ = nh.subscribe("/mission/path", 1, &DronePathTracker::pathCallback, this);
        odom_sub_ = nh.subscribe("/Odometry", 1, &DronePathTracker::odomCallback, this);
        cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        land_pub_ = nh.advertise<std_msgs::Bool>("/mission/land", 1);

        target_index_ = 0;
        has_path_ = false;
        has_odom_ = false;
        is_landing_ = false;
        
        // 3D控制参数
        nh.param("lookahead_dist_xy", lookahead_dist_xy_, 1.0);
        nh.param("lookahead_dist_z", lookahead_dist_z_, 0.5);
        nh.param("max_linear_vel_xy", max_linear_vel_xy_, 1.0);
        nh.param("max_linear_vel_z", max_linear_vel_z_, 0.5);
        nh.param("max_angular_vel", max_angular_vel_, 0.5);
        nh.param("xy_gain", xy_gain_, 1.5);
        nh.param("z_gain", z_gain_, 1.0);
        nh.param("yaw_gain", yaw_gain_, 1.0);
        nh.param("landing_speed", landing_speed_, 0.1);
        nh.param("position_tolerance", position_tolerance_, 0.1);
        nh.param("yaw_tolerance", yaw_tolerance_, 0.05);
        
        ROS_INFO("Drone Path Tracker initialized.");
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        path_ = *msg;
        target_index_ = 0;
        has_path_ = true;
        is_landing_ = false;
        ROS_INFO("Received 3D path with %lu points", path_.poses.size());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // 获取当前位置
        current_pose_.position = msg->pose.pose.position;
        current_pose_.orientation = msg->pose.pose.orientation;
        
        // 获取当前速度
        current_vel_.linear = msg->twist.twist.linear;
        current_vel_.angular = msg->twist.twist.angular;
        
        // 计算欧拉角
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
            
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
        
        has_odom_ = true;
    }

    void followPath()
    {
        if (!has_path_ || !has_odom_) return;

        // 检查是否到达路径终点
        if (target_index_ >= path_.poses.size()) 
        {
            if (!is_landing_) 
            {
                ROS_INFO("Reached final waypoint. Starting landing procedure...");
                is_landing_ = true;
            }
            publishStop();
            return;
        }

        // 获取当前目标点
        geometry_msgs::PoseStamped target_pose = path_.poses[target_index_];
        
        // 计算位置误差
        double dx = target_pose.pose.position.x - current_pose_.position.x;
        double dy = target_pose.pose.position.y - current_pose_.position.y;
        double dz = target_pose.pose.position.z - current_pose_.position.z;
        
        // 计算水平距离和总距离
        double horizontal_dist = std::hypot(dx, dy);
        double total_dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        // ROS_INFO("Target: (%.2f, %.2f, %.2f), Current: (%.2f, %.2f, %.2f), Dist: %.2f", 
        //          target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z,
        //          current_pose_.position.x, current_pose_.position.y, current_pose_.position.z, total_dist);
        
        // 检查是否到达当前目标点
        if (horizontal_dist < lookahead_dist_xy_ && std::abs(dz) < lookahead_dist_z_)
        {
            target_index_++;
            ROS_INFO("Reached waypoint %lu, moving to next", target_index_);
            return;
        }
        
        // 计算目标偏航角（朝向目标点）
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = normalizeAngle(target_yaw - current_yaw_);
        
        // 计算期望速度
        geometry_msgs::Twist cmd;
        
        // 比例控制
        cmd.linear.x = xy_gain_ * dx;  // X方向速度
        cmd.linear.y = xy_gain_ * dy;  // Y方向速度
        cmd.linear.z = z_gain_ * dz;   // Z方向速度
        
        // 速度限幅
        double vel_xy = std::hypot(cmd.linear.x, cmd.linear.y);
        if (vel_xy > max_linear_vel_xy_) 
        {
            cmd.linear.x *= max_linear_vel_xy_ / vel_xy;
            cmd.linear.y *= max_linear_vel_xy_ / vel_xy;
        }
        
        cmd.linear.z = std::max(std::min(cmd.linear.z, max_linear_vel_z_), -max_linear_vel_z_);
        
        // 偏航控制（可选，可以根据目标姿态调整）
        cmd.angular.z = yaw_gain_ * yaw_error;
        cmd.angular.z = std::max(std::min(cmd.angular.z, max_angular_vel_), -max_angular_vel_);
        
        // 发布控制指令
        cmd_pub_.publish(cmd);
        
        // 可选：发布到日志
        // ROS_WARN("Control: vx=%.2f, vy=%.2f, vz=%.2f, wz=%.2f", 
        //           cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
    }

private:
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher land_pub_;
    
    nav_msgs::Path path_;
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_vel_;
    
    size_t target_index_;
    bool has_path_;
    bool has_odom_;
    bool is_landing_;
    
    // 控制参数
    double lookahead_dist_xy_;
    double lookahead_dist_z_;
    double max_linear_vel_xy_;
    double max_linear_vel_z_;
    double max_angular_vel_;
    double xy_gain_;
    double z_gain_;
    double yaw_gain_;
    double landing_speed_;
    double position_tolerance_;
    double yaw_tolerance_;
    
    // 当前姿态
    double current_yaw_;
    
    void publishStop()
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_.publish(cmd);
        std_msgs::Bool land_msg;
        land_msg.data = true;
        land_pub_.publish(land_msg);
        ROS_INFO("Drone stopped.");
    }
    
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_path_tracker_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    DronePathTracker tracker(nh);
    
    ros::Rate rate(50);  // 50Hz控制频率
    
    while (ros::ok())
    {
        ros::spinOnce();
        tracker.followPath();
        rate.sleep();
    }
    
    return 0;
}