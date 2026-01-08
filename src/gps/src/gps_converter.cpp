// gps_converter.cpp
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <GeographicLib/UTMUPS.hpp>
#include <math.h>
#include <mutex>

class GPSConverter {
private:
    ros::NodeHandle nh_, private_nh_;
    
    // 订阅器
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber local_odom_sub_;
    
    // 发布器
    ros::Publisher odom_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher initial_pose_pub_;
    
    // TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    
    // 参数
    double origin_lat_, origin_lon_, origin_alt_;
    std::string base_frame_id_, odom_frame_id_, map_frame_id_, body_frame_id_;
    bool publish_initial_pose_;
    bool use_enu_to_ned_;
    bool publish_tf_;
    
    // 状态
    bool origin_set_;
    bool first_fix_received_;
    bool static_tf_published_;
    
    // 当前位置和姿态
    double current_x_, current_y_, current_z_;
    double current_roll_, current_pitch_, current_yaw_;
    
    // UTM相关
    double origin_easting_, origin_northing_;
    int utm_zone_;
    bool utm_north_;
    
    // 线程安全
    std::mutex mutex_;
    
public:
    GPSConverter() : 
        private_nh_("~"),
        origin_set_(false),
        first_fix_received_(false),
        static_tf_published_(false),
        current_x_(0.0), current_y_(0.0), current_z_(0.0),
        current_roll_(0.0), current_pitch_(0.0), current_yaw_(0.0),
        origin_easting_(0.0), origin_northing_(0.0),
        utm_zone_(0), utm_north_(true) {
        
        // 获取参数
        private_nh_.param("origin_lat", origin_lat_, 0.0);
        private_nh_.param("origin_lon", origin_lon_, 0.0);
        private_nh_.param("origin_alt", origin_alt_, 0.0);
        private_nh_.param("publish_initial_pose", publish_initial_pose_, true);
        private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
        private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
        private_nh_.param("map_frame_id", map_frame_id_, std::string("map"));
        private_nh_.param("body_frame_id", body_frame_id_, std::string("body"));
        private_nh_.param("use_enu_to_ned", use_enu_to_ned_, false);
        private_nh_.param("publish_tf", publish_tf_, true);
        
        ROS_INFO("GPS Converter initialized");
        ROS_INFO("Origin: lat=%f, lon=%f, alt=%f", origin_lat_, origin_lon_, origin_alt_);
        ROS_INFO("Frames: map=%s, odom=%s, base_link=%s, body=%s", 
                 map_frame_id_.c_str(), odom_frame_id_.c_str(), 
                 base_frame_id_.c_str(), body_frame_id_.c_str());
        ROS_INFO("Publish TF: %s", publish_tf_ ? "true" : "false");
        
        // 订阅话题
        gps_sub_ = nh_.subscribe("/mavros/global_position/global", 10, 
                                &GPSConverter::gpsCallback, this);
        imu_sub_ = nh_.subscribe("/mavros/imu/data", 10,
                               &GPSConverter::imuCallback, this);
        local_odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10,
                                      &GPSConverter::localOdomCallback, this);
        
        // 发布话题
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/Odometry", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/gps/pose", 10);
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
        
        ROS_INFO("GPS converter node ready");
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 从四元数提取欧拉角
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        tf2::Matrix3x3 m(q);
        m.getRPY(current_roll_, current_pitch_, current_yaw_);
    }
    
    void localOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 使用本地里程计数据（来自mavros）
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        
        // 从四元数提取欧拉角
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        m.getRPY(current_roll_, current_pitch_, current_yaw_);
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (msg->status.status < 0) {
            ROS_WARN_THROTTLE(1.0, "GPS fix not available");
            return;
        }
        
        double lat = msg->latitude;
        double lon = msg->longitude;
        double alt = msg->altitude;
        
        // ROS_INFO_THROTTLE(1.0, "GPS fix: lat=%.6f, lon=%.6f, alt=%.2f", lat, lon, alt);
        
        // 如果是第一个有效的GPS数据，设置为原点
        if (!origin_set_) {
            setOrigin(lat, lon, alt);
            return;
        }
        
        // 处理GPS数据
        processGPS(lat, lon, alt);
    }
    
private:
    void setOrigin(double lat, double lon, double alt) {
        try {
            // 将原点转换为UTM
            GeographicLib::UTMUPS::Forward(lat, lon, 
                utm_zone_, utm_north_, origin_easting_, origin_northing_);
            
            origin_lat_ = lat;
            origin_lon_ = lon;
            origin_alt_ = alt;
            origin_set_ = true;
            
            ROS_INFO("=== GPS Origin Set ===");
            ROS_INFO("WGS84: lat=%.6f, lon=%.6f, alt=%.2f", lat, lon, alt);
            ROS_INFO("UTM: zone=%d, north=%s", utm_zone_, utm_north_ ? "true" : "false");
            ROS_INFO("UTM Origin: easting=%.2f, northing=%.2f", origin_easting_, origin_northing_);
            
            // 发布静态的map->odom变换（原点对齐）
            publishStaticTF();
            
            // 发布初始位置
            if (publish_initial_pose_) {
                publishInitialPose();
            }
            
            first_fix_received_ = true;
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to convert origin to UTM: %s", e.what());
        }
    }
    
    void publishStaticTF() {
        // 发布静态的map到odom变换
        geometry_msgs::TransformStamped static_transform;
        
        static_transform.header.stamp = ros::Time::now();
        static_transform.header.frame_id = map_frame_id_;
        static_transform.child_frame_id = odom_frame_id_;
        
        static_transform.transform.translation.x = 0;
        static_transform.transform.translation.y = 0;
        static_transform.transform.translation.z = 0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        static_transform.transform.rotation = tf2::toMsg(q);
        
        static_tf_broadcaster_.sendTransform(static_transform);
        static_tf_published_ = true;
        
        ROS_INFO("Published static transform: %s -> %s", 
                 map_frame_id_.c_str(), odom_frame_id_.c_str());
    }
    
    void processGPS(double lat, double lon, double alt) {
        try {
            // 将当前GPS转换为UTM
            double easting, northing;
            int zone;
            bool north;
            GeographicLib::UTMUPS::Forward(lat, lon, zone, north, easting, northing);
            
            // 检查UTM区域是否一致
            if (zone != utm_zone_ || north != utm_north_) {
                ROS_WARN_THROTTLE(5.0, "UTM zone changed: from %d/%s to %d/%s", 
                                 utm_zone_, utm_north_ ? "north" : "south",
                                 zone, north ? "north" : "south");
            }
            
            // 计算相对于原点的局部坐标
            double local_x = easting - origin_easting_;
            double local_y = northing - origin_northing_;
            double local_z = alt - origin_alt_;
            
            // 无人机坐标系转换：ENU到NED（如果需要）
            double nav_x, nav_y, nav_z;
            if (use_enu_to_ned_) {
                // ENU (East, North, Up) 到 NED (North, East, Down)
                nav_x = local_y;  // North
                nav_y = local_x;  // East  
                nav_z = -local_z; // Down
            } else {
                // 保持ENU
                nav_x = local_x;  // East
                nav_y = local_y;  // North
                nav_z = local_z;  // Up
            }
            
            std::lock_guard<std::mutex> lock(mutex_);
            
            // 更新当前位置
            current_x_ = nav_x;
            current_y_ = nav_y;
            current_z_ = nav_z;
            
            // 发布所有消息
            ros::Time current_time = ros::Time::now();
            publishAllMessages(current_time);
            
            // 发布调试信息
            // ROS_INFO_THROTTLE(1.0, "Local: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f", 
            //                   nav_x, nav_y, nav_z, current_yaw_);
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to convert GPS to UTM: %s", e.what());
        }
    }
    
    void publishInitialPose() {
        geometry_msgs::PoseWithCovarianceStamped initial_pose;
        initial_pose.header.stamp = ros::Time::now();
        initial_pose.header.frame_id = map_frame_id_;
        
        // 设置在原点
        initial_pose.pose.pose.position.x = 0;
        initial_pose.pose.pose.position.y = 0;
        initial_pose.pose.pose.position.z = 0;
        
        // 设置姿态
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        initial_pose.pose.pose.orientation = tf2::toMsg(q);
        
        // 设置协方差
        setPoseCovariance(initial_pose.pose.covariance);
        
        initial_pose_pub_.publish(initial_pose);
        ROS_INFO("Published initial pose at origin (0,0,0)");
    }
    
    void publishAllMessages(const ros::Time& stamp) {
        // 发布动态的odom->base_link变换
        if (publish_tf_) {
            publishDynamicTF(stamp);
        }
        
        // 发布Odometry
        publishOdometry(stamp);
        
        // 发布PoseStamped
        publishPoseStamped(stamp);
    }
    
    void publishDynamicTF(const ros::Time& stamp) {
        // 发布odom到base_link的动态变换
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = odom_frame_id_;
        transform.child_frame_id = base_frame_id_;
        
        transform.transform.translation.x = current_x_;
        transform.transform.translation.y = current_y_;
        transform.transform.translation.z = current_z_;
        
        tf2::Quaternion q;
        q.setRPY(current_roll_, current_pitch_, current_yaw_);
        transform.transform.rotation = tf2::toMsg(q);
        
        tf_broadcaster_.sendTransform(transform);
        
        ROS_DEBUG_THROTTLE(1.0, "Published dynamic TF: %s -> %s (x=%.2f, y=%.2f, z=%.2f)",
                           odom_frame_id_.c_str(), base_frame_id_.c_str(),
                           current_x_, current_y_, current_z_);
    }
    
    void publishOdometry(const ros::Time& stamp) {
        nav_msgs::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_id_;
        odom.child_frame_id = base_frame_id_;
        
        // 设置位置
        odom.pose.pose.position.x = current_x_;
        odom.pose.pose.position.y = current_y_;
        odom.pose.pose.position.z = current_z_;
        
        // 设置姿态
        tf2::Quaternion q;
        q.setRPY(current_roll_, current_pitch_, current_yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);
        
        // 设置协方差
        setOdomCovariance(odom.pose.covariance);
        
        // 设置速度（暂时设为0，可以从mavros获取）
        odom.twist.twist.linear.x = 0;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = 0;
        
        // 速度协方差
        setTwistCovariance(odom.twist.covariance);
        
        odom_pub_.publish(odom);
        
        ROS_DEBUG_THROTTLE(1.0, "Published Odometry");
    }
    
    void publishPoseStamped(const ros::Time& stamp) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = map_frame_id_;
        
        // 在map坐标系中的位置
        pose.pose.position.x = current_x_;
        pose.pose.position.y = current_y_;
        pose.pose.position.z = current_z_;
        
        tf2::Quaternion q;
        q.setRPY(current_roll_, current_pitch_, current_yaw_);
        pose.pose.orientation = tf2::toMsg(q);
        
        pose_pub_.publish(pose);
    }
    
    void setPoseCovariance(boost::array<double, 36>& covariance) {
        std::fill(covariance.begin(), covariance.end(), 0.0);
        
        // 位置协方差
        covariance[0] = 0.25;  // x
        covariance[7] = 0.25;  // y
        covariance[14] = 0.25; // z
        
        // 姿态协方差
        covariance[21] = 0.0685; // roll
        covariance[28] = 0.0685; // pitch
        covariance[35] = 0.0685; // yaw
    }
    
    void setOdomCovariance(boost::array<double, 36>& covariance) {
        std::fill(covariance.begin(), covariance.end(), 0.0);
        
        // 位置协方差
        covariance[0] = 1.0;   // x
        covariance[7] = 1.0;   // y
        covariance[14] = 2.0;  // z
        
        // 姿态协方差
        covariance[21] = 0.1;  // roll
        covariance[28] = 0.1;  // pitch
        covariance[35] = 0.2;  // yaw
    }
    
    void setTwistCovariance(boost::array<double, 36>& covariance) {
        std::fill(covariance.begin(), covariance.end(), 0.0);
        
        covariance[0] = 0.1;  // 线速度x
        covariance[7] = 0.1;  // 线速度y
        covariance[14] = 0.1; // 线速度z
        covariance[21] = 0.1; // 角速度x
        covariance[28] = 0.1; // 角速度y
        covariance[35] = 0.1; // 角速度z
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_converter_node");
    
    GPSConverter converter;
    
    ros::spin();
    
    return 0;
}