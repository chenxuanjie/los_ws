#include "los_pid_follow/los_pid.h"
using namespace std;
namespace auto_recovery
{

LOS_PID::LOS_PID(const ros::NodeHandle &nh, const ros::NodeHandle &nh_p) 
: nh_(nh), nh_p_(nh_p) {

    odom_sub_ = nh_.subscribe("/wamv/sensors/position/p3d_wamv", 1, &LOS_PID::odomCallback, this);
    path_sub_ = nh_.subscribe("/path", 10, &LOS_PID::newPath, this);

    left_thrust_pub_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 10);
    right_thrust_pub_ = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/robot_path", 10);
    timer_ = nh_.createTimer(ros::Duration(0.1), &LOS_PID::timerCallback, this);
    //从参数服务器获取参数
    //los
    nh_p_.getParam("max_speed",max_speed_);
    nh_p_.getParam("max_speed_turn",max_speed_turn_);
    nh_p_.getParam("min_speed",min_speed_);
    //pid
    nh_p_.getParam("kp_psi",kp_psi_);
    nh_p_.getParam("ki_psi",ki_psi_);
    nh_p_.getParam("kd_psi",kd_psi_);
    nh_p_.getParam("kp_u",kp_u_);
    nh_p_.getParam("ki_u",ki_u_);
    nh_p_.getParam("kd_u",kd_u_);

    
}
LOS_PID::~LOS_PID() {}

void LOS_PID::newPath(const nav_msgs::Path& path) {
    if (haspath) return;
    path_ = path;
    haspath = true;
}

void LOS_PID::odomCallback(const nav_msgs::Odometry &msg) {
    
    odom_ = msg;
    path_msg_.header.stamp = ros::Time::now();
    path_msg_.header.frame_id = "map";
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = path_msg_.header.stamp;
    pose_msg.header.frame_id = path_msg_.header.frame_id;
    pose_msg.pose.position.x = msg.pose.pose.position.x; 
    pose_msg.pose.position.y = msg.pose.pose.position.y; 
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation.x = msg.pose.pose.orientation.x; 
    pose_msg.pose.orientation.y = msg.pose.pose.orientation.y; 
    pose_msg.pose.orientation.z = msg.pose.pose.orientation.z; 
    pose_msg.pose.orientation.w = msg.pose.pose.orientation.w; 
    path_msg_.poses.push_back(pose_msg);
    path_pub_.publish(path_msg_);
}
void LOS_PID::timerCallback(const ros::TimerEvent &e) {
    pubThrustCmd();
}

void LOS_PID::pubThrustCmd() {

    getDesiredSpeedHeading(desired_speed_, desired_heading_);
    double tau_surge = calculateSurgeForce(0.01, desired_speed_);
    double tau_yaw_moment = calculateYawMoment(0.01, desired_heading_);
    //cout << "tau_surge:" << tau_surge << endl;
    //cout << "tau_yaw_moment:" << tau_yaw_moment << endl;

    Eigen::Vector2d thrust_cmd = thrustAllocation(tau_surge, tau_yaw_moment);
    cout << "thrust_cmd :" << thrust_cmd[0] << " " << thrust_cmd[1] << endl;
    std_msgs::Float32 left_thrust, right_thrust;
    // left_thrust.data = static_cast<float>(thrust_cmd[0]);
    // right_thrust.data = static_cast<float>(thrust_cmd[1]);
    left_thrust.data = thrust_cmd[0];
    right_thrust.data = thrust_cmd[1];
    //cout << "left_thrust" << left_thrust.data << endl;

    left_thrust_pub_.publish(left_thrust);
    right_thrust_pub_.publish(right_thrust);

}

void LOS_PID::getDesiredSpeedHeading(double &desired_speed, double &desired_heading) {
    double x = odom_.pose.pose.position.x;
    double y = odom_.pose.pose.position.y;
    double psi = tf2::getYaw(odom_.pose.pose.orientation);
    //cout << "current pose:" << x << y << psi << endl;

    //如果执行完路径就停止保持朝向
    if (path_.poses.size() <= 10) {
        desired_speed = 0.0;
        desired_heading = psi;
        cout << "stop" << endl;
        return;
    }

    //变视距LOS
    //找到路径上距离最近的点
    std::vector<geometry_msgs::PoseStamped>::iterator closest_it;
    double min_dist = std::numeric_limits<double>::max();
    for (auto it = path_.poses.begin(); it != path_.poses.end(); ++it) {
        double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                                std::pow(y - it->pose.position.y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closest_it = it;
        }
    }

    geometry_msgs::PoseStamped pose_closest = * closest_it;
    cout <<"最近点x:" << pose_closest.pose.position.x << endl;
    cout <<"最近点y:" << pose_closest.pose.position.y << endl;

    //删除路径上路径点之前的元素
    path_.poses.erase(path_.poses.begin(), closest_it);
    cout << "path pose size:" << path_.poses.size() << endl;

    //计算最近点上路径的正切角
    double gamma_p = tf2::getYaw(pose_closest.pose.orientation);
    cout << "gamma_p:" << gamma_p << endl;

    //计算cross track error
    double y_e = - (x - pose_closest.pose.position.x) * std::sin(gamma_p) +
                 (y - pose_closest.pose.position.y) * std::cos(gamma_p);
    cout << "cross track error:" << endl;
    //计算变化的前视距离
    double delta_y_e = (delta_max_ - delta_min_) * std::exp(-delta_k_ * std::pow(y_e, 2)) + delta_min_;
    cout << "前视距离:" << delta_y_e << endl;

    //如果转向就设为最小的前视距离
    bool is_turning = false;
    if ((closest_it + 1) != path_.poses.end()) {
        double next_angle = tf2::getYaw((*(closest_it + 1)).pose.orientation);
        if (std::fabs(gamma_p - next_angle) > std::numeric_limits<double>::epsilon()) {
            delta_y_e = delta_min_;
            is_turning = true;
        }
    } 

    //得到期望朝向
    //desired_heading = gamma_p + std::atan2(-y_e, delta_y_e);
    //desired_heading = std::atan2(-y_e, delta_y_e);
    desired_heading = -3;

    //计算朝向误差
    double heading_err = desired_heading - psi;
    while (heading_err > M_PI) {
        heading_err -= 2 * M_PI;
    }
    while (heading_err < -M_PI) {
        heading_err += 2 * M_PI;
    }
    cout <<"desired heading:" << desired_heading << endl;
    cout << "current heading:" << psi << endl;
    //计算期望速度
    desired_speed = 0.3;
     desired_speed = max_speed_ * (1 - std::abs(y_e) / 5 - std::abs(heading_err) / M_PI_2);
     desired_speed = std::max(desired_speed, min_speed_);
     if (is_turning) desired_speed = max_speed_turn_;
      cout <<"desired speed:" << desired_speed << endl;
      cout <<"current speed:" << odom_.twist.twist.linear.x << endl;

}

double LOS_PID::calculateSurgeForce(double dt, double desired_speed) {
    double output = kp_u_  * (desired_speed - odom_.twist.twist.linear.x);
    //cout << "kp:" << kp_u_ << endl;
    //cout << "output:" << output << endl;
    return output ;
}
double LOS_PID::calculateYawMoment(double dt, double desired_heading) {
    double psi = tf2::getYaw(odom_.pose.pose.orientation);
    double beta_c = atan2(odom_.twist.twist.linear.y,
                            sqrt(odom_.twist.twist.linear.y * odom_.twist.twist.linear.y + 
                                odom_.twist.twist.linear.x * odom_.twist.twist.linear.x));
    
    double heading_err = desired_heading - psi - beta_c;
    while (heading_err > M_PI) {
        heading_err -= 2 * M_PI;
    }
    while (heading_err < -M_PI) {
        heading_err += 2 * M_PI;
    }
    double dev_heading_err = heading_err - prev_error_;
    cout << "heading error:" << heading_err << endl;
    double output = kp_psi_  * heading_err + kd_psi_ * dev_heading_err / dt;
    prev_error_ = heading_err;
    //cout << "moment output:" << output << endl;
    return output;
}
Eigen::Vector2d LOS_PID::thrustAllocation(double tau_surge, double tau_yaw_moment) {
    Eigen::Matrix2d T;
    T << 1, 1, -1.02, 1.02;
    Eigen::Vector2d tau;
    tau[0] = tau_surge;
    tau[1] = tau_yaw_moment;
    Eigen::Vector2d thruster;
    thruster = T.inverse() * tau;
    return thruster;
}

} // namespace auto_recovery

