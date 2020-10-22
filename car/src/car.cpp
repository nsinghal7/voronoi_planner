#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <math.h>
#define PI M_PI

class CarNode {
  ros::NodeHandle nh;

  ros::Subscriber control_sub_;
  ros::Publisher car_marker_pub_;
  ros::Timer timer_;

  tf2_ros::TransformBroadcaster br_;

  double hz_ = 30;

  double x_;
  double y_;
  double theta_;
  double v_;
  double delta_; // steering angle

  double L_ = 0.7; // length between wheels/wheelbase
  double lfw_ = 0.15; // length ahead of rear wheel to plan from
  double max_accel_ = 1; // m/s^2
  double max_vel_ = 8; // m/s
  double width_ = 0.3;
  double length_ = 0.8;
  double height_ = 0.15; // doesn't matter

  double desired_steering_angle_ = .1; // rad
  double desired_speed_ = 1; // m/s

  ros::Time last_step_;

  int car_num_;

public:
  CarNode(): x_(0), y_(0), theta_(0), v_(0), delta_(0), car_num_(0) {
    control_sub_ = nh.subscribe("car_" + std::to_string(car_num_) + "_control", 0, &CarNode::onControl, this);
    car_marker_pub_ = nh.advertise<visualization_msgs::Marker>("car_markers", 0, this);
    last_step_ = ros::Time::now();
    timer_ = nh.createTimer(ros::Duration(1/hz_), &CarNode::simLoop, this);
  }

  void onControl(const ackermann_msgs::AckermannDrive& control) {
    desired_steering_angle_ = control.steering_angle;
    desired_speed_ = control.speed;
  }

  void simLoop(const ros::TimerEvent& t) {
    ros::Time time = ros::Time::now();
    double dt = (time - last_step_).toSec();
    last_step_ = time;

    x_ += cos(theta_)*v_*dt;
    y_ += sin(theta_)*v_*dt;
    delta_ = desired_steering_angle_; // for now it updates immediately
    theta_ += v_*tan(delta_)/L_*dt;

    if(desired_speed_ < v_) {
      v_ = std::max(-max_vel_, std::max(v_ - max_accel_*dt, desired_speed_));
    } else {
      v_ = std::min(max_vel_, std::min(v_ + max_accel_*dt, desired_speed_));
    }

    geometry_msgs::TransformStamped trans;
    trans.header.stamp = time;
    trans.header.frame_id = "world";
    trans.child_frame_id = "car_" + std::to_string(car_num_) + "_baselink";
    trans.transform.translation.x = x_;
    trans.transform.translation.y = y_;
    trans.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    trans.transform.rotation.x = q.x();
    trans.transform.rotation.y = q.y();
    trans.transform.rotation.z = q.z();
    trans.transform.rotation.w = q.w();
    br_.sendTransform(trans);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = time;
    marker.ns = "car_" + std::to_string(car_num_) + "_viz";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = x_ + cos(theta_)*length_/2;
    marker.pose.position.y = y_ + sin(theta_)*length_/2;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = length_;
    marker.scale.y = width_;
    marker.scale.z = height_;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    car_marker_pub_.publish(marker);
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "car_node");
  CarNode n;
  ros::spin();
}

