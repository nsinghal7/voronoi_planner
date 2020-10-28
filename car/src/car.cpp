#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "car/car_param_parser.h"
#include "car/formation_gen.h"

#include <cmath>
#include <math.h>
#define PI M_PI

namespace car {

class CarNode {
  ros::NodeHandle nh_;

  ros::Subscriber start_sub_;
  ros::Subscriber control_sub_;
  ros::Publisher car_marker_pub_;
  ros::Publisher odom_pub_;
  ros::Timer timer_;

  tf2_ros::TransformBroadcaster br_;

  double hz_ = 30;

  double x_;
  double y_;
  double theta_;
  double v_;
  double delta_; // steering angle

  double desired_speed_;
  double desired_steering_angle_;

  CarParamParser params_;

  ros::Time last_step_;

  int car_num_;
  std::string car_name_;

public:
  CarNode(): nh_("~") {
    nh_.getParam("car_num", car_num_);
    params_.parse(nh_);
    std::cout << "found car num: " << car_num_ << std::endl;
    car_name_ = "car_" + std::to_string(car_num_);
    auto spec = generateFormationSpec(params_.formation, car_num_, params_.n_cars, params_);
    x_ = spec.start_x;
    y_ = spec.start_y;
    theta_ = spec.start_theta;
    v_ = spec.start_v;
    delta_ = spec.start_delta;

    control_sub_ = nh_.subscribe(params_.control_topic, 1, &CarNode::onControl, this);
    car_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/car_markers", 1, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(params_.odom_topic, 1, this);
    if(params_.autostart) {
      onStart(geometry_msgs::PoseWithCovarianceStamped());
    } else {
      start_sub_ = nh_.subscribe("/initialpose", 1, &CarNode::onStart, this);
      visualize();
    }
  }

  void onStart(const geometry_msgs::PoseWithCovarianceStamped& ignored) {
    last_step_ = ros::Time::now();
    timer_ = nh_.createTimer(ros::Duration(1/params_.hz), &CarNode::simLoop, this);
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
    delta_ = std::max(-params_.max_delta, std::min(params_.max_delta, delta_));
    theta_ += v_*tan(delta_)/params_.L*dt;
    theta_ = fmod(theta_, 2*PI);

    if(desired_speed_ < v_) {
      v_ = std::max(-params_.max_vel, std::max(v_ - params_.max_accel*dt, desired_speed_));
    } else {
      v_ = std::min(params_.max_vel, std::min(v_ + params_.max_accel*dt, desired_speed_));
    }
    visualize();
  }

  void visualize() {
    ros::Time time = ros::Time::now();
    geometry_msgs::TransformStamped trans;
    trans.header.stamp = time;
    trans.header.frame_id = "world";
    trans.child_frame_id = params_.baselink_frame;
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

    nav_msgs::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "world";
    odom.child_frame_id = params_.baselink_frame;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v_;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom_pub_.publish(odom);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = time;
    marker.ns = car_name_;
    marker.id = car_num_;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = x_ + cos(theta_)*params_.L/2;
    marker.pose.position.y = y_ + sin(theta_)*params_.L/2;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = params_.length;
    marker.scale.y = params_.width;
    marker.scale.z = params_.height;
    marker.color.a = 0.5;
    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    car_marker_pub_.publish(marker);
  }
};

} // namespace car

int main(int argc, char** argv) {
  ros::init(argc, argv, "car_node");
  car::CarNode n;
  ros::spin();
}

