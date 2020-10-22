#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <cmath>
#include <math.h>
#define PI M_PI

class CarNode {
  ros::NodeHandle nh;

  ros::Subscriber controlSub_;
  ros::Publisher posePub_;
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

public:
  CarNode(): x_(0), y_(0), theta_(0), v_(0), delta_(0) {
    controlSub_ = nh.subscribe("car_control", 1, &CarNode::onControl, this);
    posePub_ = nh.advertise<geometry_msgs::PoseStamped>("car_pose", 1, this);
    last_step_ = ros::Time::now();
    timer_ = nh.createTimer(ros::Duration(1/hz_), &CarNode::simLoop, this);
  }

  void onControl(const ackermann_msgs::AckermannDrive& control) {
    desired_steering_angle_ = control.steering_angle;
    desired_speed_ = control.speed;
  }

  void simLoop(const ros::TimerEvent& t) {
    std::cout << "in sim loop\n";

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

    std::cout << "x: " << x_ << " y: " << y_ << std::endl;
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "car_node");
  CarNode n;
  ros::spin();
}

