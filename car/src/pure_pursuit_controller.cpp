#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <math.h>
#define PI M_PI

class PurePursuitControllerNode {
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  ros::Publisher control_pub_;

  double hz_ = 30;

  double L_ = 0.7;
  double lfw_ = 0.15;
  double max_accel_ = 1;
  double max_vel_ = 1;
  double width_ = 0.3;
  double length_ = 0.8;
  double height_ = 0.15; // doesn't matter

  double lookahead_dist_ = 0.3;
  double lookahead_time_ = 0.05; // adds this factor times velocity to lookahead dist

  double goal_x_;
  double goal_y_;

  int car_num_;

  std::string car_name_;

public:
  PurePursuitControllerNode(): nh_("~"), goal_x_(5), goal_y_(5)  {
    // TODO: set initial and goal poses based on a common generator
    nh_.getParam("car_num", car_num_);
    std::cout << "controller found car num: " << car_num_ << std::endl;
    car_name_ = "car_" + std::to_string(car_num_);

    odom_sub_ = nh_.subscribe("/" + car_name_ + "_odom", 1, &PurePursuitControllerNode::onOdom, this);
    control_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/" + car_name_ + "_control", 1, this);
  }

  void onOdom(const nav_msgs::Odometry& odom) {
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    if(abs(q.getAxis().z()) < .99) {
      throw "Pure Pursuit Controller expected orientation to only have yaw";
    }
    double theta = q.getAngle();
    double v = odom.twist.twist.linear.x;

    // Because the goal traj always contains current position, this is exact dist to lookahead point
    double L_fw= lookahead_dist_ + lookahead_time_ * v; // TODO what happens if this is negative?
    
    double dx_g = (goal_x_ - x), dy_g = (goal_y_ - y);
    double dx_r = dx_g * cos(theta) + dy_g * sin(theta);
    double dy_r = -dy_g * sin(theta) + dy_g * cos(theta);
    double eta = atan2(dy_r, dx_r);
    double v_des = max_vel_; // TODO choose better desired velocity and allow parameterization
    if(abs(eta) > PI / 2) {
      // backwards goal. I choose to turn the nose towards the goal rather than backing up to it
      v_des *= -1;
    }
    double delta = atan2(L_*sin(eta), L_fw / 2 + lfw_ * cos(eta)); // guaranteed to be 1st/4th quad if lookahead/2>lfw

    ackermann_msgs::AckermannDrive control;
    control.steering_angle = delta;
    control.speed = v_des;
    control_pub_.publish(control);
  }

};


int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit_controller_node");
  PurePursuitControllerNode n;
  ros::spin();
}
