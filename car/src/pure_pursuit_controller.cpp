#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "car/car_param_parser.h"
#include "car/formation_gen.h"

#include <cmath>
#include <math.h>
#define PI M_PI

namespace car {

class PurePursuitControllerNode {
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  ros::Publisher control_pub_;
  ros::Publisher marker_pub_;

  CarParamParser params_;

  double goal_x_;
  double goal_y_;
  bool backing_up_ = false;

  int car_num_;

  std::string car_name_;

public:
  PurePursuitControllerNode(): nh_("~") {
    nh_.getParam("car_num", car_num_);
    params_.parse(nh_);
    std::cout << "controller found car num: " << car_num_ << std::endl;
    car_name_ = "car_" + std::to_string(car_num_);
    auto spec = generateFormationSpec(params_.formation, car_num_, params_.n_cars, params_);
    goal_x_ = spec.goal_x;
    goal_y_ = spec.goal_y;

    odom_sub_ = nh_.subscribe(params_.odom_topic, 1, &PurePursuitControllerNode::onOdom, this);
    control_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>(params_.control_topic, 1, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ppc_markers", 3, this);
  }

  void onOdom(const nav_msgs::Odometry& odom) {
    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    double theta = tf2::getYaw(q);
    double v = odom.twist.twist.linear.x;

    // Because the goal traj always contains current position, this is exact dist to lookahead point
    double L_fw= params_.lookahead_dist + params_.lookahead_time * v; // TODO what happens if this is negative?
    
    double dx_g = (goal_x_ - x - cos(theta)*params_.lfw), dy_g = (goal_y_ - y - sin(theta)*params_.lfw);
    double dx_r = dx_g * cos(theta) + dy_g * sin(theta);
    double dy_r = -dx_g * sin(theta) + dy_g * cos(theta);
    const double eta_orig = atan2(dy_r, dx_r);
    double v_des = params_.max_vel; // TODO choose better desired velocity and allow parameterization
    double eta = eta_orig;
    if((backing_up_ && abs(eta_orig) > PI/3) || (!backing_up_ && abs(eta_orig) > PI / 2)) {
      // backwards goal. I choose to turn the nose towards the goal rather than backing up to it
      backing_up_ = true;
      v_des *= -1;
      if(eta > 0) {
        eta = eta - PI;
      } else {
        eta = eta + PI;
      }
    } else {
      // not backing up
      backing_up_ = false;
    }
    double delta = atan2(params_.L*sin(eta), L_fw / 2 + params_.lfw * cos(eta)); // guaranteed to be 1st/4th quad if lookahead/2>lfw

    ackermann_msgs::AckermannDrive control;
    control.steering_angle = delta;
    control.speed = v_des;
    control_pub_.publish(control);

    visualization_msgs::Marker marker;
    marker.header.frame_id = params_.baselink_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = car_name_;
    marker.id = car_num_*3;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = params_.lfw + cos(eta) * L_fw;
    marker.pose.position.y = sin(eta) * L_fw;
    marker.pose.position.z = 0;
    marker.scale.x = .1;
    marker.scale.y = .1;
    marker.scale.z = .1;
    marker.color.a = 1.0;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker_pub_.publish(marker);

    marker.id++;
    marker.header.frame_id = "world";
    marker.pose.position.x = goal_x_;
    marker.pose.position.y = goal_y_;
    marker.pose.position.z = 0;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker_pub_.publish(marker);

    marker.id++;
    marker.header.frame_id = params_.baselink_frame;
    marker.pose.position.x = params_.lfw + cos(eta_orig) * L_fw;
    marker.pose.position.y = sin(eta) * L_fw;
    marker.pose.position.z = 0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    marker_pub_.publish(marker);
  }

};

} // namespace car

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit_controller_node");
  car::PurePursuitControllerNode n;
  ros::spin();
}

