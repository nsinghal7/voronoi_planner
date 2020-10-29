#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "car/car_param_parser.h"
#include "car/formation_gen.h"

#include <cmath>
#include <math.h>
#define PI M_PI

namespace car {

typedef struct {
  double x;
  double y;
  double theta;
  double v;
  bool valid = false;
  void extract(const nav_msgs::Odometry& odom) {
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    theta = tf2::getYaw(q);
    v = odom.twist.twist.linear.x;
    valid = true;
  }
} CarPose;

class VoronoiPlannerNode {
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  std::vector<ros::Subscriber> other_pose_subs_;
  ros::Publisher traj_pub_;
  ros::Publisher plan_marker_pub_;
  ros::Timer timer_;

  CarPose pose_;

  std::vector<CarPose> other_poses_;
  
  CarParamParser params_;

  double goal_x_;
  double goal_y_;

public:
  VoronoiPlannerNode(): nh_("~") {
    params_.parse(nh_);
    auto spec = generateFormationSpec(params_);
    goal_x_ = spec.goal_x;
    goal_y_ = spec.goal_y;
    pose_.x = spec.start_x;
    pose_.y = spec.start_y;
    pose_.theta = spec.start_theta;
    pose_.v = spec.start_v;

    odom_sub_ = nh_.subscribe(params_.odom_topic, 1, &CarPose::extract, &pose_);
    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(params_.traj_topic, 1, this);
    plan_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/plan_markers", 3, this);

    other_pose_subs_.resize(params_.n_cars - 1);
    for(int i = 1; i <= params_.n_cars; i++) {
      if(i == params_.car_num) {
        continue;
      }
      other_pose_subs_.push_back(nh_.subscribe("/car_" + std::to_string(i) + "/odom", 1, &CarPose::extract, &other_pose_subs_.at((i < params_.car_num) ? i : i - 1)));
    }

    timer_ = nh_.createTimer(ros::Duration(1/params_.traj_hz), &VoronoiPlannerNode::planLoop, this);
    plan();
  }

  void planLoop(const ros::TimerEvent& t) {
    plan();
  }

  void plan() {
    // TODO: make and publish and viz plan
  }
};

} // namespace car

int main(int argc, char** argv) {
  ros::init(argc, argv, "voronoi_planner_node");
  car::VoronoiPlannerNode n;
  ros::spin();
}

