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
  ros::Publisher traj_pub_;
  ros::Timer timer_;

  CarPose pose_;

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

    odom_sub_ = nh_.subscribe("/all_cars/odoms", 1, &VoronoiPlannerNode::onOdoms, this);
    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(params_.traj_topic, 1, this);

    plan({});
  }

  void onOdoms(const OdomArray& odoms) {
    plan(odoms.odoms);
  }

  void plan(const std::vector<nav_msgs::Odometry> odoms) {
    // always just return the goal point
    // TODO: maybe restrict goal within cell or account for closeness to edge
    trajectory_msgs::JointTrajectory traj;
    traj.header.frame_id = params_.baselink_frame;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.push_back("x");
    traj.joint_names.push_back("y");
    trajectory_msgs::Point pt;
    pt.positions.push_back(goal_x_);
    pt.positions.push_back(goal_y_);
    traj.points.push_back(std::move(pt));
    traj_pub_.publish(traj);
  }
};

} // namespace car

int main(int argc, char** argv) {
  ros::init(argc, argv, "voronoi_planner_node");
  car::VoronoiPlannerNode n;
  ros::spin();
}

