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
#include <trajectory_msgs/JointTrajectory.h>
#include <car/OdomArray.h>
#include "car/car_param_parser.h"
#include "car/formation_gen.h"

#include <cmath>
#include <math.h>
#define PI M_PI

namespace car {

class PurePursuitControllerNode {
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  ros::Subscriber odom_array_sub_;
  ros::Subscriber traj_sub_;
  ros::Publisher control_pub_;
  ros::Publisher marker_pub_;

  CarParamParser params_;
  double init_goal_x_;
  double init_goal_y_;

  trajectory_msgs::JointTrajectory traj_;
  int traj_index_ = 0;
  std::vector<nav_msgs::Odometry> all_odoms_;
  bool backing_up_ = false;

public:
  PurePursuitControllerNode(): nh_("~") {
    params_.parse(nh_);
    auto spec = generateFormationSpec(params_);
    init_goal_x_ = spec.goal_x;
    init_goal_y_ = spec.goal_y;

    odom_sub_ = nh_.subscribe(params_.odom_topic, 1, &PurePursuitControllerNode::onOdom, this);
    odom_array_sub_ = nh_.subscribe("/all_cars/odoms", 1, &PurePursuitControllerNode::onOdomArray, this);
    traj_sub_ = nh_.subscribe(params_.traj_topic, 1, &PurePursuitControllerNode::onTraj, this);
    control_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>(params_.control_topic, 1, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/ppc_markers", 3, this);
  }

  void onOdomArray(const car::OdomArray& odoms) {
    all_odoms_ = odoms.odoms;
  }

  void onTraj(const trajectory_msgs::JointTrajectory& traj) {
    traj_ = traj;
    traj_index_ = 0;
  }

  double calcDelta(double eta, double L_fw) {
    return atan2(params_.L*sin(eta), L_fw / 2 + params_.lfw * cos(eta));
  }

  std::tuple<double, double> toRelativePos(double x, double y, double theta, double ox, double oy, bool include_lfw) {
    const double lfw_temp = include_lfw ? params_.lfw : 0;
    const double dx_g = ox - x - cos(theta) * lfw_temp;
    const double dy_g = oy - y - sin(theta) * lfw_temp;
    const double dx_r = dx_g * cos(theta) + dy_g * sin(theta);
    const double dy_r = -dx_g * sin(theta) + dy_g * cos(theta);
    return std::make_tuple(dx_r, dy_r);
  }

  std::tuple<double, double> toGlobalPos(double x, double y, double theta, double rx, double ry, bool include_lfw) {
    const double lfw_temp = include_lfw ? params_.lfw : 0;
    const double dx_g = rx * cos(theta) - ry * sin(theta);
    const double dy_g = rx * sin(theta) + ry * cos(theta);
    const double ox = dx_g + cos(theta) * lfw_temp + x;
    const double oy = dy_g + sin(theta) * lfw_temp + y;
    return std::make_tuple(ox, oy);
  }

  std::tuple<double, double, double, double> toPose(const nav_msgs::Odometry& odom) {
    const double x = odom.pose.pose.position.x;
    const double y = odom.pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(odom.pose.pose.orientation, q);
    const double theta = tf2::getYaw(q);
    const double v = odom.twist.twist.linear.x;
    return std::make_tuple(x, y, theta, v);
  }

  double etaWithObstacles(const double eta, const double L_fw, double x, double y, const double theta) {
    if(all_odoms_.size() <= 1) {
      return eta;
    }
    x += cos(theta) * params_.lfw;
    y += sin(theta) * params_.lfw;
    double look_x, look_y;
    for(int side = 1; side > -2; side -= 2) {
      std::tie(look_x, look_y) = toGlobalPos(x, y, theta, cos(eta)*L_fw, sin(eta)*L_fw, false);
      int iter_count = 0;
      int start_index = params_.car_num - 1;
      bool fail = false;
      for(int index = params_.car_num % all_odoms_.size(); index != start_index; index = (index + 1) % all_odoms_.size()) {
        if(iter_count >= 50) {
          // no progress.
          std::cerr << "etaWithObstacles went 50 its without success on side: " << side << std::endl;
          fail = true;
          break;
        }
        iter_count++;
        if(index == params_.car_num - 1) {
          continue;
        }
        double ox, oy, otheta, ov;
        std::tie(ox, oy, otheta, ov) = toPose(all_odoms_.at(index));
        double sep_dist = sqrt(pow(ox - x, 2) + pow(oy - y, 2) + 1e-12);
        double sep_vx = (ox - x) / sep_dist;
        double sep_vy = (oy - y) / sep_dist;
        double look_xr = look_x - x, look_yr = look_y - y;
        double dist_along_bisector = look_xr * sep_vx + look_yr * sep_vy;
        if(dist_along_bisector > sep_dist / 2 - params_.voronoi_buffer) {
          // TODO: illegal eta. modify
          dist_along_bisector = sep_dist / 2 - params_.voronoi_buffer;
          if(dist_along_bisector < -L_fw) {
            dist_along_bisector = -L_fw;
          }
          double dist_to_right = sqrt(L_fw*L_fw - dist_along_bisector*dist_along_bisector + 1e-12);
          look_x = sep_vx * dist_along_bisector + side * sep_vy * dist_to_right;
          look_y = -side * sep_vx * dist_to_right + sep_vy * dist_along_bisector;
  
          // update start_index since this was a 'fail'
          start_index = index;
        }
      }
      if(fail) {
        continue; // try the left side
      }
      double look_xr, look_yr;
      std::tie(look_xr, look_yr) = toRelativePos(x, y, theta, look_x, look_y, false);
      return  atan2(look_yr, look_xr);
    }
    throw "no viable eta";
  }

  void vizCellBoundary(const double x, const double y) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = params_.car_name + "/cell";
    marker.id = params_.car_num* 4 + 4;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.scale.x = .1;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;

    double mx, my, mtheta, mv;
    if(all_odoms_.size() < params_.car_num) {
      return;
    }
    std::tie(mx, my, mtheta, mv) = toPose(all_odoms_.at(params_.car_num - 1));

    for(int index = 0; index < all_odoms_.size(); index++) {
      if(index == params_.car_num - 1) {
        continue;
      }
      double ox, oy, otheta, ov;
      std::tie(ox, oy, otheta, ov) = toPose(all_odoms_.at(index));
      double dx = ox - mx, dy = oy - my;
      double dist = sqrt(dx*dx + dy * dy + 1e-12);
      double vx = dx / dist, vy = dy / dist;
      dx -= vx * params_.voronoi_buffer;
      dy -= vy * params_.voronoi_buffer;
      double x1 = dx / 2 - vy * 2, y1 = dy / 2 + vx * 2;
      double x2 = dx / 2 + vy * 2, y2 = dy / 2 - vx * 2;
      double x3 = dx / 2, y3 = dy / 2;
      double x4 = dx / 2 - vx / 2, y4 = dy / 2 - dy / 2;
      geometry_msgs::Point pt;
      pt.x = x1;
      pt.y = y1;
      pt.z = 0;
      marker.points.push_back(pt);
      pt.x = x2;
      pt.y = y2;
      pt.z = 0;
      marker.points.push_back(pt);
      pt.x = x3;
      pt.y = y3;
      pt.z = 0;
      marker.points.push_back(pt);
      pt.x = x4;
      pt.y = y4;
      pt.z = 0;
      marker.points.push_back(pt);
    }

    marker_pub_.publish(marker);
  }

  void onOdom(const nav_msgs::Odometry& odom) {
    double goal_x;
    double goal_y;
    if(traj_.points.size() == 0) {
      goal_x = 100; init_goal_x_;
      goal_y = 100; init_goal_y_;
    } else {
      // TODO: use distance based metric to potentially move on to other traj points
      goal_x = traj_.points.at(traj_index_).positions.at(0);
      goal_y = traj_.points.at(traj_index_).positions.at(1);
    }
    double x, y, theta, v;
    std::tie(x, y, theta, v) = toPose(odom);

    // Because the goal traj always contains current position, this is exact dist to lookahead point
    const double L_fw= params_.lookahead_dist + params_.lookahead_time * abs(v); // TODO what happens if this is negative?
    
    double dx_r, dy_r;
    std::tie(dx_r, dy_r) = toRelativePos(x, y, theta, goal_x, goal_y, true);
    double eta_orig = etaWithObstacles(atan2(dy_r, dx_r), L_fw, x, y, theta);

    double v_des = params_.max_vel; // TODO choose better desired velocity and allow parameterization
    const double goal_dist = sqrt(dx_r*dx_r+dy_r*dy_r+1e-12);
    if(v*v/2/goal_dist < params_.max_accel) {
      // reduce velocity to stop at goal, times damping factor for stability
      v_des = goal_dist/(0.1+goal_dist)* params_.max_accel*sqrt(2*goal_dist/params_.max_accel);
    }
    double eta = eta_orig;
    if((backing_up_ && abs(eta_orig) > PI/3) || (!backing_up_ && abs(eta_orig) > PI / 2)) {
      // backwards goal. I choose to turn the nose towards the goal rather than backing up to it
      backing_up_ = true;
      v_des *= -1;
    } else {
      // not backing up
      backing_up_ = false;
    }
    if(v < 0 || (v == 0 && v_des < 0)) {
      // vehicle is moving backwards (whether desired or not), so reverse steering to point to goal
      if(eta > 0) {
        eta = eta - PI;
      } else {
        eta = eta + PI;
      }
    }
    double delta = calcDelta(eta, L_fw);

    ackermann_msgs::AckermannDrive control;
    control.steering_angle = delta;
    control.speed = v_des;
    control_pub_.publish(control);

    visualization_msgs::Marker marker;
    marker.header.frame_id = params_.baselink_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = params_.car_name;
    marker.id = params_.car_num* 4;
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
    marker.pose.position.x = goal_x;
    marker.pose.position.y = goal_y;
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
    vizCellBoundary(x, y);
  }

};

} // namespace car

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_pursuit_controller_node");
  car::PurePursuitControllerNode n;
  ros::spin();
}

