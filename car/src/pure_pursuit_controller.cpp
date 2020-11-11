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

std::vector<double> get_hom_line(std::pair<double, double> o, std::pair<double, double> m, double buffer=0) {
  double midx = (o.first + m.first) / 2, midy = (o.second + m.second) / 2;
  double vecx = o.first - m.first, vecy = o.second - m.second;
  double norm = hypot(vecx, vecy);
  vecx /= norm;
  vecy /= norm;
  midx -= buffer*vecx;
  midy -= buffer*vecy;
  return {vecx, vecy, -midx*vecx - midy*vecy};
}

std::vector<double> get_hom_intersect(std::vector<double> blin, std::vector<double> clin) {
  auto bcxp = blin[1]*clin[2] - clin[1]*blin[2];
  auto bcyp = clin[0]*blin[2] - blin[0]*clin[2];
  auto bczp = blin[0]*clin[1] - clin[0]*blin[1];
  return {bcxp, bcyp, bczp};
}

bool is_on_same_side(std::vector<double> lin, std::pair<double, double> m, std::pair<double, double> o) {
  return !((m.first*lin[0] + m.second*lin[1] + lin[2] > 0) ^ (o.first*lin[0] + o.second*lin[1] + lin[2] > 0));
}

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

  // returns positions of relevant vehicles, sorted by increasing angle from the cell center
  std::vector<std::pair<double, double>> get_voronoi_cell() {
    if(all_odoms_.size() <= 1) {
      throw std::logic_error("can't create voronoi cell");
    }

    double my_cell_x, my_cell_y, _mct, _mcv;
    std::tie(my_cell_x, my_cell_y, _mct, _mcv) = toPose(all_odoms_.at(params_.car_num - 1));

    std::vector<std::tuple<double, double, int>> angle_distance_index;
    for(int i = 0; i < all_odoms_.size(); i++) {
      if(i == params_.car_num - 1) {
        continue; // don't add any edges for own car
      }
      double ox, oy, _ot, _ov;
      std::tie(ox, oy, _ot, _ov) = toPose(all_odoms_.at(i));
      angle_distance_index.push_back(std::make_tuple(atan2(oy - my_cell_y, ox - my_cell_x), -hypot(ox - my_cell_x, oy - my_cell_y), i));
    }
    std::sort(angle_distance_index.begin(), angle_distance_index.end()); // sort by inc angle, dec dist so if same angle, later line will cut off earlier line
    auto a_cuts_off_bc = [&](std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> c, std::pair<double, double> m) {
      auto alin = get_hom_line(a, m);
      auto blin = get_hom_line(b, m);
      auto clin = get_hom_line(c, m);
      auto inter_p = get_hom_intersect(blin, clin);
      if(abs(inter_p[2]) < 1e-4) {
        return false; // b and c don't intersect, so they go off to infinity, so we don't cut it off // TODO: but maybe a still cuts off a parallel line
      }
      auto px = inter_p[0] / inter_p[2];
      auto py = inter_p[1] / inter_p[2];

      return !is_on_same_side(alin, std::make_pair(my_cell_x, my_cell_y), std::make_pair(px, py));
    };

    std::vector<std::pair<double, double>> relevant_vehicles;
    for(const auto adi : angle_distance_index) {
      double ox, oy, _ot, _ov;
      std::tie(ox, oy, _ot, _ov) = toPose(all_odoms_.at(std::get<2>(adi)));
      const auto o = std::make_pair(ox, oy);
      while(relevant_vehicles.size() >= 2 && a_cuts_off_bc(o, relevant_vehicles[relevant_vehicles.size() - 1], relevant_vehicles[relevant_vehicles.size() - 2], std::make_pair(my_cell_x, my_cell_y))) {
        // o cuts off the intersection of the last two lines, so the last line doesn't affect the voronoi cell, so we remove it
        relevant_vehicles.pop_back();
      }
      relevant_vehicles.push_back(o);
    }

    // need to check if first or last line cut each other off (iteratively)
    while(relevant_vehicles.size() >= 3 && a_cuts_off_bc(relevant_vehicles[0], relevant_vehicles[relevant_vehicles.size() - 1], relevant_vehicles[relevant_vehicles.size() - 2], std::make_pair(my_cell_x, my_cell_y))) {
      // first line cuts off intersection of last two
      relevant_vehicles.pop_back();
    }
    int to_remove = 0;
    while(relevant_vehicles.size() >= to_remove + 3 && a_cuts_off_bc(relevant_vehicles.back(), relevant_vehicles[to_remove], relevant_vehicles[to_remove + 1], std::make_pair(my_cell_x, my_cell_y))) {
      // the last line cuts off the first line
      to_remove++;
    }
    relevant_vehicles.erase(relevant_vehicles.begin(), relevant_vehicles.begin() + to_remove);
    return relevant_vehicles;
}

  std::tuple<double, double>  etaLfwWithObstacles(const double eta, const double L_fw, const double x, const double y, const double theta) {
    if(all_odoms_.size() <= 1) {
      return std::make_tuple(eta, L_fw);
    }

    const double eta_base_x = x + cos(theta) * params_.lfw;
    const double eta_base_y = y + sin(theta) * params_.lfw;

    double my_cell_x, my_cell_y, _mct, _mcv;
    std::tie(my_cell_x, my_cell_y, _mct, _mcv) = toPose(all_odoms_.at(params_.car_num - 1));

    const auto relevant_vehicles = get_voronoi_cell(); // ccw sorted list of vehicles that form edges in the voronoi cell
    double look_x, look_y;
    std::tie(look_x, look_y) = toGlobalPos(eta_base_x, eta_base_y, theta, cos(eta)*L_fw, sin(eta)*L_fw, false);

    // project look_x back into voronoi cell in a lazy way:
    // TODO: project more carefully to maintain angle
    for(int i = 0; i < relevant_vehicles.size(); i++) {
      auto lin = get_hom_line(relevant_vehicles[i], std::make_pair(my_cell_x, my_cell_y), params_.voronoi_buffer);
      if(!is_on_same_side(lin, std::make_pair(my_cell_x, my_cell_y), std::make_pair(look_x, look_y))) {
        double d = (lin[0]*look_x + lin[1]*look_y + lin[2]) / (lin[0]*lin[0] + lin[1]*lin[1]);
        double projx = look_x - lin[0]*d, projy = look_y - lin[1]*d;
        // move to the side of projected spot
        look_x = projx -lin[1]*d;
        look_y = projy + lin[0]*d;
      }
    }
    double look_xr, look_yr;
    std::tie(look_xr, look_yr) = toRelativePos(eta_base_x, eta_base_y, theta, look_x, look_y, false);
    double new_eta = atan2(look_yr, look_xr);
    double new_L_fw = hypot(look_xr, look_yr);
    return std::make_tuple(new_eta, new_L_fw);
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
    double L_fw= params_.lookahead_dist + params_.lookahead_time * abs(v); // TODO what happens if this is negative?

    double dx_r, dy_r;
    std::tie(dx_r, dy_r) = toRelativePos(x, y, theta, goal_x, goal_y, true);
    double eta_orig = atan2(dy_r, dx_r);
    std::tie(eta_orig, L_fw) = etaLfwWithObstacles(eta_orig, L_fw, x, y, theta);

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

