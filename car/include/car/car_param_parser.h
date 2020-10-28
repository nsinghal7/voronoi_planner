#pragma once

#include <ros/ros.h>

namespace car {

class CarParamParser {
public:
  double L;
  double lfw;
  double max_accel;
  double max_vel;
  double max_delta;
  double width;
  double length;
  double height;
  double lookahead_dist;
  double lookahead_time;
  double hz;

  std::string odom_topic;
  std::string control_topic;

  std::string formation;
  int n_cars;
  bool autostart;

  void parse(ros::NodeHandle nh) {
    std::string key;
    nh.searchParam("L", key);
    nh.getParam(key, L);
    nh.searchParam("lfw", key);
    nh.getParam(key, lfw);
    nh.searchParam("max_accel", key);
    nh.getParam(key, max_accel);
    nh.searchParam("max_vel", key);
    nh.getParam(key, max_vel);
    nh.searchParam("max_delta", key);
    nh.getParam(key, max_delta);
    nh.searchParam("width", key);
    nh.getParam(key, width);
    nh.searchParam("length", key);
    nh.getParam(key, length);
    nh.searchParam("height", key);
    nh.getParam(key, height);
    nh.searchParam("lookahead_dist", key);
    nh.getParam(key, lookahead_dist);
    nh.searchParam("lookahead_time", key);
    nh.getParam(key, lookahead_time);
    nh.searchParam("sim_hz", key);
    nh.getParam(key, hz);

    nh.searchParam("odom_topic", key);
    nh.getParam(key, odom_topic);
    nh.searchParam("control_topic", key);
    nh.getParam(key, control_topic);

    nh.getParam("/all_cars/formation", formation);
    nh.getParam("/all_cars/n_cars", n_cars);
    nh.getParam("/all_cars/autostart", autostart);
  }
};

} // namespace car

