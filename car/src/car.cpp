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

  ros::Subscriber controlSub;
  ros::Publisher posePub;
  ros::Timer timer;

  tf2_ros::TransformBroadcaster br;

  double x;
  double y;
  double theta;
  double v;

  double L; // length between wheels/wheelbase
  double lfw; // length ahead of rear wheel to plan from
  double max_accel;

  double desired_steering_angle; // delta, rad
  double desired_speed; // m/s

public:
  CarNode(): x(0), y(0), theta(0) {
    controlSub = nh.subscribe("car_control", 1, &CarNode::onControl, this);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("car_pose", 1, this);
    timer = nh.createTimer(ros::Duration(1/30.0), &CarNode::simLoop, this);
  }

  void onControl(const ackermann_msgs::AckermannDrive& control) {}

  void simLoop(const ros::TimerEvent& t) {
    std::cout << "in sim loop\n";
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "car_node");
  CarNode n;
  ros::spin();
}

