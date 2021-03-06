#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <car/OdomArray.h>

namespace car {


class OdomCollectorNode {
  ros::NodeHandle nh_;

  std::vector<ros::Subscriber> odom_subs_;
  ros::Publisher collection_pub_;
  std::vector<nav_msgs::Odometry> odoms_;
  ros::Timer timer_;

public:
  OdomCollectorNode() {
    int n_cars;
    nh_.getParam("/all_cars/n_cars", n_cars);
    double traj_hz;
    nh_.getParam("/all_cars/traj_hz", traj_hz);
    collection_pub_ = nh_.advertise<OdomArray>("/all_cars/odoms", 1, this);
    odoms_.resize(n_cars);
    for(int i = 1; i <= n_cars; i++) {
      odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>("/car_" + std::to_string(i) + "/odom", 1, [i, this] (const boost::shared_ptr<const nav_msgs::Odometry> odom) -> void {
              this->odoms_.at(i - 1) = *odom;
            }));
    }

    timer_ = nh_.createTimer(ros::Duration(1/traj_hz), &OdomCollectorNode::publish, this);
  }

  void publish(const ros::TimerEvent& e) {
    OdomArray output;
    output.odoms = odoms_;
    collection_pub_.publish(output);
  }
};

} // namespace car

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_collector_node");
  car::OdomCollectorNode n;
  ros::spin();
}

