#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class TfPub
{
public:
  TfPub();
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber odom_sub_;
  tf::TransformBroadcaster tf_broadcaster_;
  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);
};

TfPub::TfPub()
{
  pnh_ = ros::NodeHandle("~");
  odom_sub_ = pnh_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&TfPub::OdometryCallback, this, _1));
}

void TfPub::OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg) {

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = odometry_msg->header.stamp;
  odom_trans.header.frame_id = "world";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = odometry_msg->pose.pose.position.x;
  odom_trans.transform.translation.y = odometry_msg->pose.pose.position.y;
  odom_trans.transform.translation.z = odometry_msg->pose.pose.position.z;
  odom_trans.transform.rotation = odometry_msg->pose.pose.orientation;

  //send the transform
  tf_broadcaster_.sendTransform(odom_trans);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_pub");
  TfPub tf_pub;
  ros::spin();
}