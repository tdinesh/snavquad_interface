#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <snap_msgs/MapPointArray.h>

#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <cv_bridge/cv_bridge.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, snap_msgs::MapPointArray> ApproxSyncPolicy;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, snap_msgs::MapPointArray> ExactSyncPolicy;

class PointOverlay
{
public:
  PointOverlay();
private:
  void syncCallback(const sensor_msgs::ImageConstPtr& im_msg, const snap_msgs::MapPointArrayConstPtr& points_msg);

  ros::NodeHandle nh_, pnh_;

  image_transport::ImageTransport it_;
  image_transport::Publisher im_overlay_pub_;

  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_;
  boost::shared_ptr<message_filters::Subscriber<snap_msgs::MapPointArray>> point_sub_;

  boost::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> approx_sync_;
  boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy>> exact_sync_;

};

PointOverlay::PointOverlay(): it_(nh_)
{
  pnh_ = ros::NodeHandle("~");

  im_overlay_pub_ = it_.advertise("image_overlay", 1);

  image_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(nh_, "image_raw", 1));
  point_sub_.reset(new message_filters::Subscriber<snap_msgs::MapPointArray>(nh_, "vio/map_points", 1));

  approx_sync_.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(10), *image_sub_, *point_sub_));
  approx_sync_->registerCallback(boost::bind(&PointOverlay::syncCallback, this, _1, _2));

  //exact_sync_.reset(new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(10), *image_sub_, *point_sub_));
  //exact_sync_->registerCallback(boost::bind(&PointOverlay::syncCallback, this, _1, _2));
}

void PointOverlay::syncCallback(const sensor_msgs::ImageConstPtr& im_msg, const snap_msgs::MapPointArrayConstPtr& points_msg)
{

  cv::Mat cam_im, cam_im_rect;
  try {
    cam_im = cv_bridge::toCvShare(im_msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat cam_im_col;
  cv::cvtColor(cam_im, cam_im_col, cv::COLOR_GRAY2BGR);

  for(int i = 0; i < points_msg->map_points.size(); i++)
  {
    snap_msgs::MapPoint pt = points_msg->map_points[i];
    cv::Scalar color;
    int r = 0;

    if(pt.point_quality == 2)
    {
      //High quality points - Green
      color = cv::Scalar(0,255,0);
      r = 7;
    }
    else if(pt.point_quality == 1)
    {
      //Low quality points - Red
      color = cv::Scalar(0,0,255);
      r = 5;
    }
    else if(pt.point_quality == 0)
    {
      color = cv::Scalar(255,0,0);
      r = 4;
    }

    if ((pt.pixel_location[0] > 0) && (pt.pixel_location[1] > 0))
    {
      cv::circle(cam_im_col, cv::Point(int(pt.pixel_location[0]), int(pt.pixel_location[1])), r, color, 2);
    }
  }

  cv_bridge::CvImage out_r(im_msg->header, "bgr8", cam_im_col);
  //img.header.frame_id = 'dfc'
  im_overlay_pub_.publish(out_r.toImageMsg());

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_overlay");
  PointOverlay pt_ov;
  ros::spin();
}