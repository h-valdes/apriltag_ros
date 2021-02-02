#ifndef APRILTAG_ROS_PARTIAL_DETECTOR_H
#define APRILTAG_ROS_PARTIAL_DETECTOR_H

#include "apriltag_ros/common_functions.h"

#include <memory>

#include <nodelet/nodelet.h>
#include "std_msgs/Bool.h"

namespace apriltag_ros
{

class PartialDetector: public nodelet::Nodelet
{
 public:
  PartialDetector() = default;
  ~PartialDetector() = default;

  void onInit();

  void imageCallback(const sensor_msgs::ImageConstPtr& image_rect,
                     const sensor_msgs::CameraInfoConstPtr& camera_info);

  void is_ir_callback(const std_msgs::Bool::ConstPtr& msg);

 private:
  std::shared_ptr<TagDetector> tag_detector_;
  bool draw_tag_detections_image_;
  cv_bridge::CvImagePtr cv_image_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraSubscriber camera_image_subscriber_;
  image_transport::Publisher tag_detections_image_publisher_;
  ros::Publisher tag_detections_publisher_;

  bool is_ir = false;
  ros::Subscriber is_ir_sub;
};

} // namespace apriltag_ros

#endif // APRILTAG_ROS_PARTIAL_DETECTOR_H
