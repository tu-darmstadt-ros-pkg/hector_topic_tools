#include <nodelet_topic_tools/nodelet_throttle.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pluginlib/class_list_macros.h>

typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::PointCloud2> NodeletThrottlePointcloud;
typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::Image> NodeletThrottleImage;

PLUGINLIB_EXPORT_CLASS(NodeletThrottlePointcloud, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(NodeletThrottleImage, nodelet::Nodelet)
