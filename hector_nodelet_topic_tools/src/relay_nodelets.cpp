#include <nodelet_topic_tools/nodelet_throttle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pluginlib/class_list_macros.h>

typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::PointCloud2> NodeletThrottlePointcloud;
PLUGINLIB_DECLARE_CLASS(hector_nodelet_topic_tools, NodeletThrottlePointcloud, NodeletThrottlePointcloud, nodelet::Nodelet)
