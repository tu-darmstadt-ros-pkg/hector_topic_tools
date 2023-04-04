#ifndef HECTOR_ACTION_DEMUX_ACTION_DEMUXER_H
#define HECTOR_ACTION_DEMUX_ACTION_DEMUXER_H

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <unordered_map>

#include <hector_action_demux/action_client.h>

namespace hector_action_demux {
class ActionDemuxer {
public:
  ActionDemuxer(const ros::NodeHandle& server_nh);
  bool loadConfiguration();
  void switchClient();

  // Server Callbacks
  void goalCallback(const ShapeShifterConstPtr& msg);
  void cancelCallback(const ShapeShifterConstPtr& msg);

  // Client Callbacks
  void feedbackCallback(const ShapeShifterConstPtr& msg);
  void statusCallback(const ShapeShifterConstPtr& msg);
  void resultCallback(const ShapeShifterConstPtr& msg);

  void

private:
  // Action Server Interface
  ros::NodeHandle server_nh_;
  ros::Subscriber goal_sub_;
  ros::Subscriber cancel_sub_;
  std::shared_ptr<ros::Publisher> feedback_pub_;
  std::shared_ptr<ros::Publisher> status_pub_;
  std::shared_ptr<ros::Publisher> result_pub_;

  // Action Client Interface
  std::unordered_map<std::string, ActionClient> action_clients_;
  ActionClient* active_client_;
};
}


#endif  // HECTOR_ACTION_DEMUX_ACTION_DEMUXER_H
