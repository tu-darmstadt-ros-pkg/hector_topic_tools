#ifndef HECTOR_ACTION_DEMUX_ACTION_DEMUXER_H
#define HECTOR_ACTION_DEMUX_ACTION_DEMUXER_H

#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <unordered_map>

#include <hector_action_demux/action_client.h>

namespace hector_action_demux {
class ActionDemuxer {
public:
  ActionDemuxer(const ros::NodeHandle& pnh);
  bool loadConfiguration(const ros::NodeHandle& nh);
  void switchClient(const std::string& name);

  // Server Callbacks
  void goalCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);
  void cancelCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);

  // Client Callbacks
  void feedbackCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);
  void statusCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);
  void resultCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);

  // Dynamic Reconfigure
  void outputChangedCallback(const std::string& value);
private:
  // Action Server Interface
  ros::NodeHandle pnh_;
  ros::NodeHandle server_nh_;
  ros_babel_fish::BabelFish fish_;
  ros::Subscriber goal_sub_;
  ros::Subscriber cancel_sub_;
  ros::Publisher feedback_pub_;
  ros::Publisher status_pub_;
  ros::Publisher result_pub_;

  std::string goal_type_;
//  std::string cance_type_;
  std::string feedback_type_;
  std::string status_type_;
  std::string result_type_;

  // Action Client Interface
  std::unordered_map<std::string, ActionClientPtr> action_clients_;
  ActionClientPtr active_client_;

  ddynamic_reconfigure::DDynamicReconfigure reconfigure_server_;
  std::string active_client_name_;
};
}


#endif  // HECTOR_ACTION_DEMUX_ACTION_DEMUXER_H
