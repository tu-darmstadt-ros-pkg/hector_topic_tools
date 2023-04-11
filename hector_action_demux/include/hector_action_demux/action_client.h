#ifndef HECTOR_ACTION_DEMUX_ACTION_CLIENT_H
#define HECTOR_ACTION_DEMUX_ACTION_CLIENT_H

#include <ros/ros.h>
#include <ros_babel_fish/babel_fish.h>

namespace hector_action_demux {

class ActionClient {
public:
  explicit ActionClient(const std::string& name, const ros::NodeHandle& action_nh, const std::string& goal_type);
  void setFeedbackCallback(const std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)>& callback);
  void setStatusCallback(const std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)>& callback);
  void setResultCallback(const std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)>& callback);
  void start();
  void shutdown();

  void publishGoal(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);
  void publishCancel(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);

  std::string getName() const;
  void setGoalActive(bool active);
  bool goalActive() const;
private:
  std::string name_;
  ros::NodeHandle action_nh_;
  ros_babel_fish::BabelFish fish_;
  ros::Publisher goal_pub_;
  ros::Publisher cancel_pub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  std::string goal_type_;
  std::string cancel_type_;

  std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)> feedback_callback_;
  std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)> status_callback_;
  std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)> result_callback_;

  bool goal_active_;
};

typedef std::shared_ptr<ActionClient> ActionClientPtr;
typedef std::shared_ptr<const ActionClient> ActionClientConstPtr;

}

#endif  // HECTOR_ACTION_DEMUX_ACTION_CLIENT_H
