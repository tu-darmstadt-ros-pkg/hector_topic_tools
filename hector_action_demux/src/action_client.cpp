#include <hector_action_demux/action_client.h>

#include <hector_action_demux/common.h>

namespace hector_action_demux {

ActionClient::ActionClient(const std::string& name, const ros::NodeHandle& action_nh, const std::string& goal_type)
: name_(name), action_nh_(action_nh), goal_type_(goal_type), cancel_type_("actionlib_msgs/GoalID"), goal_active_(false)
{}

void ActionClient::start()
{
  goal_pub_ = fish_.advertise(action_nh_, goal_type_, "goal", 10, false);
  cancel_pub_ = fish_.advertise(action_nh_, cancel_type_, "cancel", 10, false);
  feedback_sub_ = action_nh_.subscribe<ros_babel_fish::BabelFishMessage>("feedback", 10, feedback_callback_);
  status_sub_ = action_nh_.subscribe<ros_babel_fish::BabelFishMessage>("status", 10, status_callback_);
  result_sub_ = action_nh_.subscribe<ros_babel_fish::BabelFishMessage>("result", 10, result_callback_);
}

void ActionClient::shutdown()
{
  goal_pub_.shutdown();
  cancel_pub_.shutdown();
  feedback_sub_.shutdown();
  status_sub_.shutdown();
  result_sub_.shutdown();
}

void ActionClient::setFeedbackCallback(const std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)>& callback)
{
  feedback_callback_ = callback;
}

void ActionClient::setStatusCallback(const std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)>& callback)
{
  status_callback_ = callback;
}

void ActionClient::setResultCallback(const std::function<void(const ros_babel_fish::BabelFishMessage::ConstPtr&)>& callback)
{
  result_callback_ = callback;
}

void ActionClient::publishGoal(const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
  goal_active_ = true;
  goal_pub_.publish(msg);
}

void ActionClient::publishCancel(const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
  goal_active_ = false;
  cancel_pub_.publish(msg);
}
void ActionClient::setGoalActive(bool active)
{
  goal_active_ = active;
}
bool ActionClient::goalActive() const
{
  return goal_active_;
}
std::string ActionClient::getName() const
{
  return name_;
}

}
