#include <hector_action_demux/action_client.h>

#include <hector_action_demux/common.h>

namespace hector_action_demux {

ActionClient::ActionClient(const std::string& name, const ros::NodeHandle& action_nh)
: name_(name), action_nh_(action_nh), goal_active_(false)
{
}

void ActionClient::start()
{
  feedback_sub_ = action_nh_.subscribe<topic_tools::ShapeShifter>("feedback", 10, feedback_callback_);
  status_sub_ = action_nh_.subscribe<topic_tools::ShapeShifter>("status", 10, status_callback_);
  result_sub_ = action_nh_.subscribe<topic_tools::ShapeShifter>("result", 10, result_callback_);
}

void ActionClient::shutdown()
{
//  goal_pub_.reset();
//  cancel_pub_.reset();
  feedback_sub_.shutdown();
  status_sub_.shutdown();
  result_sub_.shutdown();
}

void ActionClient::setFeedbackCallback(const std::function<void(const ShapeShifterConstPtr&)>& callback)
{
  feedback_callback_ = callback;
}

void ActionClient::setStatusCallback(const std::function<void(const ShapeShifterConstPtr&)>& callback)
{
  status_callback_ = callback;
}

void ActionClient::setResultCallback(const std::function<void(const ShapeShifterConstPtr&)>& callback)
{
  result_callback_ = callback;
}

void ActionClient::publishGoal(const ShapeShifterConstPtr& msg)
{
  goal_active_ = true;
  publishMessage(goal_pub_, action_nh_, "goal", *msg);
}

void ActionClient::publishCancel(const ShapeShifterConstPtr& msg)
{
  goal_active_ = false;
  publishMessage(cancel_pub_, action_nh_, "cancel", *msg);
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
