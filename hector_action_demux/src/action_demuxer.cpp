#include <hector_action_demux/action_demuxer.h>

#include <hector_action_demux/common.h>

namespace hector_action_demux {

ActionDemuxer::ActionDemuxer(const ros::NodeHandle& pnh)
: pnh_(pnh), status_type_("actionlib_msgs/GoalStatusArray"), reconfigure_server_(pnh)
{
  if (loadConfiguration(pnh)) {
    goal_sub_ = server_nh_.subscribe<ros_babel_fish::BabelFishMessage>("goal", 10, &ActionDemuxer::goalCallback, this);
    cancel_sub_ = server_nh_.subscribe<ros_babel_fish::BabelFishMessage>("cancel", 10, &ActionDemuxer::cancelCallback, this);
    feedback_pub_ = fish_.advertise(server_nh_, feedback_type_, "feedback", 10, false);
    status_pub_ = fish_.advertise(server_nh_, status_type_, "status", 10, false);
    result_pub_ = fish_.advertise(server_nh_, result_type_, "result", 10, false);
  }
}

void ActionDemuxer::goalCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Goal received");
  if (active_client_) {
    active_client_->publishGoal(msg);
  }
}
void ActionDemuxer::cancelCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Cancel received");
  if (active_client_) {
    active_client_->publishCancel(msg);
  }
}

void ActionDemuxer::feedbackCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
  feedback_pub_.publish(msg);
}

void ActionDemuxer::statusCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
  status_pub_.publish(msg);
}

void ActionDemuxer::resultCallback(const ros_babel_fish::BabelFishMessage::ConstPtr& msg)
{
  active_client_->setGoalActive(false);
  result_pub_.publish(msg);
}

void ActionDemuxer::switchClient(const std::string& name)
{
  if (active_client_) {
    if (active_client_->goalActive()) {
      ROS_WARN_STREAM("Goal active, cannot switch client");
      active_client_name_ = active_client_->getName();
      return;
    }
    if (active_client_->getName() == name) {
      return;
    }
  }

  auto it = action_clients_.find(name);
  if (it == action_clients_.end()) {
    ROS_ERROR_STREAM("Unknown action client name '" << name << "'.");
    return;
  }
  if (active_client_) {
    active_client_->shutdown();
  }
  active_client_ = it->second;
  active_client_->start();
  ROS_INFO_STREAM("Switched active action client to " << name);
}

bool ActionDemuxer::loadConfiguration(const ros::NodeHandle& nh)
{
  std::string in_action_server_ns;
  if (!nh.getParam("in_action_server_ns", in_action_server_ns)) {
    ROS_ERROR_STREAM(nh.getNamespace() + "/in_action_server_ns not found.");
    return false;
  }
  server_nh_ = ros::NodeHandle(in_action_server_ns);

  std::string action_msg_type;
  if (!nh.getParam("action_msg_type", action_msg_type)) {
    ROS_ERROR_STREAM(nh.getNamespace() + "/action_msg_type not found.");
    return false;
  }

  // Message types
  ros_babel_fish::Message::Ptr message;
  try {
    message = fish_.createMessage(action_msg_type);
  } catch (const ros_babel_fish::BabelFishException& e) {
    ROS_ERROR_STREAM(e.what());
    return false;
  }
  goal_type_ = getActionMessageGoalType(message);
  feedback_type_ = getActionMessageFeedbackType(message);
  result_type_ = getActionMessageResultType(message);

  XmlRpc::XmlRpcValue action_servers;
  nh.getParam("out_action_servers", action_servers);
  if (action_servers.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM(nh.getNamespace() << "/out_action_servers is not an array.");
    return false;
  }
  std::map<std::string, std::string> enum_map;
  for (int i = 0; i < action_servers.size(); ++i) {
    const XmlRpc::XmlRpcValue& server_dict = action_servers[i];
    if (server_dict.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      ROS_ERROR_STREAM(nh.getNamespace() << "/" << i << " is not a struct.");
      continue;
    }
    if (server_dict["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR_STREAM(nh.getNamespace() << "/" << i << "/name is not a string.");
      continue;
    }
    if (server_dict["action_ns"].getType() != XmlRpc::XmlRpcValue::TypeString) {
      ROS_ERROR_STREAM(nh.getNamespace() << "/" << i << "/action_ns is not a string.");
      continue;
    }
    std::string name = static_cast<std::string>(server_dict["name"]);
    if (i == 0) {
      active_client_name_ = name;
    }
    ros::NodeHandle action_nh(static_cast<std::string>(server_dict["action_ns"]));
    ActionClientPtr action_client = std::make_shared<ActionClient>(name, action_nh, goal_type_);
    // Set callbacks
    action_client->setFeedbackCallback([this](const ros_babel_fish::BabelFishMessage::ConstPtr& msg) { this->feedbackCallback(msg); });
    action_client->setResultCallback([this](const ros_babel_fish::BabelFishMessage::ConstPtr& msg) { this->resultCallback(msg); });
    action_client->setStatusCallback([this](const ros_babel_fish::BabelFishMessage::ConstPtr& msg) { this->statusCallback(msg); });
    ROS_INFO_STREAM("Registered action server " << name << ": " << action_nh.getNamespace());
    action_clients_.emplace(name, action_client);
    enum_map.emplace(name, name);
  }

  if (action_clients_.empty()) {
    ROS_ERROR_STREAM("No action clients configured");
    active_client_ = nullptr;
  } else {
    std::string default_client;
    if (pnh_.getParam("default", default_client)) {
      active_client_name_ = default_client;
    }
    switchClient(active_client_name_);
    reconfigure_server_.registerEnumVariable<std::string>("output", &active_client_name_, std::bind(&ActionDemuxer::outputChangedCallback, this, std::placeholders::_1), "Change the output action server.", enum_map);
  }

  reconfigure_server_.publishServicesTopics();

  return true;
}
void ActionDemuxer::outputChangedCallback(const std::string& value)
{
  switchClient(value);
}

}
