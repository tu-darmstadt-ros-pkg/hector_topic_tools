#include <hector_action_demux/common.h>
#include <ros_babel_fish/messages/compound_message.h>

namespace hector_action_demux {

std::string getActionMessageType(const ros_babel_fish::Message::Ptr& action_msg, const std::string& action_field) {
  return action_msg->as<ros_babel_fish::CompoundMessage>()[action_field].as<ros_babel_fish::CompoundMessage>().datatype();
}

std::string getActionMessageGoalType(const ros_babel_fish::Message::Ptr& action_msg) {
  return getActionMessageType(action_msg, "action_goal");
}

std::string getActionMessageFeedbackType(const ros_babel_fish::Message::Ptr& action_msg) {
  return getActionMessageType(action_msg, "action_feedback");
}

std::string getActionMessageResultType(const ros_babel_fish::Message::Ptr& action_msg) {
  return getActionMessageType(action_msg, "action_result");
}


}
