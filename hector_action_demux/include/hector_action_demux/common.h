#ifndef HECTOR_ACTION_DEMUX_COMMON_H
#define HECTOR_ACTION_DEMUX_COMMON_H

#include <ros/ros.h>
#include <ros_babel_fish/babel_fish_message.h>

namespace hector_action_demux {

std::string getActionMessageType(const ros_babel_fish::Message::Ptr& action_msg, const std::string& action_field);

std::string getActionMessageGoalType(const ros_babel_fish::Message::Ptr& action_msg);
std::string getActionMessageFeedbackType(const ros_babel_fish::Message::Ptr& action_msg);
std::string getActionMessageResultType(const ros_babel_fish::Message::Ptr& action_msg);

}

#endif  // HECTOR_ACTION_DEMUX_COMMON_H
