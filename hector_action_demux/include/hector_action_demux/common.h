#ifndef HECTOR_ACTION_DEMUX_COMMON_H
#define HECTOR_ACTION_DEMUX_COMMON_H

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace hector_action_demux {

void publishMessage(std::shared_ptr<ros::Publisher>& publisher, ros::NodeHandle& nh,
                    const std::string& topic_name, const topic_tools::ShapeShifter& msg);

void waitForSubscribers(const ros::Publisher& publisher, ros::Duration timeout=ros::Duration(0));

}

#endif  // HECTOR_ACTION_DEMUX_COMMON_H
