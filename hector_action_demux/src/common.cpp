#include <hector_action_demux/common.h>

namespace hector_action_demux {

void publishMessage(std::shared_ptr<ros::Publisher>& publisher, ros::NodeHandle& nh,
                    const std::string& topic_name, const topic_tools::ShapeShifter& msg)
{
  if (!publisher) {
    publisher = std::make_shared<ros::Publisher>(msg.advertise(nh, topic_name, 10, false));
    waitForSubscribers(*publisher, ros::Duration(1));
  }
  publisher->publish(msg);
}

void waitForSubscribers(const ros::Publisher& publisher, ros::Duration timeout) {
  ros::Time start = ros::Time::now();
  ros::Rate rate(10.0);
  while (publisher.getNumSubscribers() == 0 && ((ros::Time::now() - start) < timeout || timeout.toSec() == 0.0)) {
    rate.sleep();
  }
}


}
