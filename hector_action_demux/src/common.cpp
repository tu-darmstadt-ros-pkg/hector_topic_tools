#include <hector_action_demux/common.h>

namespace hector_action_demux {

void publishMessage(std::shared_ptr<ros::Publisher>& publisher, ros::NodeHandle& nh,
                    const std::string& topic_name, const topic_tools::ShapeShifter& msg)
{
  if (!publisher) {
    publisher = std::make_shared<ros::Publisher>(msg.advertise(nh, topic_name, 10, false));
  }
  publisher->publish(msg);
}

}
