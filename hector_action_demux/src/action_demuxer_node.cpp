#include <hector_action_demux/action_demuxer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "action_demuxer_node");

  ros::NodeHandle pnh("~");
  hector_action_demux::ActionDemuxer demuxer(pnh);
  ros::spin();

  return 0;
}
