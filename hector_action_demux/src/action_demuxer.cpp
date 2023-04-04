#include <hector_action_demux/action_demuxer.h>

namespace hector_action_demux {

ActionDemuxer::ActionDemuxer(const ros::NodeHandle& server_nh)
: server_nh_(server_nh)
{
  goal_sub_ = server_nh_.subscribe<topic_tools::ShapeShifter>("goal", 10, &ActionDemuxer::goalCallback, this);
  cancel_sub_ = server_nh_.subscribe<topic_tools::ShapeShifter>("cancel", 10, &ActionDemuxer::cancelCallback, this);
}

void ActionDemuxer::goalCallback(const ShapeShifterConstPtr& msg)
{
  active_client_->publishGoal(msg);
}
void ActionDemuxer::cancelCallback(const ShapeShifterConstPtr& msg)
{
  active_client_->publishCancel(msg);
}

void ActionDemuxer::feedbackCallback(const ShapeShifterConstPtr& msg)
{

}

void ActionDemuxer::statusCallback(const ShapeShifterConstPtr& msg)
{
}

void ActionDemuxer::resultCallback(const ShapeShifterConstPtr& msg)
{
}

}
