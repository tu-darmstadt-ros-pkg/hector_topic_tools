#ifndef HECTOR_ACTION_DEMUX_ACTION_CLIENT_H
#define HECTOR_ACTION_DEMUX_ACTION_CLIENT_H

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

namespace hector_action_demux {

typedef boost::shared_ptr<const topic_tools::ShapeShifter> ShapeShifterConstPtr;

class ActionClient {
public:
  explicit ActionClient(const ros::NodeHandle& action_nh);
  void setFeedbackCallback(const std::function<void(const ShapeShifterConstPtr&)>& callback);
  void setStatusCallback(const std::function<void(const ShapeShifterConstPtr&)>& callback);
  void setResultCallback(const std::function<void(const ShapeShifterConstPtr&)>& callback);
  void start();
  void shutdown();

  void publishGoal(const ShapeShifterConstPtr& msg);
  void publishCancel(const ShapeShifterConstPtr& msg);
private:
  ros::NodeHandle action_nh_;
  std::shared_ptr<ros::Publisher> goal_pub_;
  std::shared_ptr<ros::Publisher> cancel_pub_;
  ros::Subscriber feedback_sub_;
  ros::Subscriber status_sub_;
  ros::Subscriber result_sub_;

  std::function<void(const ShapeShifterConstPtr&)> feedback_callback_;
  std::function<void(const ShapeShifterConstPtr&)> status_callback_;
  std::function<void(const ShapeShifterConstPtr&)> result_callback_;

};
}


#endif  // HECTOR_ACTION_DEMUX_ACTION_CLIENT_H
