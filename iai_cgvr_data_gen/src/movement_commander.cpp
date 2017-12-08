#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <giskard_msgs/WholeBodyAction.h>
#include <giskard_ros/ros_utils.hpp>

giskard_msgs::ArmCommand make_cartesian_command(const std::vector<double>& pose,
    const std::map< std::string, double>& thresholds)
{
  giskard_msgs::ArmCommand msg;
  msg.goal_pose.header.stamp = ros::Time::now();
  msg.goal_pose.header.frame_id = "base_link";
  msg.goal_pose.pose = giskard_ros::make_pose(pose);
  msg.type = giskard_msgs::ArmCommand::CARTESIAN_GOAL;
  msg.convergence_thresholds = giskard_ros::to_msg(thresholds);
  return msg;
}

giskard_msgs::ArmCommand make_joint_command(const std::vector<double> config,
    const std::map<std::string, double> thresholds)
{
  giskard_msgs::ArmCommand msg;
  msg.goal_configuration = config;
  msg.type = giskard_msgs::ArmCommand::JOINT_GOAL;
  msg.convergence_thresholds = giskard_ros::to_msg(thresholds);
  return msg;
}

giskard_msgs::WholeBodyGoal all_joint_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_joint_command(
    giskard_ros::readParam< std::vector<double> >(nh, "goals/left_arm/joint"),
    giskard_ros::readParam< std::map<std::string, double> >(nh, "thresholds/left_arm/joint"));
  goal.command.right_ee = make_joint_command(
    giskard_ros::readParam< std::vector<double> >(nh, "goals/right_arm/joint"),
    giskard_ros::readParam< std::map<std::string, double> >(nh, "thresholds/right_arm/joint"));

  return goal;
}

giskard_msgs::WholeBodyGoal left_pose_goal(const ros::NodeHandle& nh)
{
  giskard_msgs::WholeBodyGoal goal;
  goal.command.left_ee = make_cartesian_command(
      giskard_ros::readParam< std::vector<double> >(nh, "goals/left_arm/cartesian"),
      giskard_ros::readParam< std::map<std::string, double> >(nh,
        "thresholds/left_arm/cartesian"));
  return goal;
}

void execute_goal(actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction>& client,
    const giskard_msgs::WholeBodyGoal& goal, const std::string& message, const ros::Duration& dur)
{
  ROS_INFO("%s", message.c_str());
  client.sendGoal(goal);
  if (client.waitForResult(dur))
    ROS_INFO("Action finished: %s", client.getState().toString().c_str());
  else
    ROS_INFO("Action timed out.");
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "controller_action_test_client");
  ros::NodeHandle nh("~");

  actionlib::SimpleActionClient<giskard_msgs::WholeBodyAction> client("/controller_action_server/move", true);

  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Action server started, sending goal.");

  ros::Duration dur(10.0);
  execute_goal(client, all_joint_goal(nh), "Sending goal. Left: joint, right: joint.", dur);

  execute_goal(client, left_pose_goal(nh), "Sending goal. Left: pose, right: ignore.", dur);

  return 0;
}
