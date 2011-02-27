#include <pr2_arm_planning_tests/control_arm.h>

ControlArm::ControlArm() 
{
  // tell the action client that we want to spin a thread by default
  traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

  // wait for action server to come up
  while(!traj_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }
}

ControlArm::~ControlArm()
{
  delete traj_client_;
}

void ControlArm::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
{
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  traj_client_->sendGoal(goal);
}

pr2_controllers_msgs::JointTrajectoryGoal ControlArm::setArmConfiguration(std::vector<double> &waypoint)
{
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
  goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
  goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
  goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

  goal.trajectory.points.resize(1);
  goal.trajectory.points[0].positions.resize(7);

  for(size_t i = 0; i < waypoint.size(); i++)
    goal.trajectory.points[0].positions[i] = angles::normalize_angle(waypoint[i]);

  goal.trajectory.points[0].velocities.resize(7);
  for (size_t j = 0; j < 7; ++j)
    goal.trajectory.points[0].velocities[j] = 0.0;

  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

  return goal;
}

actionlib::SimpleClientGoalState ControlArm::getState()
{
  return traj_client_->getState();
}

