#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>

int main(int argc, char **argv){
  ros::init (argc, argv, "move_arm_pose_goal_test");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  std::string arm_name;
  ROS_INFO("using right arm"); 
//  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(nh,"move_arm");
  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("Connected to server");
  arm_navigation_msgs::MoveArmGoal goalA;


  ph.param<std::string>("arm_name", arm_name,std::string("right_arm"));
  goalA.motion_plan_request.group_name = arm_name;
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("sbpl_planning/plan_path");

  double duration;
  ph.param<double>("planning_duration",duration,5.0);
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(duration);

  double goal_x,goal_y,goal_z,goal_roll,goal_pitch,goal_yaw;
 
  ph.param<double>("goal_x",goal_x,0.75);
  ph.param<double>("goal_y",goal_y,-0.188);
  ph.param<double>("goal_z",goal_z,0.0);

  ph.param<double>("goal_roll",goal_roll,1.0);
  ph.param<double>("goal_pitch",goal_pitch,0.5);
  ph.param<double>("goal_yaw",goal_yaw,0.2);

  //btQuaternion gripper_goal;
  tf::Quaternion gripper_goal;
  geometry_msgs::Quaternion gripper_goal_msg;
  gripper_goal.setRPY(goal_roll,goal_pitch,goal_yaw);
  tf::quaternionTFToMsg(gripper_goal,gripper_goal_msg);
 
  arm_navigation_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = "base_link";
  desired_pose.link_name = "r_wrist_roll_link";
  desired_pose.pose.position.x = goal_x;
  desired_pose.pose.position.y = goal_y;
  desired_pose.pose.position.z = goal_z;
  ROS_INFO("[send_move_arm_goal] arm_name: %s frame: %s position: %0.3f %0.3f %0.3f orientation: %0.2f %0.2f %0.2f %0.2f",arm_name.c_str(),desired_pose.header.frame_id.c_str(),goal_x,goal_y,goal_z,gripper_goal_msg.x,gripper_goal_msg.y,gripper_goal_msg.z,gripper_goal_msg.w);

  desired_pose.pose.orientation.x = gripper_goal_msg.x;
  desired_pose.pose.orientation.y = gripper_goal_msg.y;
  desired_pose.pose.orientation.z = gripper_goal_msg.z;
  desired_pose.pose.orientation.w = gripper_goal_msg.w;

  desired_pose.absolute_position_tolerance.x = 0.01;
  desired_pose.absolute_position_tolerance.y = 0.01;
  desired_pose.absolute_position_tolerance.z = 0.01;

  desired_pose.absolute_roll_tolerance = 0.08;
  desired_pose.absolute_pitch_tolerance = 0.08;
  desired_pose.absolute_yaw_tolerance = 0.08;
  
  goalA.disable_ik = true;
  goalA.disable_collision_monitoring = true;

  arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  goalA.accept_partial_plans = true;

  if (nh.ok())
  {
    bool finished_within_time = false;
    ROS_INFO("Sending Goal");
    move_arm.sendGoal(goalA);
    finished_within_time = move_arm.waitForResult(ros::Duration(100.0));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action finished: %s",state.toString().c_str());
      else
        ROS_INFO("Action failed: %s",state.toString().c_str());
    }
  }
  ros::shutdown();
}

