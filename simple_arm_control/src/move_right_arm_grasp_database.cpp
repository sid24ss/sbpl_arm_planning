#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/utils.h>
#include <pr2_grasp_database/pr2_grasp_database.h>

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
  SQLiteDB DB("/home/siddharth/ros-packages/grasp_database");


  ph.param<std::string>("arm_name", arm_name,std::string("right_arm"));
  goalA.motion_plan_request.group_name = arm_name;
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string("sbpl_planning/plan_path");

  double duration;
  ph.param<double>("planning_duration",duration,5.0);
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(duration);

  double number_of_goals;
  ph.param<double>("number_of_goals",number_of_goals,1.0);

  double goal_x,goal_y,goal_z,goal_roll,goal_pitch,goal_yaw;

  std::string obj_name;
  ph.param<std::string> ("object_name",obj_name,"default_object");


  ROS_INFO("Number of goals received: %d",int(number_of_goals));

  std::stringstream ssTemp;
  std::string sTemp;
  char paramTemp[80];
  DB.CreateObject(obj_name);


  for (int i = 0; i < int(number_of_goals); ++i)
  {
    ssTemp.str(std::string());
    ssTemp.clear();
    ssTemp << int(i);
    sTemp = ssTemp.str();
    ROS_INFO("Processing goal number : %s ",sTemp.c_str());

    paramTemp[0] = '\0';
    strcat(paramTemp,"goal_x_");
    strcat(paramTemp,sTemp.c_str());
    ph.param<double>(paramTemp,goal_x,0.75);
    paramTemp[0] = '\0';
    strcat(paramTemp,"goal_y_");
    strcat(paramTemp,sTemp.c_str());
    ph.param<double>(paramTemp,goal_y,-0.188);
    paramTemp[0] = '\0';
    strcat(paramTemp,"goal_z_");
    strcat(paramTemp,sTemp.c_str());
    ph.param<double>(paramTemp,goal_z,0.0);

    paramTemp[0] = '\0';
    strcat(paramTemp,"goal_roll_");
    strcat(paramTemp,sTemp.c_str());
    ph.param<double>(paramTemp,goal_roll,1.0);
    paramTemp[0] = '\0';
    strcat(paramTemp,"goal_pitch_");
    strcat(paramTemp,sTemp.c_str());
    ph.param<double>(paramTemp,goal_pitch,0.5);
    paramTemp[0] = '\0';
    strcat(paramTemp,"goal_yaw_");
    strcat(paramTemp,sTemp.c_str());
    ph.param<double>(paramTemp,goal_yaw,0.2);


    tf::Quaternion gripper_goal;
    geometry_msgs::Quaternion gripper_goal_msg;
    gripper_goal.setRPY(goal_roll,goal_pitch,goal_yaw);
    tf::quaternionTFToMsg(gripper_goal,gripper_goal_msg);

    geometry_msgs::Pose desired_pose;

    desired_pose.position.x = goal_x;
    desired_pose.position.y = goal_y;
    desired_pose.position.z = goal_z;
    // ROS_INFO("[send_move_arm_goal] Goal number: %d arm_name: %s frame: %s position: %0.3f %0.3f %0.3f orientation: %0.2f %0.2f %0.2f %0.2f",(i+1),arm_name.c_str(),desired_pose.header.frame_id.c_str(),goal_x,goal_y,goal_z,gripper_goal_msg.x,gripper_goal_msg.y,gripper_goal_msg.z,gripper_goal_msg.w);

    desired_pose.orientation.x = gripper_goal_msg.x;
    desired_pose.orientation.y = gripper_goal_msg.y;
    desired_pose.orientation.z = gripper_goal_msg.z;
    desired_pose.orientation.w = gripper_goal_msg.w;
    // DB.AddPregrasp(obj_name,desired_pose);

  }
  // geometry_msgs::PoseArray posearr_Pregrasp;
  // DB.GetPregrasp(obj_name,posearr_Pregrasp);


  // for (int i = 0; i < int(number_of_goals); ++i)
  // {

  //   arm_navigation_msgs::SimplePoseConstraint goal_pose;
  //   goal_pose.header.frame_id = "base_link";
  //   goal_pose.link_name = "r_wrist_roll_link";

  //   goal_pose.pose.position.x = posearr_Pregrasp.poses[i].position.x;
  //   goal_pose.pose.position.y = posearr_Pregrasp.poses[i].position.y;
  //   goal_pose.pose.position.z = posearr_Pregrasp.poses[i].position.z;
  //   goal_pose.pose.orientation.x = posearr_Pregrasp.poses[i].orientation.x;
  //   goal_pose.pose.orientation.y = posearr_Pregrasp.poses[i].orientation.y;
  //   goal_pose.pose.orientation.z = posearr_Pregrasp.poses[i].orientation.z;
  //   goal_pose.pose.orientation.w = posearr_Pregrasp.poses[i].orientation.w;

  //   goal_pose.absolute_position_tolerance.x = 0.01;
  //   goal_pose.absolute_position_tolerance.y = 0.01;
  //   goal_pose.absolute_position_tolerance.z = 0.01;

  //   goal_pose.absolute_roll_tolerance = 0.08;
  //   goal_pose.absolute_pitch_tolerance = 0.08;
  //   goal_pose.absolute_yaw_tolerance = 0.08;


  //   ROS_INFO("Attaching Pose to goal");
  //   arm_navigation_msgs::addGoalConstraintToMoveArmGoal(goal_pose,goalA);
  // }

  
  goalA.accept_partial_plans = true;
  goalA.accept_invalid_goals = true;
  goalA.disable_collision_monitoring = true;

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

