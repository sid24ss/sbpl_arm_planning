#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <sstream>
#include <string>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <arm_navigation_msgs/utils.h>
#include <sensor_msgs/JointState.h>

#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <kinematics_msgs/GetPositionFK.h>

#include <arm_navigation_msgs/FilterJointTrajectoryWithConstraints.h>

class currentJointState{
	public:
		std::vector<std::string > name;
		std::vector<double> position;
		void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

void currentJointState::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
	this->name = msg->name;
	this->position = msg->position;
	for (int i = 0; i < this->name.size(); ++i)
	{
		ROS_DEBUG("Joint name received : %s with position %1.4f",this->name[i].c_str(), this->position[i]);
	}
}


int main(int argc, char **argv){
	ros::init (argc, argv, "move_arm_pose_goal_test");
	ros::NodeHandle nh;
	ros::NodeHandle ph("~");
	
	std::string arm_name;
	ROS_INFO("using right arm"); 
//  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm(nh,"move_arm");
	// actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_right_arm",true);
	actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > traj_client("r_arm_controller/joint_trajectory_action",true);
	 // wait for action server to come up
		// move_arm.waitForServer();
	while(!traj_client.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the joint_trajectory_action server");
	}

	// ros::service::waitForService("kinematics_msgs/GetKinematicSolverInfo");
	// ros::service::waitForService("kinematics_msgs/GetPositionIK");
	// ros::service::waitForService("kinematics_msgs/GetPositionFK");

	ROS_INFO("Connected to server. Awesome.");
	arm_navigation_msgs::MoveArmGoal goalA;
	arm_navigation_msgs::GetMotionPlan::Request req;
	arm_navigation_msgs::GetMotionPlan::Response res;


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

	ROS_INFO("Number of goals received: %d",int(number_of_goals));
	double goal_x,goal_y,goal_z,goal_roll,goal_pitch,goal_yaw;
	std::stringstream ssTemp;
	std::string sTemp;
	char paramTemp[80];
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

		arm_navigation_msgs::SimplePoseConstraint desired_pose;
		desired_pose.header.frame_id = "base_link";
		desired_pose.link_name = "r_wrist_roll_link";
		desired_pose.pose.position.x = goal_x;
		desired_pose.pose.position.y = goal_y;
		desired_pose.pose.position.z = goal_z;
		ROS_INFO("[send_move_arm_goal] Goal number: %d arm_name: %s frame: %s position: %0.3f %0.3f %0.3f orientation: %0.2f %0.2f %0.2f %0.2f",(i+1),arm_name.c_str(),desired_pose.header.frame_id.c_str(),goal_x,goal_y,goal_z,gripper_goal_msg.x,gripper_goal_msg.y,gripper_goal_msg.z,gripper_goal_msg.w);

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
		ROS_INFO("Attaching Pose to goal");
		arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);
	
	}
	
	goalA.disable_ik = true;
	goalA.disable_collision_monitoring = true;

	// arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	goalA.accept_partial_plans = true;
	goalA.accept_invalid_goals = true;

	req.motion_plan_request.workspace_parameters.workspace_region_pose.header.stamp = ros::Time::now();
	req.motion_plan_request = goalA.motion_plan_request;

	currentJointState cJS;
	ros::Subscriber sub = ph.subscribe("/joint_states", 0, &currentJointState::jointStateCallback,&cJS);
	cJS.position.clear();
	cJS.name.clear();
	while(cJS.position.empty() && cJS.name.empty()){
		ros::spinOnce();
	}
	req.motion_plan_request.start_state.joint_state.position = cJS.position;
	req.motion_plan_request.start_state.joint_state.name = cJS.name;

	if (nh.ok())
	{
		bool finished_within_time = false;
		ROS_INFO("Sending Goal");

		while(!ros::service::waitForService(goalA.planner_service_name, ros::Duration(1.0))) {
			 ROS_INFO_STREAM("Waiting for requested service " << goalA.planner_service_name);
		}
		ros::ServiceClient planning_client = nh.serviceClient<arm_navigation_msgs::GetMotionPlan>(goalA.planner_service_name);
		if (planning_client.call(req, res))
		{
			if (res.trajectory.joint_trajectory.points.empty())
			{
				ROS_WARN("Motion planner was unable to plan a path to goal");
				// return false;
			}
			else{
				ROS_INFO("Motion planning succeeded");
				// Filter motion plan
				arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Request filterReq;
				arm_navigation_msgs::FilterJointTrajectoryWithConstraints::Response filterRes;
				ros::ServiceClient filter_trajectory_client_ = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectoryWithConstraints>("/trajectory_filter_server/filter_trajectory_with_constraints");
				filterReq.trajectory = res.trajectory.joint_trajectory;
				filterReq.allowed_time = ros::Duration(1.0);
				if(filter_trajectory_client_.call(filterReq,filterRes))
				{
					if(filterRes.error_code.val == filterRes.error_code.SUCCESS)
					{
					  ROS_INFO("Requested trajectory was filtered");
					 //  for(unsigned int i=0; i < filterRes.trajectory.points.size(); i++)
					 //  {
						// ROS_INFO_STREAM(filterRes.trajectory.points[i].positions[0] << "," << filterRes.trajectory.points[i].velocities[0] << "," << filterRes.trajectory.points[i].positions[1] << "," << filterRes.trajectory.points[i].velocities[1] << "," << filterRes.trajectory.points[i].time_from_start.toSec());
					 //  }
					}
					else
					  ROS_INFO("Requested trajectory was not filtered. Error code: %d",filterRes.error_code.val);
				}
				else
				{
					ROS_ERROR("Service call to filter trajectory failed %s",filter_trajectory_client_.getService().c_str());
				}

				//Execute the motion plan
				pr2_controllers_msgs::JointTrajectoryGoal executable_goal;
				executable_goal.trajectory = filterRes.trajectory;
				traj_client.sendGoal(executable_goal);
				while(!traj_client.getState().isDone() && ros::ok())
				{
					usleep(50000);
				}
				actionlib::SimpleClientGoalState state = traj_client.getState();
				bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
				if(success)
					ROS_INFO("Action finished: %s",state.toString().c_str());
				else
					ROS_INFO("Action failed: %s",state.toString().c_str());
			}
		}
		else
		{
			ROS_ERROR("Motion planning service failed on %s",planning_client.getService().c_str());
		}
		// move_arm.sendGoal(goalA);
		// finished_within_time = move_arm.waitForResult(ros::Duration(100.0));
		// if (!finished_within_time)
		// {
		//   move_arm.cancelGoal();
		//   ROS_INFO("Timed out achieving goal A");
		// }
		// else
		// {
		//   actionlib::SimpleClientGoalState state = move_arm.getState();
		//   bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		//   if(success)
		//     ROS_INFO("Action finished: %s",state.toString().c_str());
		//   else
		//     ROS_INFO("Action failed: %s",state.toString().c_str());
		// }

	}
	ros::shutdown();
}

