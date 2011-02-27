#include <pr2_arm_planning_tests/arm_planning_test.h>


ArmPlanningTest::ArmPlanningTest() : ph_("~")
{
  disable_ik_ = true;
  disable_collision_monitoring_ = true;
  accept_partial_plans_ = true;
  position_tolerance_ = 0.025;
  orientation_tolerance_ = 0.15;
  movearm_duration_ = 50.0;
  max_planning_time_ = 3.0;

  goal_frame_ = "base_link";
  planning_link_ = "r_wrist_roll_link";
  planning_service_name_ = "/sbpl_planning/plan_path";
  planner_name_ = "sbpl";
  test_file_path_ = "/home/bcohen/penn_sandbox/gokul/pr2_arm_planning_tests/tests/";
  test_file_name_ = "goal_constraints_1.env";

  plan_subscriber_ = nh_.subscribe("display_path", 3, &ArmPlanningTest::motionPlanCallback, this);

  trajectory_file_path_ = "/home/bcohen/";

  checkParams();

  //aviz_ = new VisualizeArm("right_arm");
}

void ArmPlanningTest::checkParams()
{
  ph_.param ("disable_ik", disable_ik_, true);
  ph_.param ("disable_collision_monitoring", disable_collision_monitoring_, true);
  ph_.param ("accept_partial_plans", accept_partial_plans_, true);
  ph_.param<double>("max_planning_time", max_planning_time_,3.0);
  ph_.param<double>("allowed_move_arm_duration", movearm_duration_,200.0);
  ph_.param<double>("position_tolerance", position_tolerance_,0.01);
  ph_.param<double>("orientation_tolerance", orientation_tolerance_,0.1);
  ph_.param<std::string>("output_file_path", trajectory_file_path_,"/home/bcohen/");

  ph_.param<std::string>("goal_pose_frame", goal_frame_, std::string("base_link"));
  ph_.param<std::string>("planning_link", planning_link_,std::string("r_wrist_roll_link"));
  ph_.param<std::string>("planning_service_name", planning_service_name_, std::string("sbpl_planning/plan_path"));
  ph_.param<std::string>("planner_name", planner_name_, std::string("sbpl"));
  ph_.param<std::string>("test_file_name", test_file_name_, std::string("goal_constraints_1.env"));
  ph_.param<std::string>("test_file_path", test_file_path_, std::string("/home/bcohen/penn_sandbox/gokul/pr2_arm_planning_tests/tests/"));
}

bool ArmPlanningTest::runTestFromParamServer()
{
  checkParams();

  ROS_INFO("Running test: %s", std::string(test_file_path_+test_file_name_).c_str());

  return parseEnvironmentFile(test_file_path_+test_file_name_);
}

bool ArmPlanningTest::parseEnvironmentFile(std::string filename)
{
  char sTemp[1024];
  int i;
  std::vector<double> cube(6,0);

  filename_ = filename;
  char* file = new char[filename.length()+1];
  filename.copy(file, filename.length(),0);
  file[filename.length()] = '\0';
  FILE* fCfg = fopen(file, "r");

  if(fCfg == NULL)
  {
    ROS_WARN("Can't open environment file. Exiting.");
    ROS_WARN("File: %s",file);
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1)
    printf("Parsed string has length < 1.\n");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "linkstartangles(radians):") == 0)
    {
      start_angles_.resize(7,0);
      for(i = 0; i < 7; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          printf("Parsed string has length < 1.\n");
        start_angles_[i] = atof(sTemp);
      }
    }
    else if(strcmp(sTemp, "endeffectorgoal(meters):") == 0)
    {
      goal_.resize(6);

      //read number of goals -> ignored here (assume 1)
      if(fscanf(fCfg,"%s",sTemp) < 1)
        printf("Parsed string has length < 1.\n");

      for(unsigned int k = 0; k < 6; k++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          printf("Parsed string has length < 1.\n");
        goal_[k] = atof(sTemp);
      }
      //1: 6dof goal, 0: 3dof goal -> ignored here (assume 1)
      if(fscanf(fCfg,"%s",sTemp) < 1)
        printf("Parsed string has length < 1.\n");
    }
    else if(strcmp(sTemp, "goal_tolerance(meters,radians):") == 0)
    {
      //distance tolerance (m)
      if(fscanf(fCfg,"%s",sTemp) < 1)
        printf("Parsed string has length < 1.\n");
      position_tolerance_  = atof(sTemp);
      //orientation tolerance (rad)
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        printf("Parsed string has length < 1.\n");
      orientation_tolerance_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "cube:") == 0)
    {
      for(int j = 0; j < 6; j++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          printf("Parsed string has length < 1.\n");
        cube[j] = atof(sTemp);
      }
      cubes_.push_back(cube);
    }
    else
      printf("ERROR: Unknown parameter name in environment config file: %s.\n", sTemp);

    if(fscanf(fCfg,"%s",sTemp) < 1) 
      printf("Parsed string has length < 1.\n");
  }

  fclose(fCfg);
  return true;
}

void ArmPlanningTest::printTest(FILE* fout)
{
  fprintf(fout, "goal: xyz: %0.2f %0.2f %0.2f (tol: %0.2fm)   rpy: %0.2f %0.2f %0.2f (tol: %0.2frad)\n", goal_[0],goal_[1],goal_[2],position_tolerance_,goal_[3],goal_[4],goal_[5],orientation_tolerance_);
  fprintf(fout,"Cubes: %d\n", int(cubes_.size()));
  for(size_t i = 0; i < cubes_.size(); i++)
    fprintf(fout, "%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n", cubes_[i][0],cubes_[i][1],cubes_[i][2],cubes_[i][3],cubes_[i][4],cubes_[i][5]);
}

void ArmPlanningTest::sendArmToStart()
{
  ROS_INFO("[sendArmToStart] Sending arm to start configuration and pausing for 2 seconds");
  //arm_.startTrajectory(arm_.setArmConfiguration(start_angles_));
  sleep(3);
  ROS_INFO("[sendArmToStart] Exiting.");
}

bool ArmPlanningTest::callMoveArm()
{
  ROS_INFO("[callMoveArm] calling move arm....");
  actionlib::SimpleActionClient<move_arm_msgs::MoveArmAction> move_arm("move_right_arm",true);
  move_arm.waitForServer();
  ROS_INFO("[callMoveArm] Connected to server");
  move_arm_msgs::MoveArmGoal goalA;

  goalA.motion_plan_request.group_name = "right_arm";
  goalA.motion_plan_request.num_planning_attempts = 1;
  goalA.motion_plan_request.planner_id = std::string("");
  goalA.planner_service_name = std::string(planning_service_name_);
  goalA.motion_plan_request.allowed_planning_time = ros::Duration(max_planning_time_);

  motion_planning_msgs::SimplePoseConstraint desired_pose;
  desired_pose.header.frame_id = goal_frame_;
  desired_pose.link_name = planning_link_;
  desired_pose.pose.position.x = goal_[0];
  desired_pose.pose.position.y = goal_[1];
  desired_pose.pose.position.z = goal_[2];

  btQuaternion gripper_goal;
  geometry_msgs::Quaternion gripper_goal_msg;
  gripper_goal.setRPY(goal_[3],goal_[4],goal_[5]);
  tf::quaternionTFToMsg(gripper_goal,gripper_goal_msg);

  desired_pose.pose.orientation.x = gripper_goal_msg.x;
  desired_pose.pose.orientation.y = gripper_goal_msg.y;
  desired_pose.pose.orientation.z = gripper_goal_msg.z;
  desired_pose.pose.orientation.w = gripper_goal_msg.w;

  desired_pose.absolute_position_tolerance.x = position_tolerance_;
  desired_pose.absolute_position_tolerance.y = position_tolerance_;
  desired_pose.absolute_position_tolerance.z = position_tolerance_;

  desired_pose.absolute_roll_tolerance = orientation_tolerance_;
  desired_pose.absolute_pitch_tolerance = orientation_tolerance_;
  desired_pose.absolute_yaw_tolerance = orientation_tolerance_;

  desired_pose.orientation_constraint_type = motion_planning_msgs::SimplePoseConstraint::HEADER_FRAME;

  //goalA.disable_ik = disable_ik_;
  goalA.disable_collision_monitoring = disable_collision_monitoring_;
  
  move_arm_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

  //goalA.disable_ik = disable_ik_;

  goalA.accept_partial_plans = accept_partial_plans_;

/*
  if (nh_.ok())
    {
    bool finished_within_time = false;
*/
    move_arm.sendGoal(goalA);
/*
    finished_within_time = move_arm.waitForResult(ros::Duration(movearm_duration_));
    if (!finished_within_time)
    {
      move_arm.cancelGoal();
      ROS_INFO("Timed out achieving goal A");
      return false;
    }
    else
    {
      actionlib::SimpleClientGoalState state = move_arm.getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
      {
        ROS_INFO("Action finished: %s",state.toString().c_str());
        return true;
      }
      else
      {
        ROS_INFO("Action failed: %s",state.toString().c_str());
        return false;
      }
    }
  }
*/
  return true;
}

void ArmPlanningTest::addCubesToEnvironment()
{
  addCubesToEnvironment(cubes_);
}

void ArmPlanningTest::addCubesToEnvironment(std::vector<std::vector<double> > &cubes)
{
  ROS_INFO("[addCubesToEnvironment] adding %d boxes to the environment", int(cubes.size()));
  addobj_.addBoxes(cubes);
  
  ROS_INFO("[addCubesToEnvironment] added the boxes to the environment");
}

void ArmPlanningTest::motionPlanCallback(const motion_planning_msgs::DisplayTrajectory &plan)
{
  ROS_INFO("[motionPlanCallback] Plan received.");
  plan_.clear();
  plan_.resize(plan.trajectory.joint_trajectory.points.size());

  if(plan.trajectory.joint_trajectory.points.size() < 6)
  {
    ROS_INFO("[motionPlanCallback] Plan received only has %d waypoints.", int(plan.trajectory.joint_trajectory.points.size()));
    return;
  }

  for(size_t i = 0; i < plan.trajectory.joint_trajectory.points.size(); i++)
  {
    plan_[i].resize(plan.trajectory.joint_trajectory.points[i].positions.size());
    for(size_t j = 0; j < plan.trajectory.joint_trajectory.points[i].positions.size(); j++)
    {
      plan_[i][j] = plan.trajectory.joint_trajectory.points[i].positions[j];
    }
  }

  ROS_INFO("size of plan %d ",  int(plan.trajectory.joint_trajectory.points.size()));
  //visualizePlan();

  time_t init_time = time(NULL);
  std::string str_time(asctime (localtime(&init_time)));
  std::string fname = trajectory_file_path_ + test_file_name_ + "-" + planner_name_ + "-" + str_time.substr(4,20);
  ROS_INFO("[motionPlanCallback] Saving path to %s", fname.c_str());
  FILE* fout = fopen(fname.c_str(), "w");
  printTrajectory(fout);
  fclose(fout);

  ROS_INFO("[motionPlanCallback] The path planned has %d waypoints", int(plan_.size()));
}

void ArmPlanningTest::visualizePlan()
{
  ROS_INFO("[visualizePlan] Visualizing path.");
  //aviz_->visualizeArmConfigurations(plan_, 4);
}

void ArmPlanningTest::printTrajectory(FILE* fout)
{
  ROS_INFO("[printTrajectory] Printing path.");
  for(size_t i = 0; i < plan_.size(); i++)
  {
    if(plan_[i].size() == 7)
      fprintf(fout, "%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", plan_[i][0],plan_[i][1],plan_[i][2],plan_[i][3],plan_[i][4],plan_[i][5],plan_[i][6]);
  }
}

