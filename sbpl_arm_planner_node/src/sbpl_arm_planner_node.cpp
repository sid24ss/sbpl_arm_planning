/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#include <sbpl_arm_planner_node/sbpl_arm_planner_node.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

/** Initializers -------------------------------------------------------------*/
SBPLArmPlannerNode::SBPLArmPlannerNode() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1)
{
  planner_initialized_ = false;
}

SBPLArmPlannerNode::~SBPLArmPlannerNode()
{
  delete aviz_;
  delete planner_;
}

bool SBPLArmPlannerNode::init()
{
  sleep(2);

  //planner
  node_handle_.param ("planner/search_mode", search_mode_, true); //true: stop after first solution
  node_handle_.param ("planner/allocated_time", allocated_time_, 10.0);
  node_handle_.param ("planner/forward_search", forward_search_, true);
  node_handle_.param ("debug/print_out_path", print_path_, true);
  node_handle_.param ("seconds_per_waypoint", waypoint_time_, 0.35);
  node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param ("planner/use_research_heuristic", use_research_heuristic_, false);
  node_handle_.param<std::string>("fk_service_name", fk_service_name_, "pr2_right_arm_kinematics/get_fk");
  node_handle_.param<std::string>("ik_service_name", ik_service_name_, "pr2_right_arm_kinematics/get_ik");
  node_handle_.param<std::string>("planner/arm_description_file", arm_description_filename_, " ");
  node_handle_.param<std::string>("planner/motion_primitive_file", mprims_filename_, " ");

  //robot description
  node_handle_.param<std::string>("robot/arm_name", arm_name_, "right_arm");
  std::string robot_urdf_param;
  if(!node_handle_.searchParam("robot_description",robot_urdf_param))
  {
    ROS_ERROR("Unable to find robot description on param server (/robot_description is not set). Exiting");
    return false;
  }
  node_handle_.param<std::string>(robot_urdf_param, robot_description_, "robot_description");
  node_handle_.param<std::string>("robot/planning_joint", planning_joint_, "r_wrist_roll_link");
  node_handle_.param ("robot/num_joints", num_joints_, 7);
  
  joint_names_.resize(num_joints_);
  if(arm_name_ == "left_arm")
  {
    side_ = "l";
    planning_joint_ = "l_wrist_roll_link";
  }
  else
  {
    side_ = "r";
    planning_joint_ = "r_wrist_roll_link";
  }

  //pr2 specific
  joint_names_[0] = side_ + "_shoulder_pan_joint";
  joint_names_[1] = side_ + "_shoulder_lift_joint";
  joint_names_[2] = side_ + "_upper_arm_roll_joint";
  joint_names_[3] = side_ + "_elbow_flex_joint";
  joint_names_[4] = side_ + "_forearm_roll_joint";
  joint_names_[5] = side_ + "_wrist_flex_joint";
  joint_names_[6] = side_ + "_wrist_roll_joint";
  
  //collision space
  node_handle_.param<std::string>("collision_space/collision_map_topic", collision_map_topic_, "collision_map_occ");

  //visualizations
  node_handle_.param ("visualizations/goal", visualize_goal_, true);
  node_handle_.param ("visualizations/expanded_states",visualize_expanded_states_,true);
  node_handle_.param ("visualizations/heuristic", visualize_heuristic_, false);
  node_handle_.param ("visualizations/voxel_size", env_resolution_, 0.01);
  node_handle_.param ("visualizations/trajectory", visualize_trajectory_, false);
  node_handle_.param ("visualizations/collision_model_trajectory", visualize_collision_model_trajectory_, false);
  node_handle_.param ("visualizations/trajectory_throttle", throttle_, 4);

  //initialize planner
  planner_ = new ARAPlanner(&sbpl_arm_env_, forward_search_);
  if(!initializePlannerAndEnvironment())
    return false;

  collision_map_filter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&SBPLArmPlannerNode::collisionMapCallback, this, _1));

  collision_object_subscriber_ = root_handle_.subscribe("collision_object", 1, &SBPLArmPlannerNode::collisionObjectCallback, this);

  // main planning service
  planning_service_ = root_handle_.advertiseService("/sbpl_planning/plan_path", &SBPLArmPlannerNode::planKinematicPath,this);
  object_subscriber_ = root_handle_.subscribe("attached_collision_object", 1, &SBPLArmPlannerNode::attachedObjectCallback,this);
  planner_initialized_ = true;

  ROS_INFO("The SBPL arm planner node initialized succesfully.");
  return true;
}

int SBPLArmPlannerNode::run()
{
  ros::spin();
  return 0;
}

bool SBPLArmPlannerNode::initializePlannerAndEnvironment()
{
  planner_ = new ARAPlanner(&sbpl_arm_env_, forward_search_);

  if(robot_description_.empty())
  {
    ROS_ERROR("Robot description file is empty. Exiting.");
    return false;
  }

  //initialize arm planner environment
  if(!sbpl_arm_env_.initEnvironment(arm_description_filename_,mprims_filename_))
  {
    ROS_ERROR("ERROR: initEnvironment failed");
    return false;
  }

  //initialize MDP 
  if(!sbpl_arm_env_.InitializeMDPCfg(&mdp_cfg_))
  {
    ROS_ERROR("ERROR: InitializeMDPCfg failed");
    return false;
  }

  cspace_ = sbpl_arm_env_.getCollisionSpace();

  //sad excuse for self-collision checking
  cspace_->addArmCuboidsToGrid();

  grid_ = sbpl_arm_env_.getOccupancyGrid();

  //set epsilon
  planner_->set_initialsolution_eps(sbpl_arm_env_.getEpsilon());

  //set search mode (true - settle with first solution)
  search_mode_ = false;
  planner_->set_search_mode(search_mode_);

  if(!initChain(robot_description_))
  {
    ROS_ERROR("Unable to initialize KDL chain.");
    return false;
  }

  aviz_ = new VisualizeArm();
  aviz_->setReferenceFrame(reference_frame_);

  ROS_INFO("Initialized sbpl planning environment.");
  return true;
}

/** Callbacks ----------------------------------------------------------------*/
void SBPLArmPlannerNode::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
  updateMapFromCollisionMap(collision_map);
}

void SBPLArmPlannerNode::updateMapFromCollisionMap(const mapping_msgs::CollisionMapConstPtr &collision_map)
{
  ROS_DEBUG("[updateMapFromCollisionMap] trying to get colmap_mutex_ mutex");
  if(colmap_mutex_.try_lock())
  {
    ROS_DEBUG("[updateMapFromCollisionMap] locked colmap_mutex_ mutex.");

    if(collision_map->header.frame_id.compare(reference_frame_) != 0)
    {
      ROS_WARN("collision_map_occ is in %s not in %s", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
      ROS_DEBUG("the map has %i cubic obstacles", int(collision_map->boxes.size()));
    }

    // add collision map msg
    grid_->updateFromCollisionMap(*collision_map);

    // add self collision blocks
    //cspace_->addArmCuboidsToGrid();
    
    colmap_mutex_.unlock();
    ROS_DEBUG("[updateMapFromCollisionMap] released colmap_mutex_ mutex.");

    grid_->visualize();
    return;
  }
  else
  {
    ROS_DEBUG("[updateMapFromCollisionMap] failed trying to get colmap_mutex_ mutex");
    return;
  }
}

void SBPLArmPlannerNode::attachedObjectCallback(const mapping_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  bool gripper_object = false;

  // is one of the objects attached to the gripper of the arm we are planning for
  for(size_t i=0; i <attached_object->touch_links.size(); i++)
  {
    if(attached_object->touch_links[i].find(side_ + "_gripper") != string::npos)
      gripper_object = true;
  }

  if (!gripper_object)
  {
    ROS_WARN("AttachedCollisionObjects that don't describe objects in the gripper are currently not supported.");
    return;
  }

  if(object_mutex_.try_lock())
  {
    // remove all objects
    if(attached_object->link_name.compare(mapping_msgs::AttachedCollisionObject::REMOVE_ALL_ATTACHED_OBJECTS) == 0)
    {
      ROS_INFO("Removing all attached objects.");
      object_map_.clear();
      //TODO: Clear objects in collision space
    }

    // add object
    if(attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::ADD ||
        attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
    {
      object_map_[attached_object->link_name] = attached_object->object;

      ROS_INFO("Received a message with %d attached objects.", int(attached_object->object.shapes.size()));

      for(size_t i = 0; i < attached_object->object.shapes.size(); i++)
      {
        if(attached_object->object.shapes[i].type == geometric_shapes_msgs::Shape::SPHERE)
        {
          ROS_INFO("Attaching a sphere with radius: %0.3fm", attached_object->object.shapes[i].dimensions[0]);
          cspace_->attachSphereToGripper(attached_object->object.header.frame_id, attached_object->object.poses[i], attached_object->object.shapes[i].dimensions[0]);
        }
        else if(attached_object->object.shapes[i].type == geometric_shapes_msgs::Shape::CYLINDER)
        {
          ROS_INFO("Attaching a cylinder with radius: %0.3fm & length %0.3fm", attached_object->object.shapes[i].dimensions[0], attached_object->object.shapes[i].dimensions[1]);

          cspace_->attachCylinderToGripper(attached_object->object.header.frame_id, attached_object->object.poses[i], attached_object->object.shapes[i].dimensions[0], attached_object->object.shapes[i].dimensions[1]);
        }
        else if(attached_object->object.shapes[i].type == geometric_shapes_msgs::Shape::MESH)
        {
          ROS_INFO("Attaching a mesh with %d triangles  & %d vertices.", int(attached_object->object.shapes[i].triangles.size()/3), int(attached_object->object.shapes[i].vertices.size()));

          cspace_->attachMeshToGripper(attached_object->object.header.frame_id, attached_object->object.poses[i], attached_object->object.shapes[i].triangles,attached_object->object.shapes[i].vertices);
        }

      }
    }

    // remove object
    else if(attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE ||
        attached_object->object.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
    {
      ROS_INFO("Removing object: %s", attached_object->link_name.c_str());
      object_map_.erase(attached_object->link_name);
    }
    else
      ROS_WARN("Received a collision object with an unknown operation");

    object_mutex_.unlock();
  }
}

void SBPLArmPlannerNode::collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collision_object)
{
  std::vector<double> cube(6,0);

  if(collision_object->header.frame_id != reference_frame_)
    ROS_WARN("collision object is in %s, only %s supported right now.",collision_object->header.frame_id.c_str(),reference_frame_.c_str());

  if(collision_object->operation.operation == mapping_msgs::CollisionObjectOperation::ADD)
  {
    for(size_t i = 0; i < collision_object->shapes.size(); i++)
    {
      //only support cubes right now
      if(collision_object->shapes[i].type == geometric_shapes_msgs::Shape::BOX)
      {
        cube[0] = collision_object->poses[i].position.x;
        cube[1] = collision_object->poses[i].position.y;
        cube[2] = collision_object->poses[i].position.z;
        cube[3] = collision_object->shapes[i].dimensions[0];
        cube[4] = collision_object->shapes[i].dimensions[1];
        cube[5] = collision_object->shapes[i].dimensions[2];

        //only support orientation of {0,0,0,1}
        if(collision_object->poses[i].orientation.x != 0 || collision_object->poses[i].orientation.y != 0 ||
          collision_object->poses[i].orientation.z != 0 || collision_object->poses[i].orientation.w != 1)
        {
          ROS_WARN("[collisionObjectCallback] Warning: collision object does not have an orientation of {0,0,0,1}");
        }
      }
      col_objects_.push_back(cube);
    }
  }

  ROS_INFO("%s with %d shapes has been added to the occupancy grid.",collision_object->id.c_str(), int(collision_object->shapes.size()));
}

/** Planner Interface  -------------------------------------------------------*/
bool SBPLArmPlannerNode::setStart(const sensor_msgs::JointState &start_state)
{
  std::vector<double> sbpl_start(start_state.position.size(),0);

  for(unsigned int i=0; i< start_state.position.size(); i++)
    sbpl_start[i] = (double)(start_state.position[i]);

  std::vector<std::vector<double> > xyz;
  cspace_->getAttachedObject(sbpl_start, xyz);
  aviz_->visualizeSpheres(xyz, 189, "sbpl_attached_object_start", cspace_->getAttachedObjectRadius());

  ROS_INFO("start: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", sbpl_start[0],sbpl_start[1],sbpl_start[2],sbpl_start[3],sbpl_start[4],sbpl_start[5],sbpl_start[6]);

  if(sbpl_arm_env_.setStartConfiguration(sbpl_start) == 0)
  {
    ROS_ERROR("Environment failed to set start state. Not Planning.\n");
    return false;
  }

  if(planner_->set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR("Failed to set start state. Not Planning.");
    return false;
  }

  return true;
}

bool SBPLArmPlannerNode::setGoalPosition(const motion_planning_msgs::Constraints &goals)
{
  double roll,pitch,yaw;
  geometry_msgs::Pose pose_msg;
  tf::Pose tf_pose;
  std::vector <std::vector <double> > sbpl_goal(1, std::vector<double> (7,0));
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  if(goals.position_constraints.size() != goals.orientation_constraints.size())
    ROS_WARN("There are %d position contraints and %d orientation constraints.", int(goals.position_constraints.size()),int(goals.orientation_constraints.size()));

  //currently only supports one goal
  sbpl_goal[0][0] = goals.position_constraints[0].position.x;
  sbpl_goal[0][1] = goals.position_constraints[0].position.y;
  sbpl_goal[0][2] = goals.position_constraints[0].position.z;

  //convert quaternion into roll,pitch,yaw
  pose_msg.position = goals.position_constraints[0].position;
  pose_msg.orientation = goals.orientation_constraints[0].orientation;

  tf::poseMsgToTF(pose_msg, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  sbpl_goal[0][3] = roll;
  sbpl_goal[0][4] = pitch;
  sbpl_goal[0][5] = yaw;

  //6dof goal: true, 3dof: false 
  sbpl_goal[0][6] = true;

  //allowable tolerance from goal
  sbpl_tolerance[0][0] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][2] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][3] = goals.orientation_constraints[0].absolute_roll_tolerance;
  sbpl_tolerance[0][4] = goals.orientation_constraints[0].absolute_pitch_tolerance;
  sbpl_tolerance[0][5] = goals.orientation_constraints[0].absolute_yaw_tolerance;

  ROS_INFO("goal xyz: %.2f %.2f %.2f (tol: %.3fm) rpy: %.2f %.2f %.2f (tol: %.3frad) in the %s frame", sbpl_goal[0][0],sbpl_goal[0][1],sbpl_goal[0][2],sbpl_tolerance[0][0],sbpl_goal[0][3],sbpl_goal[0][4],sbpl_goal[0][5], sbpl_tolerance[0][1], reference_frame_.c_str());

  //set sbpl environment goal
  if(!sbpl_arm_env_.setGoalPosition(sbpl_goal, sbpl_tolerance))
  {
    ROS_ERROR("Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
    return false;
  }

  //set planner goal	
  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("Failed to set goal state. Exiting.");
    return false;
  }

  return true;
}

bool SBPLArmPlannerNode::planToPosition(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
  unsigned int i;
  int nind = 0;
  std::vector<trajectory_msgs::JointTrajectoryPoint> arm_path;
  sensor_msgs::JointState start;
  tf::Stamped<tf::Pose> pose_stamped;

  starttime = clock();

  //check for an empty start state
  if(req.motion_plan_request.start_state.joint_state.position.size() <= 0)
  {
    ROS_ERROR("No start state given. Unable to plan.");
    return false;
  }

  //check if goal constraint is empty
  if(req.motion_plan_request.goal_constraints.position_constraints.size() <= 0 || 
      req.motion_plan_request.goal_constraints.orientation_constraints.size() <= 0)
  {
    ROS_ERROR("Position constraint or orientation constraint is empty. Unable to plan.");
    return false;
  }

  // add collision objects to occupancy grid 9/2/2010
  for(size_t i = 0; i < col_objects_.size(); i++)
  {
    grid_->addCollisionCuboid(col_objects_[i][0],col_objects_[i][1],col_objects_[i][2],col_objects_[i][3],col_objects_[i][4],col_objects_[i][5]);
  }

  //transform goal pose into reference_frame_
  for(i = 0; i < req.motion_plan_request.goal_constraints.position_constraints.size(); i++)
  {
    planning_joint_ = req.motion_plan_request.goal_constraints.position_constraints[i].link_name;

    if(planning_joint_ != "r_wrist_roll_link" && arm_name_ == "right_arm")
    {
      ROS_ERROR("Planner is configured to plan for the right arm. It has only been tested with pose constraints for r_wrist_roll_link");
      return false;
    }
    else if(planning_joint_ != "l_wrist_roll_link" && arm_name_ == "left_arm")
    {
      ROS_ERROR("Planner is configured to plan for the left arm. It has only been tested with pose constraints for l_wrist_roll_link. Exiting");
      return false;
    }

    ROS_DEBUG("[planToPosition] position constraint %d: xyz: %0.3f %0.3f %0.3f in %s", i, req.motion_plan_request.goal_constraints.position_constraints[i].position.x, req.motion_plan_request.goal_constraints.position_constraints[i].position.y, req.motion_plan_request.goal_constraints.position_constraints[i].position.z, req.motion_plan_request.goal_constraints.position_constraints[i].header.frame_id.c_str());

    try
    {
      tf_.lookupTransform(reference_frame_, req.motion_plan_request.goal_constraints.position_constraints[i].header.frame_id, ros::Time(0), transform_);

      ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)",reference_frame_.c_str(),req.motion_plan_request.goal_constraints.position_constraints[i].header.frame_id.c_str(), transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return false;
    }

    // cache transform for later
    setTransform();
    
    // use transform to transform goal into reference frame (should switch to tf_.transformPose() in future)
    geometry_msgs::PoseStamped goal_pose;
    KDL::Frame goal_frame;
    goal_pose.header = req.motion_plan_request.goal_constraints.position_constraints[i].header;
    goal_pose.pose.position = req.motion_plan_request.goal_constraints.position_constraints[i].position;
    goal_pose.pose.orientation = req.motion_plan_request.goal_constraints.orientation_constraints[i].orientation;
    tf::PoseMsgToKDL(goal_pose.pose,goal_frame);
    goal_frame  = kdl_transform_ * goal_frame;
    tf::PoseKDLToMsg(goal_frame, goal_pose.pose);
    req.motion_plan_request.goal_constraints.position_constraints[i].position = goal_pose.pose.position;
    req.motion_plan_request.goal_constraints.orientation_constraints[i].orientation = goal_pose.pose.orientation;
  }

  // get the initial state of the planning joints
  start.position.resize(num_joints_);
  for(i = 0; i < req.motion_plan_request.start_state.joint_state.position.size(); i++)
  {
    ROS_DEBUG("%s: %.3f",req.motion_plan_request.start_state.joint_state.name[i].c_str(),req.motion_plan_request.start_state.joint_state.position[i]);
    if(joint_names_[nind].compare(req.motion_plan_request.start_state.joint_state.name[i]) == 0)
    {
      start.position[nind] = req.motion_plan_request.start_state.joint_state.position[i];
      nind++;
    }
    if(nind == num_joints_)
      break;
  }
  if(nind != num_joints_)
    ROS_WARN("Not all of the joints in the arm were assigned a starting position.");

  allocated_time_ = req.motion_plan_request.allowed_planning_time.toSec();

  colmap_mutex_.lock();
  object_mutex_.lock();

  ROS_DEBUG("[planToPosition] About to set start configuration");
  if(setStart(start))
  {
    ROS_DEBUG("[planToPosition] successfully set starting configuration");

    if(visualize_goal_)
      visualizeGoalPosition(req.motion_plan_request.goal_constraints);

    if(setGoalPosition(req.motion_plan_request.goal_constraints))
    {
      ROS_DEBUG("Calling planner");

      if(plan(arm_path))
      {
        colmap_mutex_.unlock();
        object_mutex_.unlock();

        res.trajectory.joint_trajectory.points.resize(arm_path.size());
        res.trajectory.joint_trajectory.points = arm_path;

        // fill in the waypoint times (not scaled as of now)
        res.trajectory.joint_trajectory.points[0].time_from_start.fromSec(waypoint_time_);
        for(i = 1; i < res.trajectory.joint_trajectory.points.size(); i++)
          res.trajectory.joint_trajectory.points[i].time_from_start.fromSec(res.trajectory.joint_trajectory.points[i-1].time_from_start.toSec() + waypoint_time_);

        res.trajectory.joint_trajectory.header.seq = req.motion_plan_request.goal_constraints.position_constraints[0].header.seq; 
        res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

        if(!req.motion_plan_request.start_state.joint_state.header.frame_id.empty())
          res.trajectory.joint_trajectory.header.frame_id = req.motion_plan_request.start_state.joint_state.header.frame_id;
        else
          res.trajectory.joint_trajectory.header.frame_id = reference_frame_;
        ROS_DEBUG("path frame is %s",res.trajectory.joint_trajectory.header.frame_id.c_str());

        // fill in the joint names 
        res.trajectory.joint_trajectory.joint_names.resize(num_joints_);
        for(i = 0; i < (unsigned int)num_joints_; i++)
          res.trajectory.joint_trajectory.joint_names[i] = joint_names_[i];
  
        ROS_INFO("Planner completed in %lf seconds. Planned trajectory has %d waypoints.",(clock() - starttime) / (double)CLOCKS_PER_SEC, int(res.trajectory.joint_trajectory.points.size()));

        if(print_path_)
          printPath(res.trajectory.joint_trajectory.points);

        // visualizations
        if(visualize_expanded_states_)
          displayARAStarStates();

        //if(visualize_heuristic_)
        //  displayShortestPath();

        if(visualize_trajectory_)
        {
          ROS_INFO("[plan] Visualizing the trajectory (throttle = %d)", throttle_);
          aviz_->visualizeJointTrajectoryMsg(res.trajectory.joint_trajectory, throttle_);

          ROS_INFO("[plan] Visualizing the attached object (throttle = %d)", throttle_);
          //TODO: add in check to see if we have an attached object...
          visualizeAttachedObject(res.trajectory.joint_trajectory, throttle_);
        }

        //if(visualize_collision_model_trajectory_)
        //  aviz_->visualizeCollisionModelFromJointTrajectoryMsg(cspace_, res.trajectory.joint_trajectory);

        if(use_research_heuristic_)
          visualizeElbowPoses();

        return true;
      }
      else
      {
        ROS_ERROR("Failed to plan within alotted time frame (%0.2f seconds).", allocated_time_);
      }
    }
    else
    {
      ROS_ERROR("Failed to set goal pose.");
    }
  }
  else
  {
    ROS_ERROR("Failed to set start configuration.");
  }

  colmap_mutex_.unlock();
  object_mutex_.unlock();

  if(visualize_expanded_states_)
    displayARAStarStates();
  
  if(use_research_heuristic_)
    visualizeElbowPoses();

  return false;
}

bool SBPLArmPlannerNode::planKinematicPath(motion_planning_msgs::GetMotionPlan::Request &req, motion_planning_msgs::GetMotionPlan::Response &res)
{
  if(!planner_initialized_)
  {
    ROS_ERROR("Hold up a second...the planner isn't initialized yet. Try again in a second or two.");
    return false;
  }

  if(req.motion_plan_request.goal_constraints.position_constraints.empty())
  {
    ROS_ERROR("There are no goal pose constraints in the request message. We need those to plan :).");
    return false;
  }
  if(!planToPosition(req, res))
    return false;

  return true;
}

bool SBPLArmPlannerNode::plan(std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path)
{
  bool b_ret(false);
  int solution_cost;
  std::vector<double>angles(num_joints_,0);
  vector<int> solution_state_ids_v;

  //reinitialize the search space
  planner_->force_planning_from_scratch();

  //plan
  b_ret = planner_->replan(allocated_time_, &solution_state_ids_v, &solution_cost);

  //check if an empty plan was received.
  if(b_ret && solution_state_ids_v.size() <= 0)
    b_ret = false;

  if(visualize_heuristic_ && b_ret)
    displayShortestPath();

  //outputs debug information about the adaptive mprims
  sbpl_arm_env_.debugAdaptiveMotionPrims();

  // if a path is returned, then pack it into msg form
  if(b_ret && (solution_state_ids_v.size() > 0))
  {
    unsigned int i;
    ROS_DEBUG("[plan] A path was returned with %d waypoints.", int(solution_state_ids_v.size()));
    ROS_INFO("Initial Epsilon: %0.3f   Final Epsilon: %0.3f", planner_->get_initial_eps(),planner_->get_final_epsilon());
    arm_path.resize(solution_state_ids_v.size()+1);
    for(i=0; i < solution_state_ids_v.size(); i++)
    {       
      arm_path[i].positions.resize(num_joints_);
      sbpl_arm_env_.StateID2Angles(solution_state_ids_v[i], angles);

      //Check the length of 'angles'. If it's 14 then break into 2 joint configurations and concatenate them to the path.
      if(angles.size()==14)
      {
        arm_path[i+1].positions.resize(num_joints_);
        std::vector<double>test_angles(7,0);
        for (int p = 0; p < num_joints_; ++p)
        {
          arm_path[i].positions[p] = angles::normalize_angle(angles[p]);
          test_angles[p] = angles::normalize_angle(angles[p+7]);
          arm_path[i+1].positions[p] = angles::normalize_angle(angles[p+7]);
        }
      }
      else
      {
        for (int p = 0; p < num_joints_; ++p)
          arm_path[i].positions[p] = angles::normalize_angle(angles[p]);
      }
      ROS_DEBUG("%i: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", i, arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6]);
    }
  }
  return b_ret;
}

/* Kinematics ----------------------------------------------------------------*/
bool SBPLArmPlannerNode::computeIK(const geometry_msgs::Pose &pose, std::vector<double> jnt_pos, std::vector<double> &solution)
{
  kinematics_msgs::GetPositionIK::Request request;
  kinematics_msgs::GetPositionIK::Response response;

  request.ik_request.ik_link_name = planning_joint_;

  request.ik_request.pose_stamped.pose = pose;
  request.ik_request.pose_stamped.header.stamp = ros::Time();
  request.ik_request.pose_stamped.header.frame_id = reference_frame_;

  request.ik_request.ik_seed_state.joint_state.header.stamp = ros::Time();
  request.ik_request.ik_seed_state.joint_state.header.frame_id = reference_frame_;
  request.ik_request.ik_seed_state.joint_state.name = joint_names_;
  request.ik_request.ik_seed_state.joint_state.position.clear();

  for(int j = 0 ; j < num_joints_; ++j)
    request.ik_request.ik_seed_state.joint_state.position.push_back(jnt_pos[j]);

  ros::service::waitForService(ik_service_name_);
  ros::ServiceClient client = root_handle_.serviceClient<kinematics_msgs::GetPositionIK>(ik_service_name_, true);

  if(client.call(request, response))
  {
    ROS_DEBUG("Obtained IK solution");
    if(response.error_code.val == response.error_code.SUCCESS)
      for(unsigned int i=0; i < response.solution.joint_state.name.size(); i ++)
      {
        solution[i] = response.solution.joint_state.position[i];
        ROS_INFO("Joint: %s %f",response.solution.joint_state.name[i].c_str(),response.solution.joint_state.position[i]);
      }
    else
    {
      ROS_ERROR("Inverse kinematics failed");
      return false;
    }

    ROS_DEBUG("IK Solution");
    for(unsigned int i = 0; i < solution.size() ; ++i)
      ROS_DEBUG("%i: %f", i, solution[i]);
  }
  else
  {
    ROS_ERROR("IK service failed");
    return false;
  }
  return true;
}

bool SBPLArmPlannerNode::computeFK(const std::vector<double> &jnt_pos, geometry_msgs::Pose &pose)
{
  kinematics_msgs::GetPositionFK::Request  request;
  kinematics_msgs::GetPositionFK::Response response;

  request.header.stamp = ros::Time();
  request.header.frame_id = reference_frame_;

  request.robot_state.joint_state.name = joint_names_;

  for(unsigned int j = 0 ; j < jnt_pos.size(); ++j)
    request.robot_state.joint_state.position.push_back(jnt_pos[j]);

  request.fk_link_names.resize(1);
  request.fk_link_names[0] = planning_joint_;

  ROS_DEBUG("waiting for %s service", fk_service_name_.c_str());
  ros::service::waitForService(fk_service_name_);
  ros::ServiceClient client = root_handle_.serviceClient<kinematics_msgs::GetPositionFK>(fk_service_name_);

  if(client.call(request, response))
  {
    if(response.error_code.val == response.error_code.SUCCESS)
    {
      pose = response.pose_stamped[0].pose;
      return true;
    }
    else
      return false;
  }
  else
  {
    ROS_ERROR("FK service failed");
    return false;
  }
}

void SBPLArmPlannerNode::computeFKwithKDL(const std::vector<double> &jnt_pos, geometry_msgs::Pose &pose, int joint_num)
{
  KDL::JntArray jnt_array;
  KDL::Frame frame_out;

  jnt_array.resize(arm_chain_.getNrOfJoints());

  for(int i = 0; i < num_joints_; ++i)
    jnt_array(i+1)=jnt_pos[i];

  if(jnt_to_pose_solver_->JntToCart(jnt_array, frame_out, joint_num) < 0)
  {
    ROS_ERROR("JntToCart returned < 0. Exiting.\n");
    return;
  }

  pose.position.x = frame_out.p[0];
  pose.position.y = frame_out.p[1];
  pose.position.z = frame_out.p[2];
}

bool SBPLArmPlannerNode::initChain(std::string robot_description)
{
  KDL::Tree my_tree;

  if (!kdl_parser::treeFromString(robot_description, my_tree))
  {
    ROS_ERROR("Failed to parse tree from manipulator description file.\n");
    return false;;
  }

  if (!my_tree.getChain(reference_frame_, planning_joint_, arm_chain_))
  {
    ROS_ERROR("Could not fetch the KDL chain for the desired manipulator. Exiting.\n"); 
    return false;
  }

  jnt_to_pose_solver_ = new KDL::ChainFkSolverPos_recursive(arm_chain_);

  ROS_DEBUG("[initChain] arm_chain has %d segments and %d joints", arm_chain_.getNrOfSegments(), arm_chain_.getNrOfJoints());

  return true;
}

void SBPLArmPlannerNode::setTransform()
{
  tf::TransformTFToKDL(transform_,kdl_transform_);
  sbpl_arm_env_.setReferenceFrameTransform(kdl_transform_);
}

/* Visualizations ------------------------------------------------------------*/
void SBPLArmPlannerNode::displayARAStarStates()
{
  std::vector<std::vector<double> > expanded_states;
  std::vector<double> expanded_color(4,1);
  expanded_color[0] = 0.5;
  expanded_color[2] = 0;

  sbpl_arm_env_.getExpandedStates(&(expanded_states));

  if(!expanded_states.empty())
  {
    std::vector<std::vector<double> > detailed_color(2);
    detailed_color[0].resize(4,0);
    detailed_color[0][0] = 1;
    detailed_color[0][1] = 0;
    detailed_color[0][2] = 0;
    detailed_color[0][3] = 1;

    detailed_color[1].resize(4,0);
    detailed_color[1][0] = 0;
    detailed_color[1][1] = 1;
    detailed_color[1][2] = 0;
    detailed_color[1][3] = 1;

    aviz_->visualizeDetailedStates(expanded_states, detailed_color,"expanded",0.01);
  }

  ROS_INFO("[displayARAStarStates] displaying %d expanded states.\n",int(expanded_states.size()));
}

void SBPLArmPlannerNode::visualizeGoalPosition(const motion_planning_msgs::Constraints &goal_pose)
{
  geometry_msgs::Pose pose;
  pose.position = goal_pose.position_constraints[0].position;
  pose.orientation = goal_pose.orientation_constraints[0].orientation;
  aviz_->visualizePose(pose, "goal_pose");
  ROS_DEBUG("[visualizeGoalPosition] publishing goal marker visualizations.");
}

void SBPLArmPlannerNode::visualizeElbowPoses()
{
  std::vector<std::vector<double> > elbow_poses;
  sbpl_arm_env_.getElbowPoints(elbow_poses);

  for(size_t i = 0; i < elbow_poses.size(); i++)
  {
    aviz_->visualizeSphere(elbow_poses[i], 100, "elbow_poses" + boost::lexical_cast<std::string>(i), 0.02);
  }
  ROS_INFO("[visualizeElbowPoses] Visualizing %d elbow poses.", int(elbow_poses.size()));
}

void SBPLArmPlannerNode::visualizeAttachedObject(trajectory_msgs::JointTrajectory &traj_msg, int throttle)
{
  std::vector<double> angles(7,0);
  std::vector<std::vector<double> > xyz;
  
  if(traj_msg.points.empty())
  {
    ROS_WARN("Trajectory message is empty. Not visualizing anything.");
    return;
  }

  for(size_t i = 0; i < traj_msg.points.size(); i++)
  {
    angles = traj_msg.points[i].positions;

    if(!cspace_->getAttachedObject(angles, xyz))
      continue;
   
    //ROS_INFO("%d: Visualizing %d points",int(i),int(xyz.size()));
    aviz_->visualizeSpheres(xyz, 10*(i+1), "sbpl_attached_object_" + boost::lexical_cast<std::string>(i), cspace_->getAttachedObjectRadius());
  }

  ROS_INFO("IGNORING THROTTLE");
}

void SBPLArmPlannerNode::displayShortestPath()
{
  dpath_ = sbpl_arm_env_.getShortestPath();

  //check if the list is empty
  if(dpath_.empty())
  {
    ROS_DEBUG("The heuristic path has a length of 0");
    return;
  }
  else
    ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.",int(dpath_.size()));
 
  aviz_->visualizeSpheres(dpath_, 80, "heuristic", env_resolution_);
}

void SBPLArmPlannerNode::printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path)
{
  geometry_msgs::Pose pose;
  std::vector<double> jnt_pos(num_joints_,0);

  ROS_INFO("Path:");
  for(unsigned int i = 0; i < arm_path.size(); i++)
  {
    for(int j = 0; j < num_joints_; ++j)
      jnt_pos[j] = arm_path[i].positions[j];

    computeFK(jnt_pos, pose);
    ROS_INFO("%3d: %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f      xyz: %2.3f %2.3f %2.3f", i,arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6],pose.position.x, pose.position.y, pose.position.z);
  }
}

void SBPLArmPlannerNode::printPath(FILE* fOut, const std::vector<std::vector<double> > path)
{
  time_t init_time;
  time(&init_time);
  std::string str_time(asctime (localtime(&init_time)));

  fprintf(fOut, "%s", str_time.c_str());
  for(unsigned int i = 0; i < path.size(); i++)
    fprintf(fOut, "state %3d: %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",i,path[i][0],path[i][1],path[i][2],path[i][3],path[i][4],path[i][5],path[i][6]);
  fprintf(fOut,"---------------------------------");
}

/* Node
 * ---------------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sbpl_arm_planner");
  SBPLArmPlannerNode arm_planner;
  if(!arm_planner.init())
  {
    ROS_ERROR("Failed to initialize arm planner node. Exiting.");
    return 0;
  }

  return arm_planner.run();
}


