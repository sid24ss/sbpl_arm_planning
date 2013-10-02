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
#include <algorithm>
#include <math.h>

clock_t starttime;

using namespace std;
using namespace sbpl_arm_planner;

/** Initializers -------------------------------------------------------------*/
SBPLArmPlannerNode::SBPLArmPlannerNode() : node_handle_("~"),collision_map_subscriber_(root_handle_,"collision_map_occ",1), collision_map_filter_(NULL),jnt_to_pose_solver_(NULL),grid_(NULL),aviz_(NULL)
{
  planner_initialized_ = false;
  attached_object_ = false;
  forward_search_ = true;
  planning_joint_ = "r_wrist_roll_link";
  attached_object_frame_ = "r_gripper_r_finger_tip_link";
  allocated_time_ = 10.0;
  env_resolution_ = 0.02;
  cspace_ = NULL;
  map_frame_ = "base_link";

  stats_.resize(9);
  stats_field_names_.resize(9);
  stats_field_names_[0] = "initial solution planning time";
  stats_field_names_[1] = "initial epsilon";
  stats_field_names_[2] = "initial solution expansions";
  stats_field_names_[3] = "final epsilon planning time";
  stats_field_names_[4] = "final epsilon";
  stats_field_names_[5] = "solution epsilon";
  stats_field_names_[6] = "expansions";
  stats_field_names_[7] = "solution cost";
  stats_field_names_[8] = "path length";
}

SBPLArmPlannerNode::~SBPLArmPlannerNode()
{
  if(aviz_ != NULL)
    delete aviz_;
  if(collision_map_filter_ != NULL)
    delete collision_map_filter_;
  if(jnt_to_pose_solver_ != NULL)
    delete jnt_to_pose_solver_;
  if(planner_ != NULL)
    delete planner_;
}

bool SBPLArmPlannerNode::init()
{
  //planner
  node_handle_.param ("planner/search_mode", search_mode_, true); //true: stop after first solution
  node_handle_.param ("planner/use_research_heuristic", use_research_heuristic_, false);
  node_handle_.param<std::string>("planner/arm_description_file", arm_description_filename_, "");
  node_handle_.param<std::string>("planner/motion_primitive_file", mprims_filename_, "");
  node_handle_.param ("debug/print_out_path", print_path_, true);
  node_handle_.param ("seconds_per_waypoint", waypoint_time_, 0.35);
  node_handle_.param<std::string>("reference_frame", reference_frame_, std::string("base_link"));
  node_handle_.param<std::string>("fk_service_name", fk_service_name_, "pr2_right_arm_kinematics/get_fk");
  node_handle_.param<std::string>("ik_service_name", ik_service_name_, "pr2_right_arm_kinematics/get_ik");

  //robot description
  node_handle_.param<std::string>("robot/arm_name", arm_name_, "right_arm");
  std::string robot_urdf_param;
  if(!node_handle_.searchParam("robot_description",robot_urdf_param))
  {
    ROS_ERROR("Unable to find robot description on param server (/robot_description is not set). Exiting");
    return false;
  }
  node_handle_.param<std::string>(robot_urdf_param, robot_description_, "robot_description");
  node_handle_.param ("robot/num_joints", num_joints_, 7);
  
  joint_names_.resize(num_joints_);
  if(arm_name_ == "left_arm")
  {
    ROS_INFO("Planning for the left arm.");
    side_ = "l";
    planning_joint_ = "l_wrist_roll_link";
    attached_object_frame_ = "l_gripper_r_finger_tip_link";
  }
  else
  {
    ROS_INFO("Planning for the right arm.");
    side_ = "r";
    planning_joint_ = "r_wrist_roll_link";
    attached_object_frame_ = "r_gripper_r_finger_tip_link";
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
  node_handle_.param ("visualizations/heuristic", visualize_heuristic_, true);
  node_handle_.param ("visualizations/voxel_size", env_resolution_, 0.02);
  node_handle_.param ("visualizations/trajectory", visualize_trajectory_, false);
  node_handle_.param ("visualizations/collision_model_trajectory", visualize_collision_model_trajectory_, false);
  node_handle_.param ("visualizations/trajectory_throttle", throttle_, 4);

  //initialize planner
  if(!initializePlannerAndEnvironment())
    return false;

  collision_map_filter_ = new tf::MessageFilter<arm_navigation_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
  collision_map_filter_->registerCallback(boost::bind(&SBPLArmPlannerNode::collisionMapCallback, this, _1));

  collision_object_subscriber_ = root_handle_.subscribe("collision_object", 3, &SBPLArmPlannerNode::collisionObjectCallback, this);
  object_subscriber_ = root_handle_.subscribe("attached_collision_object", 3, &SBPLArmPlannerNode::attachedObjectCallback,this);

  joint_states_subscriber_ = root_handle_.subscribe("joint_states", 1, &SBPLArmPlannerNode::jointStatesCallback,this);

  // main planning service
  planning_service_ = root_handle_.advertiseService("/sbpl_planning/plan_path", &SBPLArmPlannerNode::planKinematicPath,this);
  
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
  //cspace_->addArmCuboidsToGrid();

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

  aviz_ = new VisualizeArm(arm_name_);
  aviz_->setReferenceFrame(reference_frame_);

  ROS_DEBUG("Initialized sbpl arm planning environment.");
  return true;
}

/** Callbacks ----------------------------------------------------------------*/
void SBPLArmPlannerNode::collisionMapCallback(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  updateMapFromCollisionMap(collision_map);
}

void SBPLArmPlannerNode::updateMapFromCollisionMap(const arm_navigation_msgs::CollisionMapConstPtr &collision_map)
{
  ROS_DEBUG("[node] trying to get colmap_mutex_");
  if(colmap_mutex_.try_lock())
  {
    ROS_DEBUG("[node] locked colmap_mutex_");

    if(collision_map->header.frame_id.compare(reference_frame_) != 0)
    {
      ROS_WARN_ONCE("collision_map_occ is in %s not in %s", collision_map->header.frame_id.c_str(), reference_frame_.c_str());
      ROS_DEBUG("the collision map has %i cubic obstacles", int(collision_map->boxes.size()));
    }

    // update collision map
    if(collision_map->boxes.empty())
      ROS_INFO("[node] Received collision map is empty.");
    else
    {
      grid_->updateFromCollisionMap(*collision_map);
      cmap_ = *collision_map;
    }

    // add self collision blocks
    //cspace_->addArmCuboidsToGrid();
  
    cspace_->putCollisionObjectsInGrid();

    map_frame_ = collision_map->header.frame_id; 
    setArmToMapTransform(map_frame_);

    colmap_mutex_.unlock();
    ROS_DEBUG("[node] released colmap_mutex_ mutex.");

    visualizeCollisionObjects();

    grid_->visualize();
    return;
  }
  else
  {
    ROS_DEBUG("[node] failed trying to get colmap_mutex_ mutex");
    return;
  }
}

void SBPLArmPlannerNode::jointStatesCallback(const sensor_msgs::JointStateConstPtr &state)
{
  if(cspace_ == NULL)
    return;

  std::vector<double> angles(7,0);

  // torso_lift_link
  cspace_->setJointPosition(state->name[12], state->position[12]);

  if(arm_name_.compare("right_arm") == 0)
  {
    // r_gripper_l_finger_link & r_gripper_r_finger_link
    cspace_->setJointPosition(state->name[25], state->position[25]);
    cspace_->setJointPosition(state->name[26], state->position[26]);

    // right arm - pr2 specific
    angles[0] = state->position[18];
    angles[1] = state->position[19];
    angles[2] = state->position[17];
    angles[3] = state->position[21];
    angles[4] = state->position[20];
    angles[5] = state->position[22];
    angles[6] = state->position[23];
  }
  else
  {
    // l_gripper_l_finger_link & l_gripper_r_finger_link
    cspace_->setJointPosition(state->name[39], state->position[39]);
    cspace_->setJointPosition(state->name[40], state->position[40]);

    // left arm - pr2 specific
    angles[0] = state->position[32];
    angles[1] = state->position[33];
    angles[2] = state->position[31];
    angles[3] = state->position[35];
    angles[4] = state->position[34];
    angles[5] = state->position[36];
    angles[6] = state->position[37];
  }
  visualizeCollisionModel(angles);
}

void SBPLArmPlannerNode::attachedObjectCallback(const arm_navigation_msgs::AttachedCollisionObjectConstPtr &attached_object)
{
  if(object_mutex_.try_lock())
  {
    // remove all objects
    if(attached_object->link_name.compare(arm_navigation_msgs::AttachedCollisionObject::REMOVE_ALL_ATTACHED_OBJECTS) == 0 &&
        attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      ROS_DEBUG("[node] Removing all attached objects.");
      attached_object_ = false;
      cspace_->removeAttachedObject();
    }
    else if(!cspace_->doesLinkExist(attached_object->link_name))
    {
      ROS_WARN("[node] This attached object is not intended for this arm.");
    }
    // add object
    else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
    {
      ROS_DEBUG("[node] Received a message to ADD an object (%s) with %d shapes.", attached_object->object.id.c_str(), int(attached_object->object.shapes.size()));
      attachObject(attached_object->object, attached_object->link_name);
    }
    // attach object and remove it from collision space
    else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
    {
      ROS_INFO("[node] Received a message to ATTACH_AND_REMOVE_AS_OBJECT of object: %s", attached_object->object.id.c_str());
      
      // have we seen this collision object before?
      if(object_map_.find(attached_object->object.id) != object_map_.end())
      {
        ROS_INFO("[node] We have seen this object (%s) before.", attached_object->object.id.c_str());
        ROS_WARN("[node] attached objects we have seen before are not handled correctly right now.");
        attachObject(object_map_.find(attached_object->object.id)->second, attached_object->link_name);
      }
      else
      {
        ROS_INFO("[node] We have NOT seen this object (%s) before.", attached_object->object.id.c_str());
        object_map_[attached_object->object.id] = attached_object->object;
        attachObject(attached_object->object, attached_object->link_name);
      }
      cspace_->removeCollisionObject(attached_object->object);
    }
    // remove object
    else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
    {
      attached_object_ = false;
      ROS_DEBUG("[node] Removing object (%s) from gripper.", attached_object->object.id.c_str());
      cspace_->removeAttachedObject();
    }
    else if(attached_object->object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
    {
      attached_object_ = false;
      ROS_DEBUG("[node] Removing object (%s) from gripper and adding to collision map.", attached_object->object.id.c_str());
      cspace_->removeAttachedObject();
      cspace_->addCollisionObject(attached_object->object);
    }
    else
      ROS_WARN("Received a collision object with an unknown operation");

    object_mutex_.unlock();
  }

  updateCollisionMap();
}

void SBPLArmPlannerNode::collisionObjectCallback(const arm_navigation_msgs::CollisionObjectConstPtr &collision_object)
{
  if(object_mutex_.try_lock())
  {
    // debug: have we seen this collision object before?
    if(object_map_.find(collision_object->id) != object_map_.end())
      ROS_DEBUG("[node] We have seen this object ('%s')  before.", collision_object->id.c_str());
    else
      ROS_DEBUG("[node] We have NOT seen this object ('%s') before.", collision_object->id.c_str());
    object_map_[collision_object->id] = (*collision_object);
    object_mutex_.unlock();
  }
  cspace_->processCollisionObjectMsg((*collision_object));

  visualizeCollisionObjects();
}

void SBPLArmPlannerNode::attachObject(const arm_navigation_msgs::CollisionObject &obj, std::string link_name)
{
  geometry_msgs::PoseStamped pose_in, pose_out;
  arm_navigation_msgs::CollisionObject object(obj);

  attached_object_ = true;

  ROS_INFO("Received a collision object message with %d shapes.", int(object.shapes.size()));

  for(size_t i = 0; i < object.shapes.size(); i++)
  {
    pose_in.header = object.header;
    pose_in.header.stamp = ros::Time();
    pose_in.pose = object.poses[i];
    tf_.transformPose(attached_object_frame_, pose_in, pose_out);
    object.poses[i] = pose_out.pose;
    ROS_WARN("[node] [attach_object] Converted shape from %s (%0.2f %0.2f %0.2f) to %s (%0.3f %0.3f %0.3f)", pose_in.header.frame_id.c_str(), pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, attached_object_frame_.c_str(), pose_out.pose.position.x, pose_out.pose.position.y, pose_out.pose.position.z);

    if(object.shapes[i].type == arm_navigation_msgs::Shape::SPHERE)
    {
      ROS_INFO("[node] Attaching a sphere with radius: %0.3fm", object.shapes[i].dimensions[0]);
      cspace_->attachSphereToGripper(link_name, object.poses[i], object.shapes[i].dimensions[0]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::CYLINDER)
    {
      ROS_INFO("[node] Attaching a cylinder with radius: %0.3fm & length %0.3fm", object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);

      //cspace_->attachCylinderToGripper(link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
      cspace_->attachCylinder(link_name, object.poses[i], object.shapes[i].dimensions[0], object.shapes[i].dimensions[1]);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::MESH)
    {
      ROS_ERROR("Not supporting meshes.");
      //ROS_INFO("[node] Attaching a mesh with %d triangles  & %d vertices.", int(object.shapes[i].triangles.size()/3), int(object.shapes[i].vertices.size()));

      //cspace_->attachMeshToGripper(object.header.frame_id, object.poses[i], object.shapes[i].triangles, object.shapes[i].vertices);
    }
    else if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      std::vector<double> dims(object.shapes[i].dimensions);
      sort(dims.begin(),dims.end());
      /*
      for(size_t k = 0; k < dims.size(); ++k)
        ROS_ERROR("[k] dim: %0.3f",dims[k]);
      */
      ROS_INFO("[node] Attaching a box as a cylinder with length: %0.3fm   radius: %0.3fm", dims[2], dims[1]);
      //cspace_->attachCylinderToGripper(link_name, object.poses[i], dims[1], dims[2]);
      cspace_->attachCylinder(link_name, object.poses[i], dims[1], dims[2]);
    }
    else
      ROS_WARN("[node] Currently attaching objects of type '%d' aren't supported.", object.shapes[i].type);
  }
}

/** Planner Interface  -------------------------------------------------------*/
bool SBPLArmPlannerNode::setStart(const sensor_msgs::JointState &start_state)
{
  std::vector<double> sbpl_start(start_state.position.size(),0);

  for(unsigned int i=0; i< start_state.position.size(); i++)
    sbpl_start[i] = (double)(start_state.position[i]);

  std::vector<std::vector<double> > xyz;

  ROS_INFO("[node] start: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f", sbpl_start[0],sbpl_start[1],sbpl_start[2],sbpl_start[3],sbpl_start[4],sbpl_start[5],sbpl_start[6]);

  if(sbpl_arm_env_.setStartConfiguration(sbpl_start) == 0)
  {
    ROS_ERROR("[node] Environment failed to set start state. Not Planning.\n");
    return false;
  }

  if(planner_->set_start(mdp_cfg_.startstateid) == 0)
  {
    ROS_ERROR("[node] Failed to set start state. Not Planning.");
    return false;
  }

  //if(attached_object_)
  //  visualizeAttachedObject(sbpl_start);

  return true;
}

bool SBPLArmPlannerNode::setGoalPosition(const arm_navigation_msgs::Constraints &goals)
{
  double roll,pitch,yaw;
  geometry_msgs::Pose pose_msg;
  tf::Pose tf_pose;
  std::vector <std::vector <double> > sbpl_goal(1, std::vector<double> (11,0));  //Changed to include Quaternion
  std::vector <std::vector <double> > sbpl_tolerance(1, std::vector<double> (12,0));

  if(goals.position_constraints.size() != goals.orientation_constraints.size())
    ROS_WARN("[node] There are %d position contraints and %d orientation constraints.", int(goals.position_constraints.size()),int(goals.orientation_constraints.size()));

  //currently only supports one goal
  sbpl_goal[0][0] = goals.position_constraints[0].position.x;
  sbpl_goal[0][1] = goals.position_constraints[0].position.y;
  sbpl_goal[0][2] = goals.position_constraints[0].position.z;

  //convert quaternion into roll,pitch,yaw
  pose_msg.position = goals.position_constraints[0].position;
  pose_msg.orientation = goals.orientation_constraints[0].orientation;

  //perturb quaternion if rpy will suffer from gimbal lock
  //if(pose_msg.orientation.x == 0 && pose_msg.orientation.z == 0)
  pose_msg.orientation.w += 0.005;

  tf::poseMsgToTF(pose_msg, tf_pose);
  tf_pose.getBasis().getRPY(roll,pitch,yaw);
  sbpl_goal[0][3] = roll;
  sbpl_goal[0][4] = pitch;
  sbpl_goal[0][5] = yaw;

  //6dof goal: true, 3dof: false 
  sbpl_goal[0][6] = true;
 
  //orientation constraint as a quaternion 
  sbpl_goal[0][7] = goals.orientation_constraints[0].orientation.x;
  sbpl_goal[0][8] = goals.orientation_constraints[0].orientation.y;
  sbpl_goal[0][9] = goals.orientation_constraints[0].orientation.z;
  sbpl_goal[0][10] = goals.orientation_constraints[0].orientation.w;

  //allowable tolerance from goal
  sbpl_tolerance[0][0] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][1] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][2] = goals.position_constraints[0].constraint_region_shape.dimensions[0] / 2.0;
  sbpl_tolerance[0][3] = goals.orientation_constraints[0].absolute_roll_tolerance;
  sbpl_tolerance[0][4] = goals.orientation_constraints[0].absolute_pitch_tolerance;
  sbpl_tolerance[0][5] = goals.orientation_constraints[0].absolute_yaw_tolerance;

  ROS_INFO("[node] goal quat from move_arm: %0.3f %0.3f %0.3f %0.3f", goals.orientation_constraints[0].orientation.x, goals.orientation_constraints[0].orientation.y, goals.orientation_constraints[0].orientation.z, goals.orientation_constraints[0].orientation.w);

  ROS_INFO("[node] goal xyz(%s): %.3f %.3f %.3f (tol: %.3fm) rpy: %.3f %.3f %.3f (tol: %.3frad)", map_frame_.c_str(),sbpl_goal[0][0],sbpl_goal[0][1],sbpl_goal[0][2],sbpl_tolerance[0][0],sbpl_goal[0][3],sbpl_goal[0][4],sbpl_goal[0][5], sbpl_tolerance[0][1]);

  //set sbpl environment goal
  if(!sbpl_arm_env_.setGoalPosition(sbpl_goal, sbpl_tolerance))
  {
    ROS_ERROR("[node] Failed to set goal state. Perhaps goal position is out of reach. Exiting.");
    return false;
  }

  //set planner goal	
  if(planner_->set_goal(mdp_cfg_.goalstateid) == 0)
  {
    ROS_ERROR("[node] Failed to set goal state. Exiting.");
    return false;
  }

  tf::Quaternion q;
  q.setRPY(roll,pitch,yaw);

  ROS_DEBUG("Quat from MoveArm: %0.3f %0.3f %0.3f %0.3f", goals.orientation_constraints[0].orientation.x, goals.orientation_constraints[0].orientation.y, goals.orientation_constraints[0].orientation.z, goals.orientation_constraints[0].orientation.w);
  ROS_DEBUG("      RPY with TF: %0.3f %0.3f %0.3f", roll,pitch,yaw);
  ROS_DEBUG("     Quat with TF: %0.3f %0.3f %0.3f %0.3f", q.x(), q.y(), q.z(), q.w());

  return true;
}

bool SBPLArmPlannerNode::planToPosition(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
{
  unsigned int i;
  int nind = 0;
  std::vector<trajectory_msgs::JointTrajectoryPoint> arm_path;
  sensor_msgs::JointState start;

  starttime = clock();

  //check for an empty start state
  if(req.motion_plan_request.start_state.joint_state.position.size() <= 0)
  {
    ROS_ERROR("[node] No start state given. Unable to plan.");
    return false;
  }

  //check if goal constraint is empty
  if(req.motion_plan_request.goal_constraints.position_constraints.size() <= 0 || 
      req.motion_plan_request.goal_constraints.orientation_constraints.size() <= 0)
  {
    ROS_ERROR("[node] Position constraint or orientation constraint is empty. Unable to plan.");
    return false;
  }

  //check if there is more than one goal constraint
  if(req.motion_plan_request.goal_constraints.position_constraints.size() > 1 || 
      req.motion_plan_request.goal_constraints.orientation_constraints.size() > 1)
    ROS_WARN("[node] The planning request message contains %d position and %d orientation constraints. Currently the planner only supports one position & orientation constraint pair at a time. Planning to the first goal may not satisfy move_arm.", int(req.motion_plan_request.goal_constraints.position_constraints.size()), int(req.motion_plan_request.goal_constraints.orientation_constraints.size()));

  // add collision objects to occupancy grid
  cspace_->putCollisionObjectsInGrid();

  //check if planning for wrist (only link supported for now)
  planning_joint_ = req.motion_plan_request.goal_constraints.position_constraints[0].link_name;

  if(planning_joint_ != "r_wrist_roll_link" && arm_name_ == "right_arm")
  {
    ROS_ERROR("[node] Planner is configured to plan for the right arm. It has only been tested with pose constraints for r_wrist_roll_link. Other links may be supported in the future.");
    return false;
  }
  else if(planning_joint_ != "l_wrist_roll_link" && arm_name_ == "left_arm")
  {
    ROS_ERROR("[node] Planner is configured to plan for the left arm. It has only been tested with pose constraints for l_wrist_roll_link. Other links may be supported in the future.");
    return false;
  }

  //transform goal pose into reference_frame_
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = req.motion_plan_request.goal_constraints.position_constraints[0].header;
  pose_in.header.stamp = ros::Time();
  pose_in.pose.position = req.motion_plan_request.goal_constraints.position_constraints[0].position;
  pose_in.pose.orientation = req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation;

  // TODO: catch the exception
  tf_.transformPose(map_frame_,pose_in,pose_out);

  req.motion_plan_request.goal_constraints.position_constraints[0].position = pose_out.pose.position;
  req.motion_plan_request.goal_constraints.orientation_constraints[0].orientation = pose_out.pose.orientation;

  ROS_DEBUG("[node] Transformed goal from (%s): %0.3f %0.3f %0.3f to (%s): %0.3f %0.3f %0.3f", req.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id.c_str(),pose_in.pose.position.x, pose_in.pose.position.y, pose_in.pose.position.z, map_frame_.c_str(), pose_out.pose.position.x,pose_out.pose.position.y,pose_out.pose.position.z);

  //get the initial state of the planning joints
  start.position.resize(num_joints_);
  for(i = 0; i < req.motion_plan_request.start_state.joint_state.position.size(); i++)
  {
    if(joint_names_[nind].compare(req.motion_plan_request.start_state.joint_state.name[i]) == 0)
    {
      start.position[nind] = req.motion_plan_request.start_state.joint_state.position[i];
      nind++;
    }
    if(nind == num_joints_)
      break;
  }
  if(nind != num_joints_)
    ROS_WARN("[node] Not all of the expected joints in the arm were assigned a starting position.");

  allocated_time_ = req.motion_plan_request.allowed_planning_time.toSec();

  colmap_mutex_.lock();
  object_mutex_.lock();

  ROS_WARN("[node]  About to set the start position, %lf seconds after request came in.",(clock() - starttime) / (double)CLOCKS_PER_SEC);
  
  ROS_DEBUG("[node] About to set start configuration");
  if(setStart(start))
  {
    ROS_DEBUG("[node] Successfully set starting configuration");

    ROS_WARN("[node]  About to visualize and set goal, %lf seconds after request came in.",(clock() - starttime) / (double)CLOCKS_PER_SEC);
    
    if(visualize_goal_)
      visualizeGoalPosition(req.motion_plan_request.goal_constraints);

    if(setGoalPosition(req.motion_plan_request.goal_constraints))
    {
      ROS_WARN("[node]  About to start planning, %lf seconds after request came in.",(clock() - starttime) / (double)CLOCKS_PER_SEC);

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

        // fill in the joint names 
        res.trajectory.joint_trajectory.joint_names.resize(num_joints_);
        for(i = 0; i < (unsigned int)num_joints_; i++)
          res.trajectory.joint_trajectory.joint_names[i] = joint_names_[i];
  
        ROS_INFO("[node] Planner completed in %lf seconds. Planned trajectory has %d waypoints.",(clock() - starttime) / (double)CLOCKS_PER_SEC, int(res.trajectory.joint_trajectory.points.size()));

        if(print_path_)
          printPath(res.trajectory.joint_trajectory.points);

        // compute distance to goal
        if(!isGoalConstraintSatisfied(res.trajectory.joint_trajectory.points[res.trajectory.joint_trajectory.points.size()-1].positions, req.motion_plan_request.goal_constraints))
          ROS_WARN("[node] Uh Oh. Goal constraint isn't satisfied.");
        
        // visualizations
        if(visualize_expanded_states_)
          displayARAStarStates();

        if(visualize_heuristic_)
          displayShortestPath();

        if(visualize_trajectory_)
          aviz_->visualizeJointTrajectoryMsg(res.trajectory.joint_trajectory, throttle_);
          
        if(use_research_heuristic_)
          visualizeElbowPoses();

        return true;
      }
      else
      {
        ROS_ERROR("[node] Failed to plan within alotted time frame (%0.2f seconds).", allocated_time_);
      }
    }
    else
    {
      ROS_ERROR("[node] Failed to set goal pose.");
    }
  }
  else
  {
    ROS_ERROR("[node] Failed to set start configuration.");
  }

  colmap_mutex_.unlock();
  object_mutex_.unlock();

  if(visualize_expanded_states_)
    displayARAStarStates();
  
  if(use_research_heuristic_)
    visualizeElbowPoses();

  return false;
}

bool SBPLArmPlannerNode::planKinematicPath(arm_navigation_msgs::GetMotionPlan::Request &req, arm_navigation_msgs::GetMotionPlan::Response &res)
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

  //outputs debug information about the adaptive mprims
  sbpl_arm_env_.debugAdaptiveMotionPrims();

  // if a path is returned, then pack it into msg form
  if(b_ret && (solution_state_ids_v.size() > 0))
  {
    ROS_DEBUG("[plan] A path was returned with %d waypoints.", int(solution_state_ids_v.size()));
    ROS_INFO("[plan] Initial Epsilon: %0.3f   Final Epsilon: %0.3f", planner_->get_initial_eps(),planner_->get_final_epsilon());
    arm_path.resize(solution_state_ids_v.size()+1);
    for(size_t i=0; i < solution_state_ids_v.size(); i++)
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
      ROS_DEBUG("%i: %.4f %.4f %.4f %.4f %.4f %.4f %.4f", int(i), arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6]);
    }

    // planner stats    
    stats_[0] = planner_->get_initial_eps_planning_time();
    stats_[1] = planner_->get_initial_eps();
    stats_[2] = planner_->get_n_expands_init_solution();
    stats_[3] = planner_->get_final_eps_planning_time();
    stats_[4] = planner_->get_final_epsilon();
    stats_[5] = planner_->get_solution_eps();
    stats_[6] = planner_->get_n_expands();
    stats_[7] = solution_cost;
    stats_[8] = arm_path.size();
    
    ROS_INFO("\n%50s","-- Planning Statistics --");
    for(size_t i = 0; i < stats_.size(); ++i)
      ROS_INFO("%44s: %0.2f", stats_field_names_[i].c_str(), stats_[i]);
    ROS_INFO("\n");
  }

  return b_ret;
}

bool SBPLArmPlannerNode::isGoalConstraintSatisfied(const std::vector<double> &angles, const arm_navigation_msgs::Constraints &goal)
{
  bool satisfied = true;
  geometry_msgs::Pose pose, err;

  if(!computeFK(angles,pose))
  {
    ROS_ERROR("Failed to check if goal constraint is satisfied because the FK service failed.");
    return false;
  }

  if(goal.position_constraints.size() > 0)
  {
    err.position.x = fabs(pose.position.x - goal.position_constraints[0].position.x);
    err.position.y = fabs(pose.position.y - goal.position_constraints[0].position.y);
    err.position.z = fabs(pose.position.z - goal.position_constraints[0].position.z);
  }

  if(goal.orientation_constraints.size() > 0)
  {
    err.orientation.x = fabs(pose.orientation.x - goal.orientation_constraints[0].orientation.x);
    err.orientation.y = fabs(pose.orientation.y - goal.orientation_constraints[0].orientation.y);
    err.orientation.z = fabs(pose.orientation.z - goal.orientation_constraints[0].orientation.z);
    err.orientation.w = fabs(pose.orientation.w - goal.orientation_constraints[0].orientation.w);
  }

  ROS_INFO(" ");
  ROS_INFO("Pose:  xyz: %0.4f %0.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  ROS_INFO("Goal:  xyz: %0.4f %0.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", goal.position_constraints[0].position.x, goal.position_constraints[0].position.y, goal.position_constraints[0].position.z, goal.orientation_constraints[0].orientation.x, goal.orientation_constraints[0].orientation.y, goal.orientation_constraints[0].orientation.z, goal.orientation_constraints[0].orientation.w);
  ROS_INFO("Error: xyz: %0.4f %0.4f %0.4f  quat: %0.4f %0.4f %0.4f %0.4f", err.position.x, err.position.y, err.position.z, err.orientation.x, err.orientation.y, err.orientation.z, err.orientation.w);
  ROS_INFO(" ");

  if(goal.position_constraints[0].constraint_region_shape.type == arm_navigation_msgs::Shape::BOX)
  {
    if(goal.position_constraints[0].constraint_region_shape.dimensions.size() < 3)
    { 
      ROS_WARN("Goal constraint region shape is a BOX but fewer than 3 dimensions are defined.");
      return false;
    }
    if(err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
      satisfied = false;
    }
    if(err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[1])
    {
      ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]); 
      satisfied = false;
    }
    if(err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[2])
    {
      ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
      satisfied = false;
    }
  }
  else if(goal.position_constraints[0].constraint_region_shape.type == arm_navigation_msgs::Shape::SPHERE)
  {
    if(goal.position_constraints[0].constraint_region_shape.dimensions.size() < 1)
    { 
      ROS_WARN("Goal constraint region shape is a SPHERE but it has no dimensions...");
      return false;
    }
    if(err.position.x >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("X is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.x, goal.position_constraints[0].constraint_region_shape.dimensions[0]);
      satisfied = false;
    }
    if(err.position.y >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("Y is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.y, goal.position_constraints[0].constraint_region_shape.dimensions[1]);
      satisfied = false;
    }
    if(err.position.z >= goal.position_constraints[0].constraint_region_shape.dimensions[0])
    {
      ROS_WARN("Z is not satisfied (error: %0.4f   tolerance: %0.4f)", err.position.z, goal.position_constraints[0].constraint_region_shape.dimensions[2]);
      satisfied = false;
    }
  }
  else
    ROS_WARN("Goal constraint region shape is of type %d.", goal.position_constraints[0].constraint_region_shape.type);

  return satisfied;
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

void SBPLArmPlannerNode::setArmToMapTransform(std::string &map_frame)
{
  std::string fk_root_frame;

  // frame that the sbpl_arm_model is working in
  sbpl_arm_env_.getArmChainRootLinkName(fk_root_frame);

  // get transform to frame that collision map is in
  try
  {
    tf_.lookupTransform(map_frame, fk_root_frame, ros::Time(0), transform_);

    ROS_DEBUG("Received transform from %s to %s (translation: %f %f %f)",fk_root_frame.c_str(),map_frame.c_str(), transform_.getOrigin().x(),transform_.getOrigin().y(),transform_.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
  }

  // convert transform to a KDL object
  tf::TransformTFToKDL(transform_,kdl_transform_);
  sbpl_arm_env_.setReferenceFrameTransform(kdl_transform_, map_frame);
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

  ROS_INFO("[node] displaying %d expanded states.\n",int(expanded_states.size()));
}

void SBPLArmPlannerNode::visualizeGoalPosition(const arm_navigation_msgs::Constraints &goal_pose)
{
  geometry_msgs::Pose pose;
  pose.position = goal_pose.position_constraints[0].position;
  pose.orientation = goal_pose.orientation_constraints[0].orientation;
  aviz_->visualizePose(pose, "goal_pose");
  ROS_DEBUG("[node] Publishing goal marker visualizations.");
}

void SBPLArmPlannerNode::visualizeElbowPoses()
{
  std::vector<std::vector<double> > elbow_poses;
  sbpl_arm_env_.getElbowPoints(elbow_poses);

  for(size_t i = 0; i < elbow_poses.size(); i++)
  {
    aviz_->visualizeSphere(elbow_poses[i], 100, "elbow_poses" + boost::lexical_cast<std::string>(i), 0.02);
  }
  ROS_INFO("[node] Visualizing %d elbow poses.", int(elbow_poses.size()));
}

void SBPLArmPlannerNode::visualizeCollisionObjects()
{
  std::vector<geometry_msgs::Pose> poses;
  std::vector<std::vector<double> > points(1,std::vector<double>(3,0));
  std::vector<double> color(4,1);
  color[2] = 0;

  cspace_->getCollisionObjectVoxelPoses(poses);

  points.resize(poses.size());
  for(size_t i = 0; i < poses.size(); ++i)
  {
    points[i].resize(3);
    points[i][0] = poses[i].position.x;
    points[i][1] = poses[i].position.y;
    points[i][2] = poses[i].position.z;
  }

  ROS_DEBUG("[node] Displaying %d known collision object voxels.", int(points.size()));
  aviz_->visualizeBasicStates(points, color, "known_objects", 0.01);
}

void SBPLArmPlannerNode::visualizeAttachedObject(trajectory_msgs::JointTrajectory &traj_msg, int throttle)
{
  std::vector<double> angles(7,0);
  std::vector<std::vector<double> > xyz;
  
  if(traj_msg.points.empty())
  {
    ROS_WARN("[node] Trajectory message is empty. Not visualizing anything.");
    return;
  }

  for(size_t i = 0; i < traj_msg.points.size(); i++)
  {
    angles = traj_msg.points[i].positions;

    if(!cspace_->getAttachedObject(angles, xyz))
      continue;
   
    aviz_->visualizeSpheres(xyz, 10*(i+1), "sbpl_attached_object_" + boost::lexical_cast<std::string>(i), cspace_->getAttachedObjectRadius());
  }
}

void SBPLArmPlannerNode::visualizeAttachedObject(const std::vector<double> &angles)
{
  std::vector<std::vector<double> > xyz;

  if(angles.size() < 7)
  {
    ROS_WARN("[node] Joint configuration is not of the right length.");
    return;
  }

  if(!cspace_->getAttachedObject(angles, xyz))
    return;

  aviz_->visualizeSpheres(xyz, 50, "sbpl_attached_object", cspace_->getAttachedObjectRadius());
}

void SBPLArmPlannerNode::displayShortestPath()
{
  dpath_ = sbpl_arm_env_.getShortestPath();

  //check if the list is empty
  if(dpath_.empty())
  {
    ROS_INFO("[node] The heuristic path has a length of 0");
    return;
  }
  else
    ROS_DEBUG("Visualizing heuristic path from start to goal with %d waypoints.",int(dpath_.size()));
 
  aviz_->visualizeSpheres(dpath_, 45, "heuristic_path", 0.04);
}

void SBPLArmPlannerNode::printPath(const std::vector<trajectory_msgs::JointTrajectoryPoint> &arm_path)
{
  double roll,pitch,yaw;
  tf::Pose tf_pose;
  geometry_msgs::Pose pose;
  std::vector<double> jnt_pos(num_joints_,0);

  ROS_INFO("Path:");
  for(unsigned int i = 0; i < arm_path.size(); i++)
  {
    for(int j = 0; j < num_joints_; ++j)
      jnt_pos[j] = arm_path[i].positions[j];

    computeFK(jnt_pos, pose);
    tf::poseMsgToTF(pose, tf_pose);
    tf_pose.getBasis().getRPY(roll,pitch,yaw);

    ROS_INFO("%3d: %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f %1.4f\n   xyz: %2.3f %2.3f %2.3f  rpy: %0.3f %0.3f %0.3f  quat: %0.2f %0.2f %0.2f %0.2f", i,arm_path[i].positions[0],arm_path[i].positions[1],arm_path[i].positions[2],arm_path[i].positions[3],arm_path[i].positions[4],arm_path[i].positions[5],arm_path[i].positions[6],pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w);
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

void SBPLArmPlannerNode::visualizeCollisionModel(const std::vector<double> &angles)
{
  std::vector<std::vector<double> > spheres;
  // arm
  cspace_->getCollisionCylinders(angles, spheres);
  aviz_->visualizeSpheres(spheres, 140, 0.8, arm_name_ + "_model");
  ROS_DEBUG("[node] Visualizing %d %s_model spheres.", int(spheres.size()), arm_name_.c_str());
  
  // attached object
  spheres.clear();
  cspace_->getAttachedObject(angles, spheres);
  if(spheres.size() > 0)
  {
    aviz_->visualizeSpheres(spheres, 250, 0.8, "attached_object");
    ROS_INFO("[node] Visualizing %d attached_object spheres.", int(spheres.size()));
  }
  else
    ROS_DEBUG("[node] No attached object.");
}

void SBPLArmPlannerNode::updateCollisionMap()
{
  if(!cmap_.boxes.empty())
    grid_->updateFromCollisionMap(cmap_);

  cspace_->putCollisionObjectsInGrid();

  // visualizations
  visualizeCollisionObjects();
  grid_->visualize();
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


