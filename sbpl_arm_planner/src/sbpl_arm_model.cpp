/*
 * Copyright (c) 2010, Maxim Likhachev
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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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

#include <sbpl_arm_planner/sbpl_arm_model.h>
#include <tf_conversions/tf_kdl.h>

using namespace std;

namespace sbpl_arm_planner {

SBPLArmModel::SBPLArmModel(FILE* arm_file) : fk_solver_(NULL), pr2_arm_ik_solver_(NULL)
{
  num_joints_ = 7;
  num_links_ = 3;
  chain_root_name_ = "base_link";
  chain_tip_name_ = "r_wrist_roll_link";
  planning_joint_name_ = "r_wrist_roll_link";
  reference_frame_ = "base_link";
    
  joints_.resize(num_joints_);
  links_.resize(num_links_);

  for(unsigned int i = 0; i < joints_.size(); ++i)
    joints_[i].continuous = true;

  //parse arm description file
  if(!getArmDescription(arm_file))
    SBPL_WARN("Parsing of sbpl arm description file failed. Will attempt to use defaults.");
 
  debug_stream_ = "sbpl_arm";
  max_radius_ = 0;
  resolution_ = 0.01;
}

SBPLArmModel::~SBPLArmModel()
{
  if(fk_solver_ != NULL)
    delete fk_solver_;
  if(pr2_arm_ik_solver_ != NULL)
    delete pr2_arm_ik_solver_;
}

void SBPLArmModel::setDebugFile(std::string stream_name)
{
  debug_stream_ = stream_name;
}

bool SBPLArmModel::getArmDescription(FILE* fCfg)
{
  if(fCfg == NULL)
    return false;

  char sTemp[1024];
  int i;

  if(fscanf(fCfg,"%s",sTemp) < 1)
    SBPL_WARN("Parsed string has length < 1.");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "number_of_joints:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      num_joints_ = atoi(sTemp);
      joints_.resize(num_joints_);
    }
    else if(strcmp(sTemp, "number_of_links:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      num_links_ = atoi(sTemp);
      links_.resize(num_links_);
    }
    else if(strcmp(sTemp, "index_of_planning_joint:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      planning_joint_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "kdl_chain_root_name:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      chain_root_name_.assign(sTemp);
    }
    else if(strcmp(sTemp, "kdl_chain_tip_name:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      chain_tip_name_.assign(sTemp);
    }
    else if(strcmp(sTemp, "planning_joint_name:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      planning_joint_name_.assign(sTemp);
    }
    else if(strcmp(sTemp, "min_joint_limits:") == 0)
    {
      for(i = 0; i < num_joints_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        joints_[i].min = atof(sTemp);

        if(joints_[i].min != 0.0)
          joints_[i].continuous = false;
      }
    }
    else if(strcmp(sTemp, "max_joint_limits:") == 0)
    {
      for(i = 0; i < num_joints_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        joints_[i].max = atof(sTemp);
        
        if(joints_[i].max != 0.0)
          joints_[i].continuous = false;
      }
    }
    else if(strcmp(sTemp, "radius_of_links(meters):") == 0)
    {
      for(i = 0; i < num_links_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        links_[i].radius = atof(sTemp);
      }
    }
    else if(strcmp(sTemp, "length_of_links(meters):") == 0)
    {
      for(i = 0; i < num_links_; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
        links_[i].length = atof(sTemp);
      }
    }
    else if(strcmp(sTemp, "collision_cuboids:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");
     
      num_collision_cuboids_  = atoi(sTemp);
      collision_cuboids_.resize(num_collision_cuboids_);
      for(i = 0; i < num_collision_cuboids_; i++)
      {
        collision_cuboids_[i].resize(6);
        for(int j = 0; j < 6; j++)
        {
          if(fscanf(fCfg,"%s",sTemp) < 1)
            SBPL_WARN("Parsed string has length < 1.");
          collision_cuboids_[i][j] = atof(sTemp);
        }
      }
      SBPL_DEBUG("Number of collision cuboids: %d", num_collision_cuboids_);
    }
    else if(strcmp(sTemp, "kdl_indeces_of_link_tips:") == 0)
    {
      joint_indeces_.resize(num_links_+1);
      for(i = 0; i < num_links_ + 1; i++)
      {
        if(fscanf(fCfg,"%s",sTemp) < 1)
          SBPL_WARN("Parsed string has length < 1.");

        joint_indeces_[i] = atoi(sTemp);
      }
    }
    
    if(fscanf(fCfg,"%s",sTemp) < 1)
      SBPL_DEBUG("Parsed string has length < 1.");
  }
  
  SBPL_DEBUG("[getArmDescription] Finished parsing arm file.");
  return true;
}

void SBPLArmModel::setResolution(double resolution)
{
  resolution_ = resolution;

  //store discretized link radii
  for(int i = 0; i < num_links_; i++)
  {
    links_[i].radius_c = (links_[i].radius / resolution_) + 0.5;
    links_[i].length_c = (links_[i].length / resolution_) + 0.5;

    if(links_[i].radius_c > max_radius_)
      max_radius_ = links_[i].radius_c;
  }
}

bool SBPLArmModel::initKDLChainFromParamServer()
{
  ros::NodeHandle nh("~");
  std::string robot_description;
  std::string robot_param;

  nh.searchParam("robot_description",robot_param);
  nh.param<std::string>(robot_param,robot_description,"");

  if(robot_description.empty())
  {
    SBPL_ERROR("[SBPLArmModel] Unable to get robot_description from rosparam server.");
    return false;
  }

  return initKDLChain(robot_description);
}

bool SBPLArmModel::initKDLChain(const std::string &fKDL)
{
  if (!kdl_parser::treeFromString(fKDL, kdl_tree_))
  {
    SBPL_ERROR("Failed to parse tree from manipulator description file.");
    return false;;
  }

  if (!kdl_tree_.getChain(chain_root_name_, chain_tip_name_, chain_))
  {
    SBPL_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_); 
  jnt_pos_in_.resize(chain_.getNrOfJoints());
  jnt_pos_out_.resize(chain_.getNrOfJoints());

  SBPL_DEBUG("[initKDLChain] The FK chain has %d segments with %d joints.", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  SBPL_DEBUG("[initKDLChain] root: %s tip: %s.", chain_root_name_.c_str(), chain_tip_name_.c_str());
  
  // initialize PR2ArmIKSolver
  robot_model_.initString(fKDL);
  pr2_arm_ik_solver_ = new pr2_arm_kinematics::PR2ArmIKSolver(robot_model_, "torso_lift_link", planning_joint_name_, 0.02, 2); 

  if(!pr2_arm_ik_solver_->active_)
  {
    SBPL_ERROR("Error: the IK solver is NOT active.");
    return false;
  }

  ik_jnt_pos_in_.resize(num_joints_);
  ik_jnt_pos_out_.resize(num_joints_);

  printArmDescription(std::string("sbpl_arm")); 
  return true;
}

bool SBPLArmModel::computeFK(const std::vector<double> angles, int frame_num, KDL::Frame *frame_out)
{
  KDL::Frame fk_frame_out;
  
  for(int i = 0; i < num_joints_; ++i)
    jnt_pos_in_(i) = angles::normalize_angle(angles[i]);

  if(fk_solver_->JntToCart(jnt_pos_in_, fk_frame_out, frame_num) < 0)
  {
    SBPL_ERROR("JntToCart returned < 0. Exiting.");
    return false;
  }

  *frame_out = fk_frame_out;
  frame_out->p = transform_ * fk_frame_out.p;

  return true;
}

bool SBPLArmModel::computeFK(const std::vector<double> angles, int frame_num, std::vector<double> &xyzrpy)
{
  KDL::Frame frame_out;
  xyzrpy.resize(6);
  if(computeFK(angles,frame_num,&frame_out))
  {
    xyzrpy[0] = frame_out.p[0];
    xyzrpy[1] = frame_out.p[1];
    xyzrpy[2] = frame_out.p[2];

    frame_out.M.GetRPY(xyzrpy[3],xyzrpy[4],xyzrpy[5]);
    return true;
  }
  
  return false;
}

bool SBPLArmModel::computeIK(const std::vector<double> pose, const std::vector<double> start, std::vector<double> &solution)
{
  //pose: {x,y,z,r,p,y}  or {x,y,z,qx,qy,qz,qw}
  KDL::Frame frame_des;

  //fill the goal frame
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);
  
  //RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  //Quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  //transform from reference_frame_ to root_link_
  frame_des.p = transform_inverse_ * frame_des.p;

  //fill the start configuration (to be used as a seed)
  for(unsigned int i = 0; i < start.size(); i++)
    ik_jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJntSearch

  //call IK solver
  if(pr2_arm_ik_solver_->CartToJntSearch(ik_jnt_pos_in_, frame_des, ik_jnt_pos_out_, 0.3) < 0)
    return false;

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = ik_jnt_pos_out_(i);

  return true;
}

bool SBPLArmModel::computeFastIK(const std::vector<double> pose, const std::vector<double> start, std::vector<double> &solution)
{
  //pose: {x,y,z,r,p,y}  or {x,y,z,qx,qy,qz,qw}
  KDL::Frame frame_des;

  //fill the goal frame
  frame_des.p.x(pose[0]);
  frame_des.p.y(pose[1]);
  frame_des.p.z(pose[2]);
  
  //RPY
  if(pose.size() == 6)
    frame_des.M = KDL::Rotation::RPY(pose[3],pose[4],pose[5]);
  //Quaternion
  else
    frame_des.M = KDL::Rotation::Quaternion(pose[3],pose[4],pose[5],pose[6]);

  //transform from reference_frame_ to root_link of the IK tree
  frame_des.p = transform_inverse_ * frame_des.p;

  //fill the start configuration (to be used as a seed)
  for(unsigned int i = 0; i < start.size(); i++)
    ik_jnt_pos_in_(i) = angles::normalize_angle(start[i]); // must be normalized for CartToJnt

  ROS_DEBUG("[ik]  xyz: %0.2f %0.2f %0.2f  rpy: %0.2f %0.2f %0.2f", pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);
  ROS_DEBUG("[ik] seed: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",ik_jnt_pos_in_(0),ik_jnt_pos_in_(1),ik_jnt_pos_in_(2),ik_jnt_pos_in_(3),ik_jnt_pos_in_(4),ik_jnt_pos_in_(5),ik_jnt_pos_in_(6));

  //call IK solver
  if(pr2_arm_ik_solver_->CartToJnt(ik_jnt_pos_in_, frame_des, ik_jnt_pos_out_) < 0)
  {
    ROS_DEBUG("[ik] Failed with error code: %d", pr2_arm_ik_solver_->CartToJnt(ik_jnt_pos_in_, frame_des, ik_jnt_pos_out_));
    return false;
  }

  solution.resize(start.size());
  for(size_t i = 0; i < solution.size(); ++i)
    solution[i] = ik_jnt_pos_out_(i);

  ROS_DEBUG("[ik] sol.: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f",ik_jnt_pos_out_(0),ik_jnt_pos_out_(1),ik_jnt_pos_out_(2),ik_jnt_pos_out_(3),ik_jnt_pos_out_(4),ik_jnt_pos_out_(5),ik_jnt_pos_out_(6));
  
  return true;
}

bool SBPLArmModel::getJointPositions(const std::vector<double> angles, std::vector<std::vector<double> > &links, KDL::Frame &f_out)
{
  KDL::Frame f;

  links.resize(joint_indeces_.size());

  for(int i = 0; i < int(joint_indeces_.size()); i++)
  {
    if(!computeFK(angles, joint_indeces_[i], &f))
      return false;
    links[i].resize(3,0);
    links[i][0] = f.p.x();
    links[i][1] = f.p.y();
    links[i][2] = f.p.z();

    // changed from planning_joint_ to 2  (6/15/11)
    if(i == 2)
      f_out = f;
  }

  return true;
}

bool SBPLArmModel::getPlanningJointPose(const std::vector<double> angles, std::vector<double> &pose)
{
  KDL::Frame planning_joint_frame;
  if(pose.size() < 6)
    pose.resize(6);
 
  if(computeFK(angles, planning_joint_, &planning_joint_frame))
  {
    pose[0] = planning_joint_frame.p.x();
    pose[1] = planning_joint_frame.p.y();
    pose[2] = planning_joint_frame.p.z();
   
    planning_joint_frame.M.GetRPY(pose[3],pose[4],pose[5]);
    return true;
  }

  return false;
}

bool SBPLArmModel::checkJointLimits(std::vector<double> angles, bool verbose)
{
  std::vector<double> norm_angles(angles.size());
  for(int i = 0; i < int(angles.size()); ++i)
  {
    if(i==2)
      continue;
    norm_angles[i] = angles::normalize_angle(angles[i]);
  }

  //upper arm roll is a special case
  if(angles[2] > joints_[2].max)
    norm_angles[2] = angles[2] + (2*-M_PI);

  if(int(norm_angles.size()) < num_joints_)
    SBPL_DEBUG_NAMED(debug_stream_,"Joint array has %d joints. (should be %d joints)\n", int(norm_angles.size()),num_joints_);

  for(int i = 0; i < num_joints_; ++i)
  {
    if(!joints_[i].continuous)
    {
      if(joints_[i].min > norm_angles[i] || norm_angles[i] > joints_[i].max)
      {
        //if(verbose)
        //  SBPL_DEBUG_NAMED(debug_stream_, "Joint %d is invalid with value %0.3f (limits are {%0.3f, %0.3f}.\n",i,norm_angles[i],joints_[i].min,joints_[i].max);
        return false;
      }
    }
  }
  return true;
}

void SBPLArmModel::printArmDescription(std::string stream)
{
  SBPL_DEBUG_NAMED(stream, "\nSBPL Arm Description:");
  SBPL_DEBUG_NAMED(stream, "# Joints: %d,   # Links: %d",num_joints_,num_links_);
  SBPL_DEBUG_NAMED(stream, "Root Frame: %s, Tip Frame: %s", chain_root_name_.c_str(),chain_tip_name_.c_str());
  SBPL_DEBUG_NAMED(stream, "Joint Limits:");
  for(int i = 0; i < num_joints_; ++i)
    SBPL_DEBUG_NAMED(stream, "%d: {%.3f, %0.3f}",i,joints_[i].min,joints_[i].max);

  SBPL_DEBUG_NAMED(stream,"Joints ");
  for(int i =0; i < num_joints_; ++i)
  {
    if(joints_[i].continuous)
      SBPL_DEBUG_NAMED(stream, "%d ",i);
  }
  SBPL_DEBUG_NAMED(stream, "are continuous.");

  SBPL_DEBUG_NAMED(stream, "Links:");
  for(int i = 0; i < num_links_; ++i)
    SBPL_DEBUG_NAMED(stream, "(%d) radius: %0.3fm (%d cells)  length: %0.3fm   KDL chain index: %d",i,links_[i].radius,links_[i].radius_c, links_[i].length,links_[i].ind_chain);

  SBPL_DEBUG_NAMED(stream,"\nKDL Arm Chain:");
  SBPL_DEBUG_NAMED(stream, "# segments: %d, # joints: %d", chain_.getNrOfSegments(), chain_.getNrOfJoints());

  double jnt_pos = 0;
  for(size_t j = 0; j < chain_.getNrOfSegments(); ++j)
  {
    SBPL_DEBUG_NAMED(stream, "frame %2d: segment: %0.3f %0.3f %0.3f  joint: %0.3f %0.3f %0.3f   joint_type: %s",int(j),
        chain_.getSegment(j).pose(jnt_pos).p.x(),
        chain_.getSegment(j).pose(jnt_pos).p.y(),
        chain_.getSegment(j).pose(jnt_pos).p.z(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.x(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.y(),
        chain_.getSegment(j).getJoint().pose(jnt_pos).p.z(),
        chain_.getSegment(j).getJoint().getTypeName().c_str());
  }
}

void SBPLArmModel::getJointLimits(int joint_num, double* min, double *max)
{
  *min = joints_[joint_num].min;
  *max = joints_[joint_num].max;
}

void SBPLArmModel::setRefFrameTransform(KDL::Frame f, std::string &name)
{
  reference_frame_ = name;

  transform_ = f;
  transform_inverse_ = transform_.Inverse();

  ROS_DEBUG("Transform  %s to torso: x: %0.4f y: %0.4f z: %0.4f", name.c_str(),transform_.p.x(),transform_.p.y(),transform_.p.z()); 
  ROS_DEBUG("Inverse Transform torso to %s: x: %0.4f y: %0.4f z: %0.4f", name.c_str(),transform_inverse_.p.x(),transform_inverse_.p.y(),transform_inverse_.p.z()); 
}

void SBPLArmModel::getArmChainRootLinkName(std::string &name)
{
  name = chain_root_name_;
}

}
