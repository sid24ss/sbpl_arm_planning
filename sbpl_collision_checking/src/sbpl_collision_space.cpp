/*
 * Copyright (c) 2011, Maxim Likhachev
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

#include <sbpl_collision_checking/sbpl_collision_space.h>

#define SMALL_NUM  0.00000001     // to avoid division overflow

namespace sbpl_arm_planner 
{

SBPLCollisionSpace::SBPLCollisionSpace(sbpl_arm_planner::OccupancyGrid* grid)
{
  grid_ = grid;
  fOut_ = stdout;
  object_attached_ = false;

  padding_ = 0.03;

  num_planning_joints_ = 7;

  // size of increments for when doing path interpolation
  inc_.resize(num_planning_joints_,0.0348);
  inc_[5] = 0.1392; // 8 degrees
  inc_[6] = M_PI;

  ROS_INFO("Setting the planning joints.");
  planning_joints_.push_back("r_shoulder_pan_joint");
  planning_joints_.push_back("r_shoulder_lift_joint");
  planning_joints_.push_back("r_upper_arm_roll_joint");
  planning_joints_.push_back("r_elbow_flex_joint");
  planning_joints_.push_back("r_forearm_roll_joint");
  planning_joints_.push_back("r_wrist_flex_joint");
  planning_joints_.push_back("r_wrist_roll_joint");
}

void SBPLCollisionSpace::setDebugFile(FILE* file_ptr)
{
  fOut_ = file_ptr;
}

bool SBPLCollisionSpace::init(std::string group_name)
{
  group_name_ = group_name;

  // initialize the collision model
  if(!model_.init())
  {
    ROS_ERROR("[cspace] The robot's collision model failed to initialize.");
    return false;
  }
  
  // initialize all collision groups (don't really need to)
  if(!model_.initAllGroups())
  {
    ROS_ERROR("[cspace] Failed to initialize the collision groups.");
    return false;
  }
  
  // set the order of the planning joints
  model_.setOrderOfJointPositions(planning_joints_, group_name_);

  // choose the group we are planning for
  model_.setDefaultGroup(group_name_);

  // get the collision spheres for the robot
  model_.getDefaultGroupSpheres(spheres_);

  model_.printGroups();

  return true;
}

bool SBPLCollisionSpace::checkCollision(const std::vector<double> &angles, bool verbose, bool check_mesh, unsigned char &dist)
{
  unsigned char dist_temp=100;
  dist = 100;
  KDL::Vector v;
  Sphere s;
  int x,y,z;
  bool in_collision = false;

  ROS_DEBUG("[cspace] Computing FK...");
  // compute foward kinematics
  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  collision_spheres_.clear(); // just for demo

  ROS_DEBUG("[cspace] Checking spheres...");
  // transform the spheres into their new positions, check collisions
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;

    grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

    //model_.printFrames(frames_);

    ROS_DEBUG("[%2d] chain: %d segment: %2d origin: {%2.2f %2.2f %2.2f}  sphere: {%3.2f %3.2f %2.2f} radius: %2.2f priority: %d", int(i), spheres_[i]->kdl_chain, spheres_[i]->kdl_segment, frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment].p.x(), frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment].p.y(), frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment].p.z(), v.x(), v.y(), v.z(), spheres_[i]->radius, spheres_[i]->priority);

    // check bounds
    if(!grid_->isInBounds(x, y, z))
    {
      if(verbose)
        ROS_INFO("Sphere %d %d %d is out of bounds.", x, y, z);
      return false;
    }

    // check for collision with world
    if((dist_temp = grid_->getCell(x,y,z)) <= int((spheres_[i]->radius + padding_) / grid_->getResolution() + 0.5))
    {
      dist = dist_temp;

      // for collision space demo only - remove 
      in_collision = true;
      s = *(spheres_[i]); 
      s.v = v;
      collision_spheres_.push_back(s);
      //return false;
    }

    ROS_DEBUG("x: %d y: %d z: %d radius: %0.3f (%d)  dist: %d (dist_min: %d)", x, y, z, spheres_[i]->radius,  int(spheres_[i]->radius / grid_->getResolution() + 0.5), grid_->getCell(x,y,z), int(dist));
    if(dist_temp < dist)
      dist = dist_temp;
  }

  //check attached object for collision
  if(!isValidAttachedObject(angles, dist_temp))
  {
    dist = dist_temp;
    return false;
  }

  if(object_attached_ && dist_temp < dist)
      dist = dist_temp;

  // temporarily just for debugging
  if(in_collision)
    return false;

  return true;
}

bool SBPLCollisionSpace::checkLinkForCollision(const std::vector<double> &angles, int link_num, bool verbose, unsigned char &dist)
{
/*
  unsigned char dist_temp = 0;
  std::vector<std::vector<int> > jnts;
  KDL::Frame f_out;

  if(link_num >= arm_->num_links_)
  {
    ROS_WARN("[checkLinkInCollision] %d is not a valid link index. There are %d links.", link_num, arm_->num_links_);
    return false;
  }
  
  //get position of joints in the occupancy grid
  if(!getJointPosesInGrid(angles, jnts, f_out, false))
    return false;

  //check bounds
  if(!grid_->isInBounds(jnts[link_num][0],jnts[link_num][1],jnts[link_num][2]))
  {
    if(verbose)
      ROS_DEBUG("End of link %d is out of bounds. (%d %d %d)", link_num, jnts[link_num][0],jnts[link_num][1],jnts[link_num][2]);
    return false;
  }

  //is link in collision?
  dist = isValidLineSegment(jnts[link_num], jnts[link_num+1], arm_->getLinkRadiusCells(link_num));

  //if the line's distance to the nearest obstacle is less than the radius
  if(dist <= arm_->getLinkRadiusCells(link_num))
  {
    if(verbose)
      ROS_DEBUG("Link %d: {%d %d %d} -> {%d %d %d} with radius %0.2f is in collision.",link_num,jnts[link_num][0],jnts[link_num][1],jnts[link_num][2],jnts[link_num+1][0],jnts[link_num+1][1],jnts[link_num+1][2],arm_->getLinkRadius(link_num));
    return false;
  }

  //check attached object for collision with world and upper_arm
  if(!isValidAttachedObject(angles, dist_temp, jnts[0], jnts[1]))
  {
    if(verbose)
      ROS_DEBUG("Attached object is in collision.");
    dist = dist_temp;
    return false;
  }

  if(dist_temp < dist)
  {
    if(verbose)
      ROS_DEBUG("Attached object is the closest thing to the nearest obstacle with dist=%d", int(dist_temp));
    dist = dist_temp;
  }
*/
  return true;
}

bool SBPLCollisionSpace::checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, unsigned char &dist)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  dist = 100;

  for(size_t i=0; i < start.size(); i++)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }
  
  if(start[2] > 0.85)
    start_norm[2] = start[2] + (2*-M_PI);

  if(end[2] > 0.85)
    end_norm[2] = end[2] + (2*-M_PI);

  getInterpolatedPath(start_norm, end_norm, inc_, path);

  /*
  if(verbose)
  {
    ROS_FPRINTF(fOut_,"      interpolated_path: %d waypoints\n",path.size());
    ROS_FPRINTF(fOut_,"           %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f --> %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", start_norm[0],start_norm[1],start_norm[2],start_norm[3],start_norm[4],start_norm[5],start_norm[6], end_norm[0],end_norm[1],end_norm[2],end_norm[3],end_norm[4],end_norm[5],end_norm[6]);

    for(int i = 0; i < int(path.size()); ++i)
        ROS_FPRINTF(fOut_,"           %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", path[i][0],path[i][1],path[i][2],path[i][3],path[i][4],path[i][5],path[i][6]);
  }
  */

  // 'optimized' collision check added 8/28/2010
  // try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        if(!checkCollision(path[j], verbose, false, dist_temp))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      if(!checkCollision(path[i], verbose, false, dist_temp))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  return true;
}

bool SBPLCollisionSpace::checkLinkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, int link_num, bool verbose, unsigned char &dist)
{
  int inc_cc = 10;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  dist = 100;

  for(size_t i=0; i < start.size(); i++)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }
 
  //problem with upper_arm_roll  
  if(start[2] > 0.85)
    start_norm[2] = start[2] + (2*-M_PI);

  if(end[2] > 0.85)
    end_norm[2] = end[2] + (2*-M_PI);

  getInterpolatedPath(start_norm, end_norm, inc_, path);

  //try to find collisions that might come later in the path earlier
  if(int(path.size()) > inc_cc)
  {
    for(int i = 0; i < inc_cc; i++)
    {
      for(size_t j = i; j < path.size(); j=j+inc_cc)
      {
        if(!checkLinkForCollision(path[j], link_num, verbose, dist_temp))
        {
          dist = dist_temp;
          return false; 
        }

        if(dist_temp < dist)
          dist = dist_temp;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < path.size(); i++)
    {
      if(!checkLinkForCollision(path[i], link_num, verbose, dist_temp))
      {
        dist = dist_temp;
        return false;
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  return true;
}

unsigned char SBPLCollisionSpace::isValidLineSegment(const std::vector<int> a, const std::vector<int> b, const short unsigned int radius)
{
  bresenham3d_param_t params;
  int nXYZ[3], retvalue = 1;
  unsigned char cell_val, min_dist = 255;
  CELL3V tempcell;
  vector<CELL3V>* pTestedCells=NULL;

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    if(!grid_->isInBounds(nXYZ[0],nXYZ[1],nXYZ[2]))
      return 0;

    cell_val = grid_->getCell(nXYZ[0],nXYZ[1],nXYZ[2]);
    if(cell_val <= radius)
    {
      if(pTestedCells == NULL)
        return cell_val;   //return 0
      else
        retvalue = 0;
    }

    if(cell_val < min_dist)
      min_dist = cell_val;

    //insert the tested point
    if(pTestedCells)
    {
      if(cell_val <= radius)
        tempcell.bIsObstacle = true;
      else
        tempcell.bIsObstacle = false;
      tempcell.x = nXYZ[0];
      tempcell.y = nXYZ[1];
      tempcell.z = nXYZ[2];
      pTestedCells->push_back(tempcell);
    }
  } while (get_next_point3d(&params));

  if(retvalue)
    return min_dist;
  else
    return 0;
}

double SBPLCollisionSpace::distanceBetween3DLineSegments(std::vector<int> l1a, std::vector<int> l1b,std::vector<int> l2a, std::vector<int> l2b)
{
  // Copyright 2001, softSurfer (www.softsurfer.com)
  // This code may be freely used and modified for any purpose
  // providing that this copyright notice is included with it.
  // SoftSurfer makes no warranty for this code, and cannot be held
  // liable for any real or imagined damage resulting from its use.
  // Users of this code must verify correctness for their application.

  double u[3];
  double v[3];
  double w[3];
  double dP[3];

  u[0] = l1b[0] - l1a[0];
  u[1] = l1b[1] - l1a[1];
  u[2] = l1b[2] - l1a[2];

  v[0] = l2b[0] - l2a[0];
  v[1] = l2b[1] - l2a[1];
  v[2] = l2b[2] - l2a[2];

  w[0] = l1a[0] - l2a[0];
  w[1] = l1a[1] - l2a[1];
  w[2] = l1a[2] - l2a[2];

  double a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // always >= 0
  double b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // dot(u,v);
  double c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // dot(v,v);        // always >= 0
  double d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // dot(u,w);
  double e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // dot(v,w);
  double D = a*c - b*b;       // always >= 0
  double sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
  double tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) { // the lines are almost parallel
    sN = 0.0;        // force using point P0 on segment S1
    sD = 1.0;        // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  }
  else {                // get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);
    if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    }
    else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0)
      sN = 0.0;
    else if (-d > a)
      sN = sD;
    else {
      sN = -d;
      sD = a;
    }
  }
  else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0)
      sN = 0;
    else if ((-d + b) > a)
      sN = sD;
    else {
      sN = (-d + b);
      sD = a;
    }
  }

  // finally do the division to get sc and tc
  sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
  tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

  // get the difference of the two closest points
  // dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

  dP[0] = w[0] + (sc * u[0]) - (tc * v[0]);
  dP[1] = w[1] + (sc * u[1]) - (tc * v[1]);
  dP[2] = w[2] + (sc * u[2]) - (tc * v[2]);

  return  sqrt(dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);   // return the closest distance
}

void SBPLCollisionSpace::addArmCuboidsToGrid()
{
/*
  std::vector<std::vector<double> > cuboids = arm_->getCollisionCuboids();

  ROS_DEBUG("[SBPLCollisionSpace] received %d cuboids\n",int(cuboids.size()));

  for(unsigned int i = 0; i < cuboids.size(); i++)
  {
    if(cuboids[i].size() == 6)
      grid_->addCollisionCuboid(cuboids[i][0],cuboids[i][1],cuboids[i][2],cuboids[i][3],cuboids[i][4],cuboids[i][5]);
    else
      ROS_DEBUG("[addArmCuboidsToGrid] Self-collision cuboid #%d has an incomplete description.\n", i);
  }
*/
}

bool SBPLCollisionSpace::getCollisionCylinders(const std::vector<double> &angles, std::vector<std::vector<double> > &cylinders)
{
  std::vector<double> xyzr(4,0);
  std::vector<std::vector<double> > object;
  KDL::Vector v;

  // compute foward kinematics
  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  // get spheres of the robot
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;
  
    xyzr[0] = v.x();
    xyzr[1] = v.y();
    xyzr[2] = v.z();
    xyzr[3] = spheres_[i]->radius;
    
    cylinders.push_back(xyzr);
  }

  // get spheres of the object
  if(object_attached_)
  {
    getAttachedObject(angles, object);
    for(size_t i = 0; i < object.size(); ++i)
    {
      xyzr[0] = object[i][0];
      xyzr[1] = object[i][1];
      xyzr[2] = object[i][2];
      xyzr[3] = double(attached_object_radius_) * grid_->getResolution();
      cylinders.push_back(xyzr);
    }
  }

  return true;
}

void SBPLCollisionSpace::getLineSegment(const std::vector<int> a,const std::vector<int> b,std::vector<std::vector<int> > &points){
  bresenham3d_param_t params;
  std::vector<int> nXYZ(3,0);

  //iterate through the points on the segment
  get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
  do {
    get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

    points.push_back(nXYZ);

  } while (get_next_point3d(&params));
}

void SBPLCollisionSpace::getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, std::vector<double> &inc, std::vector<std::vector<double> > &path)
{
  bool changed = true; 
  std::vector<double> next(start);
  
  //check if both input configurations have same size
  if(start.size() != end.size())
  {
    ROS_WARN("[getInterpolatedPath] The start and end configurations have different sizes.\n");
    return;
  }

  while(changed)
  {
    changed = false;

    for (int i = 0; i < int(start.size()); i++) 
    {
      if (fabs(next[i] - end[i]) > inc[i]) 
      {
        changed = true;
        
        if(end[i] > next[i]) 
          next[i] += inc[i];
        else
          next[i] += -inc[i];
      }
    }

    if (changed)
      path.push_back(next);
  }
}

void SBPLCollisionSpace::removeAttachedObject()
{
  object_attached_ = false;
  object_points_.clear();
  ROS_INFO("[removeAttachedObject] Removed attached object.");
}

void SBPLCollisionSpace::attachSphereToGripper(std::string frame, geometry_msgs::Pose pose, double radius)
{
  object_attached_ = true;
  attached_object_frame_ = frame;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);
  
  ROS_INFO("[addSphereToGripper] Pose of Sphere: %0.3f %0.3f %0.3f radius: %0.3f", pose.position.x,pose.position.y,pose.position.z, radius); 
  attached_object_radius_ = radius / grid_->getResolution();
  
  object_points_.resize(1);
  tf::PoseMsgToKDL(pose, object_points_[0]);

  ROS_DEBUG("[addSphereToGripper] Added collision sphere.  xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[0].p.x(), object_points_[0].p.y(), object_points_[0].p.z(), radius, attached_object_radius_);
}

void SBPLCollisionSpace::attachCylinderToGripper(std::string frame, geometry_msgs::Pose pose, double radius, double length)
{
  object_attached_ = true;
  attached_object_frame_ = frame;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);
  
  ROS_DEBUG("[addCylinderToGripper] Cylinder: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f", pose.position.x,pose.position.y,pose.position.z, radius, length); 

  attached_object_radius_ = radius / grid_->getResolution();

  if(attached_object_radius_ < 1)
    attached_object_radius_ = 1;

  object_points_.resize(2);
  tf::PoseMsgToKDL(pose, object_points_[0]);
  tf::PoseMsgToKDL(pose, object_points_[1]);

  //compute the endpoints of the cylinder
  object_points_[0].p.data[2] -= length/2.0;
  object_points_[1].p.data[2] += length/2.0;

  ROS_DEBUG("[addCylinderToGripper] Added cylinder.  Bottom: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[0].p.x(), object_points_[0].p.y(), object_points_[0].p.z(), radius, attached_object_radius_);
  ROS_DEBUG("[addCylinderToGripper] Added cylinder.     Top: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[1].p.x(), object_points_[1].p.y(), object_points_[1].p.z(), radius, attached_object_radius_);
}

void SBPLCollisionSpace::attachMeshToGripper(const std::string frame, const geometry_msgs::Pose pose, const std::vector<int32_t> &triangles, const std::vector<geometry_msgs::Point> &vertices)
{
  bodies::BoundingCylinder cyl;
  getBoundingCylinderOfMesh(triangles, vertices, cyl);
  ROS_INFO("HERE");
  attachMeshToGripper(frame, pose, cyl);
}

void SBPLCollisionSpace::attachMeshToGripper(const std::string frame, const geometry_msgs::Pose pose, const bodies::BoundingCylinder &cyl)
{
  KDL::Frame cyl_pose;

  object_attached_ = true;
  attached_object_frame_ = frame;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);
  
  ROS_INFO("[addMeshToGripper] Mesh: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f", pose.position.x,pose.position.y,pose.position.z, cyl.radius, cyl.length); 

  attached_object_radius_ = cyl.radius / grid_->getResolution();

  if(attached_object_radius_ < 1)
    attached_object_radius_ = 1;

  //get pose of cylinder in gripper frame
  object_points_.resize(2);
  ROS_INFO("Object Pose-  Position: %0.3f %0.3f %0.3f  Orientation: %0.3f %0.3f %0.3f %0.3f",pose.position.x,pose.position.y,pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

  tf::PoseMsgToKDL(pose, object_points_[0]);
  tf::PoseMsgToKDL(pose, object_points_[1]);

  //compute the endpoints of the cylinder
  //object_points_[0].p.data[2] -= cyl.length/2.0;
  //object_points_[1].p.data[2] += cyl.length/2.0;

  ROS_INFO("BEFORE TRANSFORMATION"); 
  ROS_INFO("[addMeshToGripper] Added bounding cylinder.  Bottom: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[0].p.x(), object_points_[0].p.y(), object_points_[0].p.z(), cyl.radius, attached_object_radius_);
  ROS_INFO("[addMeshToGripper] Added bounding cylinder.     Top: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[1].p.x(), object_points_[1].p.y(), object_points_[1].p.z(), cyl.radius, attached_object_radius_);
 
  tf::TransformTFToKDL(cyl.pose, cyl_pose); 
//  object_points_[0] = object_points_[0]*cyl_pose;
//  object_points_[1] = object_points_[1]*cyl_pose;

  object_points_[0].p.data[2] -= cyl.length/2.0;
  object_points_[1].p.data[2] += cyl.length/2.0;
 
  ROS_INFO("AFTER TRANSFORMATION"); 
  ROS_INFO("[addMeshToGripper] Added bounding cylinder.  Bottom: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[0].p.x(), object_points_[0].p.y(), object_points_[0].p.z(), cyl.radius, attached_object_radius_);
  ROS_INFO("[addMeshToGripper] Added bounding cylinder.     Top: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[1].p.x(), object_points_[1].p.y(), object_points_[1].p.z(), cyl.radius, attached_object_radius_);
}

bool SBPLCollisionSpace::isValidAttachedObject(const std::vector<double> &angles, unsigned char &dist, std::vector<int> j1, std::vector<int> j2)
{
  // TODO: This function ignores the inputs j1, j2 for now. 

  unsigned char dist_temp = 0;
  KDL::Vector p_base,p1,p2;
  std::vector<int> q1(3,0), q2(3,0);
  dist = 100;

  if(!object_attached_)
    return true;

  if(!model_.computeDefaultGroupFK(angles, frames_))
    return false;

  // check if the object points are valid
  for(size_t i = 0; i < object_points_.size(); i++)
  {
    p_base = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i].p;

    if(!isValidPoint(p_base.data[0],p_base.data[1],p_base.data[2],attached_object_radius_,dist))
      return false;
  }

  // check if the cylinder between the points is valid
  for(size_t i = 0; i < (object_points_.size()-1); i++)
  {
    p1 = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i].p;
    p2 = frames_[attached_object_chain_num_][attached_object_segment_num_]* object_points_[i+1].p;

    grid_->worldToGrid(p1.data[0],p1.data[1],p1.data[2],q1[0],q1[1],q1[2]);
    grid_->worldToGrid(p2.data[0],p2.data[1],p2.data[2],q2[0],q2[1],q2[2]);
      
    dist_temp = isValidLineSegment(q1, q2, attached_object_radius_);

    if(dist_temp <= attached_object_radius_)
    {
      dist = dist_temp;
      return false;
    }

    if(dist_temp < dist)
      dist = dist_temp;
  }

  return true;
}

bool SBPLCollisionSpace::isValidAttachedObject(const std::vector<double> &angles, unsigned char &dist)
{
  unsigned char dist_temp = 0;
  KDL::Vector p_base,p1,p2;
  std::vector<int> q1(3,0), q2(3,0);

  if(!object_attached_)
    return true;

  if(!model_.computeDefaultGroupFK(angles, frames_))
    return false;

  // check if the object points are valid
  for(size_t i = 0; i < object_points_.size(); i++)
  {
    p_base = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i].p;

    if(!isValidPoint(p_base.data[0],p_base.data[1],p_base.data[2],attached_object_radius_,dist))
      return false;
  }

  // check if the cylinder between the points is valid
  for(size_t i = 0; i < (object_points_.size()-1); i++)
  {
    p1 = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i].p;
    p2 = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i+1].p;

    grid_->worldToGrid(p1.data[0],p1.data[1],p1.data[2],q1[0],q1[1],q1[2]);
    grid_->worldToGrid(p2.data[0],p2.data[1],p2.data[2],q2[0],q2[1],q2[2]);
      
    dist_temp = isValidLineSegment(q1, q2, attached_object_radius_);

    if(dist_temp <= attached_object_radius_)
    { 
      dist = dist_temp;
      return false;
    }

    if(dist_temp < dist)
      dist = dist_temp;
  }

  return true;
}

bool SBPLCollisionSpace::isValidPoint(double &x, double &y, double &z, short unsigned int &radius, unsigned char &dist)
{
  int xyz_c[3]={0};

  grid_->worldToGrid(x,y,z,xyz_c[0], xyz_c[1], xyz_c[2]);
 
  ROS_DEBUG("world: %0.3f %0.3f %0.3f  grid: %d %d %d  dist: %u radius: %u",x, y, z, xyz_c[0], xyz_c[1], xyz_c[2], grid_->getCell(xyz_c[0],xyz_c[1],xyz_c[2]), radius);

  if((dist = grid_->getCell(xyz_c[0],xyz_c[1],xyz_c[2])) <= radius)
    return false;

  return true;
}

bool SBPLCollisionSpace::getAttachedObject(const std::vector<double> &angles, std::vector<std::vector<double> > &xyz)
{
  KDL::Vector p_base,p1,p2;
  std::vector<double> point(6,0);
  std::vector<int> q1(3,0), q2(3,0);
  std::vector<std::vector<int> > cells;

  if(!object_attached_)
    return false;

  if(!model_.computeDefaultGroupFK(angles, frames_))
    return false;

  xyz.resize(object_points_.size(), std::vector<double>(6,0));
  for(size_t i = 0; i < object_points_.size(); i++)
  {
    p_base =  frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i].p;
    xyz[i][0] = p_base.data[0];
    xyz[i][1] = p_base.data[1];
    xyz[i][2] = p_base.data[2];
  }

  // check if the cylinder between the points is valid
  // change this: the top and bottom points are being added twice
  for(size_t i = 0; i < (object_points_.size()-1); i++)
  {
    p1 = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i].p;
    p2 = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i+1].p;

    grid_->worldToGrid(p1.data[0],p1.data[1],p1.data[2],q1[0],q1[1],q1[2]);
    grid_->worldToGrid(p2.data[0],p2.data[1],p2.data[2],q2[0],q2[1],q2[2]);

    getLineSegment(q1, q2, cells);
    
    for(size_t j = 0; j < cells.size();j++)
    {
      grid_->gridToWorld(cells[j][0],cells[j][1],cells[j][2],point[0],point[1],point[2]);
      xyz.push_back(point);
    }
  }

  return true;
}

double SBPLCollisionSpace::getAttachedObjectRadius()
{
  return attached_object_radius_*grid_->getResolution();   
}

bool SBPLCollisionSpace::getBoundingCylinderOfMesh(std::string mesh_file, shapes::Shape *mesh, bodies::BoundingCylinder &cyl)
{
  bool retval = false;
  mesh = NULL;

  if (!mesh_file.empty())
  { 
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    bool ok = true;

    try
    { 
      res = retriever.get(mesh_file);
    }
    catch (resource_retriever::Exception& e)
    { 
      ROS_ERROR("%s", e.what());
      ok = false;
    }

    if (ok)
    { 
      if (res.size == 0)
        ROS_WARN("Retrieved empty mesh for resource '%s'", mesh_file.c_str());
      else
      { 
        mesh = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
        if (mesh == NULL)
          ROS_ERROR("Failed to load mesh '%s'", mesh_file.c_str());
        else
          retval = true;
      }
    }
  }
  else
    ROS_WARN("Empty mesh filename");

  if(retval)
  {
    const bodies::Body *body=new bodies::ConvexMesh(mesh);
    body->computeBoundingCylinder(cyl);
    ROS_INFO("Bounding cylinder has radius: %0.3f  length: %0.3f", cyl.radius, cyl.length);
  }
  
  return retval;
}

void SBPLCollisionSpace::getBoundingCylinderOfMesh(const std::vector<int32_t> &triangles, const std::vector<geometry_msgs::Point> &vertices, bodies::BoundingCylinder &cyl)
{
  shapes::Shape *mesh = NULL;

  shapes::Mesh *dest = new shapes::Mesh(vertices.size(), triangles.size()/3);
  for (size_t i = 0 ; i < vertices.size(); ++i)
  {
    dest->vertices[3*i] = vertices[i].x;
    dest->vertices[3*i+1] = vertices[i].y;
    dest->vertices[3*i+2] = vertices[i].z;
  }

  for (unsigned int i = 0 ; i < triangles.size(); ++i)
    dest->triangles[i] = triangles[i];
  
  mesh = dest;

  const bodies::Body *body=new bodies::ConvexMesh(mesh);
  body->computeBoundingCylinder(cyl);
  ROS_INFO("Bounding cylinder has radius: %0.3f  length: %0.3f", cyl.radius, cyl.length);
}

void SBPLCollisionSpace::processCollisionObjectMsg(const mapping_msgs::CollisionObject &object)
{
  if(object.operation.operation == mapping_msgs::CollisionObjectOperation::ADD)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == mapping_msgs::CollisionObjectOperation::REMOVE)
  {
    if(object.id.compare("all") == 0)
      removeAllCollisionObjects();
    else
      removeCollisionObject(object);
  }
  else if(object.operation.operation == mapping_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == mapping_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
  {
    //TODO: Attach to gripper
    removeCollisionObject(object);
  }
  else
    ROS_WARN("*** Operation isn't supported. ***\n\n");
}

void SBPLCollisionSpace::addCollisionObject(const mapping_msgs::CollisionObject &object)
{
  geometry_msgs::Pose pose;

  for(size_t i = 0; i < object.shapes.size(); ++i)
  {
    if(object.shapes[i].type == geometric_shapes_msgs::Shape::BOX)
    {
      transformPose(object.header.frame_id, grid_->getReferenceFrame(), object.poses[i], pose);
      grid_->getVoxelsInBox(pose, object.shapes[i].dimensions, object_voxel_map_[object.id]);
      
      ROS_DEBUG("[%s] TransformPose from %s to %s.", object.id.c_str(), object.header.frame_id.c_str(), grid_->getReferenceFrame().c_str());
      ROS_DEBUG("[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), object.header.frame_id.c_str(), object.poses[i].position.x,  object.poses[i].position.y, object.poses[i].position.z, object.poses[i].orientation.x, object.poses[i].orientation.y, object.poses[i].orientation.z,object.poses[i].orientation.w);
      ROS_DEBUG("[%s] %s: xyz: %0.3f %0.3f %0.3f   quat: %0.3f %0.3f %0.3f %0.3f", object.id.c_str(), grid_->getReferenceFrame().c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      ROS_DEBUG("[%s] occupies %d voxels.",object.id.c_str(), int(object_voxel_map_[object.id].size()));
    }
    else
      ROS_WARN("[addCollisionObject] Collision objects of type %d are not yet supported.", object.shapes[i].type);
  }

  // add this object to list of objects that get added to grid
  bool new_object = true;
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
      new_object = false;
  }
  if(new_object)
    known_objects_.push_back(object.id);

  grid_->addPointsToField(object_voxel_map_[object.id]);
  ROS_DEBUG("[addCollisionObject] Just added %s to the distance field.", object.id.c_str());
}

void SBPLCollisionSpace::removeCollisionObject(const mapping_msgs::CollisionObject &object)
{
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
    {
      known_objects_.erase(known_objects_.begin() + i);
      ROS_DEBUG("[removeCollisionObject] Removing %s from list of known objects.", object.id.c_str());
    }
  }
}

void SBPLCollisionSpace::removeAllCollisionObjects()
{
  known_objects_.clear();
}

void SBPLCollisionSpace::putCollisionObjectsInGrid()
{
  ROS_DEBUG("[putCollisionObjectsInGrid] Should we reset first?");

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    grid_->addPointsToField(object_voxel_map_[known_objects_[i]]);
    ROS_DEBUG("[putCollisionObjectsInGrid] Added %s to grid with %d voxels.",known_objects_[i].c_str(), int(object_voxel_map_[known_objects_[i]].size()));
  }
}

void SBPLCollisionSpace::getCollisionObjectVoxelPoses(std::vector<geometry_msgs::Pose> &points)
{
  geometry_msgs::Pose pose;

  pose.orientation.x = 0; 
  pose.orientation.y = 0;
  pose.orientation.z = 0; 
  pose.orientation.w = 1;

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    for(size_t j = 0; j < object_voxel_map_[known_objects_[i]].size(); ++j)
    {
      pose.position.x = object_voxel_map_[known_objects_[i]][j].x();
      pose.position.y = object_voxel_map_[known_objects_[i]][j].y();
      pose.position.z = object_voxel_map_[known_objects_[i]][j].z();
      points.push_back(pose);
    }
  }
}

void SBPLCollisionSpace::transformPose(const std::string &current_frame, const std::string &desired_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
{
  geometry_msgs::PoseStamped stpose_in, stpose_out;
  stpose_in.header.frame_id = current_frame;
  stpose_in.header.stamp = ros::Time();
  stpose_in.pose = pose_in;
  tf_.transformPose(desired_frame, stpose_in, stpose_out);
  pose_out = stpose_out.pose;
}

void SBPLCollisionSpace::setJointPosition(std::string name, double position)
{
  ROS_DEBUG("[cspace] Setting %s joint position = %0.3f.", name.c_str(), position);
  model_.setJointPosition(name, position);
}

}
