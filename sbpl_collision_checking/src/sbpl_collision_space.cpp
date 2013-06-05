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

double distance(const KDL::Vector &a, const KDL::Vector &b)
{
  return sqrt((a.x()-b.x())*(a.x()-b.x()) + (a.y()-b.y())*(a.y()-b.y()) + (a.z()-b.z())*(a.z()-b.z()));
}

SBPLCollisionSpace::SBPLCollisionSpace(sbpl_arm_planner::OccupancyGrid* grid)
{
  grid_ = grid;
  fOut_ = stdout;
  object_attached_ = false;
  padding_ = 0.00;
  num_planning_joints_ = 7;
  num_collision_checks_ = 0;
  num_false_collision_checks_ = 0;

  // size of increments for when doing path interpolation
  inc_.resize(num_planning_joints_,0.0348);
  inc_[5] = 0.06981;
  inc_[6] = 0.06981;

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

void SBPLCollisionSpace::setPadding(double padding)
{
  padding_ = padding;
}

void SBPLCollisionSpace::setPlanningJoints(const std::vector<std::string> &joint_names)
{
  planning_joints_ = joint_names;

  //TODO: should call setOrderOfJointPositions() ?
  //TODO: set inc_ vector
  //TODO: set num_planning_joints_
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

  // visualizer for debugging
  aviz_ = new VisualizeArm(group_name_);
  std::string frame = "base_footprint";
  aviz_->setReferenceFrame(frame);

  return true;
}

bool SBPLCollisionSpace::checkCollision(const std::vector<double> &angles, bool verbose, bool visualize, unsigned char &dist)
{
  // 'visualize' is not being used right now

  unsigned char dist_temp=100;
  dist = 100;
  KDL::Vector v;
  int x,y,z;

  // for debugging
  num_collision_checks_++;

  //clock_t fk_start = clock();
  // compute foward kinematics
  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }
  //double fk_time = (clock() - fk_start) / double(CLOCKS_PER_SEC);

  // check attached object
  if(object_attached_)
  {
    for(size_t i = 0; i < object_spheres_.size(); ++i)
    {
      v = frames_[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] /* * attached_object_pose_*/ * object_spheres_[i].v;

      grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

      //ROS_INFO("x: %d y: %d z: %d radius: %0.3f (%d)  dist: %d (dist_min: %d)", x, y, z, object_spheres_[i].radius,  int(object_spheres_[i].radius / grid_->getResolution() + 0.5), grid_->getCell(x,y,z), int(dist));

      // check bounds
      if(!grid_->isInBounds(x, y, z))
      {
        if(verbose)
          ROS_INFO("[cspace] Sphere %d %d %d is out of bounds.", x, y, z);
        return false;
      }

      // check for collision with world
      if((dist_temp = grid_->getCell(x,y,z)) <= int((object_spheres_[i].radius) / grid_->getResolution() + 0.5))
      {
        dist = dist_temp;
        return false;
      }
      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  // check arm
  //clock_t spheres_start = clock();
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;

    grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

    // check bounds
    if(!grid_->isInBounds(x, y, z))
    {
      if(verbose)
        ROS_INFO("[cspace] Sphere %d %d %d is out of bounds.", x, y, z);
      return false;
    }

    // check for collision with world
    if((dist_temp = grid_->getCell(x,y,z)) <= int((spheres_[i]->radius + padding_) / grid_->getResolution() + 0.5))
    {
      dist = dist_temp;
      return false;
    }
    if(dist_temp < dist)
      dist = dist_temp;
  }

  //printf("kinematics: %0.8fsec  spheres: %0.8fsec\n", fk_time, (clock() - spheres_start) / double(CLOCKS_PER_SEC));

  num_false_collision_checks_++;
  return true;
}

bool SBPLCollisionSpace::checkCollisionWithVisualizations(const std::vector<double> &angles, unsigned char &dist)
{
  unsigned char dist_temp=100;
  dist = 100;
  KDL::Vector v;
  Sphere s;
  int x,y,z;
  bool in_collision = false;

  // compute foward kinematics
  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  collision_spheres_.clear();

  // transform the spheres into their new positions, check collisions
  for(size_t i = 0; i < spheres_.size(); ++i)
  {
    v = frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment] * spheres_[i]->v;

    grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

    ROS_DEBUG("[%2d] chain: %d segment: %2d origin: {%2.2f %2.2f %2.2f}  sphere: {%3.2f %3.2f %2.2f} radius: %2.2f priority: %d", int(i), spheres_[i]->kdl_chain, spheres_[i]->kdl_segment, frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment].p.x(), frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment].p.y(), frames_[spheres_[i]->kdl_chain][spheres_[i]->kdl_segment].p.z(), v.x(), v.y(), v.z(), spheres_[i]->radius, spheres_[i]->priority);

    // check bounds
    if(!grid_->isInBounds(x, y, z))
    {
        ROS_INFO("Sphere %d %d %d is out of bounds.", x, y, z);
      return false;
    }

    // check for collision with world
    if((dist_temp = grid_->getCell(x,y,z)) <= int((spheres_[i]->radius + padding_) / grid_->getResolution() + 0.5))
    {
      dist = dist_temp;
      in_collision = true;
      s = *(spheres_[i]); 
      s.v = v;
      collision_spheres_.push_back(s);
    }

    ROS_DEBUG("x: %d y: %d z: %d radius: %0.3f (%d)  dist: %d (dist_min: %d)", x, y, z, spheres_[i]->radius,  int(spheres_[i]->radius / grid_->getResolution() + 0.5), grid_->getCell(x,y,z), int(dist));
    
    if(dist_temp < dist)
      dist = dist_temp;
  }
/*
  //check attached object for collision
  if(!isValidAttachedObject(angles, dist_temp))
  {
    dist = dist_temp;
    in_collision = true;
  }

  if(object_attached_ && dist_temp < dist)
      dist = dist_temp;
*/
/*
  if(in_collision)
  {
    visualizeCollisionModel(angles,"model");
    visualizeCollisions(angles,"collisions");
  }
*/

  if(object_attached_)
  {
    for(size_t i = 0; i < object_spheres_.size(); ++i)
    {
      v = frames_[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] * object_spheres_[i].v;

      grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z);

      ROS_INFO("[cspace] [%d] x: %d y: %d z: %d radius: %0.3f (%d)  dist: %d (dist_min: %d)", int(i), x, y, z, object_spheres_[i].radius,  int(object_spheres_[i].radius / grid_->getResolution() + 0.5), grid_->getCell(x,y,z), int(dist));

      // check bounds
      if(!grid_->isInBounds(x, y, z))
      {
        ROS_INFO("[cspace] Sphere %d %d %d is out of bounds.", x, y, z);
        return false;
      }

      // check for collision with world
      if((dist_temp = grid_->getCell(x,y,z)) <= int((object_spheres_[i].radius) / grid_->getResolution() + 0.5))
      {
        dist = dist_temp;
        in_collision = true;
        s = *(spheres_[i]); 
        s.v = v;
        collision_spheres_.push_back(s);
      }

      if(dist_temp < dist)
        dist = dist_temp;
    }
  }

  if(in_collision)
    return false;

  return true;
}

bool SBPLCollisionSpace::checkLinkForCollision(const std::vector<double> &angles, int link_num, bool verbose, unsigned char &dist)
{
  ROS_ERROR("[cspace] checkLinkForCollision() is commented out.");

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
  int inc_cc = 5;
  unsigned char dist_temp = 0;
  std::vector<double> start_norm(start);
  std::vector<double> end_norm(end);
  std::vector<std::vector<double> > path;
  dist = 100;

  for(size_t i=0; i < start.size(); ++i)
  {
    start_norm[i] = angles::normalize_angle(start[i]);
    end_norm[i] = angles::normalize_angle(end[i]);
  }

  // upper arm roll
  if(start[2] > 0.85)
    start_norm[2] = start[2] + (2*-M_PI);

  if(end[2] > 0.85)
    end_norm[2] = end[2] + (2*-M_PI);

  getInterpolatedPath(start_norm, end_norm, inc_, path);
  if(path.size() > 4)
    ROS_DEBUG("[cspace]  %d waypoints in interpolated path.", int(path.size()));

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

  /*
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
      cylinder(xyzr);
    }
  }
  */

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
  object_spheres_.clear();
  ROS_INFO("[cspace] Removed attached object.");
}

void SBPLCollisionSpace::attachSphereToGripper(std::string frame, geometry_msgs::Pose pose, double radius)
{
  object_attached_ = true;
  attached_object_frame_ = frame;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);
  
  ROS_INFO("[cspace] [attached_object] Attaching sphere.  pose: %0.3f %0.3f %0.3f radius: %0.3f", pose.position.x,pose.position.y,pose.position.z, radius); 
  ROS_INFO("[cspace] [attached_object] frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), group_name_.c_str(), attached_object_chain_num_, attached_object_segment_num_); 
  attached_object_radius_ = radius / grid_->getResolution() + 0.5;
  
  object_points_.resize(1);
  tf::PoseMsgToKDL(pose, object_points_[0]);

  ROS_INFO("[cspace] [attached_object] Added collision sphere.  xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[0].p.x(), object_points_[0].p.y(), object_points_[0].p.z(), radius, attached_object_radius_);
}

void SBPLCollisionSpace::attachCylinderToGripper(std::string frame, geometry_msgs::Pose pose, double radius, double length)
{
  object_attached_ = true;
  attached_object_frame_ = frame;
  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);
  
  ROS_INFO("[cspace] [attached_object] Attaching cylinder. pose: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f", pose.position.x,pose.position.y,pose.position.z, radius, length); 
  ROS_INFO("[cspace] [attached_object] frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), group_name_.c_str(), attached_object_chain_num_, attached_object_segment_num_); 
  attached_object_radius_ = radius / grid_->getResolution() + 0.5;

  if(attached_object_radius_ < 1)
    attached_object_radius_ = 1;

  object_points_.resize(2);
  tf::PoseMsgToKDL(pose, object_points_[0]);
  tf::PoseMsgToKDL(pose, object_points_[1]);

  //compute the endpoints of the cylinder
  object_points_[0].p.data[2] -= length/2.0;
  object_points_[1].p.data[2] += length/2.0;

  ROS_INFO("[cspace] [attached_object] Added cylinder.  Bottom: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[0].p.x(), object_points_[0].p.y(), object_points_[0].p.z(), radius, attached_object_radius_);
  ROS_INFO("[cspace] [attached_object] Added cylinder.     Top: xyz: %0.3f %0.3f %0.3f   radius: %0.3fm (%d cells)", object_points_[1].p.x(), object_points_[1].p.y(), object_points_[1].p.z(), radius, attached_object_radius_);
}


void SBPLCollisionSpace::attachCylinder(std::string link, geometry_msgs::Pose pose, double radius, double length)
{
  object_attached_ = true;
  attached_object_frame_ = link;
  tf::PoseMsgToKDL(pose, attached_object_pose_);

  model_.getFrameInfo(attached_object_frame_, group_name_, attached_object_chain_num_, attached_object_segment_num_);
  
  // discretize radius
  attached_object_radius_ = radius / grid_->getResolution() + 0.5;
  if(attached_object_radius_ < 1)
    attached_object_radius_ = 1;

  // compute end points of cylinder
  KDL::Frame center;
  tf::PoseMsgToKDL(pose, center);
  //KDL::Vector top(0.0,0.0,length/2.0), bottom(0.0,0.0,-length/2.0);
  KDL::Vector top(center.p), bottom(center.p);
  std::vector<KDL::Vector> points;

  top.data[2] += length/2.0; 
  bottom.data[2] -= length/2.0; 

  // get spheres 
  getIntermediatePoints(top, bottom, radius, points);
  object_spheres_.resize(points.size());
  ROS_INFO("THE SPHERES:");
  for(size_t i = 0; i < points.size(); ++i)
  {
    object_spheres_[i].name = "attached_" + boost::lexical_cast<std::string>(i);
    object_spheres_[i].v = points[i];
    object_spheres_[i].radius = radius;
    object_spheres_[i].kdl_chain = attached_object_chain_num_;
    object_spheres_[i].kdl_segment = attached_object_segment_num_;

    ROS_INFO("[%d] %0.3f %0.3f %0.3f", int(i), object_spheres_[i].v.x(), object_spheres_[i].v.y(), object_spheres_[i].v.z());
  } 

  ROS_INFO("[cspace] [attached_object] Attaching cylinder. pose: %0.3f %0.3f %0.3f radius: %0.3f length: %0.3f spheres: %d", pose.position.x,pose.position.y,pose.position.z, radius, length, int(object_spheres_.size())); 
  ROS_INFO("[cspace] [attached_object]  frame: %s  group: %s  chain: %d  segment: %d", attached_object_frame_.c_str(), group_name_.c_str(), attached_object_chain_num_, attached_object_segment_num_); 
  ROS_INFO("[cspace] [attached_object]    top: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm (%d cells)", top.x(), top.y(), top.z(), radius, attached_object_radius_);
  ROS_INFO("[cspace] [attached_object] bottom: xyz: %0.3f %0.3f %0.3f  radius: %0.3fm (%d cells)", bottom.x(), bottom.y(), bottom.z(), radius, attached_object_radius_);
}

bool SBPLCollisionSpace::isValidAttachedObject(const std::vector<double> &angles, unsigned char &dist)
{
  unsigned char dist_temp = 0;
  KDL::Vector p_base,p1,p2;
  std::vector<int> q1(3,0), q2(3,0);
  
  // for debugging
  Sphere s;
  attached_collision_spheres_.clear();

  if(!object_attached_)
    return true;

  if(!model_.computeDefaultGroupFK(angles, frames_))
    return false;

  // check if the object points are valid
  for(size_t i = 0; i < object_points_.size(); i++)
  {
    p_base = frames_[attached_object_chain_num_][attached_object_segment_num_] * object_points_[i].p;

    if(!isValidPoint(p_base.data[0],p_base.data[1],p_base.data[2],attached_object_radius_,dist))
    {
      //ROS_INFO("[cspace] Attached object point is in collision. xyz: %0.3f %0.3f %0.3f radius: %d cells (dist: %d cells).", p_base.data[0],p_base.data[1],p_base.data[2],attached_object_radius_,int(dist));

      // for debugging
      s.v.x(p_base.data[0]);
      s.v.y(p_base.data[1]);
      s.v.z(p_base.data[2]);
      s.radius = attached_object_radius_ * grid_->getResolution();
      attached_collision_spheres_.push_back(s);
      //return false;
    }
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

/*
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
    xyz[i][3] = attached_object_radius_ * grid_->getResolution();
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
      point[3] = attached_object_radius_ * grid_->getResolution();
      xyz.push_back(point);
    }
  }

  return true;
}
*/

bool SBPLCollisionSpace::getAttachedObject(const std::vector<double> &angles, std::vector<std::vector<double> > &xyz)
{
  KDL::Vector v;
  int x,y,z;
  xyz.clear();

  if(!object_attached_)
    return false;

  // compute foward kinematics
  if(!model_.computeDefaultGroupFK(angles, frames_))
  {
    ROS_ERROR("[cspace] Failed to compute foward kinematics.");
    return false;
  }

  xyz.resize(object_spheres_.size(), std::vector<double>(4,0));
  for(size_t i = 0; i < object_spheres_.size(); ++i)
  {
    v = frames_[object_spheres_[i].kdl_chain][object_spheres_[i].kdl_segment] /* * attached_object_pose_*/ * object_spheres_[i].v;

    // snap to grid
    grid_->worldToGrid(v.x(), v.y(), v.z(), x, y, z); 
    grid_->gridToWorld(x, y, z, xyz[i][0], xyz[i][1], xyz[i][2]);

    xyz[i][3] = object_spheres_[i].radius;
  }

  return true;
}

double SBPLCollisionSpace::getAttachedObjectRadius()
{
  return attached_object_radius_*grid_->getResolution();   
}

void SBPLCollisionSpace::processCollisionObjectMsg(const arm_navigation_msgs::CollisionObject &object)
{
  if(object.id.compare("all") == 0) // ignoring the operation type
  {
    removeAllCollisionObjects();
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ADD)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::REMOVE)
  {
    removeCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::DETACH_AND_ADD_AS_OBJECT)
  {
    object_map_[object.id] = object;
    addCollisionObject(object);
  }
  else if(object.operation.operation == arm_navigation_msgs::CollisionObjectOperation::ATTACH_AND_REMOVE_AS_OBJECT)
  {
    //TODO: Attach to gripper
    removeCollisionObject(object);
  }
  else
    ROS_WARN("*** Operation isn't supported. ***\n\n");
}

void SBPLCollisionSpace::addCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  geometry_msgs::Pose pose;

  for(size_t i = 0; i < object.shapes.size(); ++i)
  {
    if(object.shapes[i].type == arm_navigation_msgs::Shape::BOX)
    {
      //transformPose(object.header.frame_id, grid_->getReferenceFrame(), object.poses[i], pose);
      //grid_->getVoxelsInBox(pose, object.shapes[i].dimensions, object_voxel_map_[object.id]);
      
      std::vector<double> dims(3);
      dims[0] = object.shapes[i].dimensions[0];
      dims[1] = object.shapes[i].dimensions[1];
      dims[2] = object.shapes[i].dimensions[2];
      object_voxel_map_[object.id].clear();
      grid_->getVoxelsInBox(object.poses[i], dims, object_voxel_map_[object.id]);
    }
    else
      ROS_WARN("[cspace] Collision objects of type %d are not yet supported.", object.shapes[i].type);
  }

  // add this object to list of objects that get added to grid
  bool new_object = true;
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
    {
      ROS_WARN("[cspace] Received %s object again. Not adding.",object.id.c_str());
      new_object = false;
    }
  }
  if(new_object)
    known_objects_.push_back(object.id);

  grid_->addPointsToField(object_voxel_map_[object.id]);
  ROS_DEBUG("[cspace] Just added %s to the distance field.", object.id.c_str());
}

void SBPLCollisionSpace::removeCollisionObject(const arm_navigation_msgs::CollisionObject &object)
{
  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    if(known_objects_[i].compare(object.id) == 0)
    {
      known_objects_.erase(known_objects_.begin() + i);
      ROS_INFO("[cspace] Removing %s from list of known collision objects.", object.id.c_str());
    }
  }
}

void SBPLCollisionSpace::removeAllCollisionObjects()
{
  known_objects_.clear();
  ROS_INFO("[cspace] Clearing list of known collision objects.");
}

void SBPLCollisionSpace::putCollisionObjectsInGrid()
{
  ROS_DEBUG("[cspace] Should we reset first?");

  for(size_t i = 0; i < known_objects_.size(); ++i)
  {
    grid_->addPointsToField(object_voxel_map_[known_objects_[i]]);
    ROS_INFO("[cspace] Added %s to grid with %d voxels.",known_objects_[i].c_str(), int(object_voxel_map_[known_objects_[i]].size()));
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

void SBPLCollisionSpace::visualizeCollisionModel(const std::vector<double> &angles, std::string text)
{
  std::vector<std::vector<double> > spheres;
  // arm
  getCollisionCylinders(angles, spheres);
  aviz_->visualizeSpheres(spheres, 210, 0.5, text);
  ROS_DEBUG("[cspace] Visualizing %d spheres of the robot's collision model.", int(spheres.size()));

  // attached object
  spheres.clear();
  getAttachedObject(angles, spheres);
  if(spheres.size() > 0)
  {
    aviz_->visualizeSpheres(spheres, 210, 0.5, text+" (object)");
    ROS_DEBUG("[node] Visualizing %d attached_object spheres if its collision model.", int(spheres.size()));
    /*
    for(size_t i = 0; i < spheres.size(); ++i)
      ROS_INFO("[cspace] [%d] %0.3f %0.3f %0.3f radius: %0.3f", int(i), spheres[i][0], spheres[i][1], spheres[i][2], spheres[i][3]);
    */
  }
  else
    ROS_DEBUG("[node] No attached object.");
}

void SBPLCollisionSpace::visualizeCollisions(const std::vector<double> &angles, std::string text)
{
  std::vector<double> s(4,0);
  std::vector<std::vector<double> > spheres;
  // arm
  for(size_t i = 0; i < collision_spheres_.size(); ++i)
  {
    s[0] = collision_spheres_[i].v.x();
    s[1] = collision_spheres_[i].v.y();
    s[2] = collision_spheres_[i].v.z();
    s[3] = collision_spheres_[i].radius;
    spheres.push_back(s);
  }
  if(spheres.size() > 0)
  {
    aviz_->visualizeSpheres(spheres, 50, 0.6, text);
    ROS_INFO("[cspace] Visualizing %d spheres in collision.", int(spheres.size()));
  }

  // attached object
  spheres.clear();
  for(size_t i = 0; i < attached_collision_spheres_.size(); ++i)
  {
    s[0] = attached_collision_spheres_[i].v.x();
    s[1] = attached_collision_spheres_[i].v.y();
    s[2] = attached_collision_spheres_[i].v.z();
    s[3] = attached_collision_spheres_[i].radius;

    visualizeVoxels(s[0],s[1],s[2],s[3],"voxels near object-" + boost::lexical_cast<std::string>(i)); 
    spheres.push_back(s);
  }
  if(spheres.size() > 0)
  {
    aviz_->visualizeSpheres(spheres, 50, 0.8, text+" (object)");
    ROS_INFO("[node] Visualizing %d attached_object spheres in collision.", int(spheres.size()));
  }
  else
    ROS_DEBUG("[node] No attached object.");
}

void SBPLCollisionSpace::visualizeVoxels(double x_center, double y_center, double z_center, double radius, std::string text)
{
  int x_c, y_c, z_c, radius_c;
  std::vector<double> v(3,0);
  std::vector<std::vector<double> > voxels;
 
  grid_->worldToGrid(x_center, y_center, z_center, x_c, y_c, z_c);

  radius_c = radius/grid_->getResolution() + 0.5;
  
  for(int z = z_c-radius_c; z < z_c+radius_c; ++z)
  {
    for(int y = y_c-radius_c; y < y_c+radius_c; ++y)
    {
      for(int x = x_c-radius_c; x < x_c+radius_c; ++x)
      {
        if(grid_->getCell(x,y,z) == 0)
        {
          grid_->gridToWorld(x, y, z, v[0], v[1], v[2]);
          voxels.push_back(v);
        }
      }
    }
  }
  aviz_->visualizeSpheres(voxels, 300, text, grid_->getResolution());
  ROS_INFO("[cspace] Visualizing cells centered around cell %d %d %d with radius %d cells", x_c, y_c, z_c, radius_c);
  ROS_INFO("[cspace] Visualizing %d voxel spheres around %0.3f %0.3f %0.3f with radius %0.3fm", int(voxels.size()), x_center, y_center, z_center, radius);
}

bool SBPLCollisionSpace::doesLinkExist(std::string name)
{
  int chain, segment;
  return model_.getFrameInfo(name, group_name_, chain, segment);
}

void SBPLCollisionSpace::getIntermediatePoints(btVector3 a, btVector3 b, double d, std::vector<btVector3> &points)
{
  btVector3 pt, dir;
  int interm_points = floor(a.distance(b) / d + 0.5);
  
  dir = b - a;
  dir.normalize();
  ROS_DEBUG("# interm points: %d  unit vector: %0.3f %0.3f %0.3f", interm_points, dir.x(), dir.y(), dir.z());

  points.clear();
  points.push_back(a);
  for(int i = 1; i <= interm_points; ++i)
  {
    pt = a + dir*i*d;
    points.push_back(pt);
  }
  points.push_back(b);

  /*
  // debug & visualization
  std::vector<std::vector<double> > spheres(points.size(), std::vector<double>(4,0));
  std::vector<geometry_msgs::Point> line(points.size());
  for(size_t i = 0; i < points.size(); ++i)
  {
    if(i == 0)
      ROS_INFO("[%d]  %0.3f %0.3f %0.3f", int(i), points[i].x(), points[i].y(), points[i].z());
    else  
      ROS_INFO("[%d]  %0.3f %0.3f %0.3f dist: %0.3f", int(i), points[i].x(), points[i].y(), points[i].z(), points[i].distance(points[i-1]));

    spheres[i][0] = points[i].x();
    spheres[i][1] = points[i].y();
    spheres[i][2] = points[i].z();
    spheres[i][3] = d;

    line[i].x = points[i].x();
    line[i].y = points[i].y();
    line[i].z = points[i].z();
  }
  aviz_->visualizeSpheres(spheres, 25, 0.6, "intermediate_spheres");
  aviz_->visualizeLine(line, "line", 0, 280, 0.02);
  */
}


void SBPLCollisionSpace::getIntermediatePoints(KDL::Vector a, KDL::Vector b, double d, std::vector<KDL::Vector> &points)
{
  KDL::Vector pt, dir;
  int interm_points = floor(distance(a,b) / d + 0.5);
  
  dir = b - a;
  double norm  = dir.Normalize();
  ROS_DEBUG("# interm points: %d  unit vector: %0.3f %0.3f %0.3f norm: %0.3f", interm_points, dir.x(), dir.y(), dir.z(), norm);

  points.clear();
  points.push_back(a);
  for(int i = 1; i <= interm_points; ++i)
  {
    pt = a + dir*i*d;
    points.push_back(pt);
  }
  points.push_back(b);

  /*
  // debug & visualization
  std::vector<std::vector<double> > spheres(points.size(), std::vector<double>(4,0));
  std::vector<geometry_msgs::Point> line(points.size());
  for(size_t i = 0; i < points.size(); ++i)
  {
    if(i == 0)
      ROS_INFO("[%d]  %0.3f %0.3f %0.3f", int(i), points[i].x(), points[i].y(), points[i].z());
    else  
      ROS_INFO("[%d]  %0.3f %0.3f %0.3f dist: %0.3f", int(i), points[i].x(), points[i].y(), points[i].z(), distance(points[i],points[i-1]));

    spheres[i][0] = points[i].x();
    spheres[i][1] = points[i].y();
    spheres[i][2] = points[i].z();
    spheres[i][3] = d;

    line[i].x = points[i].x();
    line[i].y = points[i].y();
    line[i].z = points[i].z();
  }
  aviz_->visualizeSpheres(spheres, 25, 0.6, "intermediate_spheres");
  aviz_->visualizeLine(line, "line", 0, 280, 0.02);
  */
}

}
