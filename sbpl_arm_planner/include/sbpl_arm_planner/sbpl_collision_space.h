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

#include <ros/ros.h>
#include <vector>
#include <sbpl/config.h>
#include <resource_retriever/retriever.h>
#include <sbpl_arm_planner/bresenham.h>
#include <sbpl_arm_planner/sbpl_arm_model.h>
#include <sbpl_arm_planner/occupancy_grid.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <tf_conversions/tf_kdl.h>

using namespace std;

#ifndef _SBPL_COLLISION_SPACE_
#define _SBPL_COLLISION_SPACE_

/** @brief coords - used to pass around lists of valid cells */
typedef struct
{
  short unsigned int x;
  short unsigned int y;
  short unsigned int z;
  bool bIsObstacle;
} CELL3V;

namespace sbpl_arm_planner
{

class SBPLCollisionSpace
{
  public:
    /** @brief default constructor 
     * @param a pointer to the arm object used for planning
     * @param a pointer to an occupancy grid used for planning
    */
    SBPLCollisionSpace(SBPLArmModel* arm, OccupancyGrid* grid);

    ~SBPLCollisionSpace(){};

    /** @brief choose the file to output debug information */
    void setDebugFile(FILE* file_ptr);

    /** @brief check joint configuration for collision (0: collision) */
    bool checkCollision(const std::vector<double> &angles, bool verbose, bool check_mesh, unsigned char &dist);

    /** @brief check if the cell's distance to nearest obstacle > radius */
    inline bool isValidCell(const int x, const int y, const int z, const int radius, bool grid2=false);

    void addArmCuboidsToGrid();

    bool getCollisionCylinders(const std::vector<double> &angles, std::vector<std::vector<double> > &cylinders);

    void getLineSegment(const std::vector<int> a,const std::vector<int> b,std::vector<std::vector<int> > &points);

    void getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, double inc, std::vector<std::vector<double> > &path);

    void getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, std::vector<double> &inc, std::vector<std::vector<double> > &path);

    bool checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, unsigned char &dist);

    /** @brief check if a line segment lies on an invalid cell (0: collision) */
    unsigned char isValidLineSegment(const std::vector<int> a,const std::vector<int> b,const short unsigned int radius);
 
    void attachSphereToGripper(std::string frame, geometry_msgs::Pose pose, double radius);

    void attachCylinderToGripper(std::string frame, geometry_msgs::Pose pose, double radius, double length);

    void attachMeshToGripper(const std::string frame, const geometry_msgs::Pose pose, const bodies::BoundingCylinder &cyl);
    
    void attachMeshToGripper(const std::string frame, const geometry_msgs::Pose pose, const std::vector<int32_t> &triangles, const std::vector<geometry_msgs::Point> &vertices);

    void getAttachedObject(const std::vector<double> &angles, std::vector<std::vector<double> > &xyz);

    double getAttachedObjectRadius();
    
  private:


    /** @brief arm model used by planner */
    SBPLArmModel* arm_;

    /** @brief occupancy grid used by planner */
    OccupancyGrid* grid_;

    /** @brief the file for dumping debug output */
    FILE* fOut_;

    std::vector<double> inc_;

    bool object_attached_;
    int attached_object_frame_num_;
    short unsigned int attached_object_radius_;
    std::vector<KDL::Frame> object_points_;

    /** @brief get the xyz coords of each joint in the arm */ 
    bool getJointPosesInGrid(std::vector<double> angles, std::vector<std::vector<int> > &jnts, KDL::Frame &f_out, bool verbose);

     /** @brief get the shortest distance between two 3D line segments */
    double distanceBetween3DLineSegments(std::vector<int> l1a, std::vector<int> l1b,std::vector<int> l2a,std::vector<int> l2b);

    bool getBoundingCylinderOfMesh(std::string mesh_file, shapes::Shape *mesh, bodies::BoundingCylinder &cyl);
    void getBoundingCylinderOfMesh(const std::vector<int32_t> &triangles, const std::vector<geometry_msgs::Point> &vertices, bodies::BoundingCylinder &cyl);

    bool isValidPoint(double &x, double &y, double &z, short unsigned int &radius, unsigned char &dist);
    bool isValidAttachedObject(const std::vector<double> &angles, unsigned char &dist);
    bool isValidAttachedObject(const std::vector<double> &angles, unsigned char &dist, std::vector<int> j1, std::vector<int> j2);

};

inline bool SBPLCollisionSpace::isValidCell(const int x, const int y, const int z, const int radius, bool grid2)
{
  if(grid_->getCell(x,y,z,grid_->isDualGrids()) <= radius)
    return false;
  return true;
}

} 
#endif

