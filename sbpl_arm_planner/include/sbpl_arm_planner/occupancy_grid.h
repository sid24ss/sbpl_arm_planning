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

#include <sys/stat.h>
#include <fstream>
#include <sbpl/config.h>
#include <distance_field/distance_field.h>
#include <distance_field/voxel_grid.h>
#include <distance_field/propagation_distance_field.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <vector>

using namespace std;

#ifndef _OCCUPANCY_GRID_
#define _OCCUPANCY_GRID_

class OccupancyGrid{

  public:
   
    /** @brief Default constructor - sets default values */ 
    OccupancyGrid();

    /** 
     * @brief Constructor 
     * @param dimension of grid along X
     * @param dimension of grid along Y
     * @param dimension of grid along Z
     * @param resolution of grid (meters)
     * @param X coordinate of origin (meters)
     * @param Y coordinate of origin (meters)
     * @param Z coordinate of origin (meters)
    */
    OccupancyGrid(double dim_x, double dim_y, double dim_z, double resolution, double origin_x, double origin_y, double origin_z);

    /** @brief destructor */
    ~OccupancyGrid();

    /** @brief convert grid cell coords into world coords*/
    inline void gridToWorld(int x, int y, int z, bool grid2, double &wx, double &wy, double &wz);
    
    /** @brief convert world coords into grid cell coords*/
    inline void worldToGrid(double wx, double wy, double wz, bool grid2, int &x, int &y, int &z); 

    /** @brief get the cell's distance to the nearest obstacle in cells*/
    inline unsigned char getCell(int x, int y, int z, bool grid2=false);

    /** @brief get the cell's distance to the nearest obstacle in meters*/
    inline double getCell(int *xyz, bool grid2=false);

    /** @brief check if {x,y,z} is in bounds of the grid */
    inline bool isInBounds(int x, int y, int z, bool grid2=false);

    /** @brief convert coords from grid to corods in grid2*/
    void gridToGrid2(int x, int y, int z, int &x2, int &y2, int &z2);

    /** @brief return a pointer to the distance field */
    const distance_field::PropagationDistanceField* getDistanceFieldPtr(bool grid2);
    
    /** @brief get the dimensions of the grid */
    void getGridSize(int &width, int &depth, int &height, bool grid2=false);

    /** @brief get the dimensions of the grid */
    void getGridSize(short unsigned int *dims, bool grid2=false); //FILL IN THIS FUNCTION

    /** @brief set the dimensions of the world (meters)*/
    void setWorldSize(double width, double depth, double height);
    
    /** @brief get the dimensions of the world (meters)*/
    void getWorldSize(double &width, double &depth, double &height);

    /** @brief set the origin of the world (meters)*/
    void setOrigin(double wx, double wy, double wz);
    
    /** @brief get the origin of the world (meters)*/
    void getOrigin(double &wx, double &wy, double &wz);

    /** @brief set the resolution of the world (meters)*/
    void setResolution(double resolution_m, bool grid2=false);
    
    /** @brief get the resolution of the world (meters)*/
    double getResolution(bool grid2);

    /** @brief enable a second grid for a lower resolution c.c */
    void enableGrid2(double resolution);

    /** @brief returns true if two grids are enabled */
    bool isDualGrids();

    /** @brief update the distance field from the collision_map */
    void updateFromCollisionMap(const mapping_msgs::CollisionMap &collision_map);

    /** @brief display distance field visualizations to rviz */
    void visualize();
    
    /** 
     * @brief manually add a cuboid to the collision map
     * @param X_origin_of_cuboid 
     * @param Y_origin_of_cuboid 
     * @param Z_origin_of_cuboid
     * @param size along the X dimension (meters)
     * @param size along the Y dimension (meters)
     * @param size along the Z dimension (meters)
    */
    void addCollisionCuboid(double origin_x, double origin_y, double origin_z, double size_x, double size_y, double size_z);

    bool saveGridToBinaryFile(std::string filename, bool use_grid2);

    void printGridFromBinaryFile(std::string filename);

  private:

    double origin_[3];
    double grid_resolution_;
    double world_size_[3];
    double prop_distance_;
    int grid_size_[3];

    bool grid2_enabled_;
    double grid2_scale_;
    double grid2_resolution_;
    int grid2_size_[3];

    std::string reference_frame_;

    distance_field::PropagationDistanceField* grid_;
    distance_field::PropagationDistanceField* grid2_;

    std::vector<btVector3> cuboid_points_;
};

inline void OccupancyGrid::gridToWorld(int x, int y, int z, bool grid2, double &wx, double &wy, double &wz)
{
  if(grid2)
    grid2_->gridToWorld(x, y, z, wx, wy, wz); 
  else
    grid_->gridToWorld(x, y, z, wx, wy, wz); 
}

inline void OccupancyGrid::worldToGrid(double wx, double wy, double wz, bool grid2, int &x, int &y, int &z)
{
  if(grid2)
    grid2_->worldToGrid (wx, wy, wz, x, y, z);
  else
    grid_->worldToGrid (wx, wy, wz, x, y, z);
}

inline unsigned char OccupancyGrid::getCell(int x, int y, int z, bool grid2)
{
  if(grid2)
    return (unsigned char)(grid2_->getDistanceFromCell(x,y,z) / grid2_resolution_);
  else
    return (unsigned char)(grid_->getDistanceFromCell(x,y,z) / grid_resolution_);
}

inline double OccupancyGrid::getCell(int *xyz, bool grid2)
{
  if(grid2)
    return grid2_->getDistanceFromCell(xyz[0],xyz[1],xyz[2]);
  else
    return grid_->getDistanceFromCell(xyz[0],xyz[1],xyz[2]);
}

inline void OccupancyGrid::gridToGrid2(int x, int y, int z, int &x2, int &y2, int &z2)
{
  x2 = x / grid2_scale_;
  y2 = y / grid2_scale_;
  z2 = z / grid2_scale_;
}

inline bool OccupancyGrid::isInBounds(int x, int y, int z, bool grid2)
{
  if(grid2)
  {
    if(x >= grid2_size_[0] || 0 > x || y >= grid2_size_[1] || 0 > y || z >= grid2_size_[2] || 0 > z)
     return false;
  }
  else
  {
    if(x >= grid_size_[0] || 0 > x || y >= grid_size_[1] || 0 > y || z >= grid_size_[2] || 0 > z)
     return false;
  }
  return true;
}

#endif
