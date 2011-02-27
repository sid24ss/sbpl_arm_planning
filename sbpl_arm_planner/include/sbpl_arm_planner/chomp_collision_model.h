/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Benjamin Cohen */

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <kdl/jntarray.hpp>
#include <angles/angles.h>
#include <motion_planning_msgs/RobotState.h>
#include <chomp_motion_planner/chomp_robot_model.h>
#include <planning_environment/monitors/collision_space_monitor.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_optimizer.h>
#include <chomp_motion_planner/chomp_collision_space.h>


class ChompCollisionModel
{

  public:

    /* \brief Default constructor */
    ChompCollisionModel(){};
    
    /* \brief Destructor */
    ~ChompCollisionModel();

    /* \brief Initialize chomp classes
     *  @param reference_frame reference frame to be used for everything
     *  (distance field, kinematics)
     *  @return false if an error occured
    */  
    bool init(std::string &group_name, std::string &reference_frame);
    
    /* \brief Checks if a joint configuration is in collision 
     * @param angles vector of joint angles
     * @param dist minimum distance from all links to nearest obstacle
     * @return true if in collision, false otherwise
    */
    bool isInCollision(const std::vector<double> &angles, bool partial_solver, double &dist);

    /* \brief Checks if the interpolated path from start to end is free of
     * collision.
     * @param start starting joint configuration
     * @param end final joint configuration
     * @param dist minimum distance from all links to nearest obstacle along
     * the path
     * @return true if in collision, false otherwise
    */ 
    bool checkInterpolatedPathForCollision(const std::vector<double> &start, const std::vector<double> &end, double &dist);

    /* \brief Computes the interpolated path from start to end
     * @param start starting joint configuration
     * @param end final joint configuration
     * @param path the linearly interpolated path discretized by interp_res_
     * @return false if start and end have different sizes
    */ 
    bool getInterpolatedPath(const std::vector<double> &start, const std::vector<double> &end, std::vector<std::vector<double> > &path);

    /* \brief Initialize the start state of the robot */
    void setStartState(const motion_planning_msgs::RobotState& robot_state);

    int jointNameToKDLIndex(const std::string &joint_name);

    void computeFK(const std::vector<double> &angles, int kdl_joint_index, bool partial_solver, std::vector<double> &xyzrpy);

    /* \brief Print debug information */
    void printDebugOutput();

  private:
   
    ros::NodeHandle nh_;

    int num_joints_;
    int num_collision_points_;
 
    double max_radius_clearance_;

    tf::TransformListener tf_;
    std::string reference_frame_;
    std::string group_name_;
 
    chomp::ChompRobotModel chomp_robot_model_;                   /**< Chomp Robot Model */
    chomp::ChompParameters chomp_parameters_;                    /**< Chomp Parameters */
    chomp::ChompCollisionSpace chomp_collision_space_;           /**< Chomp Collision space */
  
    const chomp::ChompRobotModel::ChompPlanningGroup *planning_group_;

    planning_environment::CollisionModels* collision_models_;
    planning_environment::CollisionSpaceMonitor *monitor_;

    std::vector<int> group_joint_to_kdl_joint_index_;
    std::vector<KDL::Vector> joint_axis_;
    std::vector<KDL::Vector> joint_pos_;
    std::vector<KDL::Frame>  segment_frames_;
    std::vector<KDL::Vector> collision_point_pos_;
    std::vector<double> point_is_in_collision_;

    KDL::JntArray kdl_joint_array_;
    double x_gradient_, y_gradient_, z_gradient_;


    /* for ben's functions */
    std::vector<double> interp_res_;

    // temporary variables
    std::vector<double> start_;
    std::vector<double> start_norm_;
    std::vector<double> end_;
    std::vector<double> end_norm_;
    std::vector<std::vector<double> > path_;
};

