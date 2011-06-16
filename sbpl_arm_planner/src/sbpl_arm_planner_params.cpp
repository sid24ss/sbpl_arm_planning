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

#include <sbpl_arm_planner/sbpl_arm_planner_params.h>

SBPLArmPlannerParams::SBPLArmPlannerParams()
{
  epsilon_ = 10;
  use_multires_mprims_ = true;
  use_dijkstra_heuristic_ = true;
  use_smoothing_ = false;
  use_6d_pose_goal_ = true;
  sum_heuristics_ = false;
  use_uniform_cost_ = true;
  use_ik_ = true;
  use_orientation_solver_ = true;

  verbose_ = false;
  verbose_heuristics_ = false;
  verbose_collisions_ = false;
  angle_delta_ = 360;

  num_mprims_ = 0;
  num_long_dist_mprims_ = 0;
  num_short_dist_mprims_ = 0;
  
  short_dist_mprims_thresh_c_ = 20;
  short_dist_mprims_thresh_m_ = 0.2;
  
  cost_multiplier_ = 1000;
  cost_per_cell_ = 0;
  cost_per_meter_ = 0;

  range1_cost_ = 12;
  range2_cost_ = 7;
  range3_cost_ = 2;

  solve_for_ik_thresh_ = 1000;
  solve_for_ik_thresh_m_= 0.20;

  two_calls_to_op_ = false;
  is_goal_function_ = 0;
}

void SBPLArmPlannerParams::initFromParamServer()
{
  ros::NodeHandle nh("~");

  /* planner */
  nh.param("planner/epsilon",epsilon_, 10.0);
  nh.param("planner/use_dijkstra_heuristic",use_dijkstra_heuristic_,true);
  nh.param("planner/use_research_heuristic",use_research_heuristic_,false);
  nh.param("planner/use_multiresolution_motion_primitives",use_multires_mprims_,true);
  nh.param("planner/use_uniform_obstacle_cost", use_uniform_cost_,false);
  nh.param("planner/verbose",verbose_,false);
  nh.param("planner/obstacle_distance_cost_far",range3_cost_,12);
  nh.param("planner/obstacle_distance_cost_mid",range2_cost_,7);
  nh.param("planner/obstacle_distance_cost_near",range1_cost_,2);

  /* research params of the planner */
  nh.param("planner/research/solve_with_ik_threshold",solve_for_ik_thresh_m_,0.15);
  nh.param("planner/research/sum_heuristics",sum_heuristics_,false);
  nh.param("planner/research/short_distance_mprims_threshold",short_dist_mprims_thresh_m_, 0.2);

  /* occupancy grid */
  nh.param("collision_space/resolution",resolution_,0.02);
  nh.param("collision_space/occupancy_grid/origin_x",originX_,-0.6);
  nh.param("collision_space/occupancy_grid/origin_y",originY_,-1.15);
  nh.param("collision_space/occupancy_grid/origin_z",originZ_,-0.05);  
  nh.param("collision_space/occupancy_grid/size_x",sizeX_,1.6);
  nh.param("collision_space/occupancy_grid/size_y",sizeY_,1.8);
  nh.param("collision_space/occupancy_grid/size_z",sizeZ_,1.4);
}

bool SBPLArmPlannerParams::initFromParamFile(std::string param_file)
{
  char* filename = new char[param_file.length()+1];
  param_file.copy(filename, param_file.length(),0);
  filename[param_file.length()] = '\0';
  FILE* fCfg = fopen(filename, "r");
  
  if(initFromParamFile(fCfg))
  {
    delete filename;
    SBPL_FCLOSE(fCfg);
    delete fCfg;
    return true;
  }
  else
  {
    delete filename;
    SBPL_FCLOSE(fCfg);
    delete fCfg;
    return false;
  }
}

bool SBPLArmPlannerParams::initMotionPrimsFromFile(FILE* fCfg)
{
  char sTemp[1024];
  int nrows=0,ncols=0, short_mprims=0;


  if(fCfg == NULL)
  {
    SBPL_ERROR("ERROR: unable to open the params file. Exiting.");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1)
    SBPL_WARN("Parsed string has length < 1."); 
  if(strcmp(sTemp, "Motion_Primitives(degrees):") != 0)
  {
    SBPL_ERROR("ERROR: First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
    return false;
  }

  //number of actions
  if(fscanf(fCfg,"%s",sTemp) < 1) 
  {
    SBPL_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    nrows = atoi(sTemp);
  
  //length of joint array
  if(fscanf(fCfg,"%s",sTemp) < 1)
  {
    SBPL_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    ncols = atoi(sTemp);

  //number of short distance motion primitives
  if(fscanf(fCfg,"%s",sTemp) < 1)
  { 
    SBPL_WARN("Parsed string has length < 1.");
    return false;
  }
  else
    short_mprims = atoi(sTemp);

  std::vector<double> mprim(ncols,0);

  for (int i=0; i < nrows; ++i)
  {
    for(int j=0; j < ncols; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        mprim[j] = atof(sTemp);
      else
      {
        SBPL_ERROR("ERROR: End of parameter file reached prematurely. Check for newline.");
        return false;
      }
    }
    if(i < (nrows-short_mprims))
      addMotionPrim(mprim,true,false);
    else
      addMotionPrim(mprim,true,true);
  }
 
  max_mprim_offset_ = getLargestMotionPrimOffset(); 

  return true;
}

bool SBPLArmPlannerParams::initFromParamFile(FILE* fCfg)
{ 
  char sTemp[1024];
  int nrows=0,ncols=0, short_mprims=0;

  if(fCfg == NULL)
  {
    SBPL_ERROR("ERROR: unable to open the params file. Exiting.\n");
    return false;
  }

  if(fscanf(fCfg,"%s",sTemp) < 1) 
    SBPL_PRINTF("Parsed string has length < 1.\n");
  while(!feof(fCfg) && strlen(sTemp) != 0)
  {
    if(strcmp(sTemp, "environment_size(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_PRINTF("Parsed string has length < 1.\n");
      sizeX_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_PRINTF("Parsed string has length < 1.\n");
      sizeY_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_PRINTF("Parsed string has length < 1.\n");
      sizeZ_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "environment_origin(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_PRINTF("Parsed string has length < 1.\n");
      originX_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_PRINTF("Parsed string has length < 1.\n");
      originY_ = atof(sTemp);
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_PRINTF("Parsed string has length < 1.\n");
      originZ_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "resolution(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        SBPL_PRINTF("Parsed string has length < 1.\n");
      resolution_ = atof(sTemp);
    }
    else if(strcmp(sTemp, "use_dijkstra_heuristic:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      use_dijkstra_heuristic_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_orientation_solver:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      use_orientation_solver_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_ik:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      use_ik_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "sum_heuristics:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      sum_heuristics_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "use_research_heuristic:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      use_research_heuristic_ = atoi(sTemp);
    }
    else if(strcmp(sTemp, "plan_to_6d_pose_constraint:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      use_6d_pose_goal_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"planner_epsilon:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      epsilon_ = atof(sTemp);
    }
    else if(strcmp(sTemp,"use_multiresolution_motion_primitives:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      use_multires_mprims_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"use_uniform_obstacle_cost:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      use_uniform_cost_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"short_distance_mprims_threshold(meters):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      short_dist_mprims_thresh_m_ = atof(sTemp);
    }
    else if(strcmp(sTemp,"check_if_at_goal_function(0:IK,1:OP):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      is_goal_function_ = atoi(sTemp);
    }   
    else if(strcmp(sTemp,"two_calls_to_orientation_planner:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.\n");
      two_calls_to_op_ = atoi(sTemp);
    }   
    else if(strcmp(sTemp,"verbose:") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.");
      verbose_ = atoi(sTemp);
    }
    else if(strcmp(sTemp,"solve_for_ik_threshold(distance):") == 0)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_PRINTF("Parsed string has length < 1.");
      solve_for_ik_thresh_m_ = atof(sTemp);
    }
    //motion primitives must be last thing in the file

    else if(strcmp(sTemp, "Motion_Primitives(degrees):") == 0)
    {
      break;
    }

    else
    {
      SBPL_PRINTF("Error: Invalid Field name (%s) in parameter file.",sTemp);
      //return false;
    }
    if(fscanf(fCfg,"%s",sTemp) < 1) 
      SBPL_PRINTF("Parsed string has length < 1.");
  }


  //number of actions
  if(fscanf(fCfg,"%s",sTemp) < 1) 
  {
    SBPL_PRINTF("Parsed string has length < 1.");
    return false;
  }
  else
    nrows = atoi(sTemp);
  
  //length of joint array
  if(fscanf(fCfg,"%s",sTemp) < 1)
  {
    SBPL_PRINTF("Parsed string has length < 1.");
    return false;
  }
  else
    ncols = atoi(sTemp);

  //number of short distance motion primitives
  if(fscanf(fCfg,"%s",sTemp) < 1)
  { 
    SBPL_PRINTF("Parsed string has length < 1.");
    return false;
  }
  else
    short_mprims = atoi(sTemp);

  std::vector<double> mprim(ncols,0);

  for (int i=0; i < nrows; ++i)
  {
    for(int j=0; j < ncols; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        SBPL_WARN("Parsed string has length < 1.");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        mprim[j] = atof(sTemp);
      else
      {
        SBPL_ERROR("ERROR: End of parameter file reached prematurely. Check for newline.");
        return false;
      }
    }
    if(i < (nrows-short_mprims))
      addMotionPrim(mprim,true,false);
    else
      addMotionPrim(mprim,true,true);
  }


  short_dist_mprims_thresh_c_ = short_dist_mprims_thresh_m_ / resolution_;

  solve_for_ik_thresh_ = (solve_for_ik_thresh_m_ /resolution_) * cost_per_cell_;

  max_mprim_offset_ = getLargestMotionPrimOffset(); 
    
  SBPL_PRINTF("Successfully parsed parameters file");
  return true;
}

void SBPLArmPlannerParams::setCellCost(int cost_per_cell)
{
  cost_per_cell_ = cost_per_cell;
  solve_for_ik_thresh_ = (solve_for_ik_thresh_m_ / resolution_) * cost_per_cell_;
  short_dist_mprims_thresh_c_ = short_dist_mprims_thresh_m_/resolution_ * cost_per_cell;
}

void SBPLArmPlannerParams::addMotionPrim(std::vector<double> mprim, bool add_converse, bool short_dist_mprim)
{
  if(short_dist_mprim)
  {
    mprims_.push_back(mprim);
    num_short_dist_mprims_++;
    if(add_converse)
    {
      for(int i = 0; i < int(mprim.size()); ++i)
      {
        if(mprim[i] != 0)
          mprim[i] *=  -1;
      }
      mprims_.push_back(mprim);
      num_short_dist_mprims_++;
    }
  }
  else
  {
    std::vector<std::vector<double> >::iterator it_long_dist;
    it_long_dist = mprims_.begin();
    if(num_long_dist_mprims_ > 1)
    {
      advance(it_long_dist,num_long_dist_mprims_);
      mprims_.insert(it_long_dist, mprim);
    }
    else if(it_long_dist == mprims_.end() || mprims_.empty())
      mprims_.push_back(mprim);

    num_long_dist_mprims_++;
    if(add_converse)
    {
      for(int i = 0; i < int(mprim.size()); ++i)
      {
        if(mprim[i] != 0)
          mprim[i] *=  -1;
      }

      it_long_dist=mprims_.begin();
      advance(it_long_dist,num_long_dist_mprims_);
      mprims_.insert(it_long_dist,mprim);
      num_long_dist_mprims_++;
    } 
  }
  num_mprims_ = num_short_dist_mprims_ + num_long_dist_mprims_;
}

void SBPLArmPlannerParams::printMotionPrims(FILE* fOut)
{
  int i;
  SBPL_FPRINTF(fOut,"Long Distance Motion Primitives: %d\n", num_long_dist_mprims_);
  for(i = 0; i < num_long_dist_mprims_; ++i)
  {
    SBPL_FPRINTF(fOut,"%2d: ",i);
    for(int j = 0; j < int(mprims_[i].size()); ++j)
      SBPL_FPRINTF(fOut,"%4.1f ",mprims_[i][j]);
    SBPL_FPRINTF(fOut,"\n");
  }

  SBPL_FPRINTF(fOut,"Short Distance Motion Primitives: %d\n", num_short_dist_mprims_);
  for(; i < num_mprims_; ++i)
  {
    SBPL_FPRINTF(fOut,"%2d: ",i);
    for(int j = 0; j < int(mprims_[i].size()); ++j)
      SBPL_FPRINTF(fOut,"%4.1f ",mprims_[i][j]);
    SBPL_FPRINTF(fOut,"\n");
  }
}

void SBPLArmPlannerParams::printParams(FILE* fOut)
{
  SBPL_DEBUG_NAMED(fOut,"\nArm Planner Parameters:\n");
  SBPL_DEBUG_NAMED(fOut,"%40s: %d\n", "# motion primitives",num_mprims_);
  SBPL_DEBUG_NAMED(fOut,"%40s: %d\n", "# short distance motion primitives", num_short_dist_mprims_);
  SBPL_DEBUG_NAMED(fOut,"%40s: %d\n", "# long distance motion primitives", num_long_dist_mprims_);
  SBPL_DEBUG_NAMED(fOut,"%40s: %.2f\n", "epsilon",epsilon_);
  SBPL_DEBUG_NAMED(fOut,"%40s: %s\n", "use multi-resolution motion primitives", use_multires_mprims_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(fOut,"%40s: %s\n", "use dijkstra heuristic", use_dijkstra_heuristic_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(fOut,"%40s: %s\n", "use research heuristic", use_research_heuristic_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(fOut,"%40s: %s\n", "h = h_elbow + h_endeff", sum_heuristics_ ? "yes" : "no"); 
  SBPL_DEBUG_NAMED(fOut,"%40s: %s\n", "use a uniform cost",use_uniform_cost_ ? "yes" : "no");
  SBPL_DEBUG_NAMED(fOut,"%40s: %d\n", "cost per cell", cost_per_cell_);
  SBPL_DEBUG_NAMED(fOut,"%40s: %.5f\n", "distance from goal to start using IK:",solve_for_ik_thresh_m_);
  SBPL_DEBUG_NAMED(fOut,"%40s: %d\n", "cost from goal to start using IK:",solve_for_ik_thresh_);
  SBPL_DEBUG_NAMED(fOut,"\n");
}

void SBPLArmPlannerParams::precomputeSmoothingCosts()
{
  int i,x,y;
  double temp = 0.0;
  
  smoothing_cost_.resize(num_mprims_);

  for (x = 0; x < num_mprims_; x++)
  {
    smoothing_cost_[x].resize(num_mprims_);
    for (y = 0; y < num_mprims_; y++)
    {
      temp = 0.0;
      for (i = 0; i < int(mprims_[y].size()); i++)
        temp += ((mprims_[x][i]-mprims_[y][i])*(mprims_[x][i]-mprims_[y][i]));

      smoothing_cost_[x][y] = temp * (double)cost_multiplier_ * (double)use_smoothing_;
    }
  }
}

void SBPLArmPlannerParams::printSmoothingCosts(FILE* fOut)
{
  int x,y;
  
  if(fOut == NULL)
    fOut = stdout;

  if(smoothing_cost_.empty())
    return;

  SBPL_FPRINTF(fOut,"Smoothing Costs Table:\n");
  SBPL_FPRINTF(fOut,"    ");
  for(x = 0; x < num_mprims_; x++)
    SBPL_FPRINTF(fOut, "%4d  ",x);
  SBPL_FPRINTF(fOut,"\n");

  for (x = 0; x < int(smoothing_cost_.size()); x++)
  {
    SBPL_FPRINTF(fOut,"%2d: ",x);
    for (y = 0; y < int(smoothing_cost_[x].size()); y++)
      SBPL_FPRINTF(fOut,"%4d  ", smoothing_cost_[x][y]);
    SBPL_FPRINTF(fOut,"\n");
  }
}

double SBPLArmPlannerParams::getSmallestShoulderPanMotion()
{
  double min_pan = 360.0;
  for (int i = 0; i < num_mprims_; i++)
  {
    if(mprims_[i][0] > 0 && mprims_[i][0] < min_pan) 
      min_pan = mprims_[i][0];
  }

  min_pan = angles::normalize_angle(angles::from_degrees(min_pan));
  SBPL_PRINTF("[getSmallestShoulderPanMotion] Smallest shoulder pan motion is %0.3f rad.\n", min_pan);
  
  return min_pan;
}

double SBPLArmPlannerParams::getLargestMotionPrimOffset()
{
  double max_offset = 0;
  for (int i = 0; i < num_mprims_; i++)
  {
    for(size_t j = 0; j < mprims_[i].size(); j++)
    {
      if(fabs(mprims_[i][j]) > max_offset)
        max_offset = fabs(mprims_[i][j]);
    }
  }

  max_offset = angles::normalize_angle(angles::from_degrees(max_offset));
  SBPL_PRINTF("[getLargestMotionPrimOffset] Largest single Joint Offset in all Motion Prims is %0.3f rad.\n", max_offset);
  
  return max_offset;
}

