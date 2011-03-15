/*
 * Copyright (c) 2009, Maxim Likhachev
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

/** \author Benjamin Cohen, Maxim Likhachev */

#include <sbpl_arm_planner/bfs_3d.h>


BFS3D::BFS3D(int dim_x, int dim_y, int dim_z, int radius, int cost_per_cell)
{
  if(dim_x < 0 || dim_y < 0 || dim_z < 0)
    SBPL_ERROR("Dimensions must have positive values. Fix this.\n");

  dimX_ = dim_x;
  dimY_ = dim_y;
  dimZ_ = dim_z;
  radius_ = radius;

  cost_1_move_ = cost_per_cell;
  cost_sqrt2_move_ = cost_per_cell*sqrt(2.0);
  cost_sqrt3_move_ = cost_per_cell*sqrt(3.0);

  enable_df_ = false; //configDistanceField() should be called to enable
  radius_m_ = 0.1;

  SBPL_DEBUG("goal bounds: %d %d %d\n",dimX_, dimY_,dimZ_);
}

BFS3D::~BFS3D()
{
  if (grid3D_ != NULL)
  {
    for (short unsigned int x = 0; x < dimX_; x++)
    {
      for (short unsigned int y = 0; y < dimY_; y++)
        delete [] grid3D_[x][y];
      delete [] grid3D_[x];
    }
    delete [] grid3D_;
    grid3D_ = NULL;
  }
}

void BFS3D::init()
{
  short unsigned int x,y,z;
  grid3D_ = new unsigned char** [dimX_];
  for (x = 0; x < dimX_; x++)
  {
    grid3D_[x] = new unsigned char* [dimY_];
    for (y = 0; y < dimY_; y++)
    {
      grid3D_[x][y] = new unsigned char [dimZ_];
      for (z = 0; z < dimZ_; z++)
      {
        grid3D_[x][y][z] = 255;
      }
    }
  }
}

bool BFS3D::setGoal(std::vector<short unsigned int> goal)
{
  if(goal.empty() || goal.size() < 3)
    return false;

  goal_.clear();

  if(goal[0] < dimX_ && goal[1] < dimY_ && goal[2] < dimZ_)
    goal_.push_back(goal);

  if(goal_.empty())
  { 
    SBPL_ERROR("[bfs3d] Error: No valid goals were received.\n");
    return false;
  }
  return true;
}

bool BFS3D::setGoals(std::vector<std::vector<short unsigned int> > goals)
{
  if(goals.size() <= 0)
  {
    SBPL_ERROR("[bfs3d] No goal cell received. Exiting.\n");
    return false;
  }

  goal_.clear();

  //check if received goals are valid
  for(unsigned int i = 0; i < goals.size(); ++i)
  {
    if(goals[i].size() < 3)
      continue;

    if(goals[i][0] < dimX_ && goals[i][1] < dimY_ && goals[i][2] < dimZ_)
      goal_.push_back(goals[i]);
    else
      SBPL_DEBUG("Goal: %u %u %u is invalid.\n",goals[i][0],goals[i][1],goals[i][2]);
  }

  if(goal_.empty())
  {
    SBPL_ERROR("Error: No valid goals were received.\n");
    return false;
  }
  return true;
}

void BFS3D::reInitializeState3D(State3D* state)
{
  state->g = INFINITE_COST;
  state->iterationclosed = 0;
}

void BFS3D::initializeState3D(State3D* state, short unsigned int x, short unsigned int y, short unsigned int z)
{
  state->g = INFINITE_COST;
  state->iterationclosed = 0;
  state->x = x;
  state->y = y;
  state->z = z;
}

void BFS3D::create3DStateSpace(State3D**** statespace3D)
{
  short unsigned int  x,y,z;

  *statespace3D = new State3D** [dimX_];
  for (x = 0; x < dimX_; x++)
  {
    (*statespace3D)[x] = new State3D* [dimY_];
    for(y = 0; y < dimY_; y++)
    {
      (*statespace3D)[x][y] = new State3D [dimZ_];
      for(z = 0; z < dimZ_; z++)
      {
        initializeState3D(&(*statespace3D)[x][y][z],x,y,z);
      }
    }
  }
}

void BFS3D::delete3DStateSpace(State3D**** statespace3D)
{
  short unsigned int x,y;

  for (x = 0; x < dimX_; x++)
  {
    for (y = 0; y < dimY_; y++)
      delete [] (*statespace3D)[x][y];

    delete [] (*statespace3D)[x];
  }
  delete *statespace3D;
}

bool BFS3D::runBFS()
{
#if DEBUG_TIME
  clock_t currenttime = clock();
#endif

  if(goal_.empty())
  {
    SBPL_ERROR("[bfs3d] Goal location is not set. Exiting.\n");
    return false;
  }

  dist_length_ =  (dimX_-1) + (dimY_-1)*(dimX_) + (dimZ_-1)*(dimX_)*(dimY_) + 1;
  dist_.resize(dist_length_);

  State3D*** statespace3D;
  create3DStateSpace(&statespace3D);

  search3DwithQueue(statespace3D);

  delete3DStateSpace(&statespace3D);
#if DEBUG_TIME
  SBPL_DEBUG("completed in %.3f seconds.\n", double(clock()-currenttime) / CLOCKS_PER_SEC);
#endif

  return true;
}

void BFS3D::search3DwithQueue(State3D*** statespace)
{
  State3D* ExpState;
  int newx, newy, newz;
  short unsigned int x,y,z;
  unsigned int g_temp;

  //these are added here temporarily. should be in the class
  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

  //create a queue
  queue<State3D*> Queue;

  //initialize to infinity all
  for (x = 0; x < dimX_; x++)
  {
    for (y = 0; y < dimY_; y++)
    {
      for (z = 0; z < dimZ_; z++)
      {
        dist_[xyzToIndex(x,y,z)] = INFINITE_COST;
        reInitializeState3D(&statespace[x][y][z]);
      }
    }
  }

  //initialization - throw starting states on queue with g cost = 0
  for(unsigned int i = 0; i < goal_.size(); ++i)
  {
    statespace[goal_[i][0]][goal_[i][1]][goal_[i][2]].g = 0;
    Queue.push(&statespace[goal_[i][0]][goal_[i][1]][goal_[i][2]]);
  }

  //expand all of the states
  while((int)Queue.size() > 0)
  {
    //get the state to expand
    ExpState = Queue.front();

    Queue.pop();

    //it may be that the state is already closed
    //if(ExpState->iterationclosed == 1)
    //  continue;

    //close it
    //ExpState->iterationclosed = 1;

    //set the corresponding distances to the goal
    dist_[xyzToIndex(ExpState->x, ExpState->y, ExpState->z)] = ExpState->g;

    //iterate through neighbors
    for(int d = 0; d < DIRECTIONS3D; d++)
    {
      newx = ExpState->x + dx[d];
      newy = ExpState->y + dy[d];
      newz = ExpState->z + dz[d];

      //if(!isValidCell(newx,newy,newz))
      //  continue;
      
      //make sure it is inside the map and has no obstacle
      if(0 > newx || newx >= dimX_ || 0 > newy || newy >= dimY_ || 0 > newz || newz >= dimZ_)
        continue;

      if(statespace[newx][newy][newz].iterationclosed == 0)
      {
        ExpState->iterationclosed = 1;

        if(!isValidCell(newx,newy,newz))
          continue;
  
       //insert into the stack
        Queue.push(&statespace[newx][newy][newz]);

        //set the g-value
        if (ExpState->x != newx && ExpState->y != newy && ExpState->z != newz)
          g_temp = ExpState->g + cost_sqrt3_move_;
        else if ((ExpState->y != newy && ExpState->z != newz) ||
            (ExpState->x != newx && ExpState->z != newz) ||
            (ExpState->x != newx && ExpState->y != newy))
          g_temp = ExpState->g + cost_sqrt2_move_;
        else
          g_temp = ExpState->g + cost_1_move_;

        if(statespace[newx][newy][newz].g > g_temp)
          statespace[newx][newy][newz].g = g_temp;
        // SBPL_PRINTF("%i: %u,%u,%u, g-cost: %i\n",d,newx,newy,newz,statespace[newx][newy][newz].g);
      }
    }
  }
}

bool BFS3D::isGoal(const std::vector<int> &state)
{
  for(unsigned int i = 0; i < goal_.size(); ++i)
  {
    if((state[0] <= goal_[i][0]+GOAL_TOLERANCE && state[0] >= goal_[i][0]-GOAL_TOLERANCE) && (state[1] <= goal_[i][1]+GOAL_TOLERANCE && state[1] >= goal_[i][1]-GOAL_TOLERANCE) && (state[2] <= goal_[i][2]+GOAL_TOLERANCE && state[2] >= goal_[i][2]-GOAL_TOLERANCE))
    {
      //SBPL_PRINTF("the goal is found: %d %d %d\n", state[0],state[1],state[2]);
      return true;
    }
  }
  return false;
}

bool BFS3D::getShortestPath(std::vector<short unsigned int> start, std::vector<std::vector<int> > &path)
{
  int val = 0, cntr = 0, min_val = INFINITE_COST;
  std::vector<int> state(3,0);
  std::vector<int> next_state(3,0);
  int newx,newy,newz;

  int dx[DIRECTIONS3D] = { 1,  1,  1,  0,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0, -1, -1, -1,    1,  1,  1,  0,  0,  0, -1, -1, -1};
  int dy[DIRECTIONS3D] = { 1,  0, -1,  1,  0, -1, -1,  0,  1,    1,  0, -1,  1, -1, -1,  0,  1,    1,  0, -1,  1,  0, -1, -1,  0,  1};
  int dz[DIRECTIONS3D] = {-1, -1, -1, -1, -1, -1, -1, -1, -1,    0,  0,  0,  0,  0,  0,  0,  0,    1,  1,  1,  1,  1,  1,  1,  1,  1};

  path.resize(0);
  next_state[0] = start[0];
  next_state[1] = start[1];
  next_state[2] = start[2];

  while(!isGoal(next_state) || cntr >= dimX_*dimY_)
  {
    state = next_state;
    min_val = INFINITE_COST;

    //iterate through neighbors
    for(int d = 0; d < DIRECTIONS3D; d++)
    {
      newx = state[0] + dx[d];
      newy = state[1] + dy[d];
      newz = state[2] + dz[d];

      //make sure it is inside the map and has no obstacle
      if(0 > newx || newx >= dimX_ || 0 > newy || newy >= dimY_ || 0 > newz || newz >= dimZ_)
        continue;

      val = dist_[xyzToIndex(newx,newy,newz)];

      if(val >= INFINITE_COST)
        continue;

      if (state[0] != newx && state[1] != newy && state[2] != newz)
        val += cost_sqrt3_move_;
      else if ((state[1] != newy && state[2] != newz) ||
          (state[0] != newx && state[2] != newz) ||
          (state[0] != newx && state[1] != newy))
        val += cost_sqrt2_move_;
      else
        val += cost_1_move_;

      if(val < min_val)
      {
        min_val = val;
        next_state[0] = newx;
        next_state[1] = newy;
        next_state[2] = newz;
      }
    }
    path.push_back(next_state);

    //make sure that we aren't looping forever...
    cntr++;
  }

/*  
  SBPL_PRINTF("path:\n");
  for(unsigned int i = 0; i < path.size(); ++i)
    SBPL_PRINTF("%d: %d %d %d\n", i, path[i][0],path[i][1],path[i][2]);
  SBPL_PRINTF("------------\n");
*/
  return true;
}

void BFS3D::configDistanceField(bool enable, const distance_field::PropagationDistanceField* df)
{
  enable_df_ = enable;
  df_ = df;

  radius_m_ = double(radius_ * df_->getResolution(distance_field::PropagationDistanceField::DIM_X));
}

bool BFS3D::isValidCell(const int x, const int y, const int z)
{
  if(enable_df_)
  {
    if(df_->getDistanceFromCell(x,y,z) <= radius_m_)
      return false;
  }
  else
  {  
    if(grid3D_[x][y][z] <= radius_)
      return false;
  }

  return true;
}

void BFS3D::printConfig(FILE* fOut)
{
  SBPL_FPRINTF(fOut,"BFS_3D Configuration:\n");
  SBPL_FPRINTF(fOut,"dimX: %d   dimY: %d   dimZ: %d\n",dimX_,dimY_,dimZ_);
  SBPL_FPRINTF(fOut,"radius: %d   radius(meters): %0.3f\n",radius_,radius_m_);
}

void BFS3D::printGrid()
{
  for(unsigned int z = 0; z < dimZ_; ++z)
  {
    SBPL_DEBUG("---------------------------------");
    SBPL_DEBUG("z: %u",z);
    for(unsigned int x = 0; x < dimX_; ++x)
    {
      for(unsigned int y = 0; y < dimY_; ++y)
        SBPL_DEBUG("%u ", grid3D_[x][y][z]);
    }
  }
}

void BFS3D::printCostToGoal()
{
  {
    for(unsigned int z = 0; z < dimZ_; ++z)
    {
      SBPL_DEBUG("---------------------------------");
      SBPL_DEBUG("z: %u",z);
      for(unsigned int x = 0; x < dimX_; ++x)
      {
        for(unsigned int y = 0; y < dimY_; ++y)
          SBPL_DEBUG("%u ", dist_[xyzToIndex(x,y,z)]);
      }
    }
  }
}


