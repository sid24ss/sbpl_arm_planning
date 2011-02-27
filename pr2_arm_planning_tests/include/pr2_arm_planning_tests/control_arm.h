#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <angles/angles.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class ControlArm
{

  public:

    ControlArm(); 

    ~ControlArm();

    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal);

    pr2_controllers_msgs::JointTrajectoryGoal setArmConfiguration(std::vector<double> &waypoint);

    actionlib::SimpleClientGoalState getState();

  private:
    ros::NodeHandle nh_;
    TrajClient* traj_client_;
};

