#include <iostream>
#include <string>
#include <ros/ros.h>
#include <pr2_arm_planning_tests/arm_planning_test.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"arm_planning_test");

  ArmPlanningTest armtest;

  if(argc > 1)
    armtest.parseEnvironmentFile(std::string(argv[1]));
  else
  {
    ROS_INFO("Checking param server for the test file...");
    if(!armtest.runTestFromParamServer())
    {
      ROS_ERROR("Unable to get environment from param server");
      return 0;
    }
  }

  armtest.printTest(stdout);
  
  sleep(3);
  
  armtest.addCubesToEnvironment();

//  armtest.sendArmToStart();
//  sleep(2);

/*
  if(armtest.callMoveArm())
    ROS_INFO("Called move_arm");
  else
    ROS_WARN("TEST FAILED");
*/
  ros::spin();
  return 1;
}

