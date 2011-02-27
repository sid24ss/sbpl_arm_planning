#include <ros/ros.h>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <move_arm_msgs/MoveArmAction.h>
#include <motion_planning_msgs/DisplayTrajectory.h>
//#include <move_arm_msgs/utils.h>
#include <sbpl_arm_planner/add_objects_to_map.h>
#include <pr2_arm_planning_tests/control_arm.h>
#include <sbpl_arm_planner_node/visualize_arm.h>

class ArmPlanningTest
{
  public:
    
    ArmPlanningTest();
    ~ArmPlanningTest(){};

    bool parseEnvironmentFile(std::string filename);

    void addCubesToEnvironment();

    void addCubesToEnvironment(std::vector<std::vector<double> > &cubes);

    bool callMoveArm();

    void printTest(FILE* fout);

    void checkParams();

    void sendArmToStart();

    void visualizePlan();

    void printTrajectory(FILE* fout);

    bool runTestFromParamServer();

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Subscriber plan_subscriber_;
      
    bool disable_ik_;
    bool disable_collision_monitoring_;
    bool accept_partial_plans_;
    double position_tolerance_;
    double orientation_tolerance_;
    double movearm_duration_;
    double max_planning_time_;

    std::string filename_;
    std::string goal_frame_;
    std::string planning_link_;
    std::string planner_name_;    
    std::string test_file_path_;    
    std::string test_file_name_;    
    std::string planning_service_name_;
    std::string trajectory_file_path_;
    std::vector<double> goal_;
    std::vector<std::vector<double> > cubes_;
    std::vector<std::vector<double> > plan_;
    std::vector<double> start_angles_;

    AddObjectsToMap addobj_;
    //ControlArm arm_;
    
    //sbpl_arm_planner::VisualizeArm *aviz_;

    void motionPlanCallback(const motion_planning_msgs::DisplayTrajectory &plan);

};

