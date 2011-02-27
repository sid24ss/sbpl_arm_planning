/* \author Benjamin Cohen */

#include <string>
#include <fstream>
#include <stdlib.h>
#include <ros/ros.h>
#include <math.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf/tf.h>
#include <move_arm_msgs/MoveArmAction.h>
#include <move_arm_msgs/utils.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <boost/lexical_cast.hpp>
#include <motion_planning_msgs/DisplayTrajectory.h>
#include <planning_environment/monitors/joint_state_monitor.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include <sbpl_arm_planner/robarm3d/environment_robarm3d.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

namespace sbpl_arm_planner{
  
class VisualizeArm
{
  public:
    
    VisualizeArm();

    ~VisualizeArm();

    /* \brief set reference frame of visualizations */
    void setReferenceFrame(std::string &frame);

    /* \brief send a joint configuration to the arm controller */
    void sendArmToConfiguration(const std::vector<double> &angles);

    //void sendGoalToMoveArm(const std::vector<double> &pose);

    /* \brief send a trajectory to the motion planning plugin for rviz */
    void displayArmConfiguration(std::vector<double> angles);

    /* \brief initialize the KDL chain for the robot arm */
    bool initKDLChain();

    /* \brief compute FK for the pr2 arm meshes using the KDL chain */
    bool computeFKforVisualizationWithKDL(const std::vector<double> &jnt_pos, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief compute FK for a joint configuration using the KDL chain */
    bool computeFKwithKDL(const std::vector<double> angles, int frame_num, geometry_msgs::Pose &pose);

    /* \brief compute FK for the pr2 arm meshes using the kinematic service*/
    bool computeFKforVisualization(const std::vector<double> &jnt_pos, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief visualize the pr2 arm in the configuration */
    void visualizeArmConfiguration(double color_num, const std::vector<double> &jnt_pos);

    /* \brief visualize the arm of the pr2 in specified color (color: 1-360) */
    void visualizeArmMeshes(double color_num, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(const std::vector<double> &pose, std::string text);

    void visualizePose(const geometry_msgs::Pose &pose, std::string text);

    /* \brief visualize a list of poses (sphere, arrow, pose index number) */
    void visualizePoses(const std::vector<std::vector<double> > &poses);
   
    /* \brief visualize the trajectory stored in a CSV file */
    void visualizeTrajectoryFile(std::string filename, int throttle);

    /* \brief visualize the environment described by an env file */
    void visualizeEnvironment(std::string filename);

    /* \brief visualize cuboids */
    void visualizeObstacles(const std::vector<std::vector<double> > &obstacles);
   
    void visualize3DPath(std::vector<std::vector<double> > &dpath);

    /* \brief parse a CSV file - note: no comma at end of last line! */
    bool parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data);

    /* \brief parse a CSV file that contains poses - should rewrite it to
     * use parseCSVFile() */
    bool parsePoseFile(std::string filename, std::vector<std::vector<double> > &poses);

    /* \brief parse an environment file - currently not very robust at all */
    bool parseEnvironmentFile(std::string filename);

    /* \brief visualize in rviz each a throttled set of arm configurations in a trajectory
     * by default throttle = 5 */
    void visualizeArmConfigurations(const std::vector<std::vector<double> > &traj, int throttle);

    /* \brief print out the information that was parsed from the env file */
    void printEnvironmentInfo(FILE *fid);

    /* \brief display the cylindrical collision model of each waypoint in a path */
    void visualizeCollisionModel(const std::vector<std::vector<double> > &path);

    void visualizeGripperConfiguration(double color_num, const std::vector<double> &jnt_pos);

    void visualizeGripperMeshes(double color_num, std::vector<geometry_msgs::PoseStamped> &poses);

    void visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size);

    void visualizeSphere(std::vector<double> pose, int color, std::string text, double radius);

    void visualizeJointTrajectoryMsg(trajectory_msgs::JointTrajectory traj_msg, int throttle);

    void visualizeCollisionModelFromJointTrajectoryMsg(trajectory_msgs::JointTrajectory &traj_msg);

    void visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size);

    /* \brief DOESN'T WORK */
    void clearAllVisualizations();

    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher marker_array_publisher_;
    ros::Publisher marker_publisher_;
    ros::Publisher display_trajectory_publisher_;

    std::string reference_frame_;

    planning_environment::JointStateMonitor joint_state_monitor_;

    TrajClient* traj_client_;

    int num_joints_;

    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker marker_;

    std::vector<std::string> joint_names_;

    std::vector<std::string> pr2_arm_meshes_;
    std::vector<std::string> pr2_gripper_meshes_;

    std::vector<geometry_msgs::Vector3> pr2_arm_meshes_scale_;

    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;
    KDL::Chain chain_;
    KDL::Chain gripper_l_chain_;
    KDL::Chain gripper_r_chain_;
    KDL::ChainFkSolverPos_recursive *fk_solver_;
    KDL::ChainFkSolverPos_recursive *gripper_l_fk_solver_;
    KDL::Tree kdl_tree_;

    std::string chain_root_name_;
    std::string chain_tip_name_;

    std::vector<double> start_config_;
    std::vector<double> goal_pose_;
    std::vector<std::vector<double> > cubes_;
    double position_tolerance_;
    double orientation_tolerance_;
    bool goal_is_6dof_;
};

}
