#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/Shape.h>

class AddObjectsToMap
{
  public:

    AddObjectsToMap();
    ~AddObjectsToMap(){};

    bool parseObjectsFile(FILE* fCfg, std::vector<std::vector<double> > &objects, std::vector<std::string> &object_ids);

    void addObjectsFromFile(std::string filename);
    
    void addBox(geometry_msgs::Pose pose, std::vector<double> &dims, std::string id);

    void addBoxes(std::vector<std::vector<double> > &objects);

    void addBoxes(std::vector<std::vector<double> > &objects, std::vector<std::string> &object_ids);

    void printObjects(FILE * fOut);

  private:

    ros::NodeHandle nh_;
    ros::Publisher object_in_map_pub_;
    
    std::vector<std::vector<double> > objects_;
    std::vector<std::string> object_ids_;
};


