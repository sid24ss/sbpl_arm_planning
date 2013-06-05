#include <sbpl_arm_planner_node/add_objects_to_map.h>


AddObjectsToMap::AddObjectsToMap()
{
  object_in_map_pub_ = nh_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 1024);
}

bool AddObjectsToMap::parseObjectsFile(FILE* fCfg, std::vector<std::vector<double> > &objects, std::vector<std::string> &object_ids)
{
  char sTemp[1024];
  int num_obs = 0;

  ROS_INFO("parsing objects file...");

  if(fCfg == NULL)
  {
    ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
    return false;
  }

  // get number of objects
  if(fscanf(fCfg,"%s",sTemp) < 1)
    printf("Parsed string has length < 1.\n"); 
  
  num_obs = atoi(sTemp);  

  ROS_INFO("%i objects in file",num_obs); 

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs);
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1) 
      printf("Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    objects[i].resize(6);
    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1) 
        printf("Parsed string has length < 1.\n");
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
  }

  printObjects(stdout);
  return true;
}

void AddObjectsToMap::addObjectsFromFile(std::string filename)
{
  char* file = new char[filename.length()+1];
  filename.copy(file, filename.length(),0);
  file[filename.length()] = '\0';
  FILE* fCfg = fopen(file, "r");

  if(!parseObjectsFile(fCfg, objects_, object_ids_))
  {
    ROS_ERROR("failed to parse object file: %s", file);
    return;
  }

  addBoxes(objects_, object_ids_);
}

void AddObjectsToMap::addBoxes(std::vector<std::vector<double> > &objects)
{
  std::vector<std::string> object_ids(objects.size(), "cube");

  for(size_t i = 0; i < object_ids.size(); i++)
    object_ids[i] = "cube_" + boost::lexical_cast<std::string>(i);

  addBoxes(objects, object_ids);
}

void AddObjectsToMap::addBoxes(std::vector<std::vector<double> > &objects, std::vector<std::string> &object_ids)
{
  std::vector<double> dims(3,0);
  geometry_msgs::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  
  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return;
  }

  for(size_t i = 0; i < objects.size(); i++)
  {
      pose.position.x = objects[i][0];
      pose.position.y = objects[i][1];
      pose.position.z = objects[i][2];
      dims[0] = objects[i][3];
      dims[1] = objects[i][4];
      dims[2] = objects[i][5];

      addBox(pose, dims, object_ids.at(i));
  }
}

void AddObjectsToMap::addBox(geometry_msgs::Pose pose, std::vector<double> &dims, std::string id)
{
  arm_navigation_msgs::CollisionObject object;
  object.id = id;
  object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  object.header.frame_id = "base_link";
  object.header.stamp = ros::Time::now();
  
  arm_navigation_msgs::Shape box_object;
  box_object.type = arm_navigation_msgs::Shape::BOX;
  box_object.dimensions.resize(3);
  box_object.dimensions[0] = dims[0];
  box_object.dimensions[1] = dims[1];
  box_object.dimensions[2] = dims[2];

  object.shapes.push_back(box_object);
  object.poses.push_back(pose);
 
  sleep(1); 
  object_in_map_pub_.publish(object);
  ROS_INFO("published %s", id.c_str());
}

void AddObjectsToMap::printObjects(FILE * fOut)
{
  if(object_ids_.size() != objects_.size())
  {
    fprintf(fOut,"object_id list and objects list have different sizes\n");
    return;
  }

  for(size_t i = 0; i < objects_.size(); i++)
  {
    fprintf(fOut,"%s: %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n",object_ids_[i].c_str(),objects_[i][0],objects_[i][1],objects_[i][2],objects_[i][3],objects_[i][4],objects_[i][5]);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"add_object_to_map");
  sleep(2);
  AddObjectsToMap addobj;

  ROS_INFO("Adding known objects to map 5 times.");
  int i = 0;
  while(i<5)
  { 
    if(argc > 1)
      addobj.addObjectsFromFile(std::string(argv[1]));
    else
      addobj.addObjectsFromFile(std::string("objects.txt"));

    ROS_INFO("%d: added objects to collision map", i);

    ros::spinOnce();
    sleep(5);
    i++;
  }

  return 0;
}

