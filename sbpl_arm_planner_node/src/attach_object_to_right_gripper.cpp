#include <ros/ros.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
#include <arm_navigation_msgs/Shape.h>
#include <resource_retriever/retriever.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

/*
bool getTrianglesFromMeshFile(std::string mesh_file, std::vector<int32_t> &triangles, std::vector<geometry_msgs::Point> &vertices)
{
  bool retval = false;
  shapes::Shape *mesh = NULL;

  if (!mesh_file.empty())
  {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    bool ok = true;

    try
    {
      res = retriever.get(mesh_file);
    }
    catch (resource_retriever::Exception& e)
    {
      ROS_ERROR("%s", e.what());
      ok = false;
    }

    if (ok)
    {
      if (res.size == 0)
        ROS_WARN("Retrieved empty mesh for resource '%s'", mesh_file.c_str());
      else
      {
        mesh = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
        if (mesh == NULL)
          ROS_ERROR("Failed to load mesh '%s'", mesh_file.c_str());
        else
          retval = true;
      }
    }
  }
  else
    ROS_WARN("Empty mesh filename");

  if(retval)
  {
    triangles.resize(static_cast<shapes::Mesh*>(mesh)->triangleCount*3);
    for(size_t i=0; i<triangles.size(); ++i)
      triangles[i] = static_cast<shapes::Mesh*>(mesh)->triangles[i];

    ROS_INFO("vertexCount: %d  triangleCount: %d", static_cast<shapes::Mesh*>(mesh)->vertexCount, static_cast<shapes::Mesh*>(mesh)->triangleCount);

    vertices.resize((static_cast<shapes::Mesh*>(mesh)->vertexCount));
    for(size_t i=0; i<vertices.size(); ++i)
    {
      vertices[i].x = static_cast<shapes::Mesh*>(mesh)->vertices[3*i];
      vertices[i].y = static_cast<shapes::Mesh*>(mesh)->vertices[3*i+1];
      vertices[i].z = static_cast<shapes::Mesh*>(mesh)->vertices[3*i+2];
    }
  }

  return retval;
}
*/

int main(int argc, char** argv) {

  ros::init(argc, argv, "sbpl_attach_object_right_arm");
  double radius=0,length=0;
  std::string shape, mesh_file, operation;
  ros::Publisher att_object_in_map_pub_;
  ros::NodeHandle nh;
  arm_navigation_msgs::Shape object;
  geometry_msgs::Pose pose;

  att_object_in_map_pub_  = nh.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  sleep(2);

  ros::NodeHandle ph("~");
  ph.param<std::string>("shape",shape,"sphere");
  ph.param<std::string>("operation",operation,"add");
  ph.param<std::string>("mesh_filename",mesh_file,"");
  ph.param<double>("radius",radius,0.025);
  ph.param<double>("length",length,0.5);

  //add a cylinder into the collision space attached to the r_gripper_r_finger_tip_link
  arm_navigation_msgs::AttachedCollisionObject att_object;
  att_object.link_name = "r_gripper_r_finger_tip_link";
  att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.touch_links.push_back("r_gripper_r_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_tip_link");

  att_object.object.id = "sbpl_attached_" + shape;
  att_object.object.header.frame_id = "r_gripper_r_finger_tip_link";
  att_object.object.header.stamp = ros::Time::now();

  if(operation.compare("remove") == 0)
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  else
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  if(shape.compare("mesh") == 0)
  {
    ROS_ERROR("Not supporting meshes.");
    /*
    object.type = arm_navigation_msgs::Shape::MESH;
    
    if(!getTrianglesFromMeshFile(mesh_file, object.triangles, object.vertices))
    {
      ROS_ERROR("Unable to retrieve mesh file. Exiting");
      return 0;
    }
    */
  }
  else if(shape.compare("cylinder") == 0)
  {
    object.type = arm_navigation_msgs::Shape::CYLINDER;
    object.dimensions.resize(2);
    object.dimensions[0] = radius;
    object.dimensions[1] = length;
  }
  else if(shape.compare("box") == 0)
  {
    ROS_INFO("Box is not yet implemented. Using a sphere.");
    object.type = arm_navigation_msgs::Shape::SPHERE;
    //object.type = arm_navigation_msgs::Shape::BOX;
  }
  else
  {
    object.type = arm_navigation_msgs::Shape::SPHERE;
    object.dimensions.resize(1);
    object.dimensions[0] = radius;
  }

  pose.position.x = 0.0;
  pose.position.y = 0.03;
  pose.position.z = 0.0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  att_object.object.shapes.push_back(object);
  att_object.object.poses.push_back(pose);

  ros::Rate r(0.1);  
  while(nh.ok())
  {
    att_object_in_map_pub_.publish(att_object);
    r.sleep();
  }

  ROS_INFO("Should have published");

  ros::Duration(2.0).sleep();

  ros::shutdown();
  return 0;
}

