/* /author Benjamin Cohen */

#include <ros/ros.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>
//#include <sbpl_arm_planner/sbpl_geometry_utils.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "attach_object");
  double radius=0,length=0, dim_x=0, dim_y=0, dim_z=0;
  std::string shape, mesh_file, operation, arm_side_prefix, id;
  ros::Publisher att_object_in_map_pub_;
  ros::NodeHandle nh;
  arm_navigation_msgs::Shape object;
  geometry_msgs::Pose pose;

  att_object_in_map_pub_  = nh.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 2);
  sleep(2);

  ros::NodeHandle ph("~");
  ph.param<std::string>("shape",shape,"sphere");
  ph.param<std::string>("operation",operation,"add");
  ph.param<std::string>("mesh_filename",mesh_file,"");
  ph.param<std::string>("arm_side_prefix", arm_side_prefix, "r");
  ph.param<std::string>("object_id", id, "attached_object");
  ph.param<double>("radius",radius,0.025);
  ph.param<double>("length",length,0.5);
  ph.param<double>("dim_x",dim_x,0.05);
  ph.param<double>("dim_y",dim_y,0.05);
  ph.param<double>("dim_z",dim_z,0.05);

  if(arm_side_prefix.compare("r") != 0 && arm_side_prefix.compare("l") != 0)
  {
    ROS_ERROR("The arm_side_prefix param has to be set to 'r' or 'l'. (currently: %s)", arm_side_prefix.c_str());
    return 0;
  }
  arm_navigation_msgs::AttachedCollisionObject att_object;
  att_object.link_name = arm_side_prefix+"_gripper_r_finger_tip_link";
  att_object.touch_links.push_back(arm_side_prefix+"_gripper_palm_link");
  att_object.touch_links.push_back(arm_side_prefix+"_gripper_r_finger_link");
  att_object.touch_links.push_back(arm_side_prefix+"_gripper_l_finger_link");
  att_object.touch_links.push_back(arm_side_prefix+"_gripper_l_finger_tip_link");

  att_object.object.id = id;
  att_object.object.header.frame_id = arm_side_prefix+"_gripper_r_finger_tip_link";
  att_object.object.header.stamp = ros::Time::now();

  if(operation.compare("remove") == 0)
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  else
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  
  if(shape.compare("mesh") == 0)
  {
    ROS_ERROR("Can't attach meshes....need to update the code.");
    /*
    object.type = arm_navigation_msgs::Shape::MESH;
    if(mesh_file.empty())
    {
      ROS_ERROR("You forgot to set the mesh filename dummy.");
      return 0;
    }

    if(!sbpl_geometry_utils::getTrianglesFromMeshFile("file://" + mesh_file, object.triangles, object.vertices))
    {
      ROS_ERROR("Failed to parse mesh file. Exiting.");
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
    object.type = arm_navigation_msgs::Shape::BOX;
    object.dimensions.resize(3);
    object.dimensions[0] = dim_x;
    object.dimensions[1] = dim_y;
    object.dimensions[2] = dim_z;
  }
  else
  {
    object.type = arm_navigation_msgs::Shape::SPHERE;
    object.dimensions.resize(1);
    object.dimensions[0] = radius;
  }

  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  att_object.object.shapes.push_back(object);
  att_object.object.poses.push_back(pose);


  att_object_in_map_pub_.publish(att_object);

  /*
  ros::Rate r(0.1);  
  while(nh.ok())
  {
    att_object_in_map_pub_.publish(att_object);
    r.sleep();
  }
  */

  ros::Duration(2.0).sleep();
  ros::shutdown();
  return 0;
}

