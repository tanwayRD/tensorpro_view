#include <ros/ros.h>
#include <ros/console.h>
#include "tensorpro_view/TanwayTensor.h"



int main(int argc, char** argv) {
  ros::init(argc, argv, "tensorproview"); 
  ros::NodeHandle nh;
  ros::Rate r(20);

  ros::NodeHandle nh_private("~");


  ROS_INFO( "tensorpro viewer for ROS" );
  ROS_INFO( "Version 1.1.2" );
  ROS_INFO( "Update Date: 2020/04/16\n" );

  ROS_INFO( "View in rviz;");
  ROS_INFO( "topic= tensorpro_cloud and fixed frame= TanwayTP");

  TanwayTensor TensorPro;

  if (!TensorPro.initialize(nh,nh_private))
    return 0;

  while (ros::ok()){
    TensorPro.getPoints();
  }
  return 0;
}
