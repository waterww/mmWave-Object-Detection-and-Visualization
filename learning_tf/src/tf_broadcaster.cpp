#include <ros/ros.h>
#include <ostream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>



void poseCallback(const nav_msgs::Odometry odom){
  static tf::TransformBroadcaster br;
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "/world";
  odom_trans.child_frame_id = "/ti_mmwave";

  double x = odom.pose.pose.position.x;
  double y = odom.pose.pose.position.y;

  //set translation parameters
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;

  //set rotation parameters
  odom_trans.transform.rotation = odom.pose.pose.orientation;

  //broadcast tf transform
  br.sendTransform(odom_trans);
  //1.transform 2.timestamp 3.name of parent frame 4.name of child frame


}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle node;
  
  //subscribe odometry topic and broadcast tf transform
  ros::Subscriber sub = node.subscribe("/integrated_to_init", 10, &poseCallback);
  

  std::cout << "Start boardcasting tf..." << std::endl;

  

  ros::spin();
  return 0;
};

