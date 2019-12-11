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

  //设置transform的平移参数
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;

  //设置transform的转动参数
  odom_trans.transform.rotation = odom.pose.pose.orientation;

  //turtle_name在世界坐标系下的点坐标setOrigin,与世界坐标系之间的转角setRotation
  br.sendTransform(odom_trans);
  //1.transform 2.timestamp 3.name of parent frame 4.name of child frame


}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");

  ros::NodeHandle node;

  ros::Subscriber sub = node.subscribe("/integrated_to_init", 10, &poseCallback);//先订阅位姿消息再TF广播
  

  std::cout << "Start boardcasting tf..." << std::endl;

  

  ros::spin();
  return 0;
};

