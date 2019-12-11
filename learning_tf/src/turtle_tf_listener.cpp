#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
  //初始化节点
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;
  
  //通过调用服务，产生第二只海归（服务就是我这边发出请求，接收到请求后开始处理，然后返回数据）
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle =
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);

  //定义turtle2的速度控制发布器
  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
  //TF监听器
  tf::TransformListener listener;
  //10Hz监听TF变换
  ros::Rate rate(10.0);
  while (node.ok()){
    //创建存储坐标变化的数据
    tf::StampedTransform transform;
    try{
      //查找turtle1与turtle2的坐标变化
      listener.waitForTransform("/turtle2","/turtle1", ros::Time(0), ros::Duration(2.0));
      listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform);
    }
    //报错
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    //创建需要发布的速度信息的数据
    geometry_msgs::Twist vel_msg;
    //根据turtle1和turtle2之间的坐标变换，计算turtle2需要的线速度和角速度(twist)
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    //发布速度控制指令
    turtle_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
