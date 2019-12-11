#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <typeinfo>

#include "dbscan.h"

using namespace std;

//initialize a global publisher
ros::Publisher marker_array_pub;
ros::Publisher object_pub;

//save odometry msg
nav_msgs::Odometry odom;

//int serial = 0;
//std::vector<sensor_msgs::PointCloud2> frame_array(3);


void visualize_object (const vector<Object> &object_array)
{

  int number = object_array.size();

  visualization_msgs::MarkerArray array;

  //visualized shape of objects
  uint32_t shape_o = visualization_msgs::Marker::CYLINDER;
  //visualized shape of velocity
  uint32_t shape_v = visualization_msgs::Marker::ARROW;
  //visualized shape of car
  uint32_t shape_c = visualization_msgs::Marker::CYLINDER;

  visualization_msgs::Marker marker_c;
  marker_c.header.frame_id = "/ti_mmwave";
  marker_c.header.stamp = ros::Time::now();
  marker_c.ns = "car";
  marker_c.type = shape_c;
  marker_c.action = visualization_msgs::Marker::ADD;
  marker_c.lifetime = ros::Duration(0.3);

  //Set car position, scale and color
  marker_c.id = 0;

  marker_c.pose.position.x = 0.0;
  marker_c.pose.position.y = 0.0;
  marker_c.pose.position.z = 0.15;
  marker_c.pose.orientation.x = 0.0;
  marker_c.pose.orientation.y = 0.0;
  marker_c.pose.orientation.z = 0.0;
  marker_c.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker_c.scale.x = 0.6;
  marker_c.scale.y = 0.6;
  marker_c.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker_c.color.r = 0.0f;
  marker_c.color.g = 0.0f;
  marker_c.color.b = 1.0f;
  marker_c.color.a = 1.0;
      
  array.markers.push_back(marker_c);


  for (int i = 0; i < number; i++ ){
    
    //create markers
    visualization_msgs::Marker marker_o, marker_v;

    //general parameters setting
    marker_o.header.frame_id =  marker_v.header.frame_id = "/ti_mmwave";
    
    marker_o.header.stamp = marker_v.header.stamp = ros::Time::now();
    
    marker_o.ns = "objects";
    marker_v.ns = "velocity";
    
    marker_o.id = marker_v.id = i;
    
    //set shape
    marker_o.type = shape_o;
    marker_v.type = shape_v;

    marker_o.action = marker_v.action = visualization_msgs::Marker::ADD;

    marker_o.lifetime = marker_v.lifetime = ros::Duration(0.4);

    //set objects position
    marker_o.pose.position.x = object_array[i].x;
    marker_o.pose.position.y = object_array[i].y;
    marker_o.pose.position.z = 0.5;
    marker_o.pose.orientation.x = 0.0;
    marker_o.pose.orientation.y = 0.0;
    marker_o.pose.orientation.z = 0.0;
    marker_o.pose.orientation.w = 1.0;

    //set objects scale
    marker_o.scale.x = 2*object_array[i].r;
    marker_o.scale.y = 2*object_array[i].r;
    marker_o.scale.z = 0.5;//height
    
    //set objects color
    marker_o.color.r = 1.0f;
    marker_o.color.g = 0.0f;
    marker_o.color.b = 0.0f;
    marker_o.color.a = 1.0;

    //the velocity visualization part
    /*===========================================
    //set velocity position
    geometry_msgs::Point start, end;
    double d = distance(object_array[i].x, object_array[i].y, 0.0, 0.0);

    start.x = object_array[i].x;
    start.y = object_array[i].y;
    start.z = 1.1;

    end.x = object_array[i].x + 2*object_array[i].velocity * (object_array[i].x / d);
    end.y = object_array[i].y + 2*object_array[i].velocity * (object_array[i].y / d);
    end.z = 1.1;

    marker_v.points.push_back(start);
    marker_v.points.push_back(end);
   

    //set velocity scale
    marker_v.scale.x = 0.05;//shaft diameter
    marker_v.scale.y = 0.08;//head diameter
    //marker_o.scale.z = 0.4;//arrow height

    //set velocity color
    marker_v.color.r = 0.0f;
    marker_v.color.g = 1.0f;
    marker_v.color.b = 0.0f;
    marker_v.color.a = 1.0;
    ======================================*/

    array.markers.push_back(marker_o);
    //array.markers.push_back(marker_v);

  }

  //publish marker array of curret frame
  marker_array_pub.publish(array);
}

void positioncallback (const nav_msgs::Odometry input){

  odom = input;
}

void callback (const sensor_msgs::PointCloud2 input_cloud)
{
  //sliding window
  /***vector<sensor_msgs::PointCloud2>::iterator it; 
  it = frame_array.begin();  
  frame_array.erase(it);/ti_mmwave"
  frame_array.push_back(input_cloud);

  sensor_msgs::PointCloud sum_cloud;
  merge_pcl2(frame_array[0], frame_array[1], frame_array[2]);***/

  //convert sensor_msgs::PointCloud2 to sensor_msgs::PointCloud
  sensor_msgs::PointCloud output_cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(input_cloud, output_cloud);
  
  //N is number of points in a frame
  int N = output_cloud.points.size();

  //create variables for points of Point and label
  vector<Point> points(N);
  
  //Cluster label array
  vector<int> labels;
  
  //Object array
  vector<Object> ob;

  //tansfer sensor_msgs::PointCloud to Point
  for (int i = 0; i < N; i++)
  {

    points[i].x = output_cloud.points[i].x;
    points[i].y = output_cloud.points[i].y;
    points[i].z = output_cloud.points[i].z;
    points[i].intensity = output_cloud.channels[0].values[i];
    points[i].velocity = output_cloud.channels[1].values[i]; 

  }

  //num is number of clusters in the current frame
  int num = dbscan(points, labels, 0.7, 3);
  
  cout<<""<<endl;
  cout<<"cluster size is "<<num<<endl;

  //print points of each cluster
  for(int i = 0; i < N; i++)
  {

    cout<<"Point("<<points[i].x<<", "<<points[i].y<<"): "<<labels[i]<<"  velocity:"<<points[i].velocity<<endl;

  }

  //get obstacle information
  if (labels.size() != 0 )
  {

    //return object vector of current frame
    ob = objects(points, labels, num, odom);

  }

 
  //publish detected objects
  ros::Time currenttime = ros::Time::now();

  for(int i = 0; i < ob.size(); i++)
  {
      ti_mmwave_rospkg::Object single_object;


      Object_world ob_in_world;
      //transfer into world frame
      ob_in_world = global_position_and_velocity(ob[i],odom);

      single_object.header.frame_id =  "/world";
      single_object.header.stamp = currenttime;
      single_object.object_id = i;
      single_object.radius = ob[i].r;
      single_object.x_local = ob[i].x;
      single_object.y_local = ob[i].y;
      single_object.x_world = ob_in_world.x;
      single_object.y_world = ob_in_world.y;
      //single_object.vx_world = ob_in_world.vx;
      //single_object.vy_world = ob_in_world.vy;
      //single_object.abusolute_velocity = ob_in_world.absv;
      single_object.relatice_velocity = ob[i].velocity;

      object_pub.publish(single_object);
  }
    
  //show obstacles
  visualize_object(ob);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clustering");
  ros::NodeHandle nh;

  // Get odom transform
  ros::Subscriber sub2 = nh.subscribe("/integrated_to_init", 10, positioncallback);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("/filteredPCL", 1, callback);
  
  // Create a ROS publisher for the output point cloud
  marker_array_pub = nh.advertise<visualization_msgs::MarkerArray> ("/object_marker", 1);

  object_pub = nh.advertise<ti_mmwave_rospkg::Object> ("/object_info", 20);

  // Spin
  ros::spin ();
}
