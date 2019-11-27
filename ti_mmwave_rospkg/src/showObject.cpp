#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <typeinfo>
#include <geometry_msgs/Point.h>
#include "dbscan.h"

using namespace std;

//initialize a global publisher
ros::Publisher marker_array_pub;

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

    marker_o.lifetime = marker_v.lifetime = ros::Duration(0.3);

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
    marker_o.scale.z = 1.0;
    
    //set objects color
    marker_o.color.r = 1.0f;
    marker_o.color.g = 0.0f;
    marker_o.color.b = 0.0f;
    marker_o.color.a = 1.0;

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
    

    array.markers.push_back(marker_o);
    array.markers.push_back(marker_v);

  }

  //publish marker array of curret frame
  marker_array_pub.publish(array);
}

void callback (const sensor_msgs::PointCloud2 input_cloud)
{
  //sliding window
  /***vector<sensor_msgs::PointCloud2>::iterator it; 
  it = frame_array.begin();  
  frame_array.erase(it);
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
  vector<int> labels;
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

  //num is number of clusters
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

    ob = objects(points, labels, num);

  }

  visualize_object(ob);
    

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clustering");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filteredPCL", 1, callback);

  // Create a ROS publisher for the output point cloud
  marker_array_pub = nh.advertise<visualization_msgs::MarkerArray> ("/object_marker", 1);

  // Spin
  ros::spin ();
}
