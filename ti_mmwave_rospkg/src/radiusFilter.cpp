#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void statistical_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform statistical outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> Static;
  Static.setMeanK(20);//set number fo nearby points
  Static.setStddevMulThresh(1.2);//coefficient for variance
  Static.setInputCloud(cloud);//input cloud
  Static.filter(cloud_filtered);//output cloud

  // Publish the data
  pub.publish (cloud_filtered);
}

void radius_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform radius outlier removal
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> radiusoutlier;
  radiusoutlier.setRadiusSearch(0.5);//set radius to search
  radiusoutlier.setMinNeighborsInRadius(5);//set min number of points
  radiusoutlier.setInputCloud(cloud);
  radiusoutlier.filter(cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);
}

void passthrough_cb (const pcl::PCLPointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 cloud_filtered;

  // Perform passthrough removal
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.0, 0.3);
  pass.setFilterLimitsNegative(true);
  pass.setInputCloud(cloud);
  pass.filter(cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "radius_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/outputpoints", 5, radius_cb);//choose filter type

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("filteredPCL", 1);

  // Spin
  ros::spin ();
}
