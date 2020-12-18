# Obstacle Detection based on TI mmWave Radar IWR1443BOOST

This package is revised based on [customized ROS package developed by Leo Zhang](https://github.com/radar-lab/ti_mmwave_rospkg).

## Process raw point cloud data

Add a Radius Outlier Removal Filter and Hungarian filter designed by myself, and publish point cloud data as PointCloud2 with velocity information.

## Extract Objects

Adopt DBscan to group points and extract objects radius, position and velocity.

## Visualize in Rviz

Show objects' position and velocity in Rviz and publish object information.

## Add tf transform

To better use the radar and ROS package on mobile vehicle or robots, add listerner for odometry, calculate and visualize position of object in world frame.
