# Obstacle Detection based on TI mmWave Radar IWR1443BOOST

This package is revised based on [customized ROS package developed by Leo Zhang](https://github.com/radar-lab/ti_mmwave_rospkg) and add the obstacle detection part.

![image](https://github.com/waterww/mmWave-Object-Detection-and-Visualization/blob/master/IWR1443BOOST_6.jpg)

## Process Raw Point Cloud Data

Add a Radius Outlier Removal Filter and Hungarian Algorithm based filter designed by myself, and publish point cloud data as PointCloud2 with velocity information.

## Extract Objects

Adopt DBscan to group points and extract objects radius, position and velocity.

## Visualize in Rviz

Show objects' position and velocity in Rviz and publish object information.

## Add tf Transform

To better use the radar and ROS package on mobile vehicle or robots, add listerner for odometry, calculate and visualize position of object in world frame.

## Result

[Video for fixed radar](https://1drv.ms/v/s!Ai9O9sZe1qg8jwgaK7Fe5u1Wq007)

[Video for radar assembled on AGV](https://1drv.ms/v/s!Ai9O9sZe1qg8jwcBq-QCXvkEBM5v)
