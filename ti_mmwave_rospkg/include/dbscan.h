#ifndef DBSCAN_H
#define DBSCAN_H

#include <iostream>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

struct Point{
    double x;
    double y;
    double z;
    double intensity;
    double velocity;
};

struct Object{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int n = 0;
    double r = 0.0;
    double velocity = 0.0;
};

//static const inline double distance(double x1, double y1, double x2, double y2);

double distance(double x1, double y1, double x2, double y2);

int dbscan(const std::vector<Point> &input, std::vector<int> &labels, double eps, int min);

std::vector<Object> objects(const std::vector<Point> &input, std::vector<int> &labels, int num);

#endif /*DBSCAN_H*/
