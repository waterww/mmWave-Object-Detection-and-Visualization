#include "dbscan.h"
#include <math.h>
#include <algorithm>


double distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;

    return sqrt(dx * dx + dy * dy);
}

double dbscan_distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;

    return sqrt(dx*dx + 0.3 * dy * dy);
}

const inline int region_query(const std::vector<Point> &input, int p, std::vector<int> &output, double eps)
{
    for(int i = 0; i < (int)input.size(); i++){

        if(dbscan_distance(input[i].x, input[i].y, input[p].x, input[p].y) < eps){
            output.push_back(i);
        }
    }

    return output.size();
}

bool expand_cluster(const std::vector<Point> &input, int p, std::vector<int> &output, int cluster, double eps, int min)
{
    std::vector<int> seeds;

    if(region_query(input, p, seeds, eps) < min){

        //this point is noise
        output[p] = -1;
        return false;

    }else{

        //set cluster id
        for(int i = 0; i < (int)seeds.size(); i++){
            output[seeds[i]] = cluster;
        }

        //delete paint from seeds
        seeds.erase(std::remove(seeds.begin(), seeds.end(), p), seeds.end());

        //seed -> empty
        while((int)seeds.size() > 0){

            int cp = seeds.front();
            std::vector<int> result;

            if(region_query(input, cp, result, eps) >= min){

                for(int i = 0; i < (int)result.size(); i++){

                    int rp = result[i];

                    //this paint is noise or unmarked point
                    if(output[rp] < 1){

                        //unmarked point
                        if(!output[rp]){
                            seeds.push_back(rp);
                        }

                        //set cluster id
                        output[rp] = cluster;
                    }
                }
            }

            //delete point from seeds
            seeds.erase(std::remove(seeds.begin(), seeds.end(), cp), seeds.end());
        }
    }

    return true;
}

Object_world global_position_and_velocity(const Object &input_object, nav_msgs::Odometry &trans)
{
  Object_world ow;

  //calculate yaw angle
  double quatx = trans.pose.pose.orientation.x;
  double quaty = trans.pose.pose.orientation.y;
  double quatz = trans.pose.pose.orientation.z;
  double quatw = trans.pose.pose.orientation.w;
  
  //get yaw angle
  tf::Quaternion q(quatx,quatz,quaty,quatw);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //caculate global position
  double globalx = cos(yaw)*input_object.x + sin(yaw)*input_object.y + trans.pose.pose.position.x;
  double globaly = -sin(yaw)*input_object.x + cos(yaw)*input_object.y + trans.pose.pose.position.y;

  ow.x = globalx;
  ow.y = globaly;

  //get car velocity in world frame
  //double carvx = trans.twist.twist.linear.x;
  //double carvy = trans.twist.twist.linear.y;

  //object velosity in local frame
  //double d = distance(input_object.x, input_object.y, 0.0, 0.0);
  //double ox = input_object.velocity * (input_object.x / d);
  //double oy = input_object.velocity * (input_object.y / d);

  //ow.vx = carvx + ox;
  //ow.vy = carvy + oy;
  //ow.absv = sqrt(ow.vx * ow.vx + ow.vy * ow.vy);

  //print absolute positon and velocity of the object
  std::cout << "center in world:" << "(" << ow.x << "," << ow.y << ")" << std::endl;
  //std::cout << "velocity in world:" << ow.absv << std::endl;

  return ow;    
    
}

int dbscan(const std::vector<Point> &input, std::vector<int> &labels, double eps, int min)
{
    int size = input.size();
    int cluster = 1;

    std::vector<int> state(size);

    for(int i = 0; i < size; i++){

        if(!state[i]){

            if(expand_cluster(input, i, state, cluster, eps, min)){
                cluster++;
            }
        }
    }

    labels = state;

    return cluster - 1;
}

std::vector<int> find( std::vector<int> &array, int x)
{
    using namespace std;

    vector<int>::iterator it = array.begin();
    vector<int> positions;

    int i = 0;

    while( it != array.end())
    {
        
        if(*it == x)
        {
            positions.push_back(i);
        }
        

        ++it; 
        ++i; 
    }

    return positions;

}

std::vector<Object> objects(const std::vector<Point> &input, std::vector<int> &labels, int num, nav_msgs::Odometry trans)
{
    using namespace std;

    //size is the number of points
    int size = input.size();
    
    //num is the number of objects
    std::vector<Object> output_objects(num);


    //calculate position and velocity
    for(int i = 0; i < size; i++)
    {
        
        for(int m = 0; m < num; m++)
        {
            
            if(labels[i] == m+1)
            {

                output_objects[m].x += input[i].x;
                output_objects[m].y += input[i].y;
                output_objects[m].z += input[i].z;
                output_objects[m].velocity += input[i].velocity;

                output_objects[m].n ++;
               
                break;
                
            }

        }

    }

    //calculate radius
    for(int m = 0; m < num; m++)
    {
        
        output_objects[m].x /= output_objects[m].n;
        output_objects[m].y /= output_objects[m].n;
        output_objects[m].z /= output_objects[m].n;
        output_objects[m].velocity /= output_objects[m].n;

        std::vector<int> positions = find(labels, m+1);

        //calculate radius
        for(int n = 0; n < positions.size(); n++)
        {

            Point target_point = input[ positions[n] ];
            
            double d = distance(output_objects[m].x, output_objects[m].y, target_point.x, target_point.y );
            
            if(d >= output_objects[m].r)
            {
                output_objects[m].r = d;
            }

            //maximum radius is 0.5 m
            if(output_objects[m].r >= 0.5)
            {
                output_objects[m].r = 0.5;
            }

            //minimum radius is 0.3 m
            if(output_objects[m].r <= 0.3)
            {
                output_objects[m].r = 0.3;
            }
        }

        
        //print center position, velocty and radius of objects
        std::cout << "object:" << m+1 << std::endl;
        std::cout << "radius:" << output_objects[m].r<< std::endl;
        std::cout << "center in local:" << "(" << output_objects[m].x << "," << output_objects[m].y << ")" << std::endl;
        std::cout << "velocity in local:" << output_objects[m].velocity << std::endl;


    
        
    }

    

    //never forget to return
    return output_objects;

}
