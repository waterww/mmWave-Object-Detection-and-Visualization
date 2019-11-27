#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

#pub = rospy.Publisher('/outputpoints', PointCloud2, queue_size=10)

def callback(data):
    assert isinstance(data, PointCloud2)

    number = 0#point number      


    #use generator to get point information
    for p in pc2.read_points(data, field_names = ("x","y","z","intensity","velocity"), skip_nans=True):

        number = number + 1
		
        #print xyzi info of current frame   
        print("x:%f\ny:%f\nz:%f\nintensity:%f\nveloctity:%f\npoint number:%d\n" %(p[0],p[1],p[2],p[3],p[4],number))

      
def listener_and_talker():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('printer')
    
    rospy.Subscriber('/filteredPCL', PointCloud2, callback)
    
    
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener_and_talker()
