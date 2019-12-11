#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from xyz_to_pcl import point_cloud


serial = 0
frame_list = []
pub = rospy.Publisher('/outputpoints', PointCloud2, queue_size=1)


def points_to_pointcloud2(points, frame_id="base_radar_link"):
    '''
    Create a sensor_msgs.PointCloud2 from an array
    of points.
    '''
    msg = PointCloud2()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = points.shape[0]
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 16, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 32
    msg.row_step = 32*points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = points.astype(np.float32).tobytes()

    return msg 
    

def hungarian(xyz_array):

    #threshold
    c_corr = 0.1
    
    #input number of frames
    number = len(xyz_array)

    #the last ndarray in xyz_array
    A = xyz_array[-1]
    #calculate cost matrix
    nrows_matrix = np.empty([A.shape[0],number-1])
    
    for i in range(number-1):
        #number of points in new frame
        r = A.shape[0]
        B = xyz_array[i]
        #number of points in old frame
        c = B.shape[0]
        
        #cost matrix
        C = np.empty([r,c])
        for m in range(r):
            for n in range(c):
                #distance between m-th point in frame A and n-th point in frame B
                L = np.sum(np.power(A[m,0:1]-B[n,0:1],2))
                C[m,n] = np.sqrt(L)
        

        #compare with threshold
        min_of_row = C.min(axis=1)
        #print("min of row C")
        #print(min_of_row)
        
        match_tag = np.empty(A.shape[0])
        match_tag[min_of_row >= c_corr] = -1
        match_tag[min_of_row < c_corr] = 0
        #print("set value of min of row")
        #print(match_tag)
        
        nrows_matrix[...,i] = match_tag.T

    return nrows_matrix
       

#give the number of iterator
def count(iter):
    try:
        return len(iter)
    except TypeError:
        return sum(1 for _ in iter)    

#second strategy to judge noise points
def judgment(match_matrix):
    #number of points in the target frame
    d = np.empty(match_matrix.shape[0])
    
    for i in range(match_matrix.shape[0]):
        #save existence of one point in target frame
        a = list(match_matrix[i,:])
        
        b = ''
        #transform int list a to string b
        for x in a:
            b = b + str(int(x))
        #print(b)
        #if there is '0-10-1' or '-10-10',the return number will be 0 or posivie int; if not, the return number will be -1
        d[i] = b.find('0-10-1'or'-10-10')
    
    #find position of noise point (the corresponding number in d will not be -1)
    p = d != -1

    return p


def callback(data):
    assert isinstance(data, PointCloud2)
    global serial
    global frame_list
    global pub
    
    serial = serial + 1#frame number
    number = 0#point number
    
    #generator and get number of points in the frame
    g = pc2.read_points(data, field_names = ("x","y","z","intensity","curvature"), skip_nans=True)
    N = count(g)
    #print("total num of points in %d-th frame: %d" %(serial,N))
    
    #frame_xyz = np.empty([N,4])
    frame_xyz = np.zeros([N,5])        

    #use generator to get point information
    for p in pc2.read_points(data, field_names = ("x","y","z","intensity","curvature"), skip_nans=True):
		
        #save xyzi info of current frame
        frame_xyz[number,:] = p
        number = number + 1
		
        #print xyzi info of current frame   
        #print("x:%f y:%f z:%f intensity:%f" %(p[0],p[1],p[2],p[3]))

    #create input frame list
    if (serial <= 10):
	frame_list.append(frame_xyz)
    else:
        frame_list.append(frame_xyz)
        del frame_list[0]
    
   
   
    #decide which point is noise
    match_matrix = hungarian(frame_list)
    no_match = np.sum(match_matrix,axis=1)
    
    target_frame = frame_list[-1]

    #the count that a point doesn't exit exceed x times
    z = no_match <= -3
    
    p = judgment(match_matrix)
    P = np.arange(N)

    #remove noise points in the target frame
    target_frame = np.delete(target_frame, P[p ^ z], axis=0)
    
    #print("%d noise points are removed from %d-th frame." %(len(P[p ^ z]),serial))

    target_pcl = point_cloud(target_frame,"ti_mmwave")
    pub.publish(target_pcl)


        
def listener_and_talker():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('hungarian')
    
    rospy.Subscriber('ti_mmwave/radar_scan_pcl', PointCloud2, callback)
    
    
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener_and_talker()
