import numpy as np
from matplotlib.cm import get_cmap

import rospy
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs


def point_cloud(points, parent_frame):
    """ Creates a point cloud message.

    Args:
        points: Nx7 array of xyz positions (m) and rgba colors (0..1)
        parent_frame: frame in which the point cloud is defined

    Returns:
        sensor_msgs/PointCloud2 message

    """
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize

    data = points.astype(dtype).tobytes()

    fields = [
        sensor_msgs.PointField('x', 0, ros_dtype, 1),
        sensor_msgs.PointField('y', itemsize, ros_dtype, 1),
        sensor_msgs.PointField('z', 2*itemsize, ros_dtype, 1),
        sensor_msgs.PointField('intensity', 3*itemsize, ros_dtype, 1),
        sensor_msgs.PointField('velocity', 4*itemsize, ros_dtype, 1)]
    
    header = std_msgs.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],#number of points in the frame
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 5),
        #point_step=32,
        row_step=(itemsize * 5 * points.shape[0]),
        #row_step=32*points.shape[0],
        data=data
    )


if __name__ == '__main__':
    rospy.init_node('dragon_curve')
    pub_points = rospy.Publisher('dragon_curve', sensor_msgs.PointCloud2,
                                 queue_size=1)

    # Fractal iterations
    iters = 12
    # Number of points per segment (at least 2)
    rez = 10
    # Number of points after which the colormap repeats
    period = 200 * rez
    # Initial curve
    curve = np.vstack((np.linspace(0, 1, rez), np.zeros(rez), np.zeros(rez))).T

    rotate90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    for _ in range(iters):
        next_curve = (curve - curve[-1, :]).dot(rotate90) + curve[-1, :]
        curve = np.vstack((curve, np.flipud(next_curve)))

    curve = curve * 0.1

    colors = np.array(get_cmap('cubehelix')(
        np.cos(2 * np.pi / period * np.arange(curve.shape[0])) / 2 + 0.5))

    points = np.hstack((curve, colors))

    i = 0
    while not rospy.is_shutdown():
        i += 1
        pub_points.publish(point_cloud(points[:i, :], '/map'))
