# Obstacle Detection based on TI mmWave Radar IWR1443BOOST

This package is revised based on [customized ROS package developed by Leo Zhang](https://github.com/radar-lab/ti_mmwave_rospkg) and add the obstacle detection part.

![image](https://image.baidu.com/search/detail?ct=503316480&z=0&ipn=d&word=Ti%20IWR1443boost&step_word=&hs=0&pn=13&spn=0&di=2640&pi=0&rn=1&tn=baiduimagedetail&is=0%2C0&istype=2&ie=utf-8&oe=utf-8&in=&cl=2&lm=-1&st=-1&cs=2932990697%2C446875968&os=2833861003%2C3619704865&simid=3456904469%2C196859850&adpicid=0&lpn=0&ln=70&fr=&fmq=1608324189647_R&fm=result&ic=&s=undefined&hd=&latest=&copyright=&se=&sme=&tab=0&width=&height=&face=undefined&ist=&jit=&cg=&bdtype=0&oriquery=&objurl=https%3A%2F%2Ftimgsa.baidu.com%2Ftimg%3Fimage%26quality%3D80%26size%3Db9999_10000%26sec%3D1608334275683%26di%3Da49b59b9f2ea4557b708fa093eb9b8ae%26imgtype%3D0%26src%3Dhttp%3A%2F%2Fe2echina.ti.com%2Fresized-image%2F__size%2F2460x0%2F__key%2Fcommunityserver-discussions-components-files%2F60%2F_7F67505B_.jpg&fromurl=ippr_z2C%24qAzdH3FAzdH3Fjdjvitgw_z%26e3Bpt_z%26e3Bv54AzdH3Fq7jfpt5g_wgfoj6AzdH3Fwgws52AzdH3F5pij6_wgws52AzdH3FuAzdH3FmaAzdH3FpAzdH3F8mc09c&gsm=6&rpstart=0&rpnum=0&islist=&querylist=&force=undefined)

## Process Raw Point Cloud Data

Add a Radius Outlier Removal Filter and Hungarian filter designed by myself, and publish point cloud data as PointCloud2 with velocity information.

## Extract Objects

Adopt DBscan to group points and extract objects radius, position and velocity.

## Visualize in Rviz

Show objects' position and velocity in Rviz and publish object information.

## Add tf Transform

To better use the radar and ROS package on mobile vehicle or robots, add listerner for odometry, calculate and visualize position of object in world frame.

## Result

[Video for fixed radar](https://1drv.ms/v/s!Ai9O9sZe1qg8jwgaK7Fe5u1Wq007)

[Video for radar assembled on AGV](https://1drv.ms/v/s!Ai9O9sZe1qg8jwcBq-QCXvkEBM5v)
