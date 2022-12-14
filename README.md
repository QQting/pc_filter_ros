# PointCloud Filter for ROS

This package uses PassThrough filter of PCL for ROS development.

This package has been tested on both ROS 1 melodic and noetic.

![](resource/pc_filter_ros_screenshot.png)

## Build Code SOP

```bash
mkdir ~/pc_filter_ws/src -p
cd ~/pc_filter_ws/src
git clone https://github.com/QQting/pc_filter_ros.git

cd ~/pc_filter_ws
source /opt/ros/melodic/setup.bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run Code SOP

Before runing the code, make sure your pointcloud data is published as the topic named `/velodyne_points`. Otherwise, you can modify `pc_filter.launch` to change the topic.

Run below command to test the pc_filter:

```bash
source ~/pc_filter_ws/devel/setup.bash
roslaunch pc_filter_ros pc_filter.launch open_rviz:=true
```

You can adjust the limit_x, limit_y, limit_z in the `rqt_reconfigure` to test the filter.

If you don't have a real sensor able to publish pointcloud, please append the parameter `open_bag:=true` when running the launch file:

```bash
source ~/pc_filter_ws/devel/setup.bash
roslaunch pc_filter_ros pc_filter.launch open_rviz:=true open_bag:=true
```

It will play the rosbag of VLP-16 PointCloud.
