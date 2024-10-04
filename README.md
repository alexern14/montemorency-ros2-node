This node is used to process Montemorency MCAP files with ROS2 Pointcloud2 data. This node converts the ROS2 Pointcloud2 data to PCL Pointcloud2. Once in PCL format, it uses segmentation to to get separate tree trunks.

## Dependancies

ROS2

Docker (If you need to convert Montemorency files still)

## Building

```
colcon build
```

## Running

Run the node

```
source install/setup.bash
ros2 run montemorency montemorency_pcl
```

In a separate terminal, play the mcap file to process

```
ros2 bag play <file-to-process>.mcap
```

In a separate terminal, record. This new file will have all topics including the processed ones.

```
ros2 bag record -s mcap --all
```

# Converting Montemorency data to MCAP

[Dataset](https://norlab.ulaval.ca/research/montmorencydataset/)

1. Get ROS1 Noetic Docker

```
docker run -it --volume /bagfiles:/bagfiles ros::noetic

docker exec -it <container-id> bash
```

2. Convert .bag velodyne scan to .bag pointcloud

   a. copy bag file to docker

   ```
   docker cp <file-to-convert>.bag <container-id>:/
   ```

   b. install dependancies

   ```
   docker exec -it <container-id> bash
   sudo apt update
   sudo apt install ros-noetic-velodyne*

   # docker terminal A (ROS1)
   source /opt/ros/noetic/setup.bash
   roscore
   ```

   c. docker terminal B (ROS1)

   Run the velodyne to pointcloud node

   ```
   docker exec -it <container-id> bash
   source /opt/ros/noetic/setup.bash
   rosrun nodelet nodelet standalone velodyne_pointcloud/TransformNodelet _model:="32E" _calibration:="/opt/ros/noetic/share/velodyne_pointcloud/params/32db.yaml" _min_range:=0.0
   ```

   d. docker terminal C (ROS1)

   Record

   ```
   docker exec -it <container-id> bash
   source /opt/ros/noetic/setup.bash
   rosbag record -a
   ```

   e. docker terminal D (ROS1)

   Play the file to be converted

   ```
   docker exec -it <container-id> bash
   source /opt/ros/noetic/setup.bash
   rosbag play <file-to-convert>.bag
   ```

3. Convert .bag with ROS1 message types to .mcap with ROS2 message types

   a. install ros2 galactic

   ```
   docker exec -it <container-id> bash
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt upgrade
   sudo apt install ros-galactic-desktop
   export ROS1_INSTALL_PATH=/opt/ros/noetic
   export ROS2_INSTALL_PATH=/opt/ros/galactic
   sudo apt install ros-galactic-ros1-bridge
   sudo apt install ros-galactic-rosbag2-storage-mcap
   ```

   b. docker terminal A (ROS1)

   Get ros1 running

   ```
   docker exec -it <container-id> bash
   source /opt/ros/noetic/setup.bash
   roscore
   ```

   c. docker terminal B (ROS1 & ROS2)

   Run ros1 to ros2 bridge

   ```
   docker exec -it <container-id> bash
   source /opt/ros/noetic/setup.bash
   source /opt/ros/galactic/setup.bash
   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
   ```

   d. copy file to docker container

   ```
   docker cp <file-to-convert>.bag <container-id>:/
   ```

   e. docker terminal C (ROS2)

   Record (this will produce file mcap file)

   ```
   docker exec -it <container-id> bash
   source /opt/ros/galactic/setup.bash
   ros2 bag record -s mcap --all
   ```

   f. docker terminal D (ROS1)

   Play file to be converted (use the file produced from 2.d. above)

   ```
   docker exec -it <container-id> bash
   source /opt/ros/noetic/setup.bash
   rosbag play <file-to-convert>.bag
   ```

4. Process the data

   Follow the [Running](#running) instructions above.
