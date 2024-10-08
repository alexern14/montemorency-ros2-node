This node is used to process Montemorency data files. In order to use this node, original ROS1 Montemorency data will need to be converted to MCAP (instructions below). Once in MCAP file format, this node converts the ROS2 Pointcloud2 data to PCL Pointcloud2 in order to use segmentation to to get separate tree trunks.

[Montemorency Dataset](https://norlab.ulaval.ca/research/montmorencydataset/)

## Dependancies

[Docker Compose](https://docs.docker.com/compose/install/) for converting Montemorency data to MCAP file format.

[ROS2](https://docs.ros.org/en/jazzy/index.html) for running the montemorency_pcl processing node.

# Converting Montemorency data to MCAP

This will convert the original Montemorency data to MCAP format through the following flow:

```mermaid
  flowchart LR
    ros1Velodyne[.bag ROS1 datatypes with velodyne scan]
    ros1Pointcloud[.bag ROS1 datatypes with pointcloud]
    mcap[.mcap ROS2 datatypes]
    ros1Velodyne --> ros1Pointcloud --> mcap
```

1. Run the docker container with the following command from the root directory of this repo. This will put you directly into the container.

```
docker compose run ros1ros2
```

> **_Note:_** To find the `<container-id>` used in the commands below, either see the `root@<container-id>:/#` from the container entered using the above command or in a separate terminal run `docker ps`

2. Convert .bag ROS1 velodyne scan to .bag ROS1 pointcloud

   a. Copy the Montemorency .bag file to be converted to MCAP

   ```
   # From your local machine
   docker cp <file-to-convert>.bag <container-id>:/
   ```

   b. Run the velodyne to pointcloud node

   ```
   # docker terminal B (ROS1)

   # In a separate terminal connect to the docker container
   docker exec -it <container-id> bash

   source /opt/ros/noetic/setup.bash
   rosrun nodelet nodelet standalone velodyne_pointcloud/TransformNodelet _model:="32E" _calibration:="/opt/ros/noetic/share/velodyne_pointcloud/params/32db.yaml" _min_range:=0.0
   ```

   c. Record

   ```
   # docker terminal C (ROS1)

   # In a separate terminal connect to the docker container
   docker exec -it <container-id> bash

   source /opt/ros/noetic/setup.bash
   rosbag record -a
   ```

   d. Play the file to be converted

   ```
   # docker terminal D (ROS1)

   # In a separate terminal connect to the docker container
   docker exec -it <container-id> bash

   source /opt/ros/noetic/setup.bash
   rosbag play <file-to-convert>.bag
   ```

   Once this file is done playing go into docker terminal C (ROS1) (step 2.c.) and stop recording.

   > **_Note:_** At this point docker terminal B, C, and D (ROS1) (2.b, c, and d) can be closed. Do NOT close the original terminal where `docker compose` was ran!

3. Convert .bag with ROS1 message types to .mcap with ROS2 message types

   a. Run ros1 to ros2 bridge

   ```
   # docker terminal B (ROS1 & ROS2)

   # In a separate terminal connect to the docker container
   docker exec -it <container-id> bash

   source /opt/ros/noetic/setup.bash
   source /opt/ros/galactic/setup.bash
   ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
   ```

   b. Record (this will produce the mcap file)

   ```
   # docker terminal C (ROS2)

   # In a separate terminal connect to the docker container
   docker exec -it <container-id> bash

   source /opt/ros/galactic/setup.bash
   ros2 bag record -s mcap --all
   ```

   c. Play file to be converted (use the file produced from 2.c. above)

   ```
   # docker terminal D (ROS1)

   # In a separate terminal connect to the docker container
   docker exec -it <container-id> bash

   source /opt/ros/noetic/setup.bash
   rosbag play <file-to-convert>.bag
   ```

   Once this file is done playing go into docker terminal C (ROS2) (step 3.b.) and stop recording.

   > **_Note:_** At this point docker terminal B, C, and D (ROS1) (2.a, b, and c) can be closed. Do NOT close the original terminal where `docker compose` was ran!

   d. Copy the Montemorency .MCAP file from the docker container

   ```
   # From your local machine
   docker cp <container-id>:/<path-to-converted-file>.mcap <local-path>
   ```

# Processing Montemorency MCAP data files

This can be done either locally (you have ROS2 installed) or in a docker container.

## Running in Docker container

In order to run the processing in a docker container run the following, then run the Building and Running instructions inside the docker container

1. Run the docker container with the following command from the root directory of this repo. This will put you directly into the container.

   ```
   docker compose run ros1ros2
   ```

2. Copy the Montemorency .mcap file to be processed

   ```
   # From your local machine
   docker cp <file-to-process>.mcap <container-id>:/
   ```

> **_Note:_** Anytime below it asks for a new terminal you will have to connect to the docker container using the following command `docker exec -it <container-id> bash`

## Building

```
git clone https://github.com/alexern14/montemorency-ros2-node.git
cd montemorency-ros2-node

# source where you have ros2 installed
source /opt/ros/<ROS2 Distro>/setup.bash
colcon build
```

## Running

Source where you have ros2 installed

```
source /opt/ros/<ROS2 Distro>/setup.bash
```

Run the node from the root directory of this repo

```
source install/setup.bash
ros2 run montemorency montemorency_pcl
```

In a separate terminal, play the mcap file to process

```
# source ros2
source /opt/ros/<ROS2 Distro>/setup.bash

ros2 bag play <file-to-process>.mcap
```

In a separate terminal, record. This new file will have all topics including the processed ones.

```
# source ros2
source /opt/ros/<ROS2 Distro>/setup.bash

ros2 bag record -s mcap --all
```
