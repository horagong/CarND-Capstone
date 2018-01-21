# Self Driving Car System Integration

## Team Dragon
Name | Email
-----|------
Waleed Mansoor | waleed.mansoor@gmail.com
Jaewoo Park	| jaewoopark91@gmail.com
Sasha Jaksic | dzx303@gmail.com
Dominik Marquardt	| dominik.marquardt@outlook.de
Juil O | horagong@gmail.com

## ROS native installation on mac
This project runs with ROS. Usually ROS is installed on Linux. So we can use linux VM in mac. But when I started this project on VM, I realized that it was so slow that installing on mac natively would be better.

ROS can be [installed on mac](http://wiki.ros.org/kinetic/Installation/OSX/Homebrew/Source) and there is a [script](https://github.com/mikepurvis/ros-install-osx) for it. But ROS page say, "This is a work in progress! It really won't work right now..." 

When I tested turtlesim and simple_arm packages after installing, it didn't work as they said. However, I could run the packages as well as this project with a few changes.
* Shared libraries were installed with the suffix '.so' not '.dylib'. Change the suffix.
* close_half_closed_sockets() in rosmaster/util.py uses TCP_INFO which is not implemented on mac. Just comment out this function.

For this project, you also need to install some msg packages like Dataspeed msg. 
<div align=center>
<img width=640 src=imgs/car.gif>
</div>

## System Architecture
This system has three main parts.
<div align=center>
<img src=imgs/final-project-ros-graph-v2.png>
</div>

### Perception
The main role of this part is detecting the traffic light and publishing '/traffic_waypoint' topic so that the next planning part can generate an appropiate trajectory.
<div align=center>
<img src=imgs/tl-detector-ros-graph.png>
</div>

For this, we trained the pre-trained classifier using some dataset. The pre-trained graph is [ssd_mobilenet_v1_coco](http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v1_coco_2017_11_17.tar.gz)

We used images from ROS topics of simulator and bag files as training dataset. We can see these images following commmands.
```
roslaunch launch/styx.launch
rosrun image_view image_view _sec_pre_frame:=0.1 image:=/image_color

rosbag play just_traffic_light.bag
rosrun image_view image_view image:=/image_raw
```
And then that can be saved with this command.
```
rosrun image_view image_saver _sec_per_frame:=0.1 image:=/image_raw
```
The waypoint to be stopped at when the light is red is provided by stop_line_positions config. So the classifier tries to find a traffic light when the car comes near the closests stop_line. In tl_detector,
```
# find the closest visible traffic light (if one exists)
min_dist = 100000
for stop_line_wp_idx in self.stop_line_wp_idxs:
    dist = stop_line_wp_idx - self.car_wp_idx
    if dist >= 0 and dist < min_dist:
        min_dist = dist
        if min_dist < self.visible_distance_wps:
            # It uses the stop_line postion rather than the traffic light position
            light_wp = stop_line_wp_idx

# If there is a visible traffic light
if light_wp != -1:
    # if camera is on: through classifier
    if self.has_image:
        state = self.get_light_state(light_wp)
        return light_wp, state
    # if camera is off: through ground truth
    else:
```

### Planning
Waypoint_updater module publishes '/final_waypoints' topic considering the destination, current position and the environment like traffic lights or obstacles. 
<div align=center>
<img src=imgs/waypoint-updater-ros-graph.png>
</div>
The route for destination to some distance ahead of the current pose is calculated from '/base_waypoints'. In waypoint_updater,

```
# find next_waypoint considering car's yaw
next_wp_idx = self.find_next_waypoint()
...
end_wp_idx = min(next_wp_idx + LOOKAHEAD_WPS, len(self.base_waypoints)) - 1
```
Each waypoint in final_waypoints has velocity as attribute. If there is a stop line to stop for traffic light in the middle of the waypoints, the velocity is updated like following.

```
for i in range(next_wp_idx, stop_line_wp_idx):
    dist = self.distance(self.base_waypoints, i, stop_line_wp_idx)
    dist = max(0., dist)
    stopping_vel = math.sqrt(2*decel*dist)
    stopping_vel = min(stopping_vel, self.required_velocity)
    self.set_waypoint_velocity(self.base_waypoints, i, stopping_vel)
```
### Control
The dbw node publishes topics for actuators.
<div align=center>
<img src=imgs/dbw-node-ros-graph.png>
</div>
For acceleration and braking, I used PID controller. In twist_controller,

```
curr_time = rospy.get_time()
sample_time = curr_time - self.last_time
self.last_time = curr_time
error = proposed_linear_velocity - current_linear_velocity
accel = self.velocity_pid.step(error, sample_time)
accel = self.velocity_lowpass.filt(accel)
```

## Future work
It runs well keeping the lane and according to traffic light with the camera off. When the camera is on, the simulator needs for too much resources in my machine. So it cannot run well for lagging. I think it should be solved in the simulator.

It works well even when I speed up to 80km/h with the camera off. In that case, the car goes a little away his lane at the rapid corner. I think it could need another pid controller for yaw.


---
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
