# Car Heading Generator

Takes2 GNSS inputs and generates heading from it. Assumes one is front and the other is back. This is done in ENU frame

### Getting Started

##### Install driver dependencies:
```
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/airacingtech/car_heading_generator
$ cd ~/ros2_ws
$ sudo apt update
$ sudo rosdep update
$ rosdep install -y --ignore-src --from-paths src/
```

##### Run Car Heading Generator Node

Edit the parameters in `param/car_heading_generator.param.yaml`

In `launch/car_heading_generator.launch.py` you can change the remappings so that the data gets published/subscribed in different topics. Otherwise those will be the defaults

```
$ cd ~/ros2_ws
$ source /opt/ros/galactic/setup.bash
$ colcon build
$ source install/local_setup.bash
$ ros2 launch car_heading_generator car_heading_generator.launch.py
```
