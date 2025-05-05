# Nathan Gomez
# Mary Cottier
# Cole Gilliam
## Team 2
## CS 491 - Aerial Robotics
## Final Project

# aerial_robotics

## Install requirements
```
sudo apt-get install \
ros-melodic-libmavconn \
ros-melodic-message-to-tf \
ros-melodic-mavros \
ros-melodic-mavros-msgs \
ros-melodic-apriltag \
ros-melodic-apriltag-ros \
libgstreamer-plugins-base1.0-dev \
python3-numpy \
python3-jinja2 \
python-pexpect \
python-pip

pip install \
mavproxy==1.8.49 \
monotonic

# [MAVProxy Version: Verify you have mavproxy version 1.8.49 installed]
pip show mavproxy

# [MAVProxy Version: If you still have a different (higher) version, your pip cache might be the issue]
pip cache purge
pip install --force-reinstall mavproxy==1.8.49
pip show mavproxy

sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
```

## Setup and Build
```
mkdir -p $HOME/aerial_robotics_ws/src

cd $HOME/aerial_robotics_ws
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build

cd $HOME/aerial_robotics_ws/src
git clone --recursive -b master https://github.com/robowork/aerial_robotics

cd $HOME/aerial_robotics_ws && git clone --recursive https://github.com/ArduPilot/ardupilot && cd ardupilot
git checkout Plane-4.2
./Tools/gittools/submodule-sync.sh
./Tools/environment_install/install-prereqs-ubuntu.sh -y

# The last command will pull the newest mavproxy version, higher than the one that is known to work with the Plane-4.2 stack so we will have to downgrade
pip install mavproxy==1.8.49

# Again, verify you now have mavproxy version 1.8.49 by following the previous section's steps:
# [MAVProxy Version: Verify you have mavproxy version 1.8.49 installed]
# [MAVProxy Version: If you still have a different (higher) version, your pip cache might be the issue]

# Apply some patches and extra files
cd $HOME/aerial_robotics_ws/ardupilot
patch -p0 < ../src/aerial_robotics/deps/vehicleinfo_py.patch 
cp ../src/aerial_robotics/deps/ArduPlane_MiniHawk_Gazebo.parm ./Tools/autotest/default_params

cd $HOME/aerial_robotics_ws && git clone --recursive https://github.com/khancyr/ardupilot_gazebo && cd ardupilot_gazebo
mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 12 && sudo make install

# Apply some patches and rebuild
cd $HOME/aerial_robotics_ws/ardupilot_gazebo
patch -p0 < ../src/aerial_robotics/deps/ArduPilotPlugin_hh.patch
patch -p0 < ../src/aerial_robotics/deps/ArduPilotPlugin_cc.patch
cd build && make -j 12 && sudo make install

cd $HOME/aerial_robotics_ws && git clone --recursive https://github.com/PX4/sitl_gazebo.git && cd sitl_gazebo
git checkout 9343aaf4e275db48fce02dd25c5bd8273c2d583a
mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 12 && sudo make install

### NOTES ###
# 1) The sitl_gazebo make install script does not copy libphysics_msgs.so to the install path, do so manually:
sudo cp $HOME/aerial_robotics_ws/sitl_gazebo/build/libphysics_msgs.so /usr/lib/x86_64-linux-gnu/mavlink_sitl_gazebo/plugins/
# 2) The sitl_gazebo package installs its own libLiftDragPlugin.so (breaks flight transition simulation) which can overlap with the version of gazebo-9, as they are both installed under the /usr/lib/x86_64-linux-gnu system path, so we have to hide it:
sudo mv /usr/lib/x86_64-linux-gnu/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so /usr/lib/x86_64-linux-gnu/mavlink_sitl_gazebo/plugins/libLiftDragPlugin.so.PX4_SITL_VERSION

# Add the following lines at the end of your ~/.bashrc file
export GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:${GAZEBO_MODEL_PATH}
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=/usr/share/mavlink_sitl_gazebo/models:${GAZEBO_MODEL_PATH}
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/mavlink_sitl_gazebo/plugins:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${GAZEBO_PLUGIN_PATH}

# Copy gazebo models to gazebo resources user folder
cp -r $HOME/aerial_robotics_ws/src/aerial_robotics/robowork_minihawk_gazebo/models/* $HOME/.gazebo/models/

cd $HOME/aerial_robotics_ws
catkin build
```

## Simulation
```
[Launch ROS Gazebo SIM in new terminal]:
roslaunch robowork_minihawk_gazebo minihawk_playpen.launch

[Launch Ardupilot Gazebo SITL in new terminal]:
cd $HOME/aerial_robotics_ws/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduPlane -f gazebo-minihawk --model gazebo-quadplane-tilttri --console  # --map

[Launch Rviz in new terminal]:
rviz -d $HOME/aerial_robotics_ws/src/aerial_robotics/robowork_minihawk_launch/config/minihawk_SIM.rviz

### MAVProxy-based commanding ###

[Load sample mission waypoints in Gazebo SITL terminal]
wp load ../src/aerial_robotics/robowork_minihawk_gazebo/resources/waypoints.txt

### MAVROS-based commanding ###

[Launch ROS node in new terminal 1]:
ROS_NAMESPACE="minihawk_SIM" roslaunch robowork_minihawk_launch vehicle1_apm_SIM.launch

[Invoke ROS services in new terminal 2]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'GUIDED'"
rosservice call /minihawk_SIM/mavros/cmd/arming True
rosservice call /minihawk_SIM/mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}"

[Alternatively, invoke ROS services in new terminal 2 to run auto (waypoint) mission]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'AUTO'"
rosservice call /minihawk_SIM/mavros/cmd/arming True   ###Required if the mission hasn't started yet### 

### Once the vehicle reaches its final waypoint (over the building) the camera will be looking at an apriltag marker  ###

[Observe ROS topic output of apriltag_ros node in new terminal 3 (relative pose in camera frame coordinates with respect to detected apriltag marker)]:
rostopic echo /minihawk_SIM/MH_usb_camera_link_optical/tag_detections

### Try switching between some Quadplane modes and control the aircraft motion ###

# https://ardupilot.org/plane/docs/qloiter-mode.html#qloiter-mode
[Publish ROS topic in new terminal 3 (this is required to virtually center the rc sticks, 4 first channels are 0:roll(-left,+right), 1:pitch(-up,+down), 2:throttle(-down,+up), 3:yaw(-left,+right))]:
rostopic pub -r 10 /minihawk_SIM/mavros/rc/override  mavros_msgs/OverrideRCIn "channels: [1500, 1500, 1500, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"
[Invoke ROS service in terminal 2]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'QLOITER'"

# https://ardupilot.org/plane/docs/qhover-mode.html
[Publish ROS topic in new terminal 3 (this is required to virtually trim the rc sticks)]:
rostopic pub -r 10 /minihawk_SIM/mavros/override  mavros_msgs/OverrideRCIn "channels: [1464, 1598, 1466, 1500, 1800, 1000, 1000, 1800, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"
[Invoke ROS service in terminal 2]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'QHOVER'"

# https://ardupilot.org/plane/docs/qland-mode.html
[Invoke ROS service in terminal 2]:
rosservice call /minihawk_SIM/mavros/set_mode "custom_mode: 'QLAND'"
```
