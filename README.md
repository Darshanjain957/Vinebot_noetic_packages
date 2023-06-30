# Vinebot_noetic_packages
ROS packages developed for the vinebot, it includes updated versions of Vinbot_ros_packages, realsense ros wrapper, livox ros wrapper, and mapping packages and some additional packages. 
Prerequisites and dependencies 
before installing the catkin workspace some additional libraries and dependencies are required.

ros noetic installation:  § sudo apt install ros-noetic-desktop-full
Pcl :                     $ sudo apt install libpcl-dev  pcl-tools       
Eigen:                    $ sudo apt install libeigen3-dev               
opencv:                   $ sudo apt install libopencv-dev python3-opencv
ddynamic_reconfigure:     $ sudo apt install ros-noetic-ddynamic-reconfigure
rtabmap and rtabmap-ros : $ sudo apt install ros-noetic-rtabmap ros-noetic-rtabmap-ros
joystick driver:          $ sudo apt install ros-noetic-joy
ds4drv:                   $ sudo pip install ds4drv
LIVOX SDK Installation: 
the livox sensor requires Livox SDK a software development kit developed by livox for the communication between livox mid 40 and ros.

The package can be installed form the git hub repository provided by livox, detailed installation instructions are provided by livox in the git hub repository.

Livox SDK: https://github.com/Livox-SDK/Livox-SDK


Realsense2 sdk Installation:
detailed instructions can be found here(excluding jetson devices) : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

for jetson devices refer the following link and follow the debian package installation guide.
https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md




# Make catkin workspace if does not exist and clone this repo


mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

git clone 

# Install all dependecies

cd ..

rosdep install --from-paths src --ignore-src -r -y

(only for jetson xavier nx) for the realsense camera to function with ros the cmakelist.txt file of realsense2_camera in realsesne-ros package must be modified as per the instructions from the following link:
https://github.com/IntelRealSense/realsense-ros/issues/2326#issuecomment-1107658481

# Compile package

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc



