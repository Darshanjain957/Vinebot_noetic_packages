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


# operating the vinebot:
the vinebot run on Linux based system (Ubuntu 20.04) and uses ROS noetic for the pacckage integration. the vinebot is ecquipped with various sensors and actators. the Vinebot uses a generic version of Roboteq dual motor controller(HBL2350) to control Dual DC servomotors. The motor controller is capable of accepting commands from PC/ microcontroller via USB port and using the protocol RS-232 with support for analog and digital I/O capabilities. Figure 1 depicts the system overview  and layout of the vinebot.

![Alt text](/images/PIC_OVERVIEW.PNG "VINEBOT-LAYOUT")

Figure 1: layout of the robot. 
#  hardware Setup: 



# launch instructions 

to launch vinebot with rtabmap in the mapping node with ekf filter based fused localization

$ roslaunch roboteq_control bringup_rtab_imu.launch 


to move the vinebot using the keyboard:

$ rosrun  teleop_twist_keyboard teleop_twist_keyboard.py 

to move the vinebot using the shell scripts open a terminal in the folder with the shell shripts (shell scripts and instructions)

§ sudo ls

§ ./startjoystick.sh

 this step must be performed after the bluetooth joystick is connected to the operating system and the bringup file is launched 

to launch vinebot with rtabmap in the localization node with ekf filter based fused localization note that this step requires a previously created map in .db format 

$ roslaunch roboteq_control bringup_rtab_localization_imu.launch 





