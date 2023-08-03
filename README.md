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
# Udev Rules 
This package is binding the USD devices under static name, so you can install the udev rule to identify the device as given in launch files. In order to do so edit /etc/udev/rules.d/10-local.rules file under root user:

$sudo su root 
$nano /etc/udev/rules.d/99-usb-serial.rules
#add the following line inside the file 
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="23f3", SYMLINK+="roboteq"
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="imu"
#now reload the rules files using the following code 
$sudo udevadm control --reload-rules && udevadm trigger
Troubleshooting:

If you do not have the permission to open the serial port, try to read this article https://websistent.com/fix-serial-port-permission-denied-errors-linux/

# Compile package
change to your catkin_ws folder 
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc


# operating the vinebot:
the vinebot runS on Linux based system (Ubuntu 20.04) and uses ROS noetic for the pacckage integration and it is ecquipped with various sensors and actators. the Vinebot uses a generic version of Roboteq dual motor controller(HBL2350) to control Dual DC servomotors. The motor controller is capable of accepting commands from PC/ microcontroller via USB port and using the protocol RS-232 with support for analog and digital I/O capabilities. Figure 1 depicts the system overview  and layout of the vinebot.

![Alt text](/images/PIC_OVERVIEW.PNG "VINEBOT-LAYOUT")
<p align="center">
Figure 1: layout of the robot. 
</p>

# bring_up launch files: 
In order for the vinebot to navigate effiently a set of bring_up launch files are created, different bringup launch files have different functionalities, most customized bringup files follow the base file "bringup.launch" in "Vinebot_ROS_Packages/roboteq_control/launch/" folder.
Bringup.launch file launches multiple nodes at the same time,namely the motor controller node, the urdf node, and the imu node.
an advanced version of the bringup.launch file called as bringup_rtab_imu.launch can be used for the localization and mapping of the environment using RTAB MAP, REALSENSE D455, bno 055 IMU, and EKF  based localization filter, this paticular Launch file also utilizes the IMU data provided by the IMU sensor interated into the Depth camera.

# Roboteq Motor controller:
The dual DC motors require 48v(DC) to operate and are controlled by a generic version of the roboteq motor controller, the motor controller communicates with the operating system via USB port. the ros launch file  "differential_drive.launch "  from Vinebot_ROS_Packages/roboteq_control/launch/differential_drive.launch launches the node that can be used to send commands to the motor controller. in order for the differential_drive.launch file to recogonize the serial port of the USB connceted to the motor controller a set of udev rules must be approved. the pocedure to update the udev rules is provided in the udev rules section. Figure 2 depicts the motor controller with some other components.
![Alt text](/images/vinebot_hardware_setup.jpeg "VINEBOT-hardware setup")
<p align="center">
Figure 2: 1.- DC power source from the batteries (48v), 2.-Motor Controller, 3.- 48v to 5v DC to DC converter, 4.- 48v to 12v DC to DC converter, 5.- BNO055 IMU Sensor and 6.- Wireless Emergency stop. 
</p>

# Robot model-  the URDF file of the Vinebot.

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





