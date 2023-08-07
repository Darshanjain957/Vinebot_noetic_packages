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

# LIVOX SDK Installation: 

the livox sensor requires Livox SDK a software development kit developed by livox for the communication between livox mid 40 and ros.

The package can be installed form the git hub repository provided by livox, detailed installation instructions are provided by livox in the git hub repository.

Livox SDK: https://github.com/Livox-SDK/Livox-SDK


# Realsense2 sdk Installation:
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
The dual DC motors require 48v(DC) to operate and are controlled by a generic version of the roboteq motor controller, the motor controller communicates with the operating system via USB port. the ros launch file  "differential_drive.launch "  from Vinebot_ROS_Packages/roboteq_control/launch/differential_drive.launch launches the node that can be used to send commands to the motor controller. in order for the differential_drive.launch file to recogonize the serial port of the USB connceted to the motor controller a set of udev rules must be approved. The pocedure to update the udev rules is provided in the udev rules section.
A short description of the motors is given below, and Figure 2 depicts the motor controller with some other components.

Description Specification
Operation Voltage 48V
Output power 400W each
Rated current 8-10A
Peak Torque from the motor 15Nm
Rated RPM from the gearbox 100-200 RPM
Max velocity 6 km/h
Climbing slope 45 degrees


![Alt text](/images/vinebot_hardware_setup.jpeg "VINEBOT-hardware setup")
<p align="center">
Figure 2: 1.- DC power source from the batteries (48v), 2.-Motor Controller, 3.- 48v to 5v DC to DC converter, 4.- 48v to 12v DC to DC converter, 5.- BNO055 IMU Sensor and 6.- Wireless Emergency stop. 
</p>

# Robot model - the URDF file of the Vinebot.
URDF is an XML format for representing a robot model. THE urdf file contains the necessary joints, links and apprppriate TF transform required for the representation of the Vinebot in ROS. Vinebot_ROS_Packages/vinebot_decription/urdf contains the various xacro files that define the robot model in RVIZ.  The file "display.launch" from Vinebot_ROS_Packages/roboteq_control/launch is used in the bringup.launch files to display the vinebot, it utilizes  "differential-robot.urdf.xacro" a simplified file used for the drive of the robot is provided in the "Vinebot_ROS_Packages/roboteq_control/urdf" to represent the vinebot.

# BNO 055 9 DOF IMU Sensor:
BNO 055 is one of the IMU sensors that are being utlized in the vinebot, it is located at the rear of the vinebot(figure 2). some of its specifications are listed below:
The BNO055 can output the following sensor data:

Absolute Orientation (Euler Vector, 100Hz) Three axis orientation data based on a 360° sphere
Absolute Orientation (Quatenrion, 100Hz) Four point quaternion output for more accurate data manipulation
Angular Velocity Vector (100Hz) Three axis of 'rotation speed' in rad/s
Acceleration Vector (100Hz) Three axis of acceleration (gravity + linear motion) in m/s^2
Magnetic Field Strength Vector (20Hz) Three axis of magnetic field sensing in micro Tesla (uT)
Linear Acceleration Vector (100Hz) Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
Gravity Vector (100Hz) Three axis of gravitational acceleration (minus any movement) in m/s^2
Temperature (1Hz) Ambient temperature in degrees celsius 

for more information about the IMU sensor refer: https://www.adafruit.com/product/2472

the IMU communicates with the operating system via an UART to USB converter. a modification of the USB serial port is required for the ROS packages to recogonize the data from the IMU,an Udev rule is defined,The pocedure to update the udev rules is provided in the udev rules section. THE imu data can be accessd and used by utilizing "imu.launch" from "Vinebot_ROS_Packages/ros_imu_bno055/launch" folder and "ros_imu_bno055" package.
# Realsense D455
RealSense D455 is the depth camera used in this project the camera is mounted at the front of the vehicle, the camera can be used for various purposes like mapping, navigation, object detection and much more. a brief description of the cameras specification is given below:

Specification
Depth FOV: 87°(H)x58°(V)x 95°(D)

Colour camera FOV: 90°(H)x63°(V)x 98°(D)

Ideal Range: 0.4m to 6m

Output Resolution (Depth): Up to 1280x720

Output Resolution (RGB): Up to 1280X800

Integrated IMU: Bosch BMI055

Connector: USB-C

For additional information about RealSense D455 refer: https://www.intelrealsense.com/depth-camera-d455/

 The camera uses USB to communicate with the operating system. the integration of the camera into ros and the operating system requires a set of libraties provided by RealSense which can be installed by following instrucions provided in the  Realsense2 sdk installation section. The ROS package realsesne-ros can be use to access the camera data,there are various launch files in the launch folder  located at "realsense-ros/realsense2_camera/launch/". the launch file "realsense-ros/realsense2_camera/launch/rs_camera_imu.launch" launch the node that publishes camera data as well as the data from the IMU integrated into the camera.     Figure 3 depicts the front view for the vinebot with RealSense D455 and Livox Mid-40.
 
 ![Alt text](images/Vinbot_front.jpg "vinebot-front")
<p align="center">
Figure 2:Vinebot front view 1.- Livox Mid-40 and 2.- RealSense D455   
</p>

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





