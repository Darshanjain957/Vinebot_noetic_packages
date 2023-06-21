# Vinebot_noetic_packages
ROS packages developed for the vinebot, it includes updated versions of Vinbot_ros_packages, realsense ros wrapper, livox ros wrapper, and mapping packages and some additional packages. 
Prerequisites and dependencies 
before installing the catkin workspace some additional libraries and dependencies are required. 

Pcl :  $ sudo apt install libpcl-dev  pcl-tools       
Eigen: $ sudo apt install libeigen3-dev               
opencv:$ sudo apt install libopencv-dev python3-opencv

LIVOX SDK Installation: 
the livox sensor requires Livox SDK a software development kit developed by livox for the communication between livox mid 40 and ros.

The package can be installed form the git hub repository provided by livox, detailed installation instructions are provided by livox in the git hub repository.

Livox SDK: https://github.com/Livox-SDK/Livox-SDK


Realsense2 sdk Installation:
detailed instructions can be found here : https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
for jetson devices please refer: https://github.com/IntelRealSense/librealsense/issues/6964#issuecomment-707501049

Register the server's public key
$ sudo mkdir -p /etc/apt/keyrings

$ curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

Add the server to the list of repositories:

$ echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \

$ sudo tee /etc/apt/sources.list.d/librealsense.list

$ sudo apt-get update

install realsesne sdk:

$ sudo apt-get install librealsense2-dkms

$ sudo apt-get install librealsense2-utils

optional dev and debug packages:

$ sudo apt-get install librealsense2-dev

$ sudo apt-get install librealsense2-dbg


these packages are suitable for ubuntu 20.04 and requires ros noetic.
§ sudo apt install ros-noetic-desktop-full


# Make catkin workspace if does not exist and clone this repo


mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

git clone 

# Install all dependecies

cd ..

rosdep install --from-paths src --ignore-src -r -y

# Compile package

catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc



