# Vinebot_ROS_Packages 

The set of packages for the Vinebot robot 

HERE you can find all the nesseccary Vinebot ROS packages up to date!

# Install

Clone on your catkin workspace this repository, download all dependencies and compile!

```bash
# Make catkin workspace if does not exist and clone this repo
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/yerkgb/Vinebot_ROS_Packages.git
# Install all dependecies
cd ..
rosdep install --from-paths src --ignore-src -r -y
# Compile package
catkin_make
```

Don't forget to add in your bash your catkin sources

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Run

```bash
roslaunch roboteq_control vinebot.launch
```

:rocket: That's it!
