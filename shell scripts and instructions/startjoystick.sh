sudo pkill ds4drv
rosnode kill joy_node
rosnode kill ps4_ros


echo ---
echo Step 1: Starting driver...
echo ---

sudo ds4drv --hidraw 2>&1 | tee logsd4drv.log &
sleep 10

echo ---
echo Step 2: Start joy_node
echo ---

rosparam set joy_node/dev "/dev/input/js1"
rosrun joy joy_node 2>&1 | tee  logjoynode.log &
sleep 10

echo ---
echo Step 3: Start ps4_ros
echo ---
rosrun ps4_ros ps4_ros 


