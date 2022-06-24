#/bin/sh

wbc_activate2="source /opt/ros_wbc/melodic/setup.bash;source /opt/pal/ferrum/setup.bash; PS1='(wbc)$PS1_BACKUP'"

gnome-terminal --tab  -- bash -c  \
    "$wbc_activate2;roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch tuck_arm:=false; exec bash -i"
read -p "Launching gazebo environment, Press enter to continue"

gnome-terminal --tab  -- bash -c   \
    "$wbc_activate2; bash /home/pro_mp/catkin_ws/src/visualize_dataset/launch/start_wbc.sh; exec bash -i"
read -p "starting wbc controller Press enter to continue"

gnome-terminal --tab  -- bash -c   \
    "$wbc_activate2;roslaunch /home/pro_mp/catkin_ws/src/visualize_dataset/launch/custom_reference_tasks.launch; exec bash -i"
#read -p "pushed reference tasks Press enter to continue"

#echo "Launching rviz, good luck this time"
#gnome-terminal --tab  -- bash -c  \
#  "$wbc_activate2;rosrun rviz rviz -d /home/pro_mp/catkin_ws/src/visualize_dataset/config/rviz.rviz; exec bash -i"
