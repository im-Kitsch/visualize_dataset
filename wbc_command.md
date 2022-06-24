# WBC retarget

- ` bash /home/pro_mp/catkin_ws/src/visualize_dataset/launch/launch_wbc.sh`
- `roslaunch visualize_dataset visualize_data.launch`
- `rosbag play handshake_filtered.bag -r 0.3 --pause`
- `rosrun visualize_dataset wbc_motion_transfer.py`

# WBC Test Thing

/opt/ros  /opt/pal/ferrum 都要拷贝过去，因为有一些包相互冲突

1. roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch
2. start_wbc.sh
3. `roslaunch tiago_dual_wbc push_reference_tasks.launch`
4. runs 
    
    `rosrun rviz rviz -d $(rospack find tiago_dual_wbc)/config/rviz/tiago_dual_wbc.rviz`
    

 --
Choice 1:
     run launch/launch_wbc.sh
Choice 2:
1. roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch tuck_arm:=false
2. start_wbc
3. `roslaunch visualize_dataset custom_reference_tasks.launch`
  or `roslaunch /home/pro_mp/catkin_ws/src/visualize_dataset/launch/custom_reference_tasks.launch`
     // or `roslaunch tiago_dual_wbc    custom_reference_tasks.launch`

4. `rosrun rviz rviz -d $(rospack find visualize_dataset)/config/rviz.rviz`
    
    
    ```bash
    rostopic pub \
    /whole_body_kinematic_controller/arm_left_tool_link_goal \
    geometry_msgs/PoseStamped "
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: '/base_footprint'
    pose:
      position:
        x: 1.0
        y: 0.0
        z: 0.5
      orientation:
        x: 0.0
        y: 1.0
        z: 0.0
        w: 0.0"
    ```
    
    Check here to config WBC
    
    [https://github.com/pal-robotics/pal_wbc_utils](https://github.com/pal-robotics/pal_wbc_utils)
    
    [http://wiki.ros.org/pluginlib](http://wiki.ros.org/pluginlib)
    

Moving indivisual joints

- `roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true world:=empty tuck_arm:=false`
- `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller`
- `rosrun rviz rviz -d $(rospack find visualize_dataset)/config/rviz.rviz`

[http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/motions/rqt_joint](http://wiki.ros.org/Robots/TIAGo%2B%2B/Tutorials/motions/rqt_joint)
