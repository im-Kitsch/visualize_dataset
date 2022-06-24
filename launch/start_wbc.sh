rosservice call /controller_manager/switch_controller \
"start_controllers:
 - ''
stop_controllers:
 - 'head_controller'
 - 'arm_left_controller'
 - 'arm_right_controller'
 - 'torso_controller'
 - 'whole_body_kinematic_controller'
strictness: 0"

rosservice call /controller_manager/unload_controller "{name:
'whole_body_kinematic_controller'}"

roslaunch tiago_dual_wbc tiago_dual_wbc.launch

