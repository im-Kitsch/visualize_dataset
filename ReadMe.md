

### filter tf message

example
```commandline
rosbag filter input.bag output.bag 
 'topic == "/camera/image_raw/compressed" or 
 topic == "/scan" or topic == "/timetag" or  
 topic == "/tf" and m.transforms[0].header.frame_id != "/odom" and m.transforms[0].child_frame_id != "/odom"'
```

used for Zen2 camera
```commandline
rosbag filter handshake.bag handshake_filtered.bag  \
 'topic == "/tf" and m.transforms[0].header.frame_id != "/odom" and m.transforms[0].child_frame_id != "/odom" or \
  topic == "/tf" and m.transforms[0].header.frame_id != "/map" and m.transforms[0].child_frame_id != "/map"  or \
  topic == "/tf" and m.transforms[0].header.frame_id != "/base_link" and m.transforms[0].child_frame_id != "/base_link" or \
  topic == "/tf" and m.transforms[0].header.frame_id != "/zed_base_link" and m.transforms[0].child_frame_id != "/zed_base_link" or \
  topic == "/tf" and m.transforms[0].header.frame_id != "/zed_camera_center" and m.transforms[0].child_frame_id != "/zed_camera_center" or \
  topic == "/tf" and m.transforms[0].header.frame_id != "/zed_imu_link" and m.transforms[0].child_frame_id != "/zed_imu_link" \
  ' \
  
rosbag filter handshake.bag handshake_filtered.bag  \
 'topic == "/tf" and m.transforms[0].header.frame_id != "odom" and m.transforms[0].child_frame_id != "odom"' 
```