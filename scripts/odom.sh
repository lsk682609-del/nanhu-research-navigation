  ros2 topic pub /gps1/odom nav_msgs/msg/Odometry \
    "{header: {stamp: now, frame_id: 'map'}, \
      child_frame_id: 'base_link', \
      pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, \
      twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, \
              angular: {x: 0.0, y: 0.0, z: 0.0}}}}" -r 10
