rosrun  gazebo_ros gazebo /home/jieming/catkin_ws/src/panda_simulation/panda_simulation/config/wall_world.xml

rosservice call /world/build_octomap '{bounding_box_origin: {x: 0, y: 0, z: 15}, bounding_box_lengths: {x: 30, y: 30, z: 30}, leaf_size: 0.5, filename: output_filename.bt}'

