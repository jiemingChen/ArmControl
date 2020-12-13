rosrun  gazebo_ros gazebo /home/jieming/catkin_ws/src/panda_simulation/panda_simulation/worlds/wall_world.xml

rosservice call /world/build_octomap '{bounding_box_origin: {x: 0, y: 0, z: 15}, bounding_box_lengths: {x: 2, y: 2, z: 30}, leaf_size: 0.5, filename: output_filename.bt}'

 rosservice call /world/build_octomap '{bounding_box_origin: {x: 0, y: 0, z: 1}, bounding_box_lengths: {x: 1.8, y: 1.8, z: 2}, leaf_size: 0.05, filename: 0_05small.bt}'

