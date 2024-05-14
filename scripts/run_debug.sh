catkin_make install -DCMAKE_BUILD_TYPE=Debug
rosrun --prefix "valgrind --tool=callgrind --callgrind-out-file=outfile" map_fusion map_fusion_node
