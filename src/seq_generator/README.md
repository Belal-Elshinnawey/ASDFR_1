Package sequence_generator
-----------------------------------------------
Description:This package recieves object coordinates, and sends setpoints to follow it to the relbot to follow.

Output:
/input/twist 
        Type: geometry_msgs/msg/Twist
        Published set points to relbot.

Run:
        In a terminal run the following commands:
        ros2 run sequence_generator sequence_generator_node --ros-args --params-file src/seq_generator/config/sequence_generator.yml

Parameters:
        bool test_mode: if true, the node runs a sequence of set positions for 25 seconds, if false, the node requires coordinates to send set positions to relobtt

Note:
    Make sure to run RELBOT in twist mode.

