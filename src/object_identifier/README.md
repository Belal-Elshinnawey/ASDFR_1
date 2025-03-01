Package object_identifier
-----------------------------------------------
Description:This package recieves an black and white image, and publishes the center of mass of the white object, or (-1,-1) if none.

Input:
/bw_image 
        Type: sensor_msgs/msg/Image
        Image recieved with black and white colors.
Output:
/object_position 
        Type: geometry_msgs/msg/Point
        x,y points where the center of mass is calculated, the z coordinates contain the image dimensions
Run:
        In a terminal run the following commands:
        ros2 run object_identifier object_identifier_node --ros-args --params-file src/object_identifier/config/object_identifier.yml

Parameters:
        bool test_mode: if true, the node runs a sequence of set positions for 25 seconds, if false, the node requires coordinates to send set positions to relbot

Note:
    Make sure to run RELBOT in twist mode.
