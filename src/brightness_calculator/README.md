Package: brightness_calculator
-------------------------------------------------------------------
Description: This package implements the solution to 1.1.2 and 1.1.3 (brightness, and then brightness with params)

Inputs:
/image
    Type: sensor_msgs/msg/Image

Outputs:
/is_light
    Type: std_msgs/msg/Bool
/bw_image
    Type: std_msgs/msg/Image

Parameters:
        threshold: The default brightness value to set is_light to true. default=20.
        publish_mode : Sets whether it publishes each time or only when things change. default='continuous'
        colour_name: The name of the colour being segmented. default='green'
        show_camera: Whether it should show a cv2 imshow window. default=true
        brightness_threshold: The brightness at which to recognize a pixel when using brightness.
        lower_hue: The lower hue value to recognize as part of the object. default=27
        upper_hue: The upper hue value to recognize as part of the object. default=85
        lower_saturation: The lower saturation value to recognize as part of the object. default=50
        upper_saturation: The upper saturation value to recognize as part of the object. default=255
        lower_value: The lower value value to recognize as part of the object. default=50
        upper_value: The upper value value to recognize as part of the object. default=255
        image_source_topic: The topic to process and segment. default='/image'.

Core components: 
        dynamics_timer_callback(): Calls and runs the model every timestep
        webcam_topic_callback(): Ingests and passes on webcam image stream
        CreateCVSubimage(): Creates sub-image which is then published on /output/moving_camera

Commands:
ros2 run brightness_calculator brightness_calculator_node --ros-args --params-file src/brightness_calculator/config/brightness_calculator.yml

ros2 run brightness_calculator colour_segmenter_node --ros-args --params-file src/brightness_calculator/config/colour_segmenter.yml

