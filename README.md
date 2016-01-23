ROS nodes for image processing pipeline


Start the S100 camera node

    rosrun camera_driver s100_image_capture.py


Listen for responses from S100 camera node

    rostopic echo image_filename


Trigger an image capture from the s100 node

    rostopic pub -1 image_trigger std_msgs/Empty


