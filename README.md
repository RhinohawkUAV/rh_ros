# ROS nodes for image processing pipeline


## Cannon S100 image capture node

The s100_image_capture node listens for photo request, triggers the camera, copies the image to the localfile system, and then publishes the name of the file. 


Start the S100 camera node

    rosrun camera_driver s100_image_capture.py


Listen for responses from S100 camera node

    rostopic echo image_filename


Trigger an image capture from the s100 node

    rostopic pub -1 image_trigger std_msgs/Empty


## Camera Images Node

The camera_images node subscribes to the filename topic and then picks the image up, translates to ROS sensor_msg/Image, and publishes the ROS image. 


Start the camera images node

    rosrun camera_driver camera_images

Listen for published images

    rostopic echo /camera/image > foo

Trigger reading an image from filesystem

    rostopic pub -1 /camera_images std_msgs/String /tmp/IMG_0246.JPG

    