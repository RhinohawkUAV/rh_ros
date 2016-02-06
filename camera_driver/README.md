# ROS nodes for image processing pipeline


## Cannon S100 image capture node

A ROS camera driver node that uses CHDK to take a photo from a Canon S100


Start the S100 camera node

    rosrun camera_driver s100.py


Request a photo from the service s100/request_image

    rosrun camera_driver s100_service_client.py


Listen to the S100 photo queue at s100/image_raw

     rosrun camera_driver s100_pic_client.py


    


    