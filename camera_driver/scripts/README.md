# Nikon D5100 Camera Driver

A ROS Polled Camera Node for Nikon D5100.  The interface follows the ROS spec
at http://wiki.ros.org/camera_drivers.  To trigger an image capture call the 
request_image service.  When images are captured they are published to the
image_raw topic as a sensor_msg/Image message.

Services: 

    /d5100/request_image (polled_camera/GetPolledImage)
    /d5100/set_camera_info (sensor_msgs/CameraInfo) 

Published Topics:

    /d5100/image_raw (sensor_msgs/Image)


The current python implementation works like this:

* Set the camera to store images in RAM
* Shell out to gphoto2 to capture and transfer the image from camera to node
* Read stdout from gphoto2 and write the image to disk (a ram disk)
* Use OpenCV to read the JPEG image
* OpenCV bridge to ROS sensor_msgs/Image format
* Publish the image to /image_raw


To start the node:

    rosrun camera_driver nikon_d5100.py

To trigger a capture run the service client:

    python camera_driver/scripts/nikon_d5100_service_client.py


For best results provide a ramdisk at /mnt/ramdisk.  You can make a temporary one like this:

    # mkdir /tmp/ramdisk
    # mount -t tmpfs -o size=50m tmpfs /tmp/ramdisk/


Or add this to /etc/fstab for a ramdisk that is created at boot time 
(the filesystem does not survive reboots).

    tmpfs  /mnt/ramdisk tmpfs  nodev,nosuid,noexec,nodiratime,size=50m 0 0



# GoPro Camera driver.

A node just like the Nikon driver.  Used for testing, etc.


Services: 

    /gopro/request_image (polled_camera/GetPolledImage)
    /gopro/set_camera_info (sensor_msgs/CameraInfo) 

Published Topics:

    /gopro/image_raw (sensor_msgs/Image)


To start the node:

    rosrun camera_driver gopro.py

To trigger a capture run the service client:

    python camera_driver/scripts/gopro_service_client.py

