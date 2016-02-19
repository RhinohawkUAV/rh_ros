* Nikon D5100 Camera Driver

Implement Polled Camera Node for Nikon D5100

http://wiki.ros.org/camera_drivers

Service 
/request_image (polled_camera/GetPolledImage)

Published Topics:
/d5100/image_raw (sensor_msgs/Image)



The node currently works by

* Set the camera to store images in RAM
* Shell out to gphoto2 to capture and transfer the image
* Write the image on a ram disk
* Use OpenCV to read the JPEG image
* OpenCV bridge it ROS sensor_msgs/Image
* publish the image


This node
    root@ubuntu:~# mkdir /tmp/ramdisk
    root@ubuntu:~# chmod 777 /tmp/ramdisk/
    root@ubuntu:~# mount -t tmpfs -o size=256M tmpfs /tmp/ramdisk/


or add this to /etc/fstab

    tmpfs  /mnt/ramdisk tmpfs  nodev,nosuid,noexec,nodiratime,size=50m 0 0


