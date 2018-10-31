#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "obc_pilot_console");
    ros::NodeHandle nh;

    ros::Publisher local_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("pilot_local_sp", 100);


    // wait for FCU connection
    while(ros::ok()){
      ROS_INFO("ENTER local_pos_sp : ");
      geometry_msgs::PoseStamped pose;

        std::cin>>pose.pose.position.x>>pose.pose.position.y>>pose.pose.position.z;
         local_sp_pub.publish(pose);
        ros::spinOnce();
    }

    return 0;

}
