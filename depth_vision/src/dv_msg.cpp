/**
RHINO Hawk ROS CODE for inter device comunuication

@auther Sam Winkelstein

**/

// includes

#include <string>
#include <ros/serialization.h>
#include <std_msgs/Int64.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace std;
//main method

int main(int argc, char ** argv){

  //intitialize ros node
  ros::init(argc,argv, "dv_message");

  ros::NodeHandle n;
      
  ros::Publisher test_pub = n.advertise<std_msgs::String>("masterControl", 1000);
  
  ros::Rate loop_rate(10);

  int count = 0;

  while(ros::ok()){
    
    std_msgs::String msg;
    std::stringstream ss;
	string message1;
	string message2;
	cin >> message2 >> message1;
      ss << message2 << message1;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());


      test_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  return 0;

}


