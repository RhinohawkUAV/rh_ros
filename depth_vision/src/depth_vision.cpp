

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

bool process;
int frameInt;

void commandCallback(const std_msgs::String::ConstPtr& msg){

    ROS_INFO("Commandf001: [%s]", msg->data.c_str());

  if (msg->data.c_str() == "proccess all") {
      frameInt = 0;
      }

}

using namespace cv;

  int main(int argc, char ** argv){

  ros::init(argc, argv, "dv_vision");


  VideoCapture vcap(0);

  Mat frame;
  namedWindow("Depth_Vision", CV_WINDOW_AUTOSIZE);
  startWindowThread();

  ros::NodeHandle n;

  ros::Subscriber dv_sub = n.subscribe("masterControl", 1000, commandCallback);

  ros::Rate loop_rate(30);

  while (ros::ok()){

    //opemcv code

    //grab frame
    vcap.read(frame);

    if(frame.data == NULL){

      std::cout << "noframe!";
    }
    else{
       imshow("Depth_Vision", frame);
}
    //message part
    ros::spinOnce();
    loop_rate.sleep(); 
  } 
  // ros::spin();
  return 0;
}

