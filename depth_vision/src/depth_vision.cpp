

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
void  edgedetect(unsigned char * pix, int count, int cols);

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
//VideoCapture vcap;  
//vcap.open("http://192.168.10.170:8081");
VideoCapture vcap("/home/ubuntu/CVTestNova.mp4");
vcap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
vcap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  Mat frame2;
  Mat frame;\
  namedWindow("frame", CV_WINDOW_AUTOSIZE);
  namedWindow("Depth_Vision", CV_WINDOW_AUTOSIZE);
  startWindowThread();
  
  ros::NodeHandle n;

  ros::Subscriber dv_sub = n.subscribe("masterControl", 1000, commandCallback);

  ros::Rate loop_rate(30);
  frame = imread("/run/user/1000/gvfs/gphoto2:host=%5Busb%3A002%2C057%5D/DCIM/102D5100/DSC_0077.JPG");
  while (ros::ok()){

    //opemcv code

    //grab frame
    vcap.read(frame);

    if(frame.data == NULL){

      std::cout << "noframe!";
    }
    else{
	resize(frame, frame, Size(960,540));
	cvtColor(frame, frame2, CV_RGB2GRAY);
       unsigned char * data = (unsigned char *)(frame2.data);
       edgedetect( data, frame2.rows * frame2.cols, frame2.cols);
      // frame2.data = data;
       imshow("Depth_Vision", frame2);
       imshow("frame", frame);
}
    //message part
    ros::spinOnce();
    loop_rate.sleep(); 
  } 
  // ros::spin();
  return 0;
}

