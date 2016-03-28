/*****************************************************************************/
/** ROS Topic to video stream                                               **/
/**                                                                         **/
/*****************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

static cv::VideoWriter outputVideo;  
static int count = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Incoming image");
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  outputVideo.write(cv_ptr->image);
}


int main(int argc, char **argv)
{
  // Start ROS node 
  ros::init(argc, argv, "to_stream");
  ros::NodeHandle nh("~");

  // Subscribe to image topic
  image_transport::ImageTransport it(nh);
  std::string image;
  nh.param<std::string>("image", image, "image");
  ROS_INFO("Listening to: %s", image.c_str());
  image_transport::Subscriber sub = it.subscribe(image, 1, imageCallback);

  // Start OpenCV stream writer
  std::string stream;
  nh.param<std::string>("stream", stream, "stream.avi");
  ROS_INFO("Writing to: %s", stream.c_str());
  cv::Size S = cv::Size(1280, 960);
  int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
  outputVideo.open(stream, CV_FOURCC('X', 'V', 'I', 'D'), 10, S, true);
  if(!outputVideo.isOpened())
    {
      ROS_ERROR("Cannot open file for write");
      return -1;
    }
	
  // wait for messages
  ROS_INFO("Spining");
  ros::spin();
}
