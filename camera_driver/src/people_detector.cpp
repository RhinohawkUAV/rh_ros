/*****************************************************************************/
/** ROS Image People Detector                                               **/
/**                                                                         **/
 /*****************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>


static const std::string OPENCV_WINDOW = "people_detector/image_annotated";
static image_transport::Publisher pub;
static cv::HOGDescriptor hog;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  ROS_INFO("Looking for dudes");
  
  // ROS to openCV image format
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
  

  ROS_INFO("Classifying image");
  std::vector<cv::Rect> found, found_filtered;
  double t = (double)cv::getTickCount();
  // run the detector with default parameters. to get a higher hit-rate
  // (and more false alarms, respectively), decrease the hitThreshold and
  // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).  
  hog.detectMultiScale(cv_ptr->image, found, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
  t = (double)cv::getTickCount() - t;
  ROS_INFO("detection time = %gms", t*1000./cv::getTickFrequency());

  if(found.size() > 0)
    {
      ROS_INFO("People detected");
    }
  else
    {
      ROS_INFO("No people");
    }
  size_t i, j;
  for( i = 0; i < found.size(); i++ )
    {
      cv::Rect r = found[i];
      for( j = 0; j < found.size(); j++ )
	if( j != i && (r & found[j]) == r)
	  break;
      if( j == found.size() )
	found_filtered.push_back(r);
    }
  for( i = 0; i < found_filtered.size(); i++ )
    {
      cv::Rect r = found_filtered[i];
      // the HOG detector returns slightly larger rectangles than the real objects.
      // so we slightly shrink the rectangles to get a nicer output.
      r.x += cvRound(r.width*0.1);
      r.width = cvRound(r.width*0.8);
      r.y += cvRound(r.height*0.07);
      r.height = cvRound(r.height*0.8);
      cv::rectangle(cv_ptr->image, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
    }
  
  // Debugging
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);

  // Output modified video stream
  pub.publish(cv_ptr->toImageMsg());
}


int main(int argc, char **argv)
{
  // Start ROS node 
  ros::init(argc, argv, "people_detector");
  ros::NodeHandle nh("~");
  
  // Start image transport pipeline
  // cv::namedWindow(OPENCV_WINDOW);
  hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
  pub = it.advertise("image_people", 1);

  // loitter
  int threads;
  nh.param("threads", threads, 1);
  ROS_INFO("Threads: %d",  threads);
  ros::MultiThreadedSpinner spinner(threads); // Use more than one thread
  spinner.spin(); // spin() will not return until the node has been shutdown
  //ros::spin();
}
