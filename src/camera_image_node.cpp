/*****************************************************************************/
/** ROS Camera Driver - image                                               **/
/**                                                                         **/
/*****************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

/* Adapted from wiki.ros.org/image_transport/Tutorials/PublishingImages */

static const std::string OPENCV_WINDOW = "Image window";
static const std::string DEFAULT_IMAGE = "/tmp/IMG_0246.JPG";
static image_transport::Publisher publisher;

/* Create a cv_bridge CvImage and set the encoding so we can convert
 * it to a ROS Image message to publish to the ROS network
 * see: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * TODO: The ros Image message has a header with the current timestamp that
 * is being ignored. This will be necessiceary to do camera calibration.
 */
void imageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received image: %s", msg->data.c_str());
  cv::Mat im = cv::imread(msg->data);
  cv_bridge::CvImage out_cv_image;
  out_cv_image.encoding = "bgr8"; 
  out_cv_image.image = im; 
  sensor_msgs::ImagePtr outbound_image = out_cv_image.toImageMsg();
  publisher.publish(outbound_image);
  ROS_INFO("Published ROS image.");
}


int main(int argc, char **argv)
{
    /* Ros node initilalization */
    ros::init(argc, argv, "camera_image");
    ros::NodeHandle nh("~");
    
    /* Initialze the image transport class and create a publisher to
     * publish the images
     * see: wiki.ros.org/image_transport/Tutorials/PublishingImages
     */
    image_transport::ImageTransport it(nh);
    publisher = it.advertise("/camera/image", 1);

    // listen for inbound files to read
    ros::Subscriber sub = nh.subscribe("/camera_images", 5, imageCallback);
    
    ROS_INFO("Ready");

    ros::spin();
}
