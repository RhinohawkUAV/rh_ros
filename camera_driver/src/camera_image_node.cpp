/*****************************************************************************/
/** ROS Camera Driver - image                                               **/
/**                                                                         **/
/*****************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

/* Adapted from wiki.ros.org/image_transport/Tutorials/PublishingImages */

static const std::string OPENCV_WINDOW = "Image window";
static const std::string DEFAULT_IMAGE = "/testImages/test_image_dots.png";

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
    image_transport::Publisher pub = it.advertise("/camera/image", 1);
    
    /* Get the parameter from the parameter server to find if the user
     * set a diffrent image. To set an image start the node with the argument
     * _image_path:=<path>
     */ 
    std::string im_name;
    if(!nh.getParam("image_path", im_name))
    {
        im_name = DEFAULT_IMAGE;
    }
    
    /* Create an OpenCV image object */ 
    cv::Mat im = cv::imread(im_name);
    
    /* For some reason, the camera will not capture unless there is 
     * a window created, so create a window to make it happy
     * TODO: figure out how to get around this.
     */
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
    
    /* Create a cv_bridge CvImage and set the encoding so we can convert
     * it to a ROS Image message to publish to the ROS network
     * see: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
     * TODO: The ros Image message has a header with the current timestamp that
     * is being ignored. This will be necessiceary to do camera calibration.
     */
    cv_bridge::CvImage out_cv_image;
    out_cv_image.encoding = "bgr8"; 
    
    ros::Rate loop_rate(1); /* Loop at 1Hz */
    while(ros::ok())
    {
        /* Copy the image to the CvImage object and convert to to a
         * ROS Image message that we can publish to the channel
         */
        out_cv_image.image = im; 
        sensor_msgs::ImagePtr msg = out_cv_image.toImageMsg();
        pub.publish(msg);
        // ROS_INFO("Published Frame!");
        
        cv::waitKey(1);
        loop_rate.sleep();
    }

    cv::destroyWindow(OPENCV_WINDOW);
    return 0;
}