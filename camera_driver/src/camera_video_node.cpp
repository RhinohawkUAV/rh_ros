/*****************************************************************************/
/** ROS Camera Driver                                                       **/
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

int main(int argc, char **argv)
{
    /* Ros node initilalization */
    ros::init(argc, argv, "camera_driver");
    ros::NodeHandle nh;
    
    /* Initialze the image transport class and create a publisher to
     * publish the images
     * see: wiki.ros.org/image_transport/Tutorials/PublishingImages
     */
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    
    
    /* Create an OpenCV videocapture object */ 
    cv::VideoCapture cap("/testImages/test.MP4");
    if(!cap.isOpened())
    {
        ROS_ERROR("Could Not Open Capture!");
        return -1;
    }
    
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
    
    ros::Rate loop_rate(50); /* Loop at 50Hz */
    while(ros::ok())
    {
        cv::Mat frame;
        /* grab a frame from the camera */
        if(cap.read(frame))
        {
            //cv::imshow(OPENCV_WINDOW, frame);
            
            cv::Mat im = frame.clone(); /* Deep copy the returned image */
            
            /* Copy the image to the CvImage object and convert to to a
             * ROS Image message that we can publish to the channel
             */
            out_cv_image.image = im; 
            sensor_msgs::ImagePtr msg = out_cv_image.toImageMsg();
            pub.publish(msg);
            // ROS_INFO("Published Frame!");
        }
        cv::waitKey(1);
        loop_rate.sleep();
    }

    cv::destroyWindow(OPENCV_WINDOW);
    return 0;
}