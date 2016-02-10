/*****************************************************************************/
/** ROS Image Processing Example                                            **/
/**                                                                         **/
/*****************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

/* Adapted from: http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages */

class Threshold
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

public:
    Threshold() : it(nh)
    {
        /* Initialize a subscriber and subscribe to the camera feed */
        image_sub = it.subscribe("/camera/image", 1,
            &Threshold::imageCallback, this);
        
        /* Initialize an image publisher to publish our new image */
        image_pub = it.advertise("threshold", 1);
    }
    
    ~Threshold()
    {
        // nothing to destroy
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        /* Conver the ROS Image message to a format that we can use with OpenCV 
         * see: http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
         */
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            /* Convert to an OpenCV creating a copy of the image we can
             * modify. Also conver the image to grayscale
             */
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        /* -------------- Implement vision algorithms here ---------------- */
        
        static const int thresh = 127; /* Configure the threshold */
        
        /* Implement a simple thresholding algorithm */
        for(int r = 0; r < cv_ptr->image.rows; r++)
        {
            for(int c = 0; c < cv_ptr->image.cols; c++)
            {
                if(cv_ptr->image.at<uchar>(r, c) > thresh)
                {
                    cv_ptr->image.at<uchar>(r, c) = 255;
                }
                else
                {
                    cv_ptr->image.at<uchar>(r, c) = 0;
                }
            }
        }

        
        /* -------------------- end of implementation --------------------- */
        image_pub.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv)
{
    /* ROS node initialization */
    ros::init(argc, argv, "threshold");
    ros::NodeHandle nh;
    
    Threshold t;
    
    ros::spin();
    
    return 0;
}