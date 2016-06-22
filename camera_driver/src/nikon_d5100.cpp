/*****************************************************************************/
/** ROS Camera Driver for Nikon D5100                                       **/
/**                                                                         **/
/*****************************************************************************/

#include <fcntl.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <sstream> 
#include <iomanip>
#include <cstdlib>

#include <gphoto2/gphoto2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <polled_camera/publication_server.h>
#include <camera_info_manager/camera_info_manager.h>


using namespace std;


Camera  *camera;
GPContext *context;
string name = "d5100";
sensor_msgs::CameraInfo camera_info;


static void ctx_error_func (GPContext *context, const char *str, void *data)
{
  ROS_INFO("gphoto error: %s", str);
}


static void ctx_status_func (GPContext *context, const char *str, void *data)
{
  ROS_INFO("gphoto info: %s", str);
}


// gphoto code from from https://github.com/fape/libgphoto2/blob/master/examples/sample-capture.c
void capture_to_file(Camera *camera, GPContext *context, const char *fn) 
{
  ROS_INFO("capture_to_file.\n");
  int fd, retval;
  CameraFile *file;
  CameraFilePath camera_file_path;
  strcpy(camera_file_path.folder, "/");
  strcpy(camera_file_path.name, "foo.jpg");
  retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
  fd = open(fn, O_CREAT | O_WRONLY, 0644);
  retval = gp_file_new_from_fd(&file, fd);
  retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);
  retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);
  gp_file_free(file);
}


string get_local_filename(uint seq)
{
  stringstream number_stream;
  number_stream << setfill('0') << setw(4) << seq;
  stringstream local_filename_stream;
  local_filename_stream << "/mnt/ramdisk/d5100-" << number_stream.str() << ".jpg";
  return local_filename_stream.str();
}

void 
capture_callback(polled_camera::GetPolledImage::Request& request, 
		 polled_camera::GetPolledImage::Response& response,
		 sensor_msgs::Image& the_image, 
		 sensor_msgs::CameraInfo& the_camera_info)
{

  ROS_INFO("capture_callback");
  static uint image_seq = 0;

  // capture image and write to local disk
  string local_filename = get_local_filename(image_seq++);
  capture_to_file(camera, context, local_filename.c_str());
  ros::Time pic_time =  ros::Time::now();
  ROS_DEBUG("Image captured");

  // read jpg with openCV.
  cv::Mat image = cv::imread(local_filename, CV_LOAD_IMAGE_COLOR);
  if(!image.data)
    {
      ROS_ERROR("Opencv could not read local image file");
      return;
    }

  ROS_DEBUG("Image read");

  // convert to ROS message
  cv_bridge::CvImage cv_image;
  cv_image.image = image;
  cv_image.encoding = "bgr8";
  sensor_msgs::ImagePtr from_cv = cv_image.toImageMsg();

  // copy to passed in image reference
  the_image.header.stamp = pic_time;
  the_image.header.frame_id = "nikon";
  the_image.header.seq = image_seq; 
  the_image.height = from_cv->height;
  the_image.width = from_cv->width;
  the_image.encoding = from_cv->encoding;
  the_image.is_bigendian = from_cv->is_bigendian;
  the_image.step = from_cv->step;
  the_image.data = from_cv->data;

  // copy camera_info to the passed in camera_info
  the_camera_info.header.stamp = pic_time;
  the_camera_info.header.frame_id = "nikon";
  the_camera_info.header.seq = image_seq; 
  the_camera_info.height = camera_info.height;
  the_camera_info.width = camera_info.width;
  the_camera_info.distortion_model = camera_info.distortion_model;
  the_camera_info.D = camera_info.D;
  the_camera_info.K = camera_info.K;
  the_camera_info.R = camera_info.R;
  the_camera_info.P = camera_info.P;

  // set the response
  response.success = TRUE;
  response.status_message = "Image captured as " + local_filename;
  response.stamp = pic_time;

  /*
    cv::namedWindow("Display window", CV_WINDOW_NORMAL);
    cv::imshow("Display window", image);
    cv::waitKey(0);   
  */
}


int main(int argc, char **argv) 
{

  // Start ROS node 
  ros::init(argc, argv, name);
  ros::NodeHandle nh("~");

  // bring up camera via gphoto
  context = gp_context_new();
  gp_context_set_error_func (context, ctx_error_func, NULL);
  gp_context_set_status_func (context, ctx_status_func, NULL);
  gp_camera_new(&camera);
  int retval = gp_camera_init(camera, context);
  if(retval != GP_OK) {
    ROS_ERROR("Sorry gp_camera_init returns %d\n", retval);
    exit(1);
  }
  printf("Camera init complete\n");

  // load calibration
  camera_info_manager::CameraInfoManager camera_info_manager(nh, name);
  camera_info_manager.loadCameraInfo("package://camera_driver/calibrations/${NAME}.yaml");
  camera_info = camera_info_manager.getCameraInfo();
  ROS_INFO("camera_info (x, y): (%d, %d)", camera_info.width, camera_info.height);


  // Polled camera service
  polled_camera::PublicationServer request_image_server = 
    polled_camera::advertise(nh, "request_image", capture_callback);

  // lurk
  ros::spin();  
  // and clean up
  gp_camera_exit(camera, context);
}
