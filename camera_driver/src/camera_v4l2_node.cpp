/*****************************************************************************/
/** ROS Camera Driver                                                       **/
/**                                                                         **/
/*****************************************************************************/

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "image_transport/publisher_plugin.h"
#include <pluginlib/class_loader.h>
#include <sensor_msgs/image_encodings.h>

/* Must be included last because it #defines standard
 * types, which breaks the Boost library
 */
#include <linux/ipu.h> 

/* Adapted from wiki.ros.org/image_transport/Tutorials/PublishingImages */

typedef struct my_buffer_tag
{
    void * start;
    size_t length;
} my_buffer_t;

my_buffer_t * buffers;
int n_buffers;

my_buffer_t ipu_input_buffer;
my_buffer_t ipu_output_buffer;

static int xioctl(int fd, int request, void *arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

int init_mmap(int fd)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }
    
    n_buffers = req.count;
    buffers = (my_buffer_t *)calloc(req.count, sizeof(*buffers)); /* TODO: this should be the size of the object? */
    assert(buffers != NULL);
    
    for(int i = 0; i < req.count; i++)
    {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
        {
            perror("Querying Buffer");
            return 1;
        }
        
        buffers[i].length = buf.length; /* save for munmap */
        buffers[i].start = mmap (NULL, buf.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, buf.m.offset);
        if(MAP_FAILED == buffers[i].start)
        {
            perror("Mapping capture buffer");
            return 1;
        }
    }
    return 0;
}

int init_ipu(int fd_ipu, struct ipu_task * task)
{
    /* IPU Color converson setup */
    memset(task, 0, sizeof(struct ipu_task));
    
    task->input.width = 720;
    task->input.height = 480;
    task->input.format = v4l2_fourcc('Y', 'U', 'Y', 'V');
    
    task->output.width = 720;
    task->output.height = 480;
    task->output.format = v4l2_fourcc('B', 'G', 'R', '3');
    
    /* Alocate input and output buffers */
    ipu_input_buffer.length = task->input.paddr = \
            task->input.width * task->input.height * 2; /* 2 Bytes per pixel */
    ipu_output_buffer.length = task->output.paddr = \
            task->output.width * task->output.height * 3; /* 3 Bytes per pixel */
            
    if(-1 == xioctl(fd_ipu, IPU_ALLOC, &task->input.paddr))
    {
        perror("Allocate IPU Input Buffer");
        return 1;
    }
    ipu_input_buffer.start = mmap(0, ipu_input_buffer.length, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd_ipu, task->input.paddr);
    if(!ipu_input_buffer.start)
    {
        perror("Mapping Input Buffer");
    }


    if(-1 == xioctl(fd_ipu, IPU_ALLOC, &task->output.paddr))
    {
        perror("Alocate IPU Output Buffer");
        return 1;
    }

    ipu_output_buffer.start = mmap(0, ipu_output_buffer.length, PROT_READ | PROT_WRITE,
        MAP_SHARED, fd_ipu, task->output.paddr);
    if(!ipu_output_buffer.start)
    {
        perror("Mapping Output Buffer");
    }
    
    return 0;
}

int init_capture(int fd)
{
    /* Start video capture */
    
    for(int i = 0; i < n_buffers; i++)
    {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        {
            perror("Query Buffer");
            return 1;
        }
    }
    
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    
    if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return 1;
    }
    
    return 0;
}
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
    image_transport::CameraPublisher pub = it.advertiseCamera("camera/image", 1);
    
    /* Create a publisher to publish the jpeg image */
    typedef image_transport::PublisherPlugin Plugin;
    pluginlib::ClassLoader<Plugin> loader("image_transport", "image_transport::PublisherPlugin");
    boost::shared_ptr<Plugin> compressed_pub(loader.createInstance("image_transport/compressed_pub"));
    compressed_pub->advertise(nh, "camera/image/compressed", 1);
    
    /* Open the v4l2 video device */
    int fd;
    fd = open("/dev/video0", O_RDWR);
    if (fd == -1)
    {
        perror("Opening video device");
        return 1;
    }
    if(init_mmap(fd))
        return 1;
    
    /* Open IPU */
    int fd_ipu;
    fd_ipu = open("/dev/mxc_ipu", O_RDWR, 0);
    if(fd_ipu < 0)
    {
        printf("Failed to open IPU\n");
        return 1;
    }
    
    struct ipu_task task;
    if(init_ipu(fd_ipu, &task))
        return 1;
    
    if(init_capture(fd))
        return 1;
    
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    
    while(ros::ok())
    {
        sensor_msgs::Image msg;
        
        memset(&buf, 0, sizeof(buf));
        
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        do
        {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            struct timeval tv = {0};
            tv.tv_sec = 2;
            int r = select(fd+1, &fds, NULL, NULL, &tv);
            if(-1 == r)
            {
                if(EINTR == errno)
                {
                    continue;
                }
                else
                {
                    perror("Waiting for Frame");
                    return 1;
                }
            }
        }while(0); /* Shold only loop if we explicitly call 'continue' */
        
        if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
        {
            perror("Retrieving Frame");
            return 1;
        }
        
        uint32_t frame_error = (buf.flags & V4L2_BUF_FLAG_ERROR) ? 1: 0;
        
        if(!frame_error)
        {
            memcpy(ipu_input_buffer.start, 
                   buffers[buf.index].start,
                   ipu_input_buffer.length);
        }

        /* Set the buffer to a black image, since the driver is returning
         * incomplete frames
         */
        uint8_t * buf_ptr = (uint8_t *)buffers[buf.index].start;
        while(buf_ptr < (uint8_t *)buffers[buf.index].start + ipu_input_buffer.length)
        {
            buf_ptr[0] = 0x00;
            buf_ptr[1] = 0x80;
            buf_ptr[2] = 0x00;
            buf_ptr[3] = 0x80;
            buf_ptr += 4;
        }
        
        if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        {
            perror("Queue Buffer");
            return 1;
        }
        
        if(!frame_error)
        {
            if(-1 == ioctl(fd_ipu, IPU_QUEUE_TASK, &task))
            {
                perror("IPU_QUEUE_TASK");
            }
            
            sensor_msgs::ImagePtr image(new sensor_msgs::Image);
            
            image->header.seq = buf.sequence;
            image->header.stamp.sec = buf.timestamp.tv_sec;
            image->header.stamp.nsec = buf.timestamp.tv_usec * 1000;
            
            image->height = 480;
            image->width = 720;
            image->encoding = sensor_msgs::image_encodings::BGR8;
            
            image->step = 720 * 3;
            image->data.resize(ipu_output_buffer.length);
            
            memcpy(&image->data[0], ipu_output_buffer.start, ipu_output_buffer.length);
            
            sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo);
            ci->height = image->height;
            ci->width = image->width;
            ci->header.stamp = image->header.stamp;
            
            pub.publish(image, ci);
            compressed_pub->publish(image);
        }
    }
    return 0;
}
