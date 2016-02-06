#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <linux/ipu.h>

#include <assert.h>

typedef struct my_buffer_tag
{
    void * start;
    size_t length;
} my_buffer_t;

my_buffer_t * buffers;
int n_buffers;

static int xioctl(int fd, int request, void *arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

int print_caps(int fd)
{
    struct v4l2_capability caps = {};
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
    {
        perror("Querying Capabilities");
        return 1;
    }
    
    printf( "Driver Caps:\n"
            " Driver: \"%s\"\n"
            " Card: \"%s\"\n"
            " Bus: \"%s\"\n"
            " Version: %d.%d\n"
            " Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if(-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
    {
        perror("Querying Image Format");
        return 1;
    }
    
    char fourcc2[5] = {0};
    strncpy(fourcc2, (char *)&(fmt.fmt.pix.pixelformat), 4);
    
    printf("\n");
    printf("Image Format:\n"
           " Width: %d\n"
           " Height: %d\n"
           " Pixel Format: %s\n"
           " Interlace Field: %d\n"
           " Bytes per Line: %d\n"
           " Image Size: %d\n"
           " Color Space: %d\n"
           " Priv: %d\n",
           fmt.fmt.pix.width, fmt.fmt.pix.height, fourcc2, (int)fmt.fmt.pix.field,
           fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage, (int)fmt.fmt.pix.colorspace,
           fmt.fmt.pix.priv);
    printf("\n");

/*    struct v4l2_cropcap cropcap = {0};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
    {
        perror("Querying Cropping Capabilities");
        return 1;
    }
    printf( "Camera Cropping:\n"
            " Bounds: %dx%d+%d+%d\n"
            " Default: %dx%d+%d+%d\n"
            " Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);
*/
    int support_grbg10 = 0;

    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf(" FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
    {
        strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
        if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
            support_grbg10 = 1;
        c = fmtdesc.flags & 1? 'C' : ' ';
        e = fmtdesc.flags & 2? 'E' : ' ';
        printf(" %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
    }
    
    return 0;
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
    
    printf("Got %d capture buffers\n", req.count);
    n_buffers = req.count;
    buffers = (my_buffer_t *)calloc(req.count, sizeof(*buffers));
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
        printf("Length: %d\nAddress: %p\n", buf.length, buffers[i]);
        printf("Image Length: %d\n", buf.bytesused);
    }
    return 0;
}


int capture_image(int fd)
{

    
    /* IPU Color converson setup */
    struct ipu_task task;
    memset(&task, 0, sizeof(task));
    
    int fd_ipu;
    int input_size;
    int output_size;
    
    void * input_buffer = NULL;
    void * output_buffer = NULL;
    
    task.input.width = 720;
    task.input.height = 480;
    task.input.format = v4l2_fourcc('Y', 'U', 'Y', 'V');
    
    task.output.width = 720;
    task.output.height = 480;
    task.output.format = v4l2_fourcc('B', 'G', 'R', '3');
    
    /* Open IPU */
    fd_ipu = open("/dev/mxc_ipu", O_RDWR, 0);
    if(fd_ipu < 0)
    {
        printf("Failed to open IPU\n");
        return 1;
    }
    
    /* Alocate input and output buffers */
    input_size = task.input.paddr = task.input.width * task.input.height 
            * 2; /* 2 Bytes per pixel */
    output_size = task.output.paddr = task.output.width * task.output.height
            * 3; /* 3 Bytes per pixel */
    
    if(-1 == xioctl(fd_ipu, IPU_ALLOC, &task.input.paddr))
    {
        perror("Allocate IPU Input Buffer");
        return 1;
    }
    input_buffer = mmap(0, input_size, PROT_READ | PROT_WRITE,
                        MAP_SHARED, fd_ipu, task.input.paddr);
    if(!input_buffer)
    {
        perror("Mapping Input Buffer");
    }

    
    if(-1 == xioctl(fd_ipu, IPU_ALLOC, &task.output.paddr))
    {
        perror("Alocate IPU Output Buffer");
        return 1;
    }
    
    output_buffer = mmap(0, output_size, PROT_READ | PROT_WRITE,
        MAP_SHARED, fd_ipu, task.output.paddr);
    if(!output_buffer)
    {
        perror("Mapping Output Buffer");
    }
    
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
    
    cv::namedWindow("window",CV_WINDOW_AUTOSIZE);
    int frameCt = 0;
    do
    {
        memset(&buf, 0, sizeof(buf));
        
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        struct timeval begin, end;
        int sec, usec, run_time;
        
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
        }while(0);
    
        if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
        {
            perror("Retrieving Frame");
            return 1;
        }
    
        /*
        printf("\n");
        printf("Buffer Data\n"
               " Index: %d\n"
               " Type: %d\n"
               " Bytes Used: %d\n"
               " Flags: %d\n"
               " Field: %d\n"
               " Timestamp: %ld:%ld\n"
               " sequence: %d\n"
               " Memory: %d\n"
               " Offset: %d\n"
               " Length: %d\n",
               buf.index, buf.type, buf.bytesused, buf.flags, buf.field,
               buf.timestamp.tv_sec, buf.timestamp.tv_usec, buf.sequence,
               buf.memory, buf.m.offset, buf.length);
        printf("\n");
        */
//        printf ("saving image\n");
        
        
        uint32_t frame_error = (buf.flags & V4L2_BUF_FLAG_ERROR) ? 1: 0;
/*        printf("Timestamp: %ld:%ld, Sequence: %d, Buffer: %d, Error: %d\n",
            buf.timestamp.tv_sec, buf.timestamp.tv_usec, buf.sequence,
            buf.index, frame_error);
*/        
        gettimeofday(&begin, NULL);
        
/*        
        cv::Mat img(480, 720, CV_8UC3);
        uint8_t * imgData = img.data;
        uint8_t * bufPtr = buffer;
      
        while(bufPtr < buffer + 691200)
        {
            imgData[0] = bufPtr[0];
            imgData[1] = bufPtr[3];
            imgData[2] = bufPtr[1];
            imgData[3] = bufPtr[2];
            imgData[4] = bufPtr[3];
            imgData[5] = bufPtr[1];
        
        
            imgData += 6;
            bufPtr += 4;
        }
*/
//        if(!frame_error)
        {
//            memcpy(input_buffer, buffers[buf.index].start, input_size);
        }
        
        /* Set the buffer to a black image, since the driver is returning
         * incomplete frames
         */
        uint8_t * buf_ptr = (uint8_t *)buffers[buf.index].start;
        while(buf_ptr < (uint8_t *)buffers[buf.index].start + input_size)
        {
            buf_ptr[0] = 0x00;
            buf_ptr[1] = 0x80;
            buf_ptr[2] = 0x00;
            buf_ptr[3] = 0x80;
            buf_ptr += 4;
        }
//        memset(buffers[buf.index].start, 0, input_size); makes green
        
        if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
        {
            perror("Queue Buffer");
            return 1;
        }
        
        //if(!frame_error)
        {
            task.input.paddr = (int)buffers[buf.index].start;
            gettimeofday(&begin, NULL);
            if(-1 == ioctl(fd_ipu, IPU_QUEUE_TASK, &task))
            {
                perror("IPU_QUEUE_TASK");
            }
        
            gettimeofday(&end, NULL);
        
            cv::Mat img(480, 720, CV_8UC3);
            memcpy(img.data, output_buffer, output_size);
        
        
            gettimeofday(&end, NULL);
            //cv::imshow("window", img);
/*            sec = end.tv_sec - begin.tv_sec;
            usec = end.tv_usec - begin.tv_usec;
            if (usec < 0) {
            sec--;
            usec = usec + 1000000;
            }
            run_time = (sec * 1000000) + usec;
            printf("Conversion Time: %dusec\n", run_time);
*/    
            //cv::cvtColor(img, img, CV_YCrCb2BGR);
            cv::imshow("window", img);
            char fname[20];
            //sprintf(fname, "img%d.jpg", frameCt);
            //cv::imwrite(fname, img);
            frameCt++;
        }
    }while(-1 == cvWaitKey(1));
    
    //cv::imwrite("image.jpg", img);
    return 0;
}


int main()
{
    int fd;

    fd = open("/dev/video0", O_RDWR);
    if (fd == -1)
    {
        perror("Opening video device");
        return 1;
    }
    if(print_caps(fd))
        return 1;
        
    if(init_mmap(fd))
        return 1;
    int i;
//    for(i=0; i<5; i++)
    {
        if(capture_image(fd))
            return 1;
    }
    close(fd);
    return 0;
}