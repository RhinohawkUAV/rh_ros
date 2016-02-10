# Image Processing Examples

The nodes in this package provide a simple example of an image processing task
Each node subscribes to an image topic (which is likely produced from a camera
drivere), performs an image processing task (canny edge detector or 
thresholding), and then publishes the resulting image.

## Prerequisits

For these examples to work an image publisher must be running to produce 
the image. The resuling image can be viewed using the built-in `image_view`
package.

