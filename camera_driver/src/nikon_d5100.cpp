/*****************************************************************************/
/** ROS Camera Driver for Nikon D5100                                       **/
/**                                                                         **/
/*****************************************************************************/

#include <fcntl.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <gphoto2/gphoto2.h>

using namespace std;

static void
capture_to_file(Camera *camera, GPContext *context, char *fn) {
  int fd, retval;
  CameraFile *file;
  CameraFilePath camera_file_path;

  printf("Capturing.\n");

  /* NOP: This gets overridden in the library to /capt0000.jpg */
  strcpy(camera_file_path.folder, "/");
  strcpy(camera_file_path.name, "foo.jpg");

  retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
  printf("  Retval: %d\n", retval);

  printf("Pathname on the camera: %s/%s\n", camera_file_path.folder, camera_file_path.name);

  fd = open(fn, O_CREAT | O_WRONLY, 0644);
  retval = gp_file_new_from_fd(&file, fd);
  printf("  Retval: %d\n", retval);
  retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name,
			      GP_FILE_TYPE_NORMAL, file, context);
  printf("  Retval: %d\n", retval);

  printf("Deleting.\n");
  retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name,
				 context);
  printf("  Retval: %d\n", retval);

  gp_file_free(file);
}

static void 
capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size) {
  int retval;
  CameraFile *file;
  CameraFilePath camera_file_path;

  printf("Capturing.\n");

  /* NOP: This gets overridden in the library to /capt0000.jpg */
  strcpy(camera_file_path.folder, "/");
  strcpy(camera_file_path.name, "foo.jpg");

  retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
  printf("  Retval: %d\n", retval);

  printf("Pathname on the camera: %s/%s\n", camera_file_path.folder, camera_file_path.name);

  retval = gp_file_new(&file);
  printf("  Retval: %d\n", retval);
  retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name,
			      GP_FILE_TYPE_NORMAL, file, context);
  printf("  Retval: %d\n", retval);

  gp_file_get_data_and_size (file, ptr, size);

  printf("Deleting.\n");
  retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name,
				 context);
  printf("  Retval: %d\n", retval);
  /*gp_file_free(file); */
}


static void
ctx_error_func (GPContext *context, const char *str, void *data)
{
  fprintf  (stderr, "\n*** Contexterror ***              \n%s\n",str);
  fflush   (stderr);
}

static void
ctx_status_func (GPContext *context, const char *str, void *data)
{
  fprintf  (stderr, "%s\n", str);
  fflush   (stderr);
}


int main(int argc, char **argv) {

  Camera  *camera;
  int retval;
  GPContext *context = gp_context_new();
  char*data;
  unsigned long size;

  gp_context_set_error_func (context, ctx_error_func, NULL);
  gp_context_set_status_func (context, ctx_status_func, NULL);
  gp_camera_new(&camera);
  printf("camera init\n");
  retval = gp_camera_init(camera, context);
  if(retval != GP_OK) {
    printf("Sorry gp_camera_init returns %d\n", retval);
    exit(1);
  }
  printf("camera init worked\n");
  capture_to_file(camera, context, (char *) "/tmp/junk.jpg");
  capture_to_memory(camera, context, (const char**)&data, &size);
  printf("image size %lu\n", size);
  gp_camera_exit(camera, context);
}
