 /*********************************************************
  ---   Skeleton Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>


static carmen_bumblebee_basic_stereoimage_message msg;

static int bumblebee_basic_width;
static int bumblebee_basic_height;


void
carmen_bumblebee_publish_stereoimage_message(unsigned char *rawLeft, unsigned char *rawRight, int width, int height, int bytes_per_pixel, int isRectified, int camera)
{
  msg.width = width;
  msg.height = height;
  msg.image_size = width * height * bytes_per_pixel;
  msg.isRectified = isRectified;
  msg.raw_left = rawLeft;
  msg.raw_right = rawRight;
  msg.timestamp = carmen_get_time();
  msg.host = carmen_get_host();

  carmen_bumblebee_basic_publish_message(camera, &msg);
}


void
copy_raw_image(unsigned char *raw_image_copy, unsigned char *raw_image)
{
  int x, y, r, g, b;

  for (y = 0; y < bumblebee_basic_height; y++)
  {
    for (x = 0; x < bumblebee_basic_width; x++)
    {
      r = raw_image[3 * (y * bumblebee_basic_width + x) + 0];
      g = raw_image[3 * (y * bumblebee_basic_width + x) + 1];
      b = raw_image[3 * (y * bumblebee_basic_width + x) + 2];
      raw_image_copy[3 * (y * bumblebee_basic_width + x) + 0] = r;
      raw_image_copy[3 * (y * bumblebee_basic_width + x) + 1] = g;
      raw_image_copy[3 * (y * bumblebee_basic_width + x) + 2] = b;
    }
  }
}

void
shift_image_center(unsigned char *raw_image, unsigned char *raw_image_copy, int fraction_of_size, int shift_amount)
{
  int x, y;

  copy_raw_image(raw_image_copy, raw_image);

  for (y = bumblebee_basic_height / (2 * fraction_of_size);
       y < bumblebee_basic_height - bumblebee_basic_height / (2 * fraction_of_size); y++)
  {
    for (x = bumblebee_basic_width / (2 * fraction_of_size);
         x < bumblebee_basic_width - bumblebee_basic_width / (2 * fraction_of_size); x++)
    {
      raw_image[3 * (y * bumblebee_basic_width + x) + 0] = raw_image_copy[3 * (y * bumblebee_basic_width + x + shift_amount) + 0];
      raw_image[3 * (y * bumblebee_basic_width + x) + 1] = raw_image_copy[3 * (y * bumblebee_basic_width + x + shift_amount) + 1];;
      raw_image[3 * (y * bumblebee_basic_width + x) + 2] = raw_image_copy[3 * (y * bumblebee_basic_width + x + shift_amount) + 2];;
    }
  }
}


void
compute_simulated_image(unsigned char *raw_left_image, unsigned char *raw_right_image, unsigned char *raw_image_copy)
{
  int x, y, r, g, b;

  for (y = 0; y < bumblebee_basic_height; y++)
  {
    for (x = 0; x < bumblebee_basic_width; x++)
    {
      r = carmen_uniform_random(0, 255);
      g = carmen_uniform_random(0, 255);
      b = carmen_uniform_random(0, 255);
      raw_left_image[3 * (y * bumblebee_basic_width + x) + 0] = r;
      raw_left_image[3 * (y * bumblebee_basic_width + x) + 1] = g;
      raw_left_image[3 * (y * bumblebee_basic_width + x) + 2] = b;

      raw_right_image[3 * (y * bumblebee_basic_width + x) + 0] = r;
      raw_right_image[3 * (y * bumblebee_basic_width + x) + 1] = g;
      raw_right_image[3 * (y * bumblebee_basic_width + x) + 2] = b;
    }
  }
  shift_image_center(raw_left_image, raw_image_copy, 2, -4);
  shift_image_center(raw_right_image, raw_image_copy, 2, 4);
}


void
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
     carmen_ipc_disconnect();
     printf("bumblebee_basic_simulator was disconnected.\n");
     exit(0);
  }
}


static int
read_parameters(int argc, char **argv, int camera)
{
  int num_items;
  char bumblebee_string[256];

  sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

  carmen_param_t param_list[] = {
    {bumblebee_string, (char*)"width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
    {bumblebee_string, (char*)"height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
    };

  num_items = sizeof(param_list) / sizeof(param_list[0]);
  carmen_param_install_params(argc, argv, param_list, num_items);

  return 0;
}


int
main(int argc, char **argv)
{
  int camera = 0;

  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);

  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  if (argc != 2)
    carmen_die("%s: Wrong number of parameters. %s requires 1 parameter and received %d parameter(s). \nUsage:\n %s <camera_number>\n", argv[0], argv[0], argc-1, argv[0]);

  camera = atoi(argv[1]);

  /* Read application specific parameters (Optional) */
  read_parameters(argc, argv, camera);

  signal(SIGINT, shutdown_module);

  /* Define published messages by your module */
  carmen_bumblebee_basic_define_messages(camera);

  unsigned char *raw_left_image =  (unsigned char *) malloc ((bumblebee_basic_width * bumblebee_basic_height * 3) * sizeof(unsigned char));
  unsigned char *raw_right_image = (unsigned char *) malloc ((bumblebee_basic_width * bumblebee_basic_height * 3) * sizeof(unsigned char));
  unsigned char *raw_image_copy =  (unsigned char *) malloc ((bumblebee_basic_width * bumblebee_basic_height * 3) * sizeof(unsigned char));

  carmen_ipc_sleep(1.0);
  while (1)
  {
    compute_simulated_image(raw_left_image, raw_right_image, raw_image_copy);
    carmen_bumblebee_publish_stereoimage_message(raw_left_image, raw_right_image,
                                                 bumblebee_basic_width, bumblebee_basic_height, 3, 1, camera);
  }

  carmen_ipc_disconnect();
  return 0;
}
