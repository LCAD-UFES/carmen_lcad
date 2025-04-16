#include "camera_drivers.h"
#include "camera_drivers_process_image.hpp"

int image_index_to_show = 0;
double resize_factor = 1.0;
int camera_id = -1;
char *camera_name = NULL;;
int x_crop = -1, y_crop = -1, w_crop = -1, h_crop = -1;

void 
overlay_image(Mat* src, Mat* overlay, const Point& location)
{
    for (int y = max(location.y, 0); y < src->rows; ++y)
    {
        int fY = y - location.y;

        if (fY >= overlay->rows)
            break;

        for (int x = max(location.x, 0); x < src->cols; ++x)
        {
            int fX = x - location.x;

            if (fX >= overlay->cols)
                break;

            double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

            for (int c = 0; opacity > 0 && c < src->channels(); ++c)
            {
                unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
				src->data[y * src->step + src->channels() * x + c] = overlayPx;
            }
        }
    }
}


double
filter_fps(double timestamp)
{
    static list <double> timestamp_list;
    static unsigned int max_list_size = 5;
    double sum = 0.0;
    double cont = 0.0;

    if (timestamp_list.size() > max_list_size)
        timestamp_list.pop_front();

    timestamp_list.push_back(timestamp);

    list <double>::iterator it = timestamp_list.begin();
    list <double>::iterator next_it = timestamp_list.begin();
    next_it++;

    while(1)
    {
        if (next_it == timestamp_list.end())
            break;

        sum += *next_it - *it;
        it++;
        next_it++;
        cont += 1.0;
    }
    double fps = 1 / (sum / cont);

    if (round(fps) > max_list_size)
        max_list_size = round(fps);

    return (fps);
}


void
check_image_index(camera_message *msg)
{
    if (image_index_to_show > (msg->number_of_images - 1))
    {
        printf("\n---------------------------------------------\n The image index exceeds the maximun %d! \n \n Setting the image index to 0! \n---------------------------------------------\n\n", msg->number_of_images - 1);
    }
    image_index_to_show = 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers																					 //
///////////////////////////////////////////////////////////////////////////////////////////////

int cont = 0;
void
image_handler(camera_message *msg)
{
    // DO NOT delete, Useful for debug
    // printf ("CAMERA_MESSAGE NI%d IS%d %dx%d C%d DT%d SeE%d %lf %s\n", msg->number_of_images, msg->images[0].image_size, msg->images[0].width, msg->images[0].height, msg->images[0].number_of_channels, msg->images[0].size_in_bytes_of_each_element, msg->images[0].data_type, msg->timestamp,msg->host);
    
    static bool first_time = true;
    char info[25];
    char window_name[25];

    if (first_time)
    {
        check_image_index(msg);
        first_time = false;
    }

    Mat cv_image; // = Mat(msg->images[0].height, msg->images[0].width, CV_8UC3, msg->images[0].raw_data, 0);
    process_image(msg, image_index_to_show, camera_name, !msg->undistorted, cv_image);

    sprintf(info, "%dx%d", msg->images[image_index_to_show].width, msg->images[image_index_to_show].height);
    putText(cv_image, info, Point(10, 25), FONT_HERSHEY_SIMPLEX, .7, cvScalar(0, 0, 0), 4);
    putText(cv_image, info, Point(10, 25), FONT_HERSHEY_SIMPLEX, .7, cvScalar(255, 255, 255), 2);

    sprintf(info, "%.2fFPS", filter_fps(msg->timestamp));
    putText(cv_image, info, Point(10, 55), FONT_HERSHEY_SIMPLEX, .7, cvScalar(0, 0, 0), 4);
    putText(cv_image, info, Point(10, 55), FONT_HERSHEY_SIMPLEX, .7, cvScalar(255, 255, 255), 2);

    if (x_crop != -1 && y_crop != -1 && w_crop != -1 && h_crop != -1)
    {
        Rect myROI(x_crop, y_crop, w_crop, h_crop);
	    cv_image = cv_image(myROI);
    }

    if (resize_factor != 1.0)
        resize(cv_image, cv_image, Size(cv_image.cols * resize_factor, cv_image.rows * resize_factor));

    sprintf(window_name, "Camera %d Viewer", camera_id);

    if (msg->undistorted == 2) // retrocompatibilidade
        cvtColor(cv_image, cv_image, CV_RGB2BGR);

    imshow(window_name, cv_image);
    waitKey(1);
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT)
	{
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Signal %d received, exiting program ...\n", signo);
        exit(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations																		     //
///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    camera_drivers_subscribe_message(camera_id, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
read_parameters(int argc, char **argv)
{
    carmen_param_allow_unfound_variables(1);
	carmen_param_t optional_commandline_param_list[] =
	{
		{(char *) "commandline", (char *) "image",  CARMEN_PARAM_INT,    &image_index_to_show, 0, NULL},
		{(char *) "commandline", (char *) "resize", CARMEN_PARAM_DOUBLE, &resize_factor,       0, NULL},
        {(char *) "commandline", (char *) "crop_x", CARMEN_PARAM_INT,    &x_crop,              0, NULL},
        {(char *) "commandline", (char *) "crop_y", CARMEN_PARAM_INT,    &y_crop,              0, NULL},
        {(char *) "commandline", (char *) "crop_w", CARMEN_PARAM_INT,    &w_crop,              0, NULL},
        {(char *) "commandline", (char *) "crop_h", CARMEN_PARAM_INT,    &h_crop,              0, NULL},
	};
	carmen_param_install_params(argc, argv, optional_commandline_param_list, sizeof(optional_commandline_param_list) / sizeof(optional_commandline_param_list[0]));
}


void
check_parameters(int argc, char **argv)
{
	if (argc < 2)
	{
		carmen_die("USAGE: %s <optional camera_name> <camera_id>\n\n", argv[0]);
		exit (0);
	}
}


int
main(int argc, char **argv)
{
    check_parameters(argc, argv);

    carmen_ipc_initialize(argc, argv);

    if (argc > 2)
    {
        camera_name = (char*) malloc(sizeof(char)*strlen(argv[1]));
        strcpy(camera_name, argv[1]);
        camera_id = atoi(argv[2]);
    }
    else
        camera_id = atoi(argv[1]);

    subscribe_messages();
    
    signal(SIGINT, shutdown_module);
	
    setlocale(LC_ALL, "C");

    read_parameters(argc, argv);
	
    carmen_ipc_dispatch();
}
