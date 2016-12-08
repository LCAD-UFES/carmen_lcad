/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen_graphics.h>

#include <tf.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/src/Core/util/DisableStupidWarnings.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include <cstdio>
#include <iostream>
#include <vector>
#include <list>
#include <string>

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/localize_ackerman_messages.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/rotation_geometry.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/fused_odometry_interface.h>

// OpenCV
#if CV_MAJOR_VERSION == 2
	#include <opencv2/legacy/legacy.hpp>
#elif CV_MAJOR_VERSION == 3
	#include <opencv/cv.h>
#endif
#include <opencv2/highgui/highgui.hpp>
#include <carmen/fused_odometry_interface.h>

using namespace std;


#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH 640
#define BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT 480
#define BUMBLEBEE_BASIC_VIEW_NUM_COLORS 3

static GtkWidget *horizontal_container;
static GtkWidget *drawing_area_left;
static GdkPixbuf *image_left;
static GtkWidget *drawing_area_right;
static GdkPixbuf *image_right;

static int received_image = 0;
static void redraw(void);

static int bumblebee_basic_width;
static int bumblebee_basic_height;

static unsigned char *data_left = NULL;
static unsigned char *data_right = NULL;

static carmen_bumblebee_basic_stereoimage_message last_message;

static int msg_fps = 0, msg_last_fps = 0; //message fps
static int disp_fps = 0, disp_last_fps = 0; //display fps

//Messages
//static carmen_localize_ackerman_globalpos_message localize_message;
static carmen_fused_odometry_message localize_message;
//static carmen_rddf_annotation_message annotation_message;

//pose images
static carmen_pose_3D_t sensor_board_1_pose;
static carmen_pose_3D_t camera_pose;
static double cu_percent, cv_percent, fx_percent, fy_percent;
static double cu_value, cv_value, f_meters;

//list
//static vector<carmen_localize_ackerman_globalpos_message> lista_poses;
static vector<carmen_fused_odometry_message> lista_poses;
static vector<carmen_bumblebee_basic_stereoimage_message> lista_imagens;

//mat
static Eigen::Matrix3d mat;

//tf vectors
tf::Transform board_to_camera_pose_g;
tf::Transform car_to_board_pose;
tf::Transform world_to_car_pose_g;
tf::Transformer transformer(false);

//values to draw annotation

struct annotation
{
    double x;
    double y;
    double x1;
    double y1;
    double x2;
    double y2;
};
std::vector<annotation> annotation_point;
std::vector<carmen_rddf_annotation_message> annotations;

void
initialize_tf()
{
    // board pose with respect to the car
    car_to_board_pose.setOrigin(tf::Vector3(sensor_board_1_pose.position.x, sensor_board_1_pose.position.y, sensor_board_1_pose.position.z));
    car_to_board_pose.setRotation(tf::Quaternion(sensor_board_1_pose.orientation.yaw, sensor_board_1_pose.orientation.pitch, sensor_board_1_pose.orientation.roll)); // yaw, pitch, roll
    tf::StampedTransform car_to_board_transform(car_to_board_pose, tf::Time(0), "/car", "/board");
    transformer.setTransform(car_to_board_transform, "car_to_board_transform");

    // camera pose with respect to the board
    board_to_camera_pose_g.setOrigin(tf::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
    board_to_camera_pose_g.setRotation(tf::Quaternion(camera_pose.orientation.yaw, camera_pose.orientation.pitch, camera_pose.orientation.roll)); // yaw, pitch, roll
    tf::StampedTransform board_to_camera_transform(board_to_camera_pose_g, tf::Time(0), "/board", "/camera");
    transformer.setTransform(board_to_camera_transform, "board_to_camera_transform");

}

int
distance(carmen_rddf_annotation_message annotation_msg)
{
    double distance, orientation;
    //Fused Odometry
    distance = sqrt(pow(localize_message.pose.position.x - annotation_msg.annotation_point.x, 2) +
                    pow(localize_message.pose.position.y - annotation_msg.annotation_point.y, 2));
    orientation = fabs((atan2(localize_message.pose.position.y - annotation_msg.annotation_point.y,
                              localize_message.pose.position.x - annotation_msg.annotation_point.x) - M_PI
                       - localize_message.pose.orientation.yaw)) > M_PI_2;
    //Localize               
    //    distance = sqrt(pow(localize_message.globalpos.x - annotation_msg.annotation_point.x, 2) +
    //                    pow(localize_message.globalpos.y - annotation_msg.annotation_point.y, 2));
    //    orientation = fabs((atan2(localize_message.globalpos.y - annotation_msg.annotation_point.y,
    //                              localize_message.globalpos.x - annotation_msg.annotation_point.x) - M_PI
    //                       - localize_message.globalpos.theta)) > M_PI_2;
    if ((distance <= 200.0) && orientation == 0)
    {

        return 1;
    }
    return 0;
}

void
calculate_projection_from_point_3D()
{
    //Fused Odometry
    carmen_pose_3D_t car_pose_g = localize_message.pose;
    //Localize
    //    carmen_pose_3D_t car_pose_g;
    //    car_pose_g.position.x = localize_message.globalpos.x;
    //    car_pose_g.position.y = localize_message.globalpos.y;
    //    car_pose_g.orientation.yaw = localize_message.globalpos.theta;

    tf::StampedTransform tf_transform;

    // initial car pose with respect to the world
    world_to_car_pose_g.setOrigin(tf::Vector3(car_pose_g.position.x, car_pose_g.position.y, 0)); // car_pose_g.position.z));
    world_to_car_pose_g.setRotation(tf::Quaternion(car_pose_g.orientation.yaw, 0.0, 0.0)); // car_pose_g.orientation.pitch, car_pose_g.orientation.roll));
    tf::StampedTransform world_to_car_transform(world_to_car_pose_g, tf::Time(0), "/world", "/car");
    transformer.setTransform(world_to_car_transform, "world_to_car_transform");


    transformer.lookupTransform("/camera", "/world", tf::Time(0), tf_transform);

    annotation_point.clear();

    Eigen::MatrixXd transform(3, 4);
    Eigen::Vector3d image_point, image_point1, image_point2;

    tf::Vector3 position = tf_transform.getOrigin();
    tf::Matrix3x3 rotation = tf::Matrix3x3(tf_transform.getRotation());

    //Rotation
    transform(0, 0) = rotation[0][0];
    transform(0, 1) = rotation[0][1];
    transform(0, 2) = rotation[0][2];
    transform(1, 0) = rotation[1][0];
    transform(1, 1) = rotation[1][1];
    transform(1, 2) = rotation[1][2];
    transform(2, 0) = rotation[2][0];
    transform(2, 1) = rotation[2][1];
    transform(2, 2) = rotation[2][2];
    //Translation
    transform(0, 3) = position[0];
    transform(1, 3) = position[1];
    transform(2, 3) = position[2];

    for (uint i = 0; i < annotations.size(); i++)
    {
//    	printf("type: %d %d\n", annotations.at(i).annotation_type, RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT);
        if ((distance(annotations.at(i)) == 1) && (annotations.at(i).annotation_type == RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT))
        {
            //Central Point
            Eigen::Vector4d point;
            point(0, 0) = annotations.at(i).annotation_point.x;
            point(1, 0) = annotations.at(i).annotation_point.y;
            point(2, 0) = annotations.at(i).annotation_point.z;
            point(3, 0) = 1;

            Eigen::Vector3d point_in_camera_reference = (transform * point);

            double x = -point_in_camera_reference(1, 0);
            double y = -point_in_camera_reference(2, 0);
            double z = point_in_camera_reference(0, 0);

            point_in_camera_reference(0, 0) = x;
            point_in_camera_reference(1, 0) = y;
            point_in_camera_reference(2, 0) = z;

            image_point = mat * point_in_camera_reference;

            x = (image_point(0, 0) / image_point(2, 0));
            y = (image_point(1, 0) / image_point(2, 0));

            //Top Point
            Eigen::Vector3d point_in_camera_reference1 = (transform * point);

            double x1 = -point_in_camera_reference1(1, 0);
            double y1 = -point_in_camera_reference1(2, 0);
            double z1 = point_in_camera_reference1(0, 0);

            point_in_camera_reference1(0, 0) = x1 - .375;
            point_in_camera_reference1(1, 0) = y1 + .75;
            point_in_camera_reference1(2, 0) = z1;

            image_point1 = mat * point_in_camera_reference1;

            x1 = (image_point1(0, 0) / image_point1(2, 0));
            y1 = (image_point1(1, 0) / image_point1(2, 0));

            //Bottom Point            
            Eigen::Vector3d point_in_camera_reference2 = (transform * point);

            double x2 = -point_in_camera_reference2(1, 0);
            double y2 = -point_in_camera_reference2(2, 0);
            double z2 = point_in_camera_reference2(0, 0);

            point_in_camera_reference2(0, 0) = x2 + .375;
            point_in_camera_reference2(1, 0) = y2 - .75;
            point_in_camera_reference2(2, 0) = z2;

            image_point2 = mat * point_in_camera_reference2;

            x2 = (image_point2(0, 0) / image_point2(2, 0));
            y2 = (image_point2(1, 0) / image_point2(2, 0));

            annotation anota;
            anota.x = x;
            anota.y = y;
//            anota.x1 = x1;
//            anota.y1 = y1;
//            anota.x2 = x2;
//            anota.y2 = y2;

//            printf("anota x: %d y: %d x1: %d x2:%d y1: %d y2: %d\n", anota.x, anota.y, anota.x1, anota.x2, anota.y1, anota.y2);

            annotation_point.push_back(anota);
        }
    }
}

void
calculate_near_localize_message(carmen_bumblebee_basic_stereoimage_message *msg)
{
    carmen_fused_odometry_message menor;
    //carmen_localize_ackerman_globalpos_message menor;
    if (!lista_poses.empty())
    {
        menor = lista_poses.at(0);
        for (uint i = 0; i < lista_poses.size(); i++)
        {
            if ((lista_poses.at(i).timestamp - msg->timestamp) < (menor.timestamp - msg->timestamp))
            {

                menor = lista_poses.at(i);
            }
        }
    }
    localize_message = menor;
}

void
calculate_matrix_projection_camera(int width, int height)
{

    double ccd_width = 0.0;

    ccd_width = width * 0.00000375f;

    f_meters = ((fx_percent * width) * ccd_width) / width;
    double focal_pixel = (f_meters / ccd_width) * width;

    cu_value = width * cu_percent;
    cv_value = height * cv_percent;

    mat(0, 0) = focal_pixel;
    mat(0, 1) = 0.0;
    mat(0, 2) = cu_value;
    mat(1, 0) = 0.0;
    mat(1, 1) = focal_pixel;
    mat(1, 2) = cv_value;
    mat(2, 0) = 0.0;
    mat(2, 1) = 0.0;
    mat(2, 2) = 1.0;
}

static void
process_image(carmen_bumblebee_basic_stereoimage_message *msg)
{
    IplImage *src_image_left = NULL;
    IplImage *src_image_right = NULL;
    IplImage *resized_image_left = NULL;
    IplImage *resized_image_right = NULL;

    static char msg_fps_string[256];
    static char disp_fps_string[256];

    int scaled_width, scaled_height;

    scaled_width = floor((double) drawing_area_left->allocation.width / 8.0) * 8.0;
    scaled_height = (double) (scaled_width / (double) msg->width) * msg->height;

    src_image_left = cvCreateImage(cvSize(msg->width, msg->height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);
    src_image_right = cvCreateImage(cvSize(msg->width, msg->height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);

    if (resized_image_right)
    {
        cvReleaseImage(&resized_image_left);
        cvReleaseImage(&resized_image_right);
    }

    resized_image_left = cvCreateImage(cvSize(scaled_width, scaled_height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);
    resized_image_right = cvCreateImage(cvSize(scaled_width, scaled_height), IPL_DEPTH_8U, BUMBLEBEE_BASIC_VIEW_NUM_COLORS);

    src_image_left->imageData = (char*) msg->raw_left;
    src_image_right->imageData = (char*) msg->raw_right;

    //resizing the image
    cvResize(src_image_left, resized_image_left, CV_INTER_CUBIC);
    cvResize(src_image_right, resized_image_right, CV_INTER_CUBIC);

    CvFont font;
    sprintf(msg_fps_string, "MSG_FPS: %d", msg_last_fps);
    sprintf(disp_fps_string, "DISP_FPS: %d", disp_last_fps);
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.7, 0, 1.0, CV_AA);
    cvPutText(resized_image_left, msg_fps_string, cvPoint(10, 30), &font, cvScalar(255, 255, 0, 0));
    cvPutText(resized_image_right, disp_fps_string, cvPoint(10, 30), &font, cvScalar(255, 255, 0, 0));

//    printf("PROCESS_IMAGE\n");

    //Calculating the projection of traffic light
    if (!annotations.empty())
    {
//    	printf("HAS ANNOTATION\n");
        calculate_near_localize_message(msg);
        calculate_matrix_projection_camera(scaled_width, scaled_height);
        calculate_projection_from_point_3D();
        for (uint i = 0; i < annotation_point.size(); i++)
        {
            cvCircle(resized_image_right, cvPoint(annotation_point.at(i).x, annotation_point.at(i).y), 2, cvScalar(255, 0, 0), -1, 8);
//            cvRectangle(resized_image_right, cvPoint(annotation_point.at(i).x1, annotation_point.at(i).y1), cvPoint(annotation_point.at(i).x2, annotation_point.at(i).y2), cvScalar(0, 255, 0), 2);
        }
    }
    //End of calculating the projection of traffic light


    image_left = gdk_pixbuf_new_from_data((guchar *) resized_image_left->imageData, GDK_COLORSPACE_RGB,
                                          FALSE, 8 * sizeof (unsigned char), scaled_width,
                                          scaled_height, scaled_width * BUMBLEBEE_BASIC_VIEW_NUM_COLORS,
                                          NULL, NULL);


    image_right = gdk_pixbuf_new_from_data((guchar *) resized_image_right->imageData, GDK_COLORSPACE_RGB,
                                           FALSE, 8 * sizeof (unsigned char), scaled_width,
                                           scaled_height, scaled_width * BUMBLEBEE_BASIC_VIEW_NUM_COLORS,
                                           NULL, NULL);

    redraw();
    
    cvReleaseImage(&resized_image_left);
    cvReleaseImage(&resized_image_right);
    cvReleaseImage(&src_image_left);
    cvReleaseImage(&src_image_right);
}

static void
image_handler(carmen_bumblebee_basic_stereoimage_message* image_msg)
{
    static double last_timestamp = 0.0;
    static double last_time = 0.0;
    double time_now = carmen_get_time();

    if (!received_image)
    {
        received_image = 1;
        last_timestamp = image_msg->timestamp;
        last_time = time_now;
    }


    if ((image_msg->timestamp - last_timestamp) > 1.0)
    {
        msg_last_fps = msg_fps;
        msg_fps = 0;
        last_timestamp = image_msg->timestamp;
    }
    msg_fps++;

    if ((time_now - last_time) > 1.0)
    {

        disp_last_fps = disp_fps;
        disp_fps = 0;
        last_time = time_now;
    }
    disp_fps++;


    last_message = *image_msg;

    process_image(image_msg);
}

int
has_annotation(carmen_rddf_annotation_message *msg)
{
//	printf("HAS_ANNOTATION TYPE: %d\n", msg->annotation_type);

    for (uint i = 0; i < annotations.size(); i++)
    {
    	if ((annotations.at(i).annotation_point.x == msg->annotation_point.x) &&
            (annotations.at(i).annotation_point.y == msg->annotation_point.y) &&
            (annotations.at(i).annotation_point.z == msg->annotation_point.z) &&
            (annotations.at(i).annotation_code == msg->annotation_code) &&
            (annotations.at(i).annotation_description == msg->annotation_description) &&
            (annotations.at(i).annotation_type == msg->annotation_type))
        {

            return 1;
        }
    }
    return 0;
}

void
annotation_handler(carmen_rddf_annotation_message *msg)
{
//	printf("annotation handler: %s \n", msg->annotation_description);
    if (has_annotation(msg) == 0)
    {
        annotations.push_back(*msg);
    }
}

void
tf_handler(carmen_fused_odometry_message * msg)
//tf_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    if (lista_poses.size() >= 10)
    {
        lista_poses.pop_back();
    }
    lista_poses.insert(lista_poses.begin(), *msg);
}

void
subscribe_messages()
{

    carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) annotation_handler, CARMEN_SUBSCRIBE_LATEST);
    //carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) tf_handler, CARMEN_SUBSCRIBE_LATEST);
    carmen_fused_odometry_subscribe_fused_odometry_message(NULL, (carmen_handler_t) tf_handler, CARMEN_SUBSCRIBE_LATEST);
}

void
save_image(const unsigned char *image, int w, int h, const char *file_name)
{
    FILE *image_file = (FILE *) NULL;
    int i, r, g, b;

    if (!file_name || !image)
        return;

    if ((image_file = fopen(file_name, "w")) == (FILE *) NULL)
        return;

    // Write the image format and comment
    fprintf(image_file, "P3\n# CREATOR: bumblebee save_image ()\n");

    // Write the image dimensions and range
    fprintf(image_file, "%d %d\n%d\n", w, h, 255);

    for (i = 0; i < h * w; i++)
    {

        r = image[3 * i + 0];
        g = image[3 * i + 1];
        b = image[3 * i + 2];
        fprintf(image_file, "%d\n%d\n%d\n", r, g, b);
    }

    // Closes image file
    fclose(image_file);
}

void
compose_output_path(char *dirname, char *filename, char **composed_path)
{

    *composed_path = (char *) malloc((strlen(dirname) + strlen(filename) + 2 /* 1 for '\0' e 1 for the '/' between <dirname>/<filename> */) * sizeof (char));

    sprintf((*composed_path), "%s/%s", dirname, filename);
}

void
compose_filename_from_timestamp(double timestamp, char **filename, char *extension)
{

    *filename = (char*) malloc(256 * sizeof (char));
    sprintf((*filename), "%.25f.%s", timestamp, extension);
}

void
create_stereo_filename_from_timestamp(double timestamp, char **left_img_filename, char **right_img_filename)
{

    std::string l_ppm = "l.ppm";
    std::string r_ppm = "r.ppm";
    compose_filename_from_timestamp(timestamp, left_img_filename, (char*) l_ppm.c_str());
    compose_filename_from_timestamp(timestamp, right_img_filename, (char*) r_ppm.c_str());
}

static void
save_camera_images()
{

    std::string aux = "./";
    char *left_filename, *right_filename;
    char *left_filepath, *right_filepath;
    create_stereo_filename_from_timestamp(last_message.timestamp, &left_filename, &right_filename);
    compose_output_path((char*) aux.c_str(), left_filename, &left_filepath);
    compose_output_path((char*) aux.c_str(), right_filename, &right_filepath);
    save_image(last_message.raw_left, last_message.width, last_message.height, left_filepath);
    save_image(last_message.raw_right, last_message.width, last_message.height, right_filepath);
}

static void
shutdown_camera_view(int x)
{
    data_left = NULL;
    data_right = NULL;
    free(data_left);
    free(data_right);

    if (x == SIGINT)
    {
        carmen_ipc_disconnect();
        printf("Disconnected from robot.\n");
        exit(1);
    }
}

static gint
updateIPC(gpointer *data __attribute__((unused)))
{
    carmen_ipc_sleep(0.01);
    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);

    return 1;
}

static gint
expose_event(GtkWidget *widget __attribute__((unused)),
             GdkEventExpose *event __attribute__((unused)))
{
    if (received_image)
        process_image(&last_message);

    return 1;
}

static gint
key_press_event(GtkWidget *widget __attribute__((unused)),
                GdkEventKey *key)
{
    if (toupper(key->keyval) == 'C' && (key->state & GDK_CONTROL_MASK))
        shutdown_camera_view(SIGINT);

    if (toupper(key->keyval) == 'Q' && (key->state & GDK_CONTROL_MASK))
        shutdown_camera_view(SIGINT);

    if (toupper(key->keyval) == 'S' && (key->state & GDK_CONTROL_MASK))
        save_camera_images();

    if (key->state || key->keyval > 255)

        return 1;

    return 1;
}

static gint
key_release_event(GtkWidget *widget __attribute__((unused)),
                  GdkEventButton *key __attribute__((unused)))
{

    return 1;
}

static void
redraw(void)
{
    /* Make sure data structures are all the right size. */

    if (!received_image)

        return;

    int width, height;

    width = gdk_pixbuf_get_width(image_left);
    height = gdk_pixbuf_get_height(image_left);

    gdk_draw_pixbuf(drawing_area_left->window,
                    drawing_area_left->style->fg_gc[GTK_WIDGET_STATE(drawing_area_left)],
                    image_left, 0, 0, 0, 0,
                    width,
                    height,
                    GDK_RGB_DITHER_NONE, 0, 0);

    gdk_draw_pixbuf(drawing_area_right->window,
                    drawing_area_right->style->fg_gc[GTK_WIDGET_STATE(drawing_area_right)],
                    image_right, 0, 0, 0, 0,
                    width,
                    height,
                    GDK_RGB_DITHER_NONE, 0, 0);
}

GdkPixbuf *
create_pixbuf(const gchar * filename)
{
    GdkPixbuf *pixbuf;
    GError *error = NULL;
    pixbuf = gdk_pixbuf_new_from_file(filename, &error);
    if (!pixbuf)
    {

        fprintf(stderr, "%s\n", error->message);
        g_error_free(error);
    }

    return pixbuf;
}

static void
start_graphics(int argc, char *argv[])
{

    GtkWidget *main_window;
    char window_name[100];

    gtk_init(&argc, &argv);

    main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    sprintf(window_name, "Bumblebee Camera %s", argv[1]);
    gtk_window_set_title(GTK_WINDOW(main_window), window_name);
    std::string aux = getenv("CARMEN_HOME");
    aux.append("/data/gui/camera.png");
    gtk_window_set_icon(GTK_WINDOW(main_window), create_pixbuf(aux.c_str()));

    horizontal_container = gtk_hbox_new(TRUE, 0);
    drawing_area_left = gtk_drawing_area_new();
    drawing_area_right = gtk_drawing_area_new();

    //	if (bumblebee_basic_width <= BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH)
    //	{
    //		gtk_widget_set_usize(drawing_area_left, bumblebee_basic_width, bumblebee_basic_height);
    //		gtk_widget_set_usize(drawing_area_right, bumblebee_basic_width, bumblebee_basic_height);
    //	}
    //	else
    //	{
    //		gtk_widget_set_usize(drawing_area_left, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT);
    //		gtk_widget_set_usize(drawing_area_right, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT);
    //	}

    gtk_widget_set_size_request(horizontal_container, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * 2, BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT);


    gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_left, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(horizontal_container), drawing_area_right, TRUE, TRUE, 0);

    gtk_container_add(GTK_CONTAINER(main_window), horizontal_container);

    gtk_signal_connect(GTK_OBJECT(drawing_area_left), "expose_event",
                       (GtkSignalFunc) expose_event, NULL);
    gtk_signal_connect(GTK_OBJECT(drawing_area_right), "expose_event",
                       (GtkSignalFunc) expose_event, NULL);

    gtk_signal_connect(GTK_OBJECT(main_window), "key_press_event",
                       (GtkSignalFunc) key_press_event, NULL);
    gtk_signal_connect(GTK_OBJECT(main_window), "key_release_event",
                       (GtkSignalFunc) key_release_event, NULL);

    gtk_widget_add_events(drawing_area_left, GDK_EXPOSURE_MASK
                          | GDK_BUTTON_PRESS_MASK
                          | GDK_BUTTON_RELEASE_MASK
                          | GDK_POINTER_MOTION_MASK
                          | GDK_POINTER_MOTION_HINT_MASK
                          | GDK_KEY_PRESS_MASK
                          | GDK_KEY_RELEASE_MASK);

    gtk_widget_add_events(drawing_area_right, GDK_EXPOSURE_MASK
                          | GDK_BUTTON_PRESS_MASK
                          | GDK_BUTTON_RELEASE_MASK
                          | GDK_POINTER_MOTION_MASK
                          | GDK_POINTER_MOTION_HINT_MASK
                          | GDK_KEY_PRESS_MASK
                          | GDK_KEY_RELEASE_MASK);

    carmen_graphics_update_ipc_callbacks((GdkInputFunction) updateIPC);
    gtk_widget_show(drawing_area_left);
    gtk_widget_show(drawing_area_right);
    gtk_widget_show(horizontal_container);
    gtk_widget_show(main_window);

    gtk_main();
}

static int
read_parameters(int argc, char **argv, int camera)
{
    int num_items;

    std::string bumblebee_string = "bumblebee_basic";
    std::ostringstream myStream;
    myStream << camera << std::flush;
    bumblebee_string.append(myStream.str());
    std::string camera_string = "camera";
    camera_string.append(myStream.str());

    //  sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera);

    carmen_param_t param_list[] = {
        {(char*) bumblebee_string.c_str(), (char*) "width", CARMEN_PARAM_INT, &bumblebee_basic_width, 0, NULL},
        {(char*) bumblebee_string.c_str(), (char*) "height", CARMEN_PARAM_INT, &bumblebee_basic_height, 0, NULL},
        {(char*) bumblebee_string.c_str(), (char*) "fx", CARMEN_PARAM_DOUBLE, &fx_percent, 0, NULL},
        {(char*) bumblebee_string.c_str(), (char*) "fy", CARMEN_PARAM_DOUBLE, &fy_percent, 0, NULL},
        {(char*) bumblebee_string.c_str(), (char*) "cu", CARMEN_PARAM_DOUBLE, &cu_percent, 0, NULL},
        {(char*) bumblebee_string.c_str(), (char*) "cv", CARMEN_PARAM_DOUBLE, &cv_percent, 0, NULL},

        {(char*) "sensor_board_1", (char*) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x), 0, NULL},
        {(char*) "sensor_board_1", (char*) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y), 0, NULL},
        {(char*) "sensor_board_1", (char*) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z), 0, NULL},
        {(char*) "sensor_board_1", (char*) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll), 0, NULL},
        {(char*) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch), 0, NULL},
        {(char*) "sensor_board_1", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw), 0, NULL},

        {(char*) camera_string.c_str(), (char*) "x", CARMEN_PARAM_DOUBLE, &(camera_pose.position.x), 0, NULL},
        {(char*) camera_string.c_str(), (char*) "y", CARMEN_PARAM_DOUBLE, &(camera_pose.position.y), 0, NULL},
        {(char*) camera_string.c_str(), (char*) "z", CARMEN_PARAM_DOUBLE, &(camera_pose.position.z), 0, NULL},
        {(char*) camera_string.c_str(), (char*) "roll", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.roll), 0, NULL},
        {(char*) camera_string.c_str(), (char*) "pitch", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.pitch), 0, NULL},
        {(char*) camera_string.c_str(), (char*) "yaw", CARMEN_PARAM_DOUBLE, &(camera_pose.orientation.yaw), 0, NULL}
    };

    num_items = sizeof (param_list) / sizeof (param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}

int
main(int argc, char **argv)
{

    int camera = 0;

    if (argc != 2)
    {
        fprintf(stderr, "%s: Wrong number of parameters. stereo requires 1 parameter and received %d. \n Usage: %s <camera_number>", argv[0], argc - 1, argv[0]);
        exit(1);
    }

    camera = atoi(argv[1]);

    carmen_ipc_initialize(argc, argv);

    carmen_param_check_version(argv[0]);

    read_parameters(argc, argv, camera);

    if (bumblebee_basic_width <= BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH)
    {
        data_left = (unsigned char *) calloc(bumblebee_basic_width * bumblebee_basic_height * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
        data_right = (unsigned char *) calloc(bumblebee_basic_width * bumblebee_basic_height * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
    }
    else
    {
        data_left = (unsigned char *) calloc(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
        data_right = (unsigned char *) calloc(BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_WIDTH * BUMBLEBEE_BASIC_VIEW_MAX_WINDOW_HEIGHT * BUMBLEBEE_BASIC_VIEW_NUM_COLORS, sizeof (unsigned char));
    }

    signal(SIGINT, shutdown_camera_view);

    initialize_tf();

    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    subscribe_messages();

    start_graphics(argc, argv);

    return 0;
}
