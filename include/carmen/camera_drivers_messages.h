#ifndef CAMERA_DRIVERS_MESSAGES_H
#define CAMERA_DRIVERS_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct {
    int x;
    int y;
    int w;
    int h;
} roi;


typedef struct {
    int x1;          // Top left
    int y1;          // Top left
    int x2;          // Bottom right
    int y2;          // Bottom right
    int confidence;
    int class_id;
    int track_id;
} bbox;


// typedef struct {
//     double fx;
//     double fy;
//     double cu;
//     double cv;
//     double pixel_size;
//     unsigned int is_rectified;
//     double fov_horizontal;
//     double fov_vertical;
// } camera_parameters;


typedef struct {
    int image_size;                    // width * height * number_of_channels * size_in_bytes_of_each_element (MUST BE SET OR IPC WILL BREAK)
    int width;                         // The x dimension of the image in pixels
    int height;                        // The y dimension of the image in pixels
    int number_of_channels;            // The number of channels of the image
    int size_in_bytes_of_each_element; // 1 for char, 4 for int, 8 for double...
    int data_type;                     // String that define the data type of each pixel ex: unsigned_char, short, float...
    void *raw_data;
} camera_image;


typedef struct {
    int number_of_images;
    camera_image *images;
    double timestamp;
    char *host;
} camera_message;


#define      CAMERA_NAME       "camera"
#define      CAMERA_FMT        "{int, <{int, int, int, int, int, int, <byte:1>}:1>, double, string}"

#define      CAMERA1_NAME         "camera1"
#define      CAMERA2_NAME         "camera2"
#define      CAMERA3_NAME         "camera3"
#define      CAMERA4_NAME         "camera4"
#define      CAMERA5_NAME         "camera5"
#define      CAMERA6_NAME         "camera6"
#define      CAMERA7_NAME         "camera7"
#define      CAMERA8_NAME         "camera8"
#define      CAMERA9_NAME         "camera9"
#define      CAMERA10_NAME        "camera10"
#define      CAMERA11_NAME        "camera11"
#define      CAMERA12_NAME        "camera12"
#define      CAMERA13_NAME        "camera13"
#define      CAMERA14_NAME        "camera14"
#define      CAMERA15_NAME        "camera15"
#define      CAMERA16_NAME        "camera16"
#define      CAMERA17_NAME        "camera17"
#define      CAMERA18_NAME        "camera18"
#define      CAMERA19_NAME        "camera19"
#define      CAMERA20_NAME        "camera20"

#define MAX_CAMERA_ID 20

#ifdef __cplusplus
}
#endif

#endif