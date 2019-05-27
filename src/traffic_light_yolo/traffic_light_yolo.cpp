#include <carmen/carmen.h>
#include <carmen/carmen_darknet_interface.hpp>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/velodyne_interface.h>
#include <carmen/velodyne_camera_calibration.h>
#include <carmen/moving_objects_messages.h>
#include <carmen/moving_objects_interface.h>
#include <carmen/traffic_light_interface.h>
#include <carmen/traffic_light_messages.h>
#include <carmen/rddf_messages.h>
#include <carmen/laser_ldmrs_utils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <sys/stat.h>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>

using namespace std;
using namespace cv;
using namespace cv::ml;

const char * CARMEN_HOME = std::getenv("CARMEN_HOME");
int camera;
int camera_side;
char **classes_names;
void *network_struct;
Ptr<ml::RTrees> rf;
carmen_localize_ackerman_globalpos_message *globalpos_msg = NULL;
carmen_velodyne_partial_scan_message *velodyne_msg = NULL;
carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t camera_pose;
carmen_pose_3D_t board_pose;
tf::Transformer transformer;

map<int,Scalar> tl_rgy_colors = {
    {0, Scalar(0, 0, 255)},
    {1, Scalar(0, 255, 0)},
    {2, Scalar(0, 255, 255)},
};
map<int,string> tl_rgy_labels = {
    {0, "red"},
    {1, "green"},
    {2, "yellow"},
};

map<int,Scalar> tl_code_colors = {
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED,    Scalar(255, 0, 0)},
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN,  Scalar(0, 255, 0)},
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW, Scalar(255, 255, 0)},
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF,    Scalar(50, 50, 50)},
};
map<int,char const*> tl_code_names = {
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED,    "RED"},
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN,  "GREEN"},
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW, "YELLOW"},
    {RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF,    "OFF"},
};

vector <carmen_pose_3D_t> annotation_vector;
vector <carmen_pose_3D_t> nearests_traffic_lights_vector;

image_cartesian marker_picked_point;
// carmen_pose_3D_t picked_point;
Point screen_marker = Point(0,0);
bool screen_marker_set = false;
bool marker_point_found = false;

// These control what is displayed on the window. They are variables because it would be super cool to change them according to command line options in the future.
bool DRAW_FPS = false;
bool DRAW_TEXT = true;
bool DRAW_TEXT_BACKGROUND = true;
bool DRAW_BBS = true; // Falso para criar o GT.
bool DRAW_CLOSE_TLS = true; // Falso para criar o GT.
bool DRAW_CIRCLE_THRESHOLD = true;
bool DRAW_LIDAR_POINTS = false;
bool DRAW_FINAL_PREDICTION = true; // Falso para criar o GT.
bool RUN_YOLO = true; // Falso para criar o GT.
bool RUN_RANDOM_FOREST = true; // Falso para criar o GT.
bool COMPUTE_TL_POSES = false;
bool PRINT_FINAL_PREDICTION = true; // Falso para criar o GT.
bool PRINT_GT_PREP = false;
enum class_set {
    RG, // The only classes are Red and Green.
    RGY, // The only classes are Red, Green and Yellow.
    COCO, // All COCO classes.
};
class_set YOLO_CLASS_SET = COCO;
class_set FINAL_CLASS_SET = RGY;

#define TRAFFIC_LIGHT_GROUPING_TRESHOLD 20 // Distance in meters to consider traffic light position
// #define MAX_TRAFFIC_LIGHT_DIST 100
#define MAX_TRAFFIC_LIGHT_DIST 110 // For annotating ground truth.
#define MIN_TRAFFIC_LIGHT_THRESHOLD 10 // 30
#define MAX_TRAFFIC_LIGHT_THRESHOLD 90 // 1000
#define DEFAULT_TRAFFIC_LIGHT_THRESHOLD 60 // pixels
#define TRAFFIC_LIGHT_IMAGE_THRESHOLD 1.5 // meters
#define ORIENTATION_RESTRICTION 60 // degrees
#define CAMERA_HFOV_2 20 // Camera hfov / 2 = 33
#define CONFIDENCE_THRESHOLD 0.2


// #define RESIZED_W 1280
// #define RESIZED_H 960
#define RESIZED_W 640
#define RESIZED_H 480

vector<Mat>
make_box_views(Mat img, vector<bbox_t> predictions)
{
    vector<Mat> views;
    for (bbox_t &pred : predictions)
    {
        // Mat view(img, Rect(pred.x, pred.y, pred.w, pred.h));
        Mat view = img(Rect(pred.x, pred.y, pred.w, pred.h));
        views.push_back(view);
    }
    return (views);
}

void
save_mats_as_images(vector<Mat> matrices, string dest_dir)
{
    string file_path;
    int i = 0;
    for (Mat &m : matrices)
    {
        std::ostringstream oss;
        oss << dest_dir << "/" << i << ".png";
        file_path = oss.str();
        imwrite(file_path, m);
        i++;
    }
}

/**
 * Resize the detection crops and pack them into a single Mat, the way Jean's random forest is expecting.
 * i.e., a single image per row, and BGR values of row after row put together on a single row.
 **/
Mat
pack_images_together(vector<Mat> imgs)
{
    Size new_size(10, 30);
    vector<Mat> single_row_imgs;
    for (int i = 0; i < imgs.size(); i++)
    {
        Mat resized_img(new_size, imgs[0].type());
        resize(imgs[i], resized_img, new_size);
        Mat single_row_img = resized_img.reshape(1,1);
        single_row_imgs.push_back(single_row_img);
    }
    Mat concatenated;
    vconcat(single_row_imgs, concatenated);
    concatenated.convertTo(concatenated, CV_32F);
    return concatenated;
}

// vector<int>
// make_rf_classifications(vector<Mat> imgs) {
//     vector<int> class_predictions(imgs.size())
//     for (size_t i = 0; i < imgs.size(); i++)
//     {
//         rf.predict()
//     }
    
// }

double
compute_distance_to_the_traffic_light()
{
    double nearest_traffic_light_distance = DBL_MAX;
    carmen_pose_3D_t nearest_traffic_light;
    nearests_traffic_lights_vector.clear();

    if (velodyne_msg == NULL || globalpos_msg == NULL)
        return DBL_MAX;

    for (unsigned int i = 0; i < annotation_vector.size(); i++)
    {
            double distance = sqrt(pow(globalpos_msg->globalpos.x - annotation_vector[i].position.x, 2) +
                            pow(globalpos_msg->globalpos.y - annotation_vector[i].position.y, 2));
            // if (distance < nearest_traffic_light_distance)
            if (distance <= MAX_TRAFFIC_LIGHT_DIST && distance < nearest_traffic_light_distance)
            {
                bool orientation_ok = fabs(carmen_radians_to_degrees(globalpos_msg->globalpos.theta - annotation_vector[i].orientation.yaw)) < ORIENTATION_RESTRICTION ? 1 : 0;
                // bool orientation_ok = true;

                // auto dx = globalpos_msg->globalpos.x - annotation_vector[i].position.x;
                // auto dy = (globalpos_msg->globalpos.y - annotation_vector[i].position.y) * -1;
                // auto tl_angle = carmen_normalize_theta(atan2(dy, dx));
                // fprintf(stderr, "tl_angle=%f\n", tl_angle);
                // auto d_angle_tl_iara = fabs(carmen_normalize_theta(tl_angle - M_PI - globalpos_msg->globalpos.theta));
                // fprintf(stderr, "d_angle_tl_iara=%f\n", d_angle_tl_iara);
                // bool in_camera = carmen_radians_to_degrees(d_angle_tl_iara) < CAMERA_HFOV_2;

                bool behind = fabs(carmen_normalize_theta((atan2(globalpos_msg->globalpos.y - annotation_vector[i].position.y,
                                globalpos_msg->globalpos.x - annotation_vector[i].position.x) - M_PI - globalpos_msg->globalpos.theta))) > M_PI_2;

                // if (distance <= MAX_TRAFFIC_LIGHT_DISTANCE && orientation_ok && behind == false && distance < nearest_traffic_light_distance)
                if (distance <= MAX_TRAFFIC_LIGHT_DIST && orientation_ok && behind == false && distance < nearest_traffic_light_distance)
                // if (distance <= MAX_TRAFFIC_LIGHT_DIST && orientation_ok && in_camera && distance < nearest_traffic_light_distance)
                {
                    nearest_traffic_light_distance = distance;
                    nearest_traffic_light = annotation_vector[i];
                }
            }
    }
    nearests_traffic_lights_vector.push_back(nearest_traffic_light);

    if (nearests_traffic_lights_vector.size() > 0)
    {
        for (unsigned int i = 0; i < annotation_vector.size(); i++)
        {
            double distance = sqrt(pow(nearests_traffic_lights_vector[0].position.x - annotation_vector[i].position.x, 2) +
                    pow(nearests_traffic_lights_vector[0].position.y - annotation_vector[i].position.y, 2));
            if (distance < TRAFFIC_LIGHT_GROUPING_TRESHOLD)
            {
                bool orientation_ok = fabs(carmen_radians_to_degrees(globalpos_msg->globalpos.theta - annotation_vector[i].orientation.yaw)) < ORIENTATION_RESTRICTION ? 1 : 0;

                bool behind = fabs(carmen_normalize_theta((atan2(globalpos_msg->globalpos.y - annotation_vector[i].position.y,
                                globalpos_msg->globalpos.x - annotation_vector[i].position.x) - M_PI - globalpos_msg->globalpos.theta))) > M_PI_2;
                if (orientation_ok && behind == false)
                {
                    nearests_traffic_lights_vector.push_back(annotation_vector[i]);
                }
            }
        }
    }

    return (nearest_traffic_light_distance);
}


void
carmen_translte_2d(double *x, double *y, double offset_x, double offset_y)
{
    *x += offset_x;
    *y += offset_y;
}


void
show_LIDAR_points(Mat &image, vector<image_cartesian> all_points, int r, int g, int b, int point_radius=1)
{
    for (unsigned int i = 0; i < all_points.size(); i++)
        circle(image, Point(all_points[i].image_x, all_points[i].image_y), point_radius, cvScalar(b, g, r), -1, 8, 0);
}


void
show_LIDAR(Mat &image, vector<vector<image_cartesian>> points_lists, int r, int g, int b)
{
    for (unsigned int i = 0; i < points_lists.size(); i++)
    {
        for (unsigned int j = 0; j < points_lists[i].size(); j++)
            circle(image, Point(points_lists[i][j].image_x, points_lists[i][j].image_y), 1, cvScalar(b, g, r), 5, 8, 0);
    }
}


void
show_all_points(Mat &image, unsigned int image_width, unsigned int image_height, unsigned int crop_x, unsigned int crop_y, unsigned int crop_width, unsigned int crop_height)
{
    vector<carmen_velodyne_points_in_cam_t> all_points = carmen_velodyne_camera_calibration_lasers_points_in_camera(
                    velodyne_msg, camera_parameters, velodyne_pose, camera_pose, image_width, image_height);

    int max_x = crop_x + crop_width, max_y = crop_y + crop_height;

    for (unsigned int i = 0; i < all_points.size(); i++)
        if (all_points[i].ipx >= crop_x && all_points[i].ipx <= max_x && all_points[i].ipy >= crop_y && all_points[i].ipy <= max_y)
            circle(image, Point(all_points[i].ipx - crop_x, all_points[i].ipy - crop_y), 1, cvScalar(0, 0, 255), 5, 8, 0);
}

/** For debugging. */
string
predictions_to_str(vector<bbox_t> predictions) {
    ostringstream oss;
    oss << "vector<bbox_t> (size=" << predictions.size() << "):" << endl;
    for (size_t i = 0; i < predictions.size(); i++)
    {
        oss << " -"
            << " x=" << predictions[i].x
            << " y=" << predictions[i].y
            << " w=" << predictions[i].w
            << " h=" << predictions[i].h
            << " prob=" << predictions[i].prob
            << " obj_id=" << predictions[i].obj_id
            << " track_id=" << predictions[i].track_id
            << endl;
    }
    return oss.str();
}

void
draw_final_prediction(Mat image, int tl_code)
{
    Size image_size = image.size();
    int margin = 10;
    int circle_radius = 50;
    int circle_x = image_size.width - circle_radius - margin;
    int circle_y = image_size.height - circle_radius - margin;
    Scalar color(203,192,255); // Pink by default.
    if (tl_code_colors.find(tl_code) != tl_code_colors.end())
    {
        color = tl_code_colors[tl_code];
    }

    circle(image, Point(circle_x, circle_y), circle_radius, color, -1);

}

void
display(Mat image, vector<bbox_t> predictions, double fps, vector<image_cartesian> lidar_points)
{
    char object_info[25];
    char frame_rate[25];

    if (DRAW_LIDAR_POINTS)
    {
        show_LIDAR_points(image, lidar_points, 100, 255, 100, 1);
    }

    cvtColor(image, image, COLOR_RGB2BGR);

    if (DRAW_FPS)
    {
        snprintf(frame_rate, 25, "FPS = %.2f", fps);
        putText(image, frame_rate, Point(10, 25), FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);
    }

    if (DRAW_BBS)
    {
        for (unsigned int i = 0; i < predictions.size(); i++)
        {
            Scalar bb_color = Scalar(255,255,255);
            if (tl_rgy_colors.find(predictions[i].obj_id) != tl_rgy_colors.end()) {
                bb_color = tl_rgy_colors[predictions[i].obj_id];
            }
            rectangle(image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)), bb_color, 1);

            if (DRAW_TEXT) {
                string class_name = classes_names[predictions[i].obj_id];
                if ((FINAL_CLASS_SET == RGY) && (tl_rgy_labels.find(predictions[i].obj_id) != tl_rgy_labels.end()))
                {
                    class_name = tl_rgy_labels[predictions[i].obj_id];
                }

                snprintf(object_info, 25, "%d %s %d", predictions[i].obj_id, class_name.c_str(), (int)predictions[i].prob);

                if (DRAW_TEXT_BACKGROUND)
                {
                    //Drawing backgound into text
                    int baseline=0;
                    Size textSize = getTextSize(object_info, FONT_HERSHEY_PLAIN, 1, 1, &baseline);
                    baseline += 1;
                    // center the text
                    Point textPoint = Point(predictions[i].x + 1, predictions[i].y - 3);
                    // draw the box
                    rectangle(image, textPoint + Point(textSize.width, -textSize.height), textPoint + Point(0, 1), Scalar(0,0,0), -1);
                    // ... and the baseline first
                    //line(image, textPoint + Point(0, 1), textPoint + Point(textSize.width, 1), Scalar(0, 0, 255));
                }

                putText(image, object_info/*(char*) "Obj"*/, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, bb_color, 1);
            }
        }
    }

    // resize(image, image, Size(640, 480));
    imshow("Yolo Traffic Light", image);
    waitKey(1);
}


unsigned char *
crop_raw_image(int image_width, int image_height, unsigned char *raw_image, int displacement_x, int displacement_y, int crop_width, int crop_height)
{
    unsigned char *cropped_image = (unsigned char *) malloc (crop_width * crop_height * 3 * sizeof(unsigned char));  // Only works for 3 channels image

    displacement_x = (displacement_x - 2) * 3;
    displacement_y = (displacement_y - 2) * image_width * 3;
    crop_width     = displacement_x + ((crop_width + 1) * 3);
    crop_height    = displacement_y + ((crop_height + 1) * image_width * 3);
    image_height   = image_height * image_width * 3;
    image_width   *= 3;

    for (int line = 0, index = 0; line < image_height; line += image_width)
    {
        for (int column = 0; column < image_width; column += 3)
        {
            if (column > displacement_x && column < crop_width && line > displacement_y && line < crop_height)
            {
                cropped_image[index]     = raw_image[line + column];
                cropped_image[index + 1] = raw_image[line + column + 1];
                cropped_image[index + 2] = raw_image[line + column + 2];

                index += 3;
            }
        }
    }
    return (cropped_image);
}


vector<vector<image_cartesian>>
get_points_inside_bounding_boxes(vector<bbox_t> &predictions, vector<image_cartesian> &velodyne_points_vector)
{
    vector<vector<image_cartesian>> laser_list_inside_each_bounding_box; //each_bounding_box_laser_list

    for (unsigned int i = 0; i < predictions.size();)
    {
        vector<image_cartesian> lasers_points_inside_bounding_box;

        for (unsigned int j = 0; j < velodyne_points_vector.size(); j++)
        {
            if (velodyne_points_vector[j].image_x >  predictions[i].x &&
                velodyne_points_vector[j].image_x < (predictions[i].x + predictions[i].w) &&
                velodyne_points_vector[j].image_y >  predictions[i].y &&
                velodyne_points_vector[j].image_y < (predictions[i].y + predictions[i].h))
            {
                lasers_points_inside_bounding_box.push_back(velodyne_points_vector[j]);
            }
        }
        if (lasers_points_inside_bounding_box.size() > 0)
        {
            laser_list_inside_each_bounding_box.push_back(lasers_points_inside_bounding_box);
            i++;
        }
        else
        {
            predictions.erase(predictions.begin()+i);
        }

    }
    return laser_list_inside_each_bounding_box;
}


vector<image_cartesian>
get_biggest_cluster(vector<vector<image_cartesian>> &clusters)
{
    unsigned int max_size = 0, max_index = 0;

    for (unsigned int i = 0; i < clusters.size(); i++)
    {
        if (clusters[i].size() > max_size)
        {
            max_size = clusters[i].size();
            max_index = i;
        }
    }
    return (clusters[max_index]);
}


inline double
distance2(image_cartesian a, image_cartesian b)
{
    double dx = a.cartesian_x - b.cartesian_x;
    double dy = a.cartesian_y - b.cartesian_y;

    return (dx * dx + dy * dy);
}


vector<int>
query(double d2, int i, const vector<image_cartesian> &points, std::vector<bool> clustered)
{
    vector<int> neighbors;
    const image_cartesian &point = points[i];

    for (size_t j = 0; j < points.size(); j++)
    {
        if ((distance2(point, points[j]) < d2) && !clustered[j])
            neighbors.push_back(j);
    }

    return (neighbors);
}


vector<vector<image_cartesian>>
dbscan_compute_clusters(double d2, size_t density, const vector<image_cartesian> &points)
{
    vector<vector<image_cartesian>> clusters;
    vector<bool> clustered(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i)
    {
        // Ignore already clustered points.
        if (clustered[i])
            continue;

        // Ignore points without enough neighbors.
        vector<int> neighbors = query(d2, i, points, clustered);
        if (neighbors.size() < density)
            continue;

        // Create a new cluster with the i-th point as its first element.
        vector<image_cartesian> c;
        clusters.push_back(c);
        vector<image_cartesian> &cluster = clusters.back();
        cluster.push_back(points[i]);
        clustered[i] = true;

        // Add the point's neighbors (and possibly their neighbors) to the cluster.
        for (size_t j = 0; j < neighbors.size(); ++j)
        {
            int k = neighbors[j];
            if (clustered[k])
                continue;

            cluster.push_back(points[k]);
            clustered[k] = true;

            vector<int> farther = query(d2, k, points, clustered);
            if (farther.size() >= density)
                neighbors.insert(neighbors.end(), farther.begin(), farther.end());
        }
    }
    return (clusters);
}


vector<vector<image_cartesian>>
filter_object_points_using_dbscan(vector<vector<image_cartesian>> &points_lists)
{
    vector<vector<image_cartesian>> filtered_points;

    for (unsigned int i = 0; i < points_lists.size(); i++)
    {
        vector<vector<image_cartesian>> clusters = dbscan_compute_clusters(0.5, 3, points_lists[i]);        // Compute clusters using dbscan

        if (clusters.size() == 0)                                          // If dbscan returned zero clusters
        {
            vector<image_cartesian> empty_cluster;
            filtered_points.push_back(empty_cluster);                      // An empty cluster is added to the clusters vector
        }
        else if (clusters.size() == 1)
        {
            filtered_points.push_back(clusters[0]);
        }
        else                                                               // dbscan returned more than one cluster
        {
            filtered_points.push_back(get_biggest_cluster(clusters));      // get the biggest, the biggest cluster will always better represent the object
        }
    }
    return (filtered_points);
}

vector<image_cartesian>
compute_detected_objects_poses(vector<vector<image_cartesian>> filtered_points)
{
    vector<image_cartesian> objects_positions;
    unsigned int i, j;

    for(i = 0; i < filtered_points.size(); i++)
    {
        image_cartesian point;

        if (filtered_points[i].size() == 0)
        {
            point.cartesian_x = -999.0;    // This error code is set, probably the object is out of the LiDAR's range
            point.cartesian_y = -999.0;
            point.cartesian_z = -999.0;
            //printf("Empty Bbox\n");
        }
        else
        {
            point.cartesian_x = 0.0;
            point.cartesian_y = 0.0;
            point.cartesian_z = 0.0;

            for(j = 0; j < filtered_points[i].size(); j++)
            {
                point.cartesian_x += filtered_points[i][j].cartesian_x;
                point.cartesian_y += filtered_points[i][j].cartesian_y;
                point.cartesian_z += filtered_points[i][j].cartesian_z;
            }
            point.cartesian_x = point.cartesian_x / j;
            point.cartesian_y = point.cartesian_y / j;
            point.cartesian_z = point.cartesian_z / j;
        }
        objects_positions.push_back(point);
    }
    return (objects_positions);
}


int
compute_num_measured_objects(vector<image_cartesian> objects_poses)
{
    int num_objects = 0;

    for (int i = 0; i < objects_poses.size(); i++)
    {
        if (objects_poses[i].cartesian_x > 0.0 || objects_poses[i].cartesian_y > 0.0)
            num_objects++;
    }
    return (num_objects);
}


vector<bbox_t>
filter_predictions_traffic_light(vector<bbox_t> &predictions)
{
    vector<bbox_t> filtered_predictions;

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
        if (predictions[i].obj_id == 9)    // traffic light
            filtered_predictions.push_back(predictions[i]);
    }
    return (filtered_predictions);
}


void
compute_annotation_specifications(vector<vector<image_cartesian>> traffic_light_clusters)
{
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    int i, j;

    for (i = 0; i < traffic_light_clusters.size(); i++)
    {
        for (j = 0; j < traffic_light_clusters[i].size(); j++)
        {
            mean_x += traffic_light_clusters[i][j].cartesian_x;
            mean_y += traffic_light_clusters[i][j].cartesian_y;
            mean_z += traffic_light_clusters[i][j].cartesian_z;
        }
        printf("TL %lf %lf %lf %lf Cluster Size %d\n", globalpos_msg->globalpos.theta, mean_x/j, mean_y/j, mean_z/j, (int)traffic_light_clusters[i].size());

        mean_x = 0.0;
        mean_y = 0.0;
        mean_z = 0.0;
    }
}


void
carmen_translte_3d(double *x, double *y, double *z, double offset_x, double offset_y, double offset_z)
{
    *x += offset_x;
    *y += offset_y;
    *z += offset_z;
}


void
generate_traffic_light_annotations(vector<bbox_t> predictions, vector<vector<image_cartesian>> points_inside_bbox)
{
    static vector<image_cartesian> traffic_light_points;
    static int count = 0;
    int traffic_light_found = 1;

    for (int i = 0; i < predictions.size(); i++)
    {
        for (int j = 0; j < points_inside_bbox[i].size(); j++)
        {
            carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, board_pose.position.x, board_pose.position.y);
            carmen_rotate_2d  (&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos_msg->globalpos.theta);
            carmen_translte_2d(&points_inside_bbox[i][j].cartesian_x, &points_inside_bbox[i][j].cartesian_y, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y);

            points_inside_bbox[i][j].cartesian_z += board_pose.position.z + velodyne_pose.position.z;

            traffic_light_points.push_back(points_inside_bbox[i][j]);
        }
        count = 0;
        traffic_light_found = 0;
    }
    count += traffic_light_found;

    if (count > 8)                  // If stays without see a traffic light for more than N frames
    {                               // Compute traffic light positions and generate annotations
        vector<vector<image_cartesian>> traffic_light_clusters = dbscan_compute_clusters(0.5, 3, traffic_light_points);

        compute_annotation_specifications(traffic_light_clusters);
        traffic_light_points.clear();
        count = 0;
    }
    //printf("Cont %d\n", count);
}


void
compute_picked_point(vector<image_cartesian> points)
{
    if (screen_marker_set && !marker_point_found)
    {
        double marker_picked_point_pixel_distance = DBL_MAX;
        for (int i = 0; i < points.size(); ++i)
        {
            double pixel_distance = sqrt(pow(screen_marker.x - points[i].image_x, 2) +
                            pow(screen_marker.y - points[i].image_y, 2));
            if (pixel_distance < marker_picked_point_pixel_distance) {
                marker_picked_point_pixel_distance = pixel_distance;
                marker_picked_point = points[i];
            }
        }

        carmen_translte_2d(&marker_picked_point.cartesian_x, &marker_picked_point.cartesian_y, board_pose.position.x, board_pose.position.y);
        carmen_rotate_2d  (&marker_picked_point.cartesian_x, &marker_picked_point.cartesian_y, globalpos_msg->globalpos.theta);
        carmen_translte_2d(&marker_picked_point.cartesian_x, &marker_picked_point.cartesian_y, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y);
        marker_picked_point.cartesian_z += board_pose.position.z + velodyne_pose.position.z;
        fprintf(stderr, "Picked point: %lf\t%lf\t%lf; theta=%lf\n", marker_picked_point.cartesian_x, marker_picked_point.cartesian_y, marker_picked_point.cartesian_z, globalpos_msg->globalpos.theta);
        fprintf(stderr, "RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT\t8\t0\t%lf\t%lf\t%lf\t%lf\n", globalpos_msg->globalpos.theta, marker_picked_point.cartesian_x, marker_picked_point.cartesian_y, marker_picked_point.cartesian_z);
        marker_point_found = true;
    }
}

void
compute_traffic_light_pose(vector<bbox_t> predictions, int resized_w, int resized_h, int crop_x, int crop_y, int crop_w, int crop_h, vector<image_cartesian> points)
{
    vector<vector<image_cartesian>> points_inside_bbox = get_points_inside_bounding_boxes(predictions, points); // TODO remover bbox que nao tenha nenhum ponto, int threshold

    generate_traffic_light_annotations(predictions, points_inside_bbox);
}


int
compute_threshold()
{
    // ## Hyperbolic
    // double dist = DIST2D(nearests_traffic_lights_vector[0].position, globalpos_msg->globalpos);
    // int threshold = 1000 / dist;

    // ## Linear
    // double dist = DIST2D(nearests_traffic_lights_vector[0].position, globalpos_msg->globalpos);
    // static double m = (MAX_TRAFFIC_LIGHT_THRESHOLD - MIN_TRAFFIC_LIGHT_THRESHOLD) / (double)MAX_TRAFFIC_LIGHT_DIST;
    // int threshold = MIN_TRAFFIC_LIGHT_THRESHOLD + m * (MAX_TRAFFIC_LIGHT_DIST - dist);

    // ## Hyperbolic using projection.
    tf::StampedTransform world_to_camera_pose = get_world_to_camera_transformer(&transformer, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y, 0.0,
                0.0, 0.0, globalpos_msg->globalpos.theta);
    carmen_position_t tl_point = convert_rddf_pose_to_point_in_image(nearests_traffic_lights_vector[0].position.x, nearests_traffic_lights_vector[0].position.y, nearests_traffic_lights_vector[0].position.z,
                   world_to_camera_pose, camera_parameters, RESIZED_W, RESIZED_H);
    carmen_position_t below_point = convert_rddf_pose_to_point_in_image(nearests_traffic_lights_vector[0].position.x, nearests_traffic_lights_vector[0].position.y, nearests_traffic_lights_vector[0].position.z + TRAFFIC_LIGHT_IMAGE_THRESHOLD,
                   world_to_camera_pose, camera_parameters, RESIZED_W, RESIZED_H);
    // fprintf(stderr, "tl_point: %lf %lf\n", tl_point.x, tl_point.y);
    // fprintf(stderr, "below_point: %lf %lf\n", below_point.x, below_point.y);

    int threshold;
    if (tl_point.x >= 0 && tl_point.y >= 0 && tl_point.x < RESIZED_W && tl_point.y < RESIZED_H
        && below_point.x >= 0 && below_point.y >= 0 && below_point.x < RESIZED_W && below_point.y < RESIZED_H) {
        threshold = (int) sqrt(pow(tl_point.x - below_point.x, 2) + pow(tl_point.y - below_point.y, 2));
    } else {
        threshold = DEFAULT_TRAFFIC_LIGHT_THRESHOLD;
    }

    // fprintf(stderr, "threshold: %d\n", threshold);

    if (threshold > MAX_TRAFFIC_LIGHT_THRESHOLD)
        return MAX_TRAFFIC_LIGHT_THRESHOLD;
    else if (threshold < MIN_TRAFFIC_LIGHT_THRESHOLD)
        return MIN_TRAFFIC_LIGHT_THRESHOLD;
    else
        return threshold;
}


carmen_traffic_light*
get_main_traffic_light(vector<bbox_t> predictions,  vector<carmen_position_t> tf_annotations_on_image, int threshold)
{
    carmen_traffic_light *main_traffic_light = (carmen_traffic_light *) malloc (1 * sizeof(carmen_traffic_light));
    bbox_t main_bbox;
    double dist = 0.0, main_dist = DBL_MAX, x_centroid = 0.0, y_centroid = 0.0;

    for (unsigned int i = 0; i < predictions.size(); i++)
    {
        for (unsigned int j = 0; j < tf_annotations_on_image.size(); j++)
        {
            x_centroid = predictions[i].x + (predictions[i].w / 2);
            y_centroid = predictions[i].y + (predictions[i].h / 2);

            x_centroid = x_centroid - tf_annotations_on_image[j].x;
            y_centroid = y_centroid - tf_annotations_on_image[j].y;

            dist = sqrt((x_centroid * x_centroid) + (y_centroid * y_centroid));

            if (dist < main_dist && dist < threshold)
            {
                main_dist = dist;
                main_bbox = predictions[i];
            }
        }
    }

    main_traffic_light->x1 = 0;
    main_traffic_light->y1 = 0;
    main_traffic_light->x2 = 0;
    main_traffic_light->y2 = 0;
    main_traffic_light->color = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF;                    // In case of any failure, TRAFFIC_LIGHT_OFF message is sent

    if (main_dist < DBL_MAX)                                                               //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED
    {                                                                                      //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN
        main_traffic_light->x1 = main_bbox.x;
        main_traffic_light->y1 = main_bbox.y;
        main_traffic_light->x2 = main_bbox.x + main_bbox.w;
        main_traffic_light->y2 = main_bbox.y + main_bbox.h;
        if (main_bbox.obj_id == 0)                                                         //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW
            main_traffic_light->color = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_RED;            //RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_OFF
        else if (main_bbox.obj_id == 1)                                                    //RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT
            main_traffic_light->color = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_GREEN;          //RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT
        else if (main_bbox.obj_id == 2)                                                    //RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_RIGHT
            main_traffic_light->color = RDDF_ANNOTATION_CODE_TRAFFIC_LIGHT_YELLOW;         //RDDF_ANNOTATION_CODE_TRAFFIC_SIGN_TURN_LEFT
    }

    return (main_traffic_light);
}


carmen_traffic_light_message
build_traffic_light_message(carmen_bumblebee_basic_stereoimage_message *image_msg, vector<bbox_t> predictions, vector<carmen_position_t> tf_annotations_on_image, int tl_threshold)
{
    carmen_traffic_light_message traffic_light_message;

    traffic_light_message.num_traffic_lights = 1;
    traffic_light_message.traffic_lights = get_main_traffic_light(predictions, tf_annotations_on_image, tl_threshold);
    traffic_light_message.traffic_light_annotation_distance = 9999.0;
    traffic_light_message.timestamp = image_msg->timestamp;
    traffic_light_message.host = carmen_get_host();

    return (traffic_light_message);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_moving_objects_message(double timestamp, carmen_moving_objects_point_clouds_message *msg)
{
    msg->timestamp = timestamp;
    msg->host = carmen_get_host();

    carmen_moving_objects_point_clouds_publish_message(msg);
}


static void
publish_traffic_lights(carmen_traffic_light_message *traffic_light_message)
{
    carmen_traffic_light_publish_message(camera, traffic_light_message);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
    if (image_msg == NULL)
        return;

    double fps;
    static double start_time = 0.0;
    unsigned char *img;

    if (camera_side == 0)
        img = image_msg->raw_left;
    else
        img = image_msg->raw_right;

    int crop_x = 0;
    int crop_y = 0;
    int crop_w = RESIZED_W;
    int crop_h = RESIZED_H;

    Mat open_cv_image = Mat(image_msg->height, image_msg->width, CV_8UC3, img, 0);
    if (Size(RESIZED_W, RESIZED_H) != open_cv_image.size())
    {
        resize(open_cv_image, open_cv_image, Size(RESIZED_W, RESIZED_H));
    }

    vector<bbox_t> predictions;
    if (RUN_YOLO) {
        predictions = run_YOLO(open_cv_image.data, open_cv_image.cols, open_cv_image.rows, network_struct, classes_names, CONFIDENCE_THRESHOLD);
    } else {
        fprintf(stderr, "==============================\n");
        fprintf(stderr, "WARNING: YOLO is not running!!\n");
        fprintf(stderr, "==============================\n");
    }

    // Filter irrelevant COCO classes, when using the COCO model.
    // This may be unnecessary overhead. We could just ignore the other classes...
    if (YOLO_CLASS_SET == COCO) {
        auto new_end = std::remove_if(predictions.begin(), predictions.end(),
            [](bbox_t bb){return bb.obj_id != 9;});
        predictions.erase(new_end, predictions.end());
    }

    if (RUN_RANDOM_FOREST) {
        vector<Mat> views = make_box_views(open_cv_image, predictions);
        // mkdir("/tmp/tf_bboxes", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH); // Debugging...
        // save_mats_as_images(views, "/tmp/tf_bboxes"); // Debugging...
        Mat imgs_data = pack_images_together(views);
        Mat rf_predictions;
        rf->predict(imgs_data, rf_predictions);
        rf_predictions -= 1; // Jean did it so 1=red, 2=green, 3=yellow... weird.
        for (size_t i = 0; i < predictions.size(); i++)
        {
            predictions[i].obj_id = static_cast<int>(rf_predictions.at<float>(i));
        }
    }

    fps = 1.0 / (carmen_get_time() - start_time);
    start_time = carmen_get_time();

    vector<image_cartesian> lidar_points;

    if (globalpos_msg != NULL && velodyne_msg != NULL)
    {
        lidar_points = velodyne_camera_calibration_fuse_camera_lidar(velodyne_msg, camera_parameters, velodyne_pose, camera_pose,
                RESIZED_W, RESIZED_H, crop_x, crop_y, crop_w, crop_h);

        tf::StampedTransform world_to_camera_pose = get_world_to_camera_transformer(&transformer, globalpos_msg->globalpos.x, globalpos_msg->globalpos.y, 0.0,
                0.0, 0.0, globalpos_msg->globalpos.theta);

        double distance_to_tf = compute_distance_to_the_traffic_light();

        vector<carmen_position_t> tf_annotations_on_image;
        const size_t line_size = 100;
        char final_prediction_line[line_size];
        char gt_prep_line[line_size];
        if (distance_to_tf < DBL_MAX)
        {
            int tl_threshold = compute_threshold();

            for (int i = 0; i < nearests_traffic_lights_vector.size(); i++)
            {
                carmen_position_t tl_on_cam = convert_rddf_pose_to_point_in_image(nearests_traffic_lights_vector[i].position.x, nearests_traffic_lights_vector[i].position.y, nearests_traffic_lights_vector[i].position.z,
                        world_to_camera_pose, camera_parameters, RESIZED_W, RESIZED_H);

                // Check if TL annotation is really on camera.
                if (tl_on_cam.x >= 0 && tl_on_cam.y >= 0 && tl_on_cam.x < RESIZED_W && tl_on_cam.y < RESIZED_H) {
                    tf_annotations_on_image.push_back(tl_on_cam);

                    if (DRAW_CIRCLE_THRESHOLD)
                    {
                        // circle(open_cv_image, Point((int)tf_annotations_on_image[i].x, (int)tf_annotations_on_image[i].y), 3, Scalar(255, 255, 0), -1, 8);
                        // circle(open_cv_image, Point((int)tf_annotations_on_image[i].x, (int)tf_annotations_on_image[i].y), compute_threshold(), Scalar(255, 255, 0), 1, 8);
                        circle(open_cv_image, Point((int)tl_on_cam.x, (int)tl_on_cam.y), tl_threshold, Scalar(255, 255, 0), 1, 8);
                    }
                }
            }

            // if (predictions.size() > 0 && COMPUTE_TL_POSES)
            if (COMPUTE_TL_POSES)
            {
                // FIXME: Quase certeza que da pra tirar o `predictions.size() > 0`.
                compute_traffic_light_pose(predictions, RESIZED_W, RESIZED_H, crop_x, crop_y, crop_w, crop_h, lidar_points);
            }

            if (tf_annotations_on_image.size() > 0) {
                carmen_traffic_light_message traffic_light_message = build_traffic_light_message(image_msg, predictions, tf_annotations_on_image, tl_threshold);
                publish_traffic_lights(&traffic_light_message);

                if (DRAW_FINAL_PREDICTION)
                {
                    draw_final_prediction(open_cv_image, traffic_light_message.traffic_lights->color);
                }

                char const* tl_code_name = tl_code_names[traffic_light_message.traffic_lights->color];
                snprintf(final_prediction_line, line_size, "timestamp(image)=%lf; distance=%lf; tl_state=%s\n", image_msg->timestamp, distance_to_tf, tl_code_name);
                snprintf(gt_prep_line, line_size,          "timestamp(image)=%lf; distance=%lf; tl_state=\n", image_msg->timestamp, distance_to_tf);
            }
        }

        if (tf_annotations_on_image.size() == 0) {
            snprintf(final_prediction_line, line_size, "timestamp(image)=%lf; distance=NaN; tl_state=NONE\n", image_msg->timestamp);
            snprintf(gt_prep_line, line_size,          "timestamp(image)=%lf; distance=NaN; tl_state=\n", image_msg->timestamp);
        }

        if (PRINT_FINAL_PREDICTION) {
            printf("%s", final_prediction_line);
        } else if (PRINT_GT_PREP) {
            printf("%s", gt_prep_line);
        }

        // @possatti Debug TL annotations within a distance.
        if (DRAW_CLOSE_TLS)
        {
            for (unsigned int i = 0; i < annotation_vector.size(); i++)
            {
                double distance = sqrt(pow(globalpos_msg->globalpos.x - annotation_vector[i].position.x, 2) +
                                   pow(globalpos_msg->globalpos.y - annotation_vector[i].position.y, 2));
                bool behind = fabs(carmen_normalize_theta((
                    atan2(globalpos_msg->globalpos.y - annotation_vector[i].position.y, globalpos_msg->globalpos.x - annotation_vector[i].position.x)
                     - M_PI - globalpos_msg->globalpos.theta))) > M_PI_2;
                if (distance < 100 && ! behind)
                {
                   carmen_position_t point = convert_rddf_pose_to_point_in_image(annotation_vector[i].position.x, annotation_vector[i].position.y, annotation_vector[i].position.z,
                       world_to_camera_pose, camera_parameters, RESIZED_W, RESIZED_H);
                   circle(open_cv_image, Point((int)point.x, (int)point.y), 1, Scalar(255, 0, 0), -1, 8);
               }
            }
        }

        if (COMPUTE_TL_POSES)
        {
            compute_picked_point(lidar_points);
        }

        // Draw the point picked by hand.
        if (marker_point_found)
        {
            // Scalar(253,106,2) # Orange
            carmen_position_t point = convert_rddf_pose_to_point_in_image(marker_picked_point.cartesian_x, marker_picked_point.cartesian_y, marker_picked_point.cartesian_z,
                   world_to_camera_pose, camera_parameters, RESIZED_W, RESIZED_H);
            circle(open_cv_image, Point((int)point.x, (int)point.y), 5, Scalar(253,106,2), -1, 8);
        }
    }

    if (screen_marker_set)
    {
        circle(open_cv_image, Point((int)screen_marker.x, (int)screen_marker.y), 3.0, Scalar(100, 100, 255), -1, 8);
    }

    display(open_cv_image, predictions, fps, lidar_points);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
    velodyne_msg = velodyne_message;

    carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_msg);
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
    globalpos_msg = globalpos_message;
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Neural Object Detector: Disconnected.\n");
        exit(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////


void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);
}


void
load_traffic_light_annotation_file(char* path)
{
    carmen_pose_3D_t pose;
    
    std::ifstream ifile(path);
    if (!(bool)ifile) {
        printf("Neural Object Detector: File %s doesn't exist.\n", path);
        exit(1);
    }

    FILE *annotation_file = fopen(path,"r");

    int annotation_type, annotation_code;
    char line[1024], annotation_description[1024];
    cerr << "INFO: Loading traffic light annotation file: " << path << endl;
    cerr << "INFO: Traffic light annotations (yaw, x, y, z):" << endl;
    while (fgets(line, 1023, annotation_file) != NULL)
    {
        // Strips # comments off.
        size_t hashtag_i = strcspn(line, "#");
        line[hashtag_i] = '\0';

        annotation_description[0] = '\0';
        int scan_count = sscanf(line, "%s %d %d %lf %lf %lf %lf ", annotation_description, &annotation_type, &annotation_code,
                        &pose.orientation.yaw, &pose.position.x, &pose.position.y, &pose.position.z);

        if (scan_count == 7
            && strcmp("RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT", annotation_description) == 0)
        {
            printf("%lf %lf %lf %lf\n", pose.orientation.yaw, pose.position.x, pose.position.y, pose.position.z);
            pose.orientation.roll = 0.0;
            pose.orientation.pitch = 0.0;
            annotation_vector.push_back(pose);
        }
    }

    fclose(annotation_file);
}


void
read_parameters(int argc, char **argv)
{
    if ((argc != 4))
        carmen_die("%s: Wrong number of parameters. neural_object_detector requires 3 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)> <rddf_file_path>\n", argv[0], argc - 1, argv[0]);

    camera = atoi(argv[1]);             // Define the camera to be used
    camera_side = atoi(argv[2]);        // 0 For left image 1 for right image

    load_traffic_light_annotation_file(argv[3]);

    int num_items;

    char bumblebee_string[256];
    char camera_string[256];

    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Gather the cameri ID
    sprintf(camera_string, "%s%d", "camera", camera);

    carmen_param_t param_list[] =
    {
        {bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
        {bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
        {bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
        {bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
        {bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

        {camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
        {camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
        {camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
        {camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
        {camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
        {camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL },

        {(char *) "velodyne", (char *) "x",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
        {(char *) "velodyne", (char *) "y",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
        {(char *) "velodyne", (char *) "z",     CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
        {(char *) "velodyne", (char *) "roll",  CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
        {(char *) "velodyne", (char *) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
        {(char *) "velodyne", (char *) "yaw",   CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},

        {(char *) "sensor_board_1", (char*) "x",     CARMEN_PARAM_DOUBLE, &board_pose.position.x, 0, NULL },
        {(char *) "sensor_board_1", (char*) "y",     CARMEN_PARAM_DOUBLE, &board_pose.position.y, 0, NULL },
        {(char *) "sensor_board_1", (char*) "z",     CARMEN_PARAM_DOUBLE, &board_pose.position.z, 0, NULL },
        {(char *) "sensor_board_1", (char*) "roll",  CARMEN_PARAM_DOUBLE, &board_pose.orientation.roll, 0, NULL },
        {(char *) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &board_pose.orientation.pitch, 0, NULL },
        {(char *) "sensor_board_1", (char*) "yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL }
    };

    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);
}

void
onMouse(int event, int x, int y, int, void*)
{
    if (event == EVENT_MBUTTONDOWN) {
        // fprintf(stderr, "MIDDLE CLICK: x=%d y=%d\n", x, y);
        screen_marker = Point(x,y);
        screen_marker_set = true;
        marker_point_found = false;
    } else if (event == EVENT_RBUTTONDOWN) {
        screen_marker_set = false;
        marker_point_found = false;
    }
}

inline bool
file_exists(const std::string& name)
{
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

inline bool
file_exists_and_is_not_empty(const std::string& name)
{
    struct stat buffer;
    return ((stat(name.c_str(), &buffer) == 0) && (buffer.st_size > 0));
}

inline void
file_ok_or_error(const std::string& name) {
    if (!file_exists_and_is_not_empty(name)) {
        cerr << "ERROR: File does not exist or is empty '" << name << "'!\n";
        exit(1);
    }
}

void
initializer()
{
    initialize_transformations(board_pose, camera_pose, &transformer);

    // string darknet_names_file_path, darknet_cfg_file_path, darknet_weights_file_path;
    char darknet_names_file_path[1024], darknet_cfg_file_path[1024], darknet_weights_file_path[1024];

    /** YOLOv3 treinada pelo Possatti. */
    // darknet_names_file_path = "../sharedlib/darknet2/data/traffic_light.names";
    // darknet_cfg_file_path = "../sharedlib/darknet2/cfg/traffic_light.cfg";
    // darknet_weights_file_path = "../sharedlib/darknet2/yolov3_traffic_light_rgo.weights";

    /** YOLOv3 treinada no COCO. */
    sprintf(darknet_names_file_path, "%s/%s", CARMEN_HOME, "sharedlib/darknet2/data/coco.names");
    sprintf(darknet_cfg_file_path, "%s/%s", CARMEN_HOME, "sharedlib/darknet2/cfg/yolov3.cfg");
    sprintf(darknet_weights_file_path, "%s/%s", CARMEN_HOME, "sharedlib/darknet2/yolov3.weights");

    file_ok_or_error(darknet_names_file_path);
    file_ok_or_error(darknet_cfg_file_path);
    file_ok_or_error(darknet_weights_file_path);

    cerr << "INFO: Darknet class names file: " << darknet_names_file_path << endl;
    cerr << "INFO: Darknet CFG file: " << darknet_cfg_file_path << endl;
    cerr << "INFO: Darknet weights: " << darknet_cfg_file_path << endl;

    // classes_names = get_classes_names(const_cast<char*>(darknet_names_file_path.c_str()));
    // network_struct = initialize_YOLO(const_cast<char*>(darknet_cfg_file_path.c_str()), const_cast<char*>(darknet_weights_file_path.c_str()));

    classes_names = get_classes_names(darknet_names_file_path);
    network_struct = initialize_YOLO(darknet_cfg_file_path, darknet_weights_file_path);

    // Load Random Forest.
    char * rf_weights_path;
    sprintf(rf_weights_path, "%s/%s", CARMEN_HOME, "data/ml_models/traffic_lights/random_forest_classifier/cv_rtrees_weights_tl_rgy.yml");
    cerr << "INFO: Weights for OpenCV's RTrees: " << rf_weights_path << endl;
    rf = StatModel::load<RTrees>(rf_weights_path);

    //namedWindow("Neural Object Detector", WINDOW_AUTOSIZE);
    setMouseCallback("Yolo Traffic Light", onMouse);

    cerr << "INFO: Initialization done." << endl;
}


//IPC_RETURN_TYPE
//carmen_traffic_light_define_messages(int camera)
//{
//    IPC_RETURN_TYPE err;
//
//    char *message_name = carmen_traffic_light_message_name(camera);
//    err = IPC_defineMsg(message_name, IPC_VARIABLE_LENGTH, CARMEN_TRAFFIC_LIGHT_FMT);
//    carmen_test_ipc_exit(err, "Could not define", message_name);
//    free(message_name);
//    return err;
//}

int
main(int argc, char **argv)
{
    carmen_ipc_initialize(argc, argv);

    read_parameters(argc, argv);

    camera = atoi(argv[1]);
    carmen_traffic_light_define_messages(camera);

    subscribe_messages();

    signal(SIGINT, shutdown_module);

    initializer();

    setlocale(LC_ALL, "C");

    carmen_ipc_dispatch();

    return 0;
}
