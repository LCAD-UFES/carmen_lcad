
/*********************************************************
	---  Neural Mapper ---
 **********************************************************/
#include <carmen/carmen.h>

#include <carmen/velodyne_interface.h>
#include <carmen/mapper_interface.h>
#include <carmen/base_ackerman_interface.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <carmen/rotation_geometry.h>
#include <math.h>
#include <carmen/grid_mapping.h>
#include <string.h>
#include <stdlib.h>
#include <prob_map.h>
#include <prob_measurement_model.h>
#include <prob_transforms.h>
#include <carmen/stereo_mapping_interface.h>
#include <carmen/map_server_interface.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define N_LASER_POINTS 32

#define kitti_uu_training_size 97 
#define kitti_um_training_size 94
#define kitti_umm_training_size 95

/*********************VELODYNE*********************/
static double vertical_correction[32] = {-30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999,
-25.33, -4.0,
-24.0, -2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001,
-17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001,
-13.33, 8.0, -12.0, 9.3299999, -10.67, 10.67};
/**************************************************/

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
static carmen_pose_3D_t velodyne_pose;
static carmen_pose_3D_t sensor_board_1_pose;
static carmen_pose_3D_t car_fused_pose;

static rotation_matrix* velodyne_to_board_matrix;
static rotation_matrix* board_to_car_matrix;

static double car_fused_time;
static double car_fused_velocity;
static double car_phi;
static double distance_between_front_and_rear_axles;
static double time_spent_by_each_scan;

//velodyne map static information
static float map_resolution;	//meters
static int map_x_size;			//meters
static int map_y_size;			//meters
static int point_clouds_step_size;
static int n_x_pixels;
static int n_y_pixels;
static int use_kitti = 0;
static double mapper_timestamp = 0;
static double velodyne_timestamp = 0;
static bool get_next_mapper = true;
static bool get_next_velodyne = true;

// Valores limites das estatisticas do velodyne
static double max_hight = 5;// metros
// para teste
static int count = 1;

carmen_map_t raw_mean_hight_map, raw_max_hight_map, raw_min_hight_map,
			raw_hight_standard_deviation_map, raw_reflectivity_map, raw_number_of_lasers_map, raw_label_map, raw_square_sum_map;

static carmen_vector_3D_t move_velodyne_point_to_car_reference(carmen_vector_3D_t velodyne_reference,
        rotation_matrix *velodyne_to_board_matrix, rotation_matrix *board_to_car_matrix,
        carmen_vector_3D_t velodyne_pose_position, carmen_vector_3D_t sensor_board_1_pose_position)
{
    carmen_vector_3D_t board_reference = multiply_matrix_vector(velodyne_to_board_matrix, velodyne_reference);
    board_reference = add_vectors(board_reference, velodyne_pose_position);

    carmen_vector_3D_t car_reference = multiply_matrix_vector(board_to_car_matrix, board_reference);
    car_reference = add_vectors(car_reference, sensor_board_1_pose_position);

    return (car_reference);
}


static carmen_vector_3D_t spherical_to_cartesian_coordinates(double rot_angle, double vert_angle, double range) {
	carmen_vector_3D_t velodyne_reference;
	double cos_rot_angle = cos(rot_angle);
	double sin_rot_angle = sin(rot_angle);
	double cos_vert_angle = cos(vert_angle);
	double sin_vert_angle = sin(vert_angle);
	double xy_distance = range * cos_vert_angle;
	velodyne_reference.x = (xy_distance * cos_rot_angle);
	velodyne_reference.y = (xy_distance * sin_rot_angle);
	velodyne_reference.z = (range * sin_vert_angle);
	return velodyne_reference;
}

void map_to_csv(carmen_map_t complete_map, char* csv_name){
	
	char csv_file_name[1024];

	sprintf(csv_file_name,"%s.csv",csv_name);

	ofstream myfile;
    myfile.open (csv_file_name);

    int width = complete_map.config.x_size;
	int height = complete_map.config.y_size;

	for (int y = 0; y < height; y++){
		for (int x = 0; x < width; x++){
			myfile << complete_map.map[x][y];
			if(x < width-1)
				myfile << ",";
		}
		if(y < height-1)
			myfile << "\n";
    }
    myfile.close();
}

int map_to_bitmap(carmen_map_t complete_map, char* bitmap_name){
	// headers do bitmap
	FILE *f;

	unsigned char *img = NULL;

	int width = complete_map.config.x_size;
	int height = complete_map.config.y_size;
	int i;
	unsigned char r,g,b,prob;

	int filesize = 54 + 3*width*height;

	if(img)
	    free(img);

	img = (unsigned char *)calloc(3 * width * height, sizeof(unsigned char));


	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			if(complete_map.map[x][y] == -1)
			{
				r = 0;
				g = 0;
				b = 255;
			} else {
				prob = (unsigned char) ((1 - complete_map.map[x][y]) * 255.0); // converte a probabilidade para [0-255]
				r = prob;
				g = prob;
				b = prob;
			}
			img[(x+(height-1-y)*width)*3+2] = (unsigned char)(r);	// height -1 -y pois no mapa a origem é
			img[(x+(height-1-y)*width)*3+1] = (unsigned char)(g);	// no canto inferior esquerdo e no
			img[(x+(height-1-y)*width)*3+0] = (unsigned char)(b);	// bitmap é no canto superior esquerdo
		}
	}

	// preenche os headers do bitmap
	unsigned char bmpfileheader[14] = {'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0};
	unsigned char bmpinfoheader[40] = {40,0,0,0, 0,0,0,0, 0,0,0,0, 1,0, 24,0};
	unsigned char bmppad[3] = {0,0,0};

	bmpfileheader[ 2] = (unsigned char)(filesize    );
	bmpfileheader[ 3] = (unsigned char)(filesize>> 8);
	bmpfileheader[ 4] = (unsigned char)(filesize>>16);
	bmpfileheader[ 5] = (unsigned char)(filesize>>24);

	bmpinfoheader[ 4] = (unsigned char)(width    );
	bmpinfoheader[ 5] = (unsigned char)(width>> 8);
	bmpinfoheader[ 6] = (unsigned char)(width>>16);
	bmpinfoheader[ 7] = (unsigned char)(width>>24);
	bmpinfoheader[ 8] = (unsigned char)(height    );
	bmpinfoheader[ 9] = (unsigned char)(height>> 8);
	bmpinfoheader[10] = (unsigned char)(height>>16);
	bmpinfoheader[11] = (unsigned char)(height>>24);

	char bitmap_file_name[1024];

	sprintf(bitmap_file_name,"%s.bmp",bitmap_name);

	f = fopen(bitmap_file_name,"wb");
	fwrite(bmpfileheader,1,14,f);
	fwrite(bmpinfoheader,1,40,f);
	for(i=0; i<height; i++)
	{
	    fwrite(img+(width*(height-i-1)*3),3,width,f);
	    fwrite(bmppad,1,(4-(width*3)%4)%4,f);			// a largura deve ser multipla de 4 no bitmap
	}
	fclose(f);
	printf("Completo!\n%d %d\n", complete_map.config.x_size, complete_map.config.y_size);

	return 1;
}

// joga todos os valores para -1
void
set_map_to_void(carmen_map_t *value_map)
{
	for(int i = 0; i < value_map->config.y_size;i++){
		for(int j = 0; j < value_map->config.x_size;j++){
			value_map->map[j][i]= -1;
		}
	}
}

// joga todos os valores para 0
void 
set_map_values_to_zero(carmen_map_t *value_map){
	for(int i = 0; i < value_map->config.y_size;i++){
		for(int j = 0; j < value_map->config.x_size;j++){
			value_map->map[j][i]= 0;
		}
	}
}

// normalize map to 0-1 values
carmen_map_t normalize_map(carmen_map_t *value_map){
	double max = -2.0;
	double min = 10.0;
	int max_index = 0;
	int min_index = 0;

	carmen_map_t normalized;
	carmen_grid_mapping_create_new_map(&normalized, value_map->config.x_size, value_map->config.y_size, value_map->config.resolution, 'u');

	//find mean, min and max values
	for(int i = 0; i < value_map->config.y_size;i++){
		for(int j = 0; j < value_map->config.x_size;j++){
			if(value_map->map[j][i] != -1){
				if(max < value_map->map[j][i]){
					max = value_map->map[j][i];
				}
				if(min > value_map->map[j][i]){
					min = value_map->map[j][i];
				}
			}
		}
	}

	//normalization
	for(int i = 0; i < value_map->config.y_size;i++){
		for(int j = 0; j < value_map->config.x_size;j++){
			if(value_map->map[j][i] != -1){
				float temp = ((value_map->map[j][i]-min)/(max-min));
				normalized.map[j][i] = temp;
			}
		}
	}
	//printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized;
}

// normalize map to 0-255 values
carmen_map_t
fixed_normalize_map(carmen_map_t *value_map, double new_max)
{
	double min = -1.0;
	double max = max_hight;

	carmen_map_t normalized;
	carmen_grid_mapping_create_new_map(&normalized, value_map->config.x_size, value_map->config.y_size, value_map->config.resolution, 'u');

	//normalization
	for(int i = 0; i < value_map->config.y_size;i++){
		for(int j = 0; j < value_map->config.x_size;j++){
			float temp = (value_map->map[j][i]-min)*(new_max)/(max-min);
			normalized.map[j][i] = temp;
			//printf("%lf\n", temp);
		}
	}

	//printf("max = %lf | min = %lf | norm_max = %d | norm_min = %d\n", max, min, normalized[max_index], normalized[min_index]);
	return normalized;
}

int
map_to_png(carmen_map_t *complete_map, char* png_name)
{
	carmen_map_t normalized = fixed_normalize_map(complete_map, 255.);

	Mat png_mat = Mat(complete_map->config.x_size, complete_map->config.y_size, CV_64FC1, *normalized.map);

	char png_file_name[100];
	sprintf(png_file_name,"%s.png",png_name);

	imwrite(png_file_name, png_mat);
}

// display the value of interest of the map in a grayscale img
void
show_map(carmen_map_t *value_map, char *name)
{
	//map_to_csv(*value_map, name);
	map_to_png(value_map, name);
	set_map_to_void(value_map);
}

void update_map(carmen_vector_3D_t p, carmen_velodyne_partial_scan_message* velodyne_message, int laser_number, int vertical_position){

	int x_index = (int)(p.x/map_resolution);
    int y_index = (int)(p.y/map_resolution);

    // transform x and y positions to map reference
    int transform_x_index = (int)(raw_reflectivity_map.config.x_origin + x_index);
    int transform_y_index = (int)(raw_reflectivity_map.config.y_origin + y_index);

    // from the top to the bottom
    // check if the positions are outside the map area
    if(transform_x_index >= n_x_pixels || transform_x_index < 0 || transform_y_index >= n_y_pixels || transform_y_index < 0 || p.z > max_hight){
		return;
    }

    // variables to save last values
	double last_variance;
	if(raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index] == -1)
		last_variance = 0;
    else{
    	last_variance = carmen_square(raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index]);
    }

 	double last_mean;   
	if(raw_mean_hight_map.map[transform_x_index][transform_y_index] == -1)
		last_mean = 0;
    else{
    	last_mean = raw_mean_hight_map.map[transform_x_index][transform_y_index];
    }
    
    // update number of points
    if(raw_number_of_lasers_map.map[transform_x_index][transform_y_index]==-1){
    	raw_number_of_lasers_map.map[transform_x_index][transform_y_index] = 1;
    }
    else{
	    raw_number_of_lasers_map.map[transform_x_index][transform_y_index]++;
    }
    int n = raw_number_of_lasers_map.map[transform_x_index][transform_y_index];


    // update min and max values
	if(n == 1){
		raw_min_hight_map.map[transform_x_index][transform_y_index] = p.z;
		raw_max_hight_map.map[transform_x_index][transform_y_index] = p.z;		
	}
	else if(p.z < raw_min_hight_map.map[transform_x_index][transform_y_index]){
	    raw_min_hight_map.map[transform_x_index][transform_y_index] = p.z;
	}
	else if(p.z > raw_max_hight_map.map[transform_x_index][transform_y_index]){
	    raw_max_hight_map.map[transform_x_index][transform_y_index] = p.z;
	}
	
	// update mean
	if(n==1){
		raw_mean_hight_map.map[transform_x_index][transform_y_index] = p.z;
	}
	else{
		raw_mean_hight_map.map[transform_x_index][transform_y_index] = ((raw_mean_hight_map.map[transform_x_index][transform_y_index]*(n-1)) + p.z)/(n);
	}

	//update variance
	if(n > 0)
	{
		double V1 = (raw_square_sum_map.map[transform_x_index][transform_y_index]*(n-1) + carmen_square(p.z))/n;
		double V2 = carmen_square(raw_mean_hight_map.map[transform_x_index][transform_y_index]*(n-1))/n;
		raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index] = sqrt(V1 + V2);
	}
	else
		raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index] = 0;		

	raw_square_sum_map.map[transform_x_index][transform_y_index] += carmen_square(p.z);

	// update reflectivity
	double newRefleflectivity = velodyne_message->partial_scan[laser_number].intensity[vertical_position];
	if(n==1){
		raw_reflectivity_map.map[transform_x_index][transform_y_index] = newRefleflectivity;
	}
	else{
		double last_meanReflectivity = raw_reflectivity_map.map[transform_x_index][transform_y_index];
		raw_reflectivity_map.map[transform_x_index][transform_y_index] = ((last_meanReflectivity*(n-1))+newRefleflectivity)/n;
	}
}

void compute_velodyne_points(carmen_velodyne_partial_scan_message* velodyne_message){

    carmen_pose_3D_t car_interpolated_position;
    rotation_matrix car_to_global_matrix;

    double dt = velodyne_message->timestamp - car_fused_time - velodyne_message->number_of_32_laser_shots * time_spent_by_each_scan;
   
    int range_max_points = 0;

    for (int laser_number = 0; laser_number < velodyne_message->number_of_32_laser_shots; laser_number++, dt += time_spent_by_each_scan)
    {
        //Car position when velodyne message is received
        car_interpolated_position = carmen_ackerman_interpolated_robot_position_at_time(
                    car_fused_pose,
                    dt,
                    car_fused_velocity,
                    car_phi,
                    distance_between_front_and_rear_axles);

        //move car to correct global coordinates
        compute_rotation_matrix(&car_to_global_matrix, car_interpolated_position.orientation);

        // the current vertical shot
        carmen_velodyne_32_laser_shot &laser_shot(velodyne_message->partial_scan[laser_number]);

        // get the current horizontal angle
        double h_angle = carmen_normalize_theta(-carmen_degrees_to_radians(laser_shot.angle));

        for (int vertical_position = 0; vertical_position < N_LASER_POINTS; vertical_position++)
        {
            if (laser_shot.distance[vertical_position] == 0)
            {
                range_max_points++;
                continue;
            }

            unsigned index = laser_number * (N_LASER_POINTS) + vertical_position - range_max_points; 

            // get the vertical angle (hardcode)
            double v_angle = carmen_normalize_theta(carmen_degrees_to_radians(vertical_correction[vertical_position]));

            // get the current beam range
            double range = double(laser_shot.distance[vertical_position]) * 0.002; //hardcode 0.002 ???

            // convert velodyne to cartesian coordinates
            carmen_vector_3D_t velodyne_point = spherical_to_cartesian_coordinates(h_angle, v_angle, range);

            carmen_vector_3D_t point = move_velodyne_point_to_car_reference(velodyne_point,
                                                                            velodyne_to_board_matrix,
                                                                            board_to_car_matrix, 
                                                                            velodyne_pose.position, 
                                                                            sensor_board_1_pose.position);
            update_map(point, velodyne_message, laser_number, vertical_position);
        }
    }

    // when points are computed, create bitmaps
    char dataset_name[1024];
    char* path_to_save = "/dados/volta_da_ufes17102018/";

    sprintf(dataset_name,"%s%lf_num", path_to_save, velodyne_message->timestamp);
    show_map(&raw_number_of_lasers_map, dataset_name);
    memset (&dataset_name, 0, sizeof (dataset_name) );
    sprintf(dataset_name,"%s%lf_mean", path_to_save, velodyne_message->timestamp);
    show_map(&raw_mean_hight_map, dataset_name);
    memset (&dataset_name, 0, sizeof (dataset_name) );
    sprintf(dataset_name,"%s%lf_max", path_to_save, velodyne_message->timestamp);
    show_map(&raw_max_hight_map, dataset_name);
    memset (&dataset_name, 0, sizeof (dataset_name) );
    sprintf(dataset_name,"%s%lf_min", path_to_save, velodyne_message->timestamp);
    show_map(&raw_min_hight_map, dataset_name);
    memset (&dataset_name, 0, sizeof (dataset_name) );
    sprintf(dataset_name,"%s%lf_std", path_to_save, velodyne_message->timestamp);
    show_map(&raw_hight_standard_deviation_map, dataset_name);
    memset (&dataset_name, 0, sizeof (dataset_name) );
    sprintf(dataset_name,"%s%lf_intensity", path_to_save, velodyne_message->timestamp);
    show_map(&raw_reflectivity_map, dataset_name);
}

void update_map_from_kitti(carmen_vector_3D_t p, float remission){

	int x_index = (int)(p.x/map_resolution);
    int y_index = (int)(p.y/map_resolution);

    // transform x and y positions to map reference
    int transform_x_index = (int) x_index;//(int)(raw_reflectivity_map.config.x_origin + x_index);
    int transform_y_index = (int)(raw_reflectivity_map.config.y_origin + y_index);

    // from the top to the bottom
    // check if the positions are outside the map area
    if(transform_x_index >= n_x_pixels || transform_x_index < 0 || transform_y_index >= n_y_pixels || transform_y_index < 0){
		return;
    }

    // variables to save last values
	double last_variance;
	if(raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index] == -1)
		last_variance = 0;
    else{
    	last_variance = carmen_square(raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index]);
    }

 	double last_mean;   
	if(raw_mean_hight_map.map[transform_x_index][transform_y_index] == -1)
		last_mean = 0;
    else{
    	last_mean = raw_mean_hight_map.map[transform_x_index][transform_y_index];
    }
    
    // update number of points
    if(raw_number_of_lasers_map.map[transform_x_index][transform_y_index]==-1){
    	raw_number_of_lasers_map.map[transform_x_index][transform_y_index] = 1;
    }
    else{
	    raw_number_of_lasers_map.map[transform_x_index][transform_y_index]++;
    }
    int n = raw_number_of_lasers_map.map[transform_x_index][transform_y_index];


    // update min and max values
	if(n == 1){
		raw_min_hight_map.map[transform_x_index][transform_y_index] = p.z;
		raw_max_hight_map.map[transform_x_index][transform_y_index] = p.z;		
	}
	else if(p.z < raw_min_hight_map.map[transform_x_index][transform_y_index]){
	    raw_min_hight_map.map[transform_x_index][transform_y_index] = p.z;
	}
	else if(p.z > raw_max_hight_map.map[transform_x_index][transform_y_index]){
	    raw_max_hight_map.map[transform_x_index][transform_y_index] = p.z;
	}
	
	// update mean
	if(n==1){
		raw_mean_hight_map.map[transform_x_index][transform_y_index] = p.z;
	}
	else{
		raw_mean_hight_map.map[transform_x_index][transform_y_index] = ((raw_mean_hight_map.map[transform_x_index][transform_y_index]*(n-1)) + p.z)/(n);
	}

	if(n > 2){
		double new_variance = (((n-2)/(n-1))*last_variance) + (1/n)*(raw_mean_hight_map.map[transform_x_index][transform_y_index] - last_mean);
		raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index] = sqrt(new_variance);
	}
	else{
		raw_hight_standard_deviation_map.map[transform_x_index][transform_y_index] = 0;		
	}

	// update reflectivity
	double newRefleflectivity = (double)remission;
	if(n==1){
		raw_reflectivity_map.map[transform_x_index][transform_y_index] = newRefleflectivity;
	}
	else{
		double last_meanReflectivity = raw_reflectivity_map.map[transform_x_index][transform_y_index];
		raw_reflectivity_map.map[transform_x_index][transform_y_index] = ((last_meanReflectivity*(n-1))+newRefleflectivity)/n;
	}
}

void read_kitti_velodyne_and_update_maps(char *dir, char* data_kitti_type, int file_id)
{
	FILE *stream;
	double x, y, z, r;
	static char filename[1024];


	sprintf(filename, "%s/%s_%06d.bin", dir, data_kitti_type, file_id);
	printf("%s\n", filename);


	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	int32_t num = 1000000;
	float* data = (float*) malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;

	stream = fopen(filename, "rb");
	if(stream != NULL){
		printf("CORRETO\n");
	}

	num = fread(data, sizeof(float), num, stream);
	printf("%d\n", num);
	num = num / 4;

// preenchimento
	for (int i = 0; i < num; i++)
	{

		x = *px;
		y = *py;
		z = *pz;
		r = *pr;
		carmen_vector_3D_t point;
		point.x = x;
		point.y = y;
		point.z = z;

		update_map_from_kitti(point, r);

		//cloud->points[i].rgba = *pr;
		px += 4;
		py += 4;
		pz += 4;
		pr += 4;
	}

	//printf("max: %lf min: %lf diff: %lf\n", max, min,max-min);
	free(data);
	fclose(stream);
}

void writeCSV(string filename, Mat image)
{
   ofstream myfile;
   myfile.open(filename.c_str());

	for(int j = 0;j < image.rows;j++){
    	for(int i = 0;i < image.cols;i++){
	 		myfile << to_string(image.at<uchar>(j,i));
	 		if(i < image.cols-1)
	 			myfile << ", ";
	 	}
	 	if(j < image.rows-1)
		 	myfile << std::endl;
	}

   myfile.close();
}

void
png2csv(char* output_name)
{
	static char output_png_name[100];
	sprintf(output_png_name, "%s.png", output_name);
	printf("%s\n", output_png_name);
	static char output_csv_name[100];
	sprintf(output_csv_name, "%s.csv", output_name);

    Mat image = imread(output_png_name, CV_8UC1);
    if(! image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return ;
    }

	// concerta ground truth do Luca 1 ==> pista
			
	for(int j = 0;j < image.rows;j++){
    	for(int i = 0;i < image.cols;i++){

        	if(image.at<uchar>(j,i) == 3){
        		image.at<uchar>(j,i) = 0;
        	}
        	else if(image.at<uchar>(j,i) == 2)
        		image.at<uchar>(j,i) = 0;
        	else
        		image.at<uchar>(j,i) = 1;
    	}
	}

	writeCSV(output_csv_name, image);
}

void 
generate_inputs_and_output_pngs_from_kitti_files(char *dir, char* data_kitti_type)
{
	int size = 0;
	if(strcmp(data_kitti_type, "uu") == 0){
		size = kitti_uu_training_size;
	}
	else if(strcmp(data_kitti_type, "umm") == 0){
		size = kitti_umm_training_size;
	}
	else if(strcmp(data_kitti_type, "um") == 0){
		size = kitti_um_training_size;
	}
	else{
		printf("Nao existe esse tipo do KITTI: Apenas uu, umm ou um\n");
		return;
	}
	for(int i = 0; i < size; i++){
		/*
		read_kitti_velodyne_and_update_maps(dir, data_kitti_type, i);

		static char n_points_name[50];
		sprintf(n_points_name, "KITTI_dataset/%s/number_of_points/%s_%06d", data_kitti_type,data_kitti_type, i);

		static char mean_hight_name[50];
		sprintf(mean_hight_name, "KITTI_dataset/%s/mean_hight/%s_%06d", data_kitti_type,data_kitti_type, i);

		static char max_hight_name[50];
		sprintf(max_hight_name, "KITTI_dataset/%s/max_hight/%s_%06d", data_kitti_type,data_kitti_type, i);

		static char min_hight_name[50];
		sprintf(min_hight_name, "KITTI_dataset/%s/min_hight/%s_%06d", data_kitti_type,data_kitti_type, i);

		static char std_name[50];
		sprintf(std_name, "KITTI_dataset/%s/standard_deviation/%s_%06d", data_kitti_type,data_kitti_type, i);

		static char reflection_name[50];
		sprintf(reflection_name, "KITTI_dataset/%s/mean_reflection/%s_%06d", data_kitti_type,data_kitti_type, i);

		// when points are computed, create bitmaps and csv files
	    show_map(&raw_number_of_lasers_map, n_points_name);
	    show_map(&raw_mean_hight_map, mean_hight_name);
	    show_map(&raw_max_hight_map, max_hight_name);
	    show_map(&raw_min_hight_map, min_hight_name);
	    show_map(&raw_hight_standard_deviation_map, std_name);
	    show_map(&raw_reflectivity_map, reflection_name);
	    */
		static char output_name[50];
	    //create from KITTI bird's eye view dataset
	    //sprintf(output_name, "KITTI_dataset/%s/ground_truth/%s_%06d", data_kitti_type,data_kitti_type, i);
		// create from ground truth given by Luca 
		sprintf(output_name, "/home/parallels/Documents/road-detection-neural-network/Dataset/Dataset/%s_%06d_gt", data_kitti_type, i);

	    // creates csv outputs from marked png grund truth images
	    png2csv(output_name);
	}
}

void
compute_mapper_map(carmen_mapper_map_message *mapper_msg)
{
	if (raw_label_map.complete_map == NULL)
	{
		carmen_grid_mapping_create_new_map(&raw_label_map, mapper_msg->config.x_size, mapper_msg->config.y_size, map_resolution, 'u');
	}

    char dataset_name[1024];
    char* path_to_save = "/dados/volta_da_ufes17102018/";

    sprintf(dataset_name,"%s%lf_label", path_to_save, mapper_msg->timestamp);


	for (int i = 0; i < raw_label_map.config.y_size;i++){
		for(int j = 0; j < raw_label_map.config.x_size; j++){
			raw_label_map.map[i][j] = mapper_msg->complete_map[i*mapper_msg->config.y_size + j];
//			printf("%lf", raw_label_map.map[i][j]);
		}
//		printf("\n");
	}

    show_map(&raw_label_map, dataset_name);	
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
carmen_mapper_map_message_handler(carmen_mapper_map_message *message)
{
	if(mapper_timestamp <= velodyne_timestamp)
	{
		mapper_timestamp = message->timestamp;
		if(mapper_timestamp == velodyne_timestamp)
		{
			compute_mapper_map(message);
			printf("COMPUTOU MAPPER %lf ||||| %lf\n", velodyne_timestamp, mapper_timestamp);
		}
	}
}

void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *message)
{
	if(velodyne_timestamp <= mapper_timestamp)
	{
		velodyne_timestamp = message->timestamp;
		compute_velodyne_points(message);
		printf("COMPUTOU VELODYNE %lf ||||| %lf\n", velodyne_timestamp, mapper_timestamp);
	}	
}

void
base_ackerman_odometry_message_handler(carmen_base_ackerman_odometry_message *message)
{
	car_fused_time = message->timestamp;
	car_fused_velocity = message->v;
	car_phi = message->phi;
}

void
subscribe_messages()
{

	carmen_mapper_subscribe_map_message(NULL,
			(carmen_handler_t) carmen_mapper_map_message_handler, CARMEN_SUBSCRIBE_LATEST);

	carmen_velodyne_subscribe_partial_scan_message(NULL,
										 (carmen_handler_t) velodyne_partial_scan_message_handler,
										 CARMEN_SUBSCRIBE_LATEST);

	carmen_base_ackerman_subscribe_odometry_message(NULL,
													(carmen_handler_t) base_ackerman_odometry_message_handler,
													CARMEN_SUBSCRIBE_LATEST);
}

void
read_parameters(int argc, char **argv)
{
	carmen_param_t param_list[] = {
		{(char*) "sensor_board_1", (char*) "x", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.x), 0, NULL},
	    {(char*) "sensor_board_1", (char*) "y", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.y), 0, NULL},
	    {(char*) "sensor_board_1", (char*) "z", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.position.z), 0, NULL},
	    {(char*) "sensor_board_1", (char*) "roll", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.roll), 0, NULL},
	    {(char*) "sensor_board_1", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.pitch), 0, NULL},
	    {(char*) "sensor_board_1", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(sensor_board_1_pose.orientation.yaw), 0, NULL},
	    {(char*) "velodyne", (char*) "x", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.x), 0, NULL},
	    {(char*) "velodyne", (char*) "y", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.y), 0, NULL},
	    {(char*) "velodyne", (char*) "z", CARMEN_PARAM_DOUBLE, &(velodyne_pose.position.z), 0, NULL},
	    {(char*) "velodyne", (char*) "roll", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.roll), 0, NULL},
	    {(char*) "velodyne", (char*) "pitch", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.pitch), 0, NULL},
	    {(char*) "velodyne", (char*) "yaw", CARMEN_PARAM_DOUBLE, &(velodyne_pose.orientation.yaw), 0, NULL},
	    {(char*) "robot", (char*) "distance_between_front_and_rear_axles", CARMEN_PARAM_DOUBLE, &(distance_between_front_and_rear_axles), 0, NULL},
	    {(char*) "velodyne", (char*) "time_spent_by_each_scan", CARMEN_PARAM_DOUBLE, &(time_spent_by_each_scan), 0, NULL}

	};

	carmen_param_install_params(argc, argv, param_list, sizeof(param_list) / sizeof(param_list[0]));
	
	if(argc < 5){
		carmen_die("%s: Wrong number of parameters. This module requires 4 parameter and received %d parameter(s). \nUsage:\n %s <map_length> <map_width> <map_resolution> <number_of_velodyne_point_clouds> \n",
				argv[0], argc - 1, argv[0]);
	}
	else{
		map_x_size = atoi(argv[1]);
		map_y_size = atoi(argv[2]);
		map_resolution = atof(argv[3]);
		point_clouds_step_size = atof(argv[4]);
		n_x_pixels = (int)map_x_size/map_resolution;
		n_y_pixels = (int)map_y_size/map_resolution;

		if(argc == 6){
			use_kitti = 1;
		}
	}
}

void generate_maps(){
	carmen_grid_mapping_create_new_map(&raw_mean_hight_map, n_x_pixels, n_y_pixels, map_resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_max_hight_map, n_x_pixels, n_y_pixels, map_resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_min_hight_map, n_x_pixels, n_y_pixels, map_resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_hight_standard_deviation_map, n_x_pixels, n_y_pixels, map_resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_reflectivity_map, n_x_pixels, n_y_pixels, map_resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_number_of_lasers_map, n_x_pixels, n_y_pixels, map_resolution, 'u');
	carmen_grid_mapping_create_new_map(&raw_square_sum_map, n_x_pixels, n_y_pixels, map_resolution, 'u');

    int center_pos_x = (int)(n_x_pixels/2);
    int center_pos_y = (int)(n_y_pixels/2);

    raw_reflectivity_map.config.x_origin = center_pos_x;
    raw_mean_hight_map.config.x_origin = center_pos_x;
    raw_max_hight_map.config.x_origin = center_pos_x;
    raw_min_hight_map.config.x_origin = center_pos_x;
    raw_number_of_lasers_map.config.x_origin = center_pos_x;
    raw_hight_standard_deviation_map.config.x_origin = center_pos_x;
    raw_square_sum_map.config.x_origin = center_pos_x;

    raw_reflectivity_map.config.y_origin = center_pos_y;
    raw_mean_hight_map.config.y_origin = center_pos_y;
    raw_max_hight_map.config.y_origin = center_pos_y;
    raw_min_hight_map.config.y_origin = center_pos_y;
    raw_number_of_lasers_map.config.y_origin = center_pos_y;
    raw_hight_standard_deviation_map.config.y_origin = center_pos_y;
    raw_square_sum_map.config.y_origin = center_pos_y;

	set_map_values_to_zero(&raw_square_sum_map);
}

void destroy_maps(){
	// TODO: destruir carmen_map_t's
}

int main(int argc, char **argv){
	/* connect to IPC server */
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

	// generates the statistics maps, based on the parametters passed on the command-line
	generate_maps();

	if(use_kitti == 0){

		velodyne_to_board_matrix = create_rotation_matrix(velodyne_pose.orientation);
	    board_to_car_matrix = create_rotation_matrix(sensor_board_1_pose.orientation);

		subscribe_messages();

		/* Loop forever waiting for messages */
		carmen_ipc_dispatch();

		destroy_rotation_matrix(velodyne_to_board_matrix);
		destroy_rotation_matrix(board_to_car_matrix);
	}
	else{
		generate_inputs_and_output_pngs_from_kitti_files("/home/parallels/Documents/data_road_velodyne/training/velodyne", "um");
		generate_inputs_and_output_pngs_from_kitti_files("/home/parallels/Documents/data_road_velodyne/training/velodyne", "umm");
		generate_inputs_and_output_pngs_from_kitti_files("/home/parallels/Documents/data_road_velodyne/training/velodyne", "uu");

	}
	destroy_maps();
	return 0;
}
