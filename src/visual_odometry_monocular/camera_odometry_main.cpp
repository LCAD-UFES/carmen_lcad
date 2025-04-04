#include "camera_odomery.h"
#include <cmath>
#include "BigFloat.h"
#include <vector>

#include <carmen/tf.h>

using namespace tf;
char *log_filename = NULL;
int initial_pose_found = 0;
// As variáveis abaixo servem para fornecer as coordenadas iniciais do mapa. Existe um bug no mapper que, quando a origem do x é 0, a construção do mapa fica cortada nesses trechos. Para evitar isso adiciono os valores abaixo nas posições
double initial_x = 7757721.8;
double initial_y = -363569.5;


std::vector<std::string>
string_split(std::string s, std::string pattern)
{
	std::vector<std::string> splitted, splitted_without_empties;

	boost::split(splitted, s, boost::is_any_of(pattern));

	for (unsigned int i = 0; i < splitted.size(); i++)
	{
		if (splitted[i].size() > 0)
			splitted_without_empties.push_back(splitted[i]);
	}

	return splitted_without_empties;
}

carmen_gps_xyz_message
read_gps(std::string line, int gps_to_use)
{
	//gps id 0 lt_dm lt_orientation lg_dm lg_orientation 0 0 0 sea_level 0 0 0 0 0
	std::vector<std::string> splitted_string = string_split(line, " ");

	double lt_dm, lt, lg_dm, lg, sea_level;
	char lt_orientation, lg_orientation;
	int quality, gps_id;

	carmen_gps_xyz_message m;
	memset(&m, 0, sizeof(m));

	gps_id = std::stoi(splitted_string[1].c_str());
	m.nr = gps_id;

	if (gps_to_use == gps_id)
	{
		//gps id 0 lt_dm lt_orientation lg_dm lg_orientation 0 0 0 sea_level 0 0 0 0 0
		lt_dm = 			std::stod(splitted_string[3].c_str());
		lt_orientation = 	      	  splitted_string[4].c_str()[0];
		lg_dm = 			std::stod(splitted_string[5].c_str());
		lg_orientation = 			  splitted_string[6].c_str()[0];
		sea_level = 		std::stod(splitted_string[10].c_str());

		lt = carmen_global_convert_degmin_to_double(lt_dm);
		lg = carmen_global_convert_degmin_to_double(lg_dm);

		// verify the latitude and longitude orientations
		if ('S' == lt_orientation) lt = -lt;
		if ('W' == lg_orientation) lg = -lg;

		// convert to x and y coordinates
		Gdc_Coord_3d gdc = Gdc_Coord_3d(lt, lg, sea_level);

		// Transformando o z utilizando como altitude a altitude mesmo - que esta vindo como zero
		Utm_Coord_3d utm;
		Gdc_To_Utm_Converter::Init();
		Gdc_To_Utm_Converter::Convert(gdc , utm);

		m.x = utm.y;
		m.y = -utm.x;
	}

	return (m);
}

void
initialize_car_objects_poses_in_transformer(Transformer &tf_transformer, double x_gps_init, double y_gps_init, double yaw_gps, double x_odometry_init, double y_odometry_init, double yaw_odometry)
{


	tf_transformer.setTransform(tf::StampedTransform(Transform(Quaternion(yaw_gps, 0, 0), Vector3(x_gps_init, y_gps_init, 0)), tf::Time(0), "/world", "/gps"));
	tf_transformer.setTransform(tf::StampedTransform(Transform(Quaternion(yaw_odometry, 0, 0), Vector3(x_odometry_init, y_odometry_init, 0)), tf::Time(0), "/world", "/odometry"));
}


std::vector<std::vector<double>>
read_data_from_log(char *log_file)
{

	std::string line;
	// the input file stream
    std::ifstream logfile(log_file);

    if (!logfile.is_open())
    {
        std::cerr << "Unable to open the input file: " << log_file << std::endl;
        exit(-1);
    }
    log_filename = log_file;
	std::vector<std::vector<double>> poses_gps;
	std::vector<double> aux_gps;
	FILE *poses_gps_file = fopen("poses_gps.txt", "a");
	BigFloat x_sum (0.);
	BigFloat y_sum (0.);
	while (std::getline(logfile, line))
	{
		// the input tag
		std::string tag = line.substr(0, line.find(" "));

		

		if ("NMEAGGA" == tag && initial_pose_found == 0)
		{
			carmen_gps_xyz_message m = read_gps(line, 1);
			initial_x = m.x;
			initial_y = m.y;
			if (m.x != 0.0 && m.y != 0.0)
				fprintf(poses_gps_file, "%lf %lf\n", m.x, m.y);
			initial_pose_found = 1;
			aux_gps.push_back(m.x);
			aux_gps.push_back(m.y);
			poses_gps.push_back(aux_gps);
		}
		else
		{
			if ("NMEAGGA" == tag)
			{
				carmen_gps_xyz_message m = read_gps(line, 1);
				
				aux_gps.push_back(m.x);
				aux_gps.push_back(m.y);
				if (m.x != 0.0 && m.y != 0.0)
					fprintf(poses_gps_file, "%lf %lf\n", m.x, m.y);
				poses_gps.push_back(aux_gps);
				x_sum = x_sum + m.x;
				y_sum = y_sum + m.y;
				//std::cout << y_sum.ToString() << std::endl;
			}
		}
    }
	fclose(poses_gps_file);
	double centroid_x_gps, centroid_y_gps ;
	double yaw_gps = atan2(poses_gps[1][0], poses_gps[1][1]);
	yaw_gps = yaw_gps + M_PI / 6.0; //+ M_PI / 4.0 ;
	std::cout << yaw_gps << " " << poses_gps[1][1] << " " << poses_gps[1][0] << std::endl;
	printf("aaaa %lf %lf\n", centroid_x_gps, centroid_y_gps);
	logfile.close();
    std::ifstream poses_file("/media/lume/f3bce44c-b721-4f2d-8fd6-91d76b9581b5/voldor-easyuse/saida.txt");
    std::getline(poses_file, line);
    std::vector<std::vector<double>> poses_vector;
	std::vector<double> aux_vector;
	FILE *poses_file_write = fopen("poses_file.txt", "a");
	int j = 0;
	double yaw_odometry = atan2(-0.9450341 + 0.93798876, -3.5824656 + 3.594625);
	bool already_computed = false;
	double x_odometry_init, y_odometry_init;
	Transformer tf_transformer;
    while (std::getline(poses_file, line))
    {
        std::vector<std::string> splitted_string = string_split(line, " ");
        double x, y;
        
		//x = (double(15. * std::stod(splitted_string[1])) * sin(100 * M_PI / 180.)  + double(15. * std::stod(splitted_string[2])) * cos(100 * M_PI / 180.)) + initial_x;
		//y = (-1 * double(15. * std::stod(splitted_string[2])) * sin(100 * M_PI / 180.) + double(15. * std::stod(splitted_string[1])) * cos( 100 * M_PI / 180.))  + initial_y;
		if (j >= 149 )
		{	
			x = std::stod(splitted_string[1]);
			y = std::stod(splitted_string[2]);
			if(!already_computed)
			{
				x_odometry_init = x;
				y_odometry_init = y;

				initialize_car_objects_poses_in_transformer(tf_transformer, initial_x, initial_y, yaw_gps, x_odometry_init, y_odometry_init, yaw_odometry);
				already_computed = true;
			}
			x = x * -15;//-15.5;//-20;
			y = y * 150;//155;//60;
			x = x - 50; //-100;
			y =y + 300;
			tf::StampedTransform a_to_b;
			tf_transformer.lookupTransform("/world", "/gps", tf::Time(0), a_to_b);

			tf::Point point_odometry_tf = tf::Point(x, y, 0.0);
			tf::Point reference_adjusted = a_to_b * point_odometry_tf;
			//x = x  * sin(100 * M_PI / 180.) + y * cos(100 * M_PI / 180.);
			//y = x * cos( 100 * M_PI / 180.) - y * sin(100 * M_PI / 180.);

			x = reference_adjusted.x();
			y = reference_adjusted.y();
			//x = x + 7.75735e+06;
			//y = y + -363846;
			//std::cout << yaw_gps;
			//printf("%lf\n", x);
			fprintf(poses_file_write, "%lf %lf\n", x, y);
			aux_vector.push_back(x);
			aux_vector.push_back(y);
			poses_vector.push_back(aux_vector);
		}
		j++;
		//resize
		//x = 30 * x;
		//y = 30 * y;
		//rotation
		//x = x  * sin(100 * M_PI / 180.) + y * cos(100 * M_PI / 180.);
		//y = x * cos( 100 * M_PI / 180.) - y * sin(100 * M_PI / 180.);
		//translation
		//x = x + initial_x;
		//y = y + initial_y;
		
		//x = x  * sin(100 * M_PI / 180.) + y * cos(100 * M_PI / 180.);
		//y = x * cos( 100 * M_PI / 180.) - y * sin(100 * M_PI / 180.);
		



		
		
		
		//fprintf(poses_file_write, "%lf %lf\n", x, y);
       
		
    }
	//x_sum = y_sum = 0.;
	/*double x_sum_slam = 0., y_sum_slam = 0.;
	double centroid_x_slam = 0.; 
	double centroid_y_slam = 0.;
	
	for(uint i = 0 ; i < poses_vector.size(); i++)
	{
		x_sum_slam = x_sum_slam + poses_vector[i][0];
		y_sum_slam = y_sum_slam + poses_vector[i][1];
	}
	centroid_x_slam = x_sum_slam / poses_vector.size();
	centroid_y_slam = y_sum_slam / poses_vector.size();
	centroid_x_gps = 7.75735e+06; //+ 0.00010e+06;
	//centroid_y_gps = -363785; //+50;
	//centroid_x_gps = 7.75639e+06;
	centroid_y_gps = -363746;
	//centroid_x_gps = 7.75639e+06;
	for(uint i = 0 ; i < poses_vector.size(); i++)
	{
		double r = sqrt(pow((poses_vector[i][0] - centroid_x_slam), 2)  + pow((poses_vector[i][1] - centroid_y_slam), 2));
		double theta = atan2(poses_vector[i][1] - centroid_y_slam , poses_vector[i][0] - centroid_x_slam);
		double x_slam = centroid_x_gps + r * cos(theta);
		
		double y_slam = centroid_y_gps + r * sin(theta);
		fprintf(poses_file_write, "%lf %lf\n", x_slam, y_slam);
		
	}


	printf("%lf %lf\n",centroid_x_gps, centroid_y_gps );*/
	fclose(poses_file_write);
	return poses_vector;
}

int
main(int argc, char **argv)
{
	std::vector<std::vector<double>> poses_vector = read_data_from_log(argv[1]);
	/*
	for(unsigned int i = 0; i < poses_vector.size(); i++)
	{
		printf("%lf %lf\n", poses_vector[i][0], poses_vector[i][1]);
	}
	*/
	return (0);
}
