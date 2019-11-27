
#include <cmath>
#include <vector>
#include <algorithm>
#include <carmen/carmen.h>
#include <carmen/carmen_stdio.h>
#include <carmen/fused_odometry_messages.h>
#include <carmen/readlog.h>
#include <carmen/rddf_messages.h>
#include "rddf_index.h"

/*
 * ****************************************
 * ATENCAO: Modulo precisa de refatoracao!
 *
 * - Acho que os indices de pose nao sao usados
 * mais.
 * - Checar quais funcionalidades precisam ser
 * feitas publicas e que podem ser uteis para
 * modulos futuros.
 *****************************************
 */

using namespace std;

int global_dim_to_sort_and_search = -1;

carmen_pose_index carmen_pose_index_ordered_by_x;
carmen_pose_index carmen_pose_index_ordered_by_y;
carmen_timestamp_index carmen_index_ordered_by_timestamp;

// @filipe: ao descomentar as linhas abaixo, o programa comeca a dar seg fault. investigar por que.
// se encontrar a resposta, me conte. nao consegui entender.
//bool index_is_sorted;
//vector<carmen_timestamp_index_element> index;

template<class T>
long get_min_dist_index (vector<T> vect, long upper, long lower, long middle, T &key, double dist(T, T))
{
	double dist_to_upper = dist (key, vect[upper]);
	double dist_to_lower = dist (key, vect[lower]);
	double dist_to_middle = dist(key, vect[middle]);

	if (dist_to_upper < dist_to_lower && dist_to_upper < dist_to_middle)
		return upper;
	else if (dist_to_lower < dist_to_middle)
		return lower;
	else
		return middle;
}


template<class T>
long binary_search_nearest (vector<T> vect, T key, int cmp_function(T, T), double dist (T, T))
{
	char is_first_iteration = 1;
	long upper, lower, new_middle = 0 , old_middle = 0;

	lower = 0;
	upper = vect.size() - 1;

	if (cmp_function(vect[upper], key) < 0)
		return upper;

	if (cmp_function(key, vect[lower]) <= 0)
		return lower;

	while (lower < upper)
	{
		new_middle = (lower + upper) / 2;

		if (!is_first_iteration)
		{
			if (old_middle == new_middle)
				// it's important because we're finding the nearest, not the equals
				return get_min_dist_index(vect, upper, lower, new_middle, key, dist);
		}
		else
			is_first_iteration = 0;

		if (cmp_function(key, vect[new_middle]) < 0)
			upper = new_middle;
		else
			lower = new_middle;

		old_middle = new_middle;
	}

	// it's important because we're finding the nearest, not the equals
	return get_min_dist_index(vect, upper, lower, new_middle, key, dist);
}


double& carmen_pose_index_element::operator[](int position)
{
	if (position < 0 || position > 6)
		exit(printf("Error: trying to access an invalid area!\n"));

	switch (position)
	{
	case 0:
		return x;
		break;
	case 1:
		return y;
		break;
	case 2:
		return z;
		break;
	case 3:
		return roll;
		break;
	case 4:
		return pitch;
		break;
	case 5:
		return yaw;
		break;
	case 6:
		return timestamp_index_position;
	default:
		break;
	}

	exit(printf("Error: carmen_pose_index_element::operator[]: Trying to access invalid position\n"));
}


double& carmen_timestamp_index_element::operator[](int position)
{
	if (position < 0 || position > 6)
		exit(printf("Error: trying to access an invalid area!\n"));

	switch (position)
	{
		case 0:
			return x;
			break;
		case 1:
			return y;
			break;
		case 2:
			return z;
			break;
		case 3:
			return roll;
			break;
		case 4:
			return pitch;
			break;
		case 5:
			return yaw;
			break;
		case 6:
			return timestamp;
		default:
			break;
	}

	exit(printf("Error: carmen_timestamp_index_element::operator[]: Trying to access invalid position\n"));
	return x;
}


static int
cmp_timestamp (carmen_timestamp_index_element a, carmen_timestamp_index_element b)
{
	if (a.timestamp < b.timestamp) return -1;
	else if (a.timestamp == b.timestamp) return 0;
	else return 1;
}


static bool
cmp_timestamp_bool(carmen_timestamp_index_element a, carmen_timestamp_index_element b)
{
	if (a.timestamp < b.timestamp) return true;
	else return false;
}


static double
dist_between_timestamps(carmen_timestamp_index_element a, carmen_timestamp_index_element b)
{
	return sqrt(pow((a.timestamp - b.timestamp), 2));
}


static double
euclidean_dist(carmen_pose_index_element a, carmen_pose_index_element b)
{
	double dist =
		sqrt (
				pow(a.x - b.x, 2) +
				pow(a.y - b.y, 2)
		);

	return dist;
}


static int
cmp_pose(carmen_pose_index_element a, carmen_pose_index_element b)
{
	if (global_dim_to_sort_and_search == -1)
		exit(printf("Error: please set the dim to sort and search\n"));

	if (a[global_dim_to_sort_and_search] < b[global_dim_to_sort_and_search]) return -1;
	else if (a[global_dim_to_sort_and_search] == b[global_dim_to_sort_and_search]) return 0;
	else return 1;
}


static bool
cmp_pose_bool(carmen_pose_index_element a, carmen_pose_index_element b)
{
	if (global_dim_to_sort_and_search == -1)
		exit(printf("Error: please set the dim to sort and search\n"));

	if (a[global_dim_to_sort_and_search] < b[global_dim_to_sort_and_search]) return true;
	else return false;
}


carmen_pose_index::carmen_pose_index()
{
	dim_to_sort_and_search = -1;
	index_is_sorted = false;
}


carmen_pose_index::carmen_pose_index(int dim)
{
	if (dim < 0 || dim > 5)
		exit(printf("Error: trying sort and search invalid data!\n"));

	dim_to_sort_and_search = dim;
	index_is_sorted = false;
}


carmen_pose_index_element carmen_pose_index::operator[](uint position)
{
	if (position >= index.size())
		exit(printf("Error: trying to access invalid position in the pose index\n"));
	else
		return index[position];
}


void carmen_pose_index::set_dim_to_sort_and_search (int dim)
{
	dim_to_sort_and_search = dim;
}


void carmen_pose_index::add(double x, double y, double z, double roll, double pitch, double yaw, long carmen_timestamp_index_position)
{
	carmen_pose_index_element elem;

	elem.x = x;
	elem.y = y;
	elem.z = z;
	elem.roll = roll;
	elem.pitch = pitch;
	elem.yaw = yaw;
	elem.timestamp_index_position = carmen_timestamp_index_position;

	index.push_back(elem);
	index_is_sorted = false;
}


long carmen_pose_index::size()
{
	return index.size();
}


void carmen_pose_index::sort()
{
	global_dim_to_sort_and_search = dim_to_sort_and_search;

	std::sort(index.begin(), index.end(), cmp_pose_bool);
	index_is_sorted = true;
}


long carmen_pose_index::search(double x, double y)
{
	if (!index_is_sorted)
	{
		exit(printf("Trying to search an unordered index!\n"));
		return -1;
	}
	else
	{
		carmen_pose_index_element key;

		key.x = x;
		key.y = y;

		global_dim_to_sort_and_search = dim_to_sort_and_search;
		long p = binary_search_nearest<carmen_pose_index_element>(index, key, cmp_pose, euclidean_dist);

		return p;
	}
}


void carmen_pose_index::save_to_file(char *filename)
{
	if (!index_is_sorted)
		exit(printf("Error: trying save an unsorted index!\n"));

	if (dim_to_sort_and_search == -1)
		exit(printf("Error: trying to save index with invalid search dim!\n"));

	unsigned int i;

	FILE *fileptr = fopen(filename, "w");

	if (fileptr == NULL)
		exit(printf("Error: pose index file couldn't be created!\n"));

	fprintf(fileptr, "%d %d\n", (int) index.size(), dim_to_sort_and_search);

	for (i = 0; i < index.size(); i++)
	{
		fprintf(fileptr,
				"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
				index[i].x,
				index[i].y,
				index[i].z,
				index[i].roll,
				index[i].pitch,
				index[i].yaw,
				index[i].timestamp_index_position
		);
	}

	fclose(fileptr);
}


void carmen_pose_index::load_from_file(char *filename)
{
	unsigned int i, n;

	FILE *fileptr = fopen(filename, "r");

	if (fileptr == NULL)
		exit(printf("Error: pose index file couldn't be open to read!\n"));

	fscanf(fileptr, "%d %d\n", &n, &dim_to_sort_and_search);

	for (i = 0; i < n; i++)
	{
		carmen_pose_index_element elem;

		fscanf(fileptr,
				"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
				&(elem.x),
				&(elem.y),
				&(elem.z),
				&(elem.roll),
				&(elem.pitch),
				&(elem.yaw),
				&(elem.timestamp_index_position)
		);

		index.push_back(elem);
	}

	fclose(fileptr);
	index_is_sorted = true;
}


carmen_timestamp_index::carmen_timestamp_index()
{
	index_is_sorted = false;
}

carmen_timestamp_index_element carmen_timestamp_index::operator[](uint position)
{
	if (position >= index.size())
		exit(printf("Error: trying to access invalid position in the timestamp index\n"));
	else
		return index[position];
}

void carmen_timestamp_index::add(
	double x, double y, double z,
	double roll, double pitch, double yaw,
	double origin_x, double origin_y, double origin_z,
	double velocity_x, double velocity_y, double velocity_z,
	double angular_velocity_roll, double angular_velocity_pitch, double angular_velocity_yaw, double phi,
	double timestamp, long rddf_file_offset, long rddf_data_length, int annotation)
{
	carmen_timestamp_index_element elem;

	elem.x = x;
	elem.y = y;
	elem.z = z;

	elem.roll = roll;
	elem.pitch = pitch;
	elem.yaw = yaw;

	elem.origin_x = origin_x;
	elem.origin_y = origin_y;
	elem.origin_z = origin_z;

	elem.velocity_x = velocity_x;
	elem.velocity_y = velocity_y;
	elem.velocity_z = velocity_z;

	elem.angular_velocity_roll = angular_velocity_roll;
	elem.angular_velocity_pitch = angular_velocity_pitch;
	elem.angular_velocity_yaw = angular_velocity_yaw;

	elem.phi = phi;

	elem.timestamp = timestamp;
	elem.rddf_offset = rddf_file_offset;
	elem.rddf_data_length = rddf_data_length;
	elem.anottation = annotation;

	index.push_back(elem);
	index_is_sorted = false;
}

long carmen_timestamp_index::size()
{
	return index.size();
}

void carmen_timestamp_index::sort()
{
	std::sort(index.begin(), index.end(), cmp_timestamp_bool);
	index_is_sorted = true;
}

long carmen_timestamp_index::search(double timestamp)
{
	if (!index_is_sorted)
	{
		exit(printf("Trying to search an unordered index!\n"));
		return -1;
	}
	else
	{
		carmen_timestamp_index_element key;

		key.timestamp = timestamp;

		return binary_search_nearest(index, key, cmp_timestamp, dist_between_timestamps);
	}
}

void carmen_timestamp_index::print()
{
	for(uint i = 0; i < index.size(); i++)
	{
		printf("\t%lf %lf %lf %lf %lf %lf\n",
				index[i].x,
				index[i].y,
				index[i].z,
				index[i].roll,
				index[i].pitch,
				index[i].yaw
		);
	}
}

void carmen_timestamp_index::save_to_file(char *filename)
{
	if (!index_is_sorted)
		exit(printf("Error: trying save an unsorted index!\n"));

	unsigned int i;

	FILE *fileptr = fopen(filename, "w");

	if (fileptr == NULL)
		exit(printf("Error: pose index file couldn't be created!\n"));

	fprintf(fileptr, "%d\n", (int) index.size());

	for (i = 0; i < index.size(); i++)
	{
		fprintf(fileptr,
				"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%ld\t%ld\t%d\n",
				index[i].x,
				index[i].y,
				index[i].z,
				index[i].roll,
				index[i].pitch,
				index[i].yaw,
				index[i].origin_x,
				index[i].origin_y,
				index[i].origin_z,
				index[i].velocity_x,
				index[i].velocity_y,
				index[i].velocity_z,
				index[i].angular_velocity_roll,
				index[i].angular_velocity_pitch,
				index[i].angular_velocity_yaw,
				index[i].phi,
				index[i].timestamp,
				index[i].rddf_offset,
				index[i].rddf_data_length,
				index[i].anottation
		);
	}

	fclose(fileptr);
}

void carmen_timestamp_index::load_from_file(char *filename)
{
	unsigned int i, n;

	FILE *fileptr = fopen(filename, "r");

	if (fileptr == NULL)
		exit(printf("Error: pose index file couldn't be open to read!\n"));

	fscanf(fileptr, "%d\n", &n);

	for (i = 0; i < n; i++)
	{
		carmen_timestamp_index_element elem;

		fscanf(fileptr,
				"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%ld\t%ld\t%d\n",
				&(elem.x),
				&(elem.y),
				&(elem.z),
				&(elem.roll),
				&(elem.pitch),
				&(elem.yaw),
				&(elem.origin_x),
				&(elem.origin_y),
				&(elem.origin_z),
				&(elem.velocity_x),
				&(elem.velocity_y),
				&(elem.velocity_z),
				&(elem.angular_velocity_roll),
				&(elem.angular_velocity_pitch),
				&(elem.angular_velocity_yaw),
				&(elem.phi),
				&(elem.timestamp),
				&(elem.rddf_offset),
				&(elem.rddf_data_length),
				&(elem.anottation)
		);

		index.push_back(elem);
	}

	fclose(fileptr);
	index_is_sorted = true;
}


void carmen_write_index (char *rddf_filename)
{
	/** index filename is rddf_filename<index-extension>.index **/
	char rddf_index_x_filename [2048];
	char rddf_index_y_filename [2048];
	char rddf_index_timestamp_filename [2048];

	strcpy (rddf_index_x_filename, rddf_filename);
	strcpy (rddf_index_y_filename, rddf_filename);
	strcpy (rddf_index_timestamp_filename, rddf_filename);

	strcat (rddf_index_x_filename, ".x.index");
	strcat (rddf_index_y_filename, ".y.index");
	strcat (rddf_index_timestamp_filename, ".timestamp.index");

	/** index are sorted in the create function, so here we need just save them to file **/
	carmen_pose_index_ordered_by_x.save_to_file(rddf_index_x_filename);
	carmen_pose_index_ordered_by_y.save_to_file(rddf_index_y_filename);
	carmen_index_ordered_by_timestamp.save_to_file(rddf_index_timestamp_filename);
}


int
is_white_space (char c)
{
	if (c == ' ')
		return 1;
	if (c == '\n')
		return 1;
	if (c == '\t')
		return 1;
	if (c == '\r')
		return 1;

	return 0;
}


//void
//parse_data_and_add_to_timestamp_index (char *buffer, long data_offset, long data_length)
//{
//	int pos_buffer = 0;
//
//	// ignore empty characters at the beginning of lines
//	while (is_white_space (buffer[pos_buffer]))
//		pos_buffer ++;
//
//	// ignore comments
//	if (buffer[pos_buffer] == '#')
//		return;
//
//	// se for uma mensagem de nuvem de pontos com pose
//	if (strncmp(buffer + pos_buffer, "RDDF_FUSED_ODOMETRY ", 19) == 0)
//	{
//		carmen_fused_odometry_message *fused_odometry_message = (carmen_fused_odometry_message *) calloc (1, sizeof(carmen_fused_odometry_message));
//
//		carmen_logread_string_to_fused_odometry(buffer + pos_buffer, fused_odometry_message);
//
//		carmen_timestamp_index.add(
//			fused_odometry_message->pose.position.x, fused_odometry_message->pose.position.y, fused_odometry_message->pose.position.z,
//			fused_odometry_message->pose.orientation.roll, fused_odometry_message->pose.orientation.pitch, fused_odometry_message->pose.orientation.yaw,
//			fused_odometry_message->gps_position_at_turn_on.x, fused_odometry_message->gps_position_at_turn_on.y, fused_odometry_message->gps_position_at_turn_on.z,
//			fused_odometry_message->velocity.x, fused_odometry_message->velocity.y, fused_odometry_message->velocity.z,
//			fused_odometry_message->angular_velocity.roll, fused_odometry_message->angular_velocity.pitch, fused_odometry_message->angular_velocity.yaw,
//			fused_odometry_message->phi, fused_odometry_message->timestamp,
//			data_offset, data_length
//		);
//
//		free (fused_odometry_message->host);
//		free (fused_odometry_message);
//	}
//}


void
carmen_create_pose_index_from_timestamp_index()
{
	/**
	 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 * @IMPORTANT: the timestamp index MUST be sorted
	 * here!!! The x and y pose index has pointers to
	 * positions in the timestamp index. If you add more
	 * elements to the timestamp index and re-sort, the
	 * pose index will became inconsistent.
	 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 */

	double x, y, z, roll, pitch, yaw;

	for(int i = 0; i < carmen_index_ordered_by_timestamp.size(); i++)
	{
		x = carmen_index_ordered_by_timestamp[i].x;
		y = carmen_index_ordered_by_timestamp[i].y;
		z = carmen_index_ordered_by_timestamp[i].z;
		roll = carmen_index_ordered_by_timestamp[i].roll;
		pitch = carmen_index_ordered_by_timestamp[i].pitch;
		yaw = carmen_index_ordered_by_timestamp[i].yaw;

		carmen_pose_index_ordered_by_x.add (x, y, z, roll, pitch, yaw, i);
		carmen_pose_index_ordered_by_y.add (x, y, z, roll, pitch, yaw, i);
	}

	carmen_pose_index_ordered_by_x.set_dim_to_sort_and_search(0 /* x dim */);
	carmen_pose_index_ordered_by_y.set_dim_to_sort_and_search(1 /* y dim */);

	carmen_pose_index_ordered_by_x.sort();
	carmen_pose_index_ordered_by_y.sort();
}


//void
//carmen_rddf_create_index_from_rddf_log (char *rddf_filename)
//{
//	FILE *rddf_file = fopen (rddf_filename, "r");
//
//	if (rddf_file == NULL)
//		exit(fprintf(stderr, "rddf file \"%s\" nao pode ser encontrado\n", rddf_filename));
//
//	long offset_start = 0;
//	long offset_end = 0;
//	long file_length = 0;
//	long num_messages_indexed = 0;
//
//	const long MAX_LINE_LENGTH = 10000000; // max line length = 10 MB
//	char *line = (char*) calloc (MAX_LINE_LENGTH, sizeof(char));
//
//	struct stat stat_buf;
//	fstat(fileno(rddf_file), &stat_buf);
//	file_length = stat_buf.st_size;
//
//	fseek(rddf_file, 0L, SEEK_SET);
//
//	while (!feof (rddf_file))
//	{
//		offset_start = ftell (rddf_file);
//		fscanf (rddf_file, "\n%[^\n]\n", line);
//		offset_end = ftell (rddf_file);
//
//		// only EOF or '\n' characters has been read
//		if (strlen(line) <= 2)
//			continue;
//
//		parse_data_and_add_to_timestamp_index(line, offset_start, offset_end - offset_start);
//
//		if(num_messages_indexed % 500 == 0)
//			printf("\nIndexing messages (%.0lf%%)", ((float) offset_end / (float)file_length) * 100.0);
//
//		num_messages_indexed++;
//	}
//
//	free (line);
//
//	carmen_timestamp_index.sort();
//	carmen_create_pose_index_from_timestamp_index();
//	carmen_write_index (rddf_filename);
//
//	fclose (rddf_file);
//}


long
find_index_pos_with_correct_yaw(carmen_pose_index _carmen_pose_index, long index_pos, double yaw)
{
	int radius = 1;
	int p = index_pos;

	while ((p + radius < _carmen_pose_index.size()) || (p - radius >= 0))
	{
		if (p + radius < _carmen_pose_index.size())
		{
			if (fabs(carmen_normalize_theta(_carmen_pose_index[p + radius].yaw - yaw)) < (M_PI / 2.0))
			{
				 p += radius;
				 break;
			}
		}

		if (p - radius >= 0)
		{
			if (fabs(carmen_normalize_theta(_carmen_pose_index[p - radius].yaw - yaw)) < (M_PI / 2.0))
			{
				 p -= radius;
				 break;
			}
		}

		radius++;
	}

	return p;
}


long
find_nearest_point_around_point_found(carmen_pose_index index, long position, double x, double y)
{
	int radius = 1;
	carmen_pose_index_element key;

	key.x = x;
	key.y = y;

	while (((position - radius) >= 0) || ((position + radius) < index.size()))
	{
		if ((position - radius) >= 0)
		{
			if (euclidean_dist(key, index[position - radius]) < 5.0)
			{
				position = position - radius;
				break;
			}
		}

		if ((position + radius) < index.size())
		{
			if (euclidean_dist(key, index[position + radius]) < 5.0)
			{
				position = position + radius;
				break;
			}
		}
		radius++;
	}
	return position;
}

//
//long
//find_timestamp_index_position(double x, double y, double yaw, int test_orientation)
//{
//	carmen_pose_index_element key;
//	long index_found_timestamp_index = 0;
//
//	long index_found_by_x_index = carmen_pose_index_ordered_by_x.search(x, y); // @@@ Alberto: usar a outra dimensao para desambiguar em casos de duplicidade?
//	long index_found_by_y_index = carmen_pose_index_ordered_by_y.search(x, y);
//
//	key.x = x;
//	key.y = y;
//
//	double dist_key_to_x = euclidean_dist(key, carmen_pose_index_ordered_by_x[index_found_by_x_index]);
//	double dist_key_to_y = euclidean_dist(key, carmen_pose_index_ordered_by_y[index_found_by_y_index]);
//
//	// em uma trajetoria com loops, o indice ordenado por x pode encontrar um numero de pontos ambiguos igual ao (numero de loops + 1)
//	// isso pode ser observado da seguinte forma: trace um circulo e divida-o em um quadriculado. ao escolher uma determinada linha,
//	// voce interceptara o circulo em 2 pontos.
//	// o teste abaixo eh realizado para fazer essa desambiguacao. se o ponto com o x mais proximo ao ponto que estamos procurando estiver
//	// a uma distancia maior que 1.0 metro, ele estara do outro lado da curva.
//	if (dist_key_to_x > 5.0)
//		index_found_by_x_index = find_nearest_point_around_point_found(carmen_pose_index_ordered_by_x, index_found_by_x_index, x, y);
//
//	// o mesmo teste eh realizado para a coordenada y.
//	if (dist_key_to_y > 5.0)
//		index_found_by_y_index = find_nearest_point_around_point_found(carmen_pose_index_ordered_by_y, index_found_by_y_index, x, y);
//
//	if (test_orientation)
//	{
//		if (fabs(carmen_normalize_theta(carmen_pose_index_ordered_by_x[index_found_by_x_index].yaw - yaw)) > (M_PI / 2.0))
//			index_found_by_x_index = find_index_pos_with_correct_yaw(carmen_pose_index_ordered_by_x, index_found_by_x_index, yaw);
//
//		if (fabs(carmen_normalize_theta(carmen_pose_index_ordered_by_y[index_found_by_y_index].yaw - yaw)) > (M_PI / 2.0))
//			index_found_by_y_index = find_index_pos_with_correct_yaw(carmen_pose_index_ordered_by_y, index_found_by_y_index, yaw);
//	}
//
//	if (dist_key_to_x < dist_key_to_y)
//		index_found_timestamp_index = carmen_pose_index_ordered_by_x[index_found_by_x_index].timestamp_index_position;
//	else
//		index_found_timestamp_index = carmen_pose_index_ordered_by_y[index_found_by_y_index].timestamp_index_position;
//
//	return index_found_timestamp_index;
//}


long
find_timestamp_index_position_with_full_index_search(double x, double y, double yaw, int test_orientation, double timestamp_ignore_neighborhood, int search_only_in_the_begining)
{
	int i, min_dist_pos = 0;
	double dist, min_dist = -1;

	for(i = 0; i < carmen_index_ordered_by_timestamp.size(); i++)
	{
		if (search_only_in_the_begining && i > 100)
			break;

		// esse if eh para tratar os fechamentos de loop. se estamos no fim do rddf, buscamos a pose mais proxima 
		// com uma diferenca temporal maior que 3 minutos para evitar que poses proximas a posicao atual sejam retornados.
		if (timestamp_ignore_neighborhood > 0)
			if (fabs(timestamp_ignore_neighborhood - carmen_index_ordered_by_timestamp[i].timestamp) < 3 * 60)
				continue;
			
		dist = sqrt(pow(x - carmen_index_ordered_by_timestamp[i].x, 2) + pow(y - carmen_index_ordered_by_timestamp[i].y, 2));

		if ((dist < min_dist) || (min_dist == -1))
		{
			// ignore points with incorrect orientation
			if (test_orientation)
				if (fabs(carmen_normalize_theta(carmen_index_ordered_by_timestamp[i].yaw - yaw)) > (M_PI / 2.0))
					continue;

			min_dist = dist;
			min_dist_pos = i;
		}
	}

	return min_dist_pos;
}

long
find_timestamp_index_position_with_full_index_search_near_timestamp(double x, double y, double yaw, int test_orientation, double timestamp_near)
{
	int i, min_dist_pos = 0;
	double dist, min_dist = -1;

	for(i = 0; i < carmen_index_ordered_by_timestamp.size(); i++)
	{
		if (fabs(timestamp_near - carmen_index_ordered_by_timestamp[i].timestamp) > 30.0)
			continue;

		dist = sqrt(pow(x - carmen_index_ordered_by_timestamp[i].x, 2) + pow(y - carmen_index_ordered_by_timestamp[i].y, 2));

		if ((dist < min_dist) || (min_dist == -1))
		{
			// ignore points with incorrect orientation
			if (test_orientation)
				if (fabs(carmen_normalize_theta(carmen_index_ordered_by_timestamp[i].yaw - yaw)) > (M_PI / 2.0))
					continue;

			min_dist = dist;
			min_dist_pos = i;
		}
	}

	return min_dist_pos;
}

carmen_ackerman_traj_point_t
create_ackerman_traj_point_struct(double x, double y, double velocity_x, double phi, double yaw)
{
	carmen_ackerman_traj_point_t point;

	point.x = x;
	point.y = y;
	point.v = velocity_x;
	point.phi = phi;
	point.theta = yaw;

	return point;
}


int
fill_in_waypoints_array(long timestamp_index_position, carmen_ackerman_traj_point_t *poses_ahead, int num_poses_desired, carmen_ackerman_traj_point_t *last_pose_acquired, int *annotations)
{
	//double dist;
	int i, num_poses_aquired;
	carmen_ackerman_traj_point_t last_pose, current_pose;
	carmen_timestamp_index_element index_element;

	num_poses_aquired = 0;
	index_element = carmen_index_ordered_by_timestamp[timestamp_index_position];
	poses_ahead[num_poses_aquired] = last_pose = create_ackerman_traj_point_struct(index_element.x, index_element.y, index_element.velocity_x, index_element.phi, index_element.yaw);
	annotations[num_poses_aquired] = index_element.anottation;
	num_poses_aquired++;
	i = 0;
	while ((num_poses_aquired < num_poses_desired) && ((timestamp_index_position + i) < carmen_index_ordered_by_timestamp.size()))
	{
		index_element = carmen_index_ordered_by_timestamp[timestamp_index_position + i];
		current_pose = create_ackerman_traj_point_struct (index_element.x, index_element.y, index_element.velocity_x, index_element.phi, index_element.yaw);

		//dist = sqrt(pow(current_pose.x - last_pose.x, 2.0) + pow(current_pose.y - last_pose.y, 2.0));

		//if (dist > 1.0) // get waypoints 1 meter apart // @@@ Alberto: este parametro devia estar no carmen ini
		//{
			last_pose = current_pose;
			poses_ahead[num_poses_aquired] = current_pose;
			annotations[num_poses_aquired] = index_element.anottation;

			num_poses_aquired++;
		//}

		i++;
	}

	(*last_pose_acquired) = poses_ahead[num_poses_aquired - 1];
	return num_poses_aquired;
}


int
fill_in_backward_waypoints_array(long timestamp_index_position, carmen_ackerman_traj_point_t *poses_back, int num_poses_desired)
{
	//double dist;
	int i, num_poses_aquired;
	carmen_ackerman_traj_point_t last_pose, current_pose;
	carmen_timestamp_index_element index_element;

	num_poses_aquired = 0;
	index_element = carmen_index_ordered_by_timestamp[timestamp_index_position];
	poses_back[num_poses_aquired] = last_pose = create_ackerman_traj_point_struct(index_element.x, index_element.y, index_element.velocity_x, index_element.phi, index_element.yaw);
	num_poses_aquired++;
	i = 0;

	while ((num_poses_aquired < num_poses_desired) && ((timestamp_index_position - i) >= 0))
	{
		index_element = carmen_index_ordered_by_timestamp[timestamp_index_position - i];
		current_pose = create_ackerman_traj_point_struct (index_element.x, index_element.y, index_element.velocity_x, index_element.phi, index_element.yaw);

		//dist = sqrt(pow(current_pose.x - last_pose.x, 2.0) + pow(current_pose.y - last_pose.y, 2.0));

		//if (dist > 1.0) // get waypoints 1 meter apart // @@@ Alberto: este parametro devia estar no carmen ini
		//{
			last_pose = current_pose;
			poses_back[num_poses_aquired] = current_pose;

			num_poses_aquired++;
		//}

		i++;
	}

	return num_poses_aquired;
}


int
carmen_rddf_has_closed_loop()
{
	double dist = 0;
	carmen_timestamp_index_element first, last;

	first = carmen_index_ordered_by_timestamp[0];
	last = carmen_index_ordered_by_timestamp[carmen_index_ordered_by_timestamp.size() - 1];

	dist = sqrt(pow(first.x - last.x, 2) + pow(first.y - last.y, 2));

	if (dist < 5.0)
		return 1;
	else
		return 0;
}


int
get_more_more_poses_from_begining(int num_poses_desired, carmen_ackerman_traj_point_t* poses_ahead, carmen_ackerman_traj_point_t last_pose_acquired_at_end_of_index, int num_poses_acquired_before_end_of_index, int *annotations)
{
	double dist;
	int i, num_poses_aquired;
	carmen_ackerman_traj_point_t last_pose, current_pose;
	carmen_timestamp_index_element index_element;

	num_poses_aquired = 0;
	//i = 0;
	i = find_timestamp_index_position_with_full_index_search(
		carmen_index_ordered_by_timestamp[carmen_index_ordered_by_timestamp.size() - 1].x, 
		carmen_index_ordered_by_timestamp[carmen_index_ordered_by_timestamp.size() - 1].y, 
		carmen_index_ordered_by_timestamp[carmen_index_ordered_by_timestamp.size() - 1].yaw, 
		1, 
		carmen_index_ordered_by_timestamp[carmen_index_ordered_by_timestamp.size() - 1].timestamp);
		
	last_pose = last_pose_acquired_at_end_of_index;

	dist = sqrt(pow(carmen_index_ordered_by_timestamp[i].x - last_pose.x, 2.0) + pow(carmen_index_ordered_by_timestamp[i].y - last_pose.y, 2.0));

	if (dist >= 7.0) // @Filipe: Colocar no carmen.ini
		return 0;
	
	while ((num_poses_aquired < num_poses_desired) && (i < carmen_index_ordered_by_timestamp.size()))
	{
		index_element = carmen_index_ordered_by_timestamp[i];
		current_pose = create_ackerman_traj_point_struct (index_element.x, index_element.y, index_element.velocity_x, index_element.phi, index_element.yaw);

		//dist = sqrt(pow(current_pose.x - last_pose.x, 2.0) + pow(current_pose.y - last_pose.y, 2.0));

		//if (dist > 1.0) // get waypoints 1 meter apart // @@@ Alberto: este parametro devia estar no carmen ini
		//{
			last_pose = current_pose;
			poses_ahead[num_poses_acquired_before_end_of_index + num_poses_aquired] = current_pose;
			annotations[num_poses_acquired_before_end_of_index + num_poses_aquired] = index_element.anottation;

			num_poses_aquired++;
		//}

		i++;
	}

	return num_poses_aquired;
}


int
carmen_search_next_poses_index(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t* poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int num_poses_desired, int *annotations, int perform_loop = 0)
{
//	static int closed_loop = 0;
//	//static double time_when_closed_loop = 0;
//	static double timestamp_last_pose = 0;
//	static double timestamp_after_closure = 0;
//
	long timestamp_index_position;
	int num_poses_aquired = 0;
	carmen_ackerman_traj_point_t last_pose_acquired;
	(void) timestamp; // to not warning. I use it sometimes to debug.

//	//timestamp_index_position = find_timestamp_index_position(x, y, yaw, 1);
//	if (closed_loop)
//		timestamp_index_position = find_timestamp_index_position_with_full_index_search_near_timestamp(x, y, yaw, 1, timestamp_after_closure);
//	else
	timestamp_index_position = find_timestamp_index_position_with_full_index_search(x, y, yaw, 1);

	num_poses_aquired = fill_in_waypoints_array(timestamp_index_position, poses_ahead, num_poses_desired, &last_pose_acquired, annotations);
	(*num_poses_back) = fill_in_backward_waypoints_array(timestamp_index_position, poses_back, num_poses_desired / 3);
	if (perform_loop)
		if (/*carmen_rddf_has_closed_loop() && */ (num_poses_aquired < num_poses_desired))
			num_poses_aquired += get_more_more_poses_from_begining(num_poses_desired - num_poses_aquired, poses_ahead, last_pose_acquired, num_poses_aquired, annotations);

//	if ((timestamp_last_pose != 0) && fabs(carmen_index_ordered_by_timestamp[timestamp_index_position].timestamp - timestamp_last_pose) > 30.0 && (!closed_loop))
//	{
//		closed_loop = 1;
//		//time_when_closed_loop = carmen_get_time();
//		timestamp_after_closure = carmen_index_ordered_by_timestamp[timestamp_index_position].timestamp;
//	}
//
//	timestamp_last_pose = carmen_index_ordered_by_timestamp[timestamp_index_position].timestamp;
//
//	if (closed_loop && fabs(carmen_index_ordered_by_timestamp[timestamp_index_position].timestamp - timestamp_after_closure) > 3.0 * 60.0)
//	{
//		closed_loop = 0;
//		timestamp_last_pose = 0;
//	}

	return num_poses_aquired;
}


int
fill_in_waypoints_around_point(long timestamp_index_position, carmen_ackerman_traj_point_t* poses_ahead, int num_poses_desired)
{
	//double dist;
	int i, num_poses_aquired = 0;
	carmen_ackerman_traj_point_t last_pose, current_pose;
	carmen_timestamp_index_element index_element;

	i = timestamp_index_position;

	last_pose.x = carmen_index_ordered_by_timestamp[i].x;
	last_pose.y = carmen_index_ordered_by_timestamp[i].y;

	// add points backward to the current position
	while ((num_poses_aquired < (num_poses_desired / 2)) && (i > 0))
	{
		index_element = carmen_index_ordered_by_timestamp[i];
		current_pose = create_ackerman_traj_point_struct (index_element.x, index_element.y, index_element.velocity_x, index_element.phi, index_element.yaw);

		//dist = sqrt(pow(current_pose.x - last_pose.x, 2.0) + pow(current_pose.y - last_pose.y, 2.0));

		//if (dist > 1.0)
		//{
			last_pose = current_pose;
			poses_ahead[num_poses_aquired] = current_pose;

			num_poses_aquired++;
		//}

		i--;
	}

	i = timestamp_index_position;

	// add points forward to the current pose
	while ((num_poses_aquired < num_poses_desired) && (i < carmen_index_ordered_by_timestamp.size()))
	{
		index_element = carmen_index_ordered_by_timestamp[i];
		current_pose = create_ackerman_traj_point_struct (index_element.x, index_element.y, index_element.velocity_x, index_element.phi, index_element.yaw);

		//dist = sqrt(pow(current_pose.x - last_pose.x, 2.0) + pow(current_pose.y - last_pose.y, 2.0));

		//if (dist > 1.0)
		//{
			last_pose = current_pose;
			poses_ahead[num_poses_aquired] = current_pose;

			num_poses_aquired++;
		//}

		i++;
	}

	return num_poses_aquired;
}


int
carmen_find_poses_around(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t* poses_ahead, int num_poses_desired)
{
	long timestamp_index_position;
	int num_poses_aquired = 0;

	(void) timestamp; // to not warning. I use it sometimes to debug.

	//timestamp_index_position = find_timestamp_index_position(x, y, yaw, 0);
	timestamp_index_position = find_timestamp_index_position_with_full_index_search(x, y, yaw, 0);
	num_poses_aquired = fill_in_waypoints_around_point(timestamp_index_position, poses_ahead, num_poses_desired);

	return num_poses_aquired;
}


void
carmen_rddf_load_index(char *rddf_filename)
{
	char rddf_index_x_filename[2048];
	char rddf_index_y_filename[2048];
	char rddf_index_timestamp_filename[2048];

	strcpy (rddf_index_x_filename, rddf_filename);
	strcpy (rddf_index_y_filename, rddf_filename);
	strcpy (rddf_index_timestamp_filename, rddf_filename);

	strcat (rddf_index_x_filename, ".x.index");
	strcat (rddf_index_y_filename, ".y.index");
	strcat (rddf_index_timestamp_filename, ".timestamp.index");

	carmen_index_ordered_by_timestamp.load_from_file(rddf_index_timestamp_filename);
	carmen_pose_index_ordered_by_x.load_from_file(rddf_index_x_filename);
	carmen_pose_index_ordered_by_y.load_from_file(rddf_index_y_filename);

	carmen_pose_index_ordered_by_x.set_dim_to_sort_and_search(0 /* x dim */);
	carmen_pose_index_ordered_by_y.set_dim_to_sort_and_search(1 /* y dim */);

	printf("Index loaded with success!\n");
}


int
carmen_rddf_index_exists(char *rddf_filename)
{
	char index_timestamp_filename [2048];
	char index_x_filename [2048];
	char index_y_filename [2048];

	strcpy (index_timestamp_filename, rddf_filename);
	strcpy (index_x_filename, rddf_filename);
	strcpy (index_y_filename, rddf_filename);

	strcat (index_timestamp_filename, ".timestamp.index");
	strcat (index_x_filename, ".x.index");
	strcat (index_y_filename, ".y.index");

	//
	// Test if the x_index exists
	//

	FILE *rddf_file_x_index = fopen (index_x_filename, "r");

	if (rddf_file_x_index == NULL)
		return 0;
	else
		fclose (rddf_file_x_index);

	//
	// Test if the y_index exists
	//

	FILE *rddf_file_y_index = fopen (index_y_filename, "r");

	if (rddf_file_y_index == NULL)
		return 0;
	else
		fclose (rddf_file_y_index);

	//
	// Test if the timestamp_index exists
	//

	FILE *rddf_file_timestamp_index = fopen (index_timestamp_filename, "r");

	if (rddf_file_timestamp_index == NULL)
		return 0;
	else
		fclose (rddf_file_timestamp_index);

	//
	// Return 1 if all index exists
	//

	return 1;
}

void
carmen_rddf_index_add(const carmen_fused_odometry_message *fused_odometry_message, long data_offset, long data_length, int annotation)
{
	carmen_index_ordered_by_timestamp.add(
		fused_odometry_message->pose.position.x, fused_odometry_message->pose.position.y, fused_odometry_message->pose.position.z,
		fused_odometry_message->pose.orientation.roll, fused_odometry_message->pose.orientation.pitch, fused_odometry_message->pose.orientation.yaw,
		fused_odometry_message->gps_position_at_turn_on.x, fused_odometry_message->gps_position_at_turn_on.y, fused_odometry_message->gps_position_at_turn_on.z,
		fused_odometry_message->velocity.x, fused_odometry_message->velocity.y, fused_odometry_message->velocity.z,
		fused_odometry_message->angular_velocity.roll, fused_odometry_message->angular_velocity.pitch, fused_odometry_message->angular_velocity.yaw,
		fused_odometry_message->phi, fused_odometry_message->timestamp,
		data_offset, data_length, annotation
	);
}

void
carmen_rddf_index_save(char *rddf_filename)
{
	carmen_index_ordered_by_timestamp.sort();
	carmen_create_pose_index_from_timestamp_index();
	carmen_write_index(rddf_filename);
}

carmen_timestamp_index*
get_timestamp_index()
{
	return &carmen_index_ordered_by_timestamp;
}

void
carmen_rddf_index_clear()
{
	carmen_pose_index_ordered_by_x.index.clear();
	carmen_pose_index_ordered_by_x.index_is_sorted = false;
	carmen_pose_index_ordered_by_y.index.clear();
	carmen_pose_index_ordered_by_y.index_is_sorted = false;
	carmen_index_ordered_by_timestamp.index.clear();
	carmen_index_ordered_by_timestamp.index_is_sorted = false;
}
