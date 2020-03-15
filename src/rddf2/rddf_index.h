/*
 * rddf_index.h
 *
 *  Created on: 17/07/2012
 *      Author: filipe
 */

#ifndef RDDF_INDEX_H_
#define RDDF_INDEX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <carmen/carmen.h>
#include <vector>

using namespace std;


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

class carmen_pose_index_element
{
	public:

		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
		double timestamp_index_position;

		double& operator[](int position);
		carmen_pose_index_element()
		{
			x = y = z = roll = pitch = yaw = timestamp_index_position = 0;
		}
};


class carmen_timestamp_index_element
{
	public:

		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
		double origin_x;
		double origin_y;
		double origin_z;
		double velocity_x;
		double velocity_y;
		double velocity_z;
		double angular_velocity_roll;
		double angular_velocity_pitch;
		double angular_velocity_yaw;
		double phi;
		double timestamp;
		long rddf_offset;
		long rddf_data_length;
		int anottation;

		double& operator[](int position);
};

// @Filipe: VERIFICAR SE ESSE INDICE AINDA EH USADO. SE NAO, REMOVER.
class carmen_pose_index
{
	public:

		bool index_is_sorted;
		int dim_to_sort_and_search;
		vector<carmen_pose_index_element> index;

		carmen_pose_index();
		carmen_pose_index(int dim);

		carmen_pose_index_element operator[](uint position);
		void set_dim_to_sort_and_search(int dim);

		void add(double x, double y, double z, double roll, double pitch, double yaw,
				long carmen_timestamp_index_position);

		long size();
		void sort();
		long search (double x, double y);
		void save_to_file (char *filename);
		void load_from_file (char *filename);
};


class carmen_timestamp_index
{
	public:

		bool index_is_sorted;
		vector<carmen_timestamp_index_element> index;

		carmen_timestamp_index();
		carmen_timestamp_index_element operator[](uint position);

		void add(
			double x, double y, double z,
			double roll, double pitch, double yaw,
			double origin_x, double origin_y, double origin_z,
			double velocity_x, double velocity_y, double velocity_z,
			double angular_velocity_roll, double angular_velocity_pitch, double angular_velocity_yaw, double phi,
			double timestamp, long rddf_file_offset, long rddf_data_length, int annotation);

		long size();
		void sort();
		long search(double timestamp);
		void print();
		void save_to_file(char *filename);
		void load_from_file (char *filename);
};

// void carmen_rddf_create_index_from_rddf_log(char *rddf_filename);
int carmen_search_next_poses_index(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t *poses_ahead, carmen_ackerman_traj_point_t *poses_back, int *num_poses_back, int n, int *annotations, int perform_loop);
int carmen_find_poses_around(double x, double y, double yaw, double timestamp /* only for debugging */, carmen_ackerman_traj_point_t *poses_ahead, int n);
void carmen_rddf_load_index(char *rddf_filename);
int carmen_rddf_index_exists(char *rddf_filename);

void carmen_rddf_index_add(const carmen_fused_odometry_message *fused_odometry_message, long data_offset, long data_length, int annotation);
void carmen_rddf_index_save(char *rddf_filename);

long find_timestamp_index_position_with_full_index_search(double x, double y, double yaw, int test_orientation, double timestamp_ignore_neighborhood = 0, int search_only_in_the_begining = 0);
carmen_timestamp_index* get_timestamp_index();
carmen_ackerman_traj_point_t create_ackerman_traj_point_struct(double x, double y, double velocity_x, double phi, double yaw);

void carmen_rddf_index_clear();

#ifdef __cplusplus
}
#endif

#endif /* RDDF_INDEX_H_ */
