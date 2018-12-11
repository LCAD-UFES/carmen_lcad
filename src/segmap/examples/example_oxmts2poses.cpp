
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;


vector<double>
load_timestamps()
{
	vector<double> times;
	char *time_name = "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/oxts/timestamps.txt";

	int dummy;
	double t;
	FILE *f = fopen(time_name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", time_name));

	while(fscanf(f, "%d-%d-%d %d:%d:%lf", &dummy, &dummy, &dummy, &dummy, &dummy, &t) == 6)
		times.push_back(t);

	fclose(f);
	return times;
}


vector<double>
read_vector(char *name)
{
	double d;
	vector<double> data;

	FILE *f = fopen(name, "r");

	if (f == NULL)
		exit(printf("File '%s' not found.", name));

	while (!feof(f))
	{
		fscanf(f, "%lf\n", &d);
		data.push_back(d);
	}

	fclose(f);
	return data;
}


// debug
void
print_vector(vector<double> &v)
{
	for (int i = 0; i < v.size(); i++)
		printf("%.2lf ", v[i]);

	printf("\n");
}


vector<vector<double>>
load_oxts(vector<double> &times)
{
	vector<vector<double>> data;

	char *dir = "/dados/kitti_stuff/kitti_2011_09_26/2011_09_26_data/2011_09_26_drive_0048_sync/oxts/data";
	char name[1024];

	for (int i = 0; i < times.size(); i++)
	{
		sprintf(name, "%s/%010d.txt", dir, i);
		data.push_back(read_vector(name));
	}

	return data;
}


vector<Matrix<double, 4, 4>>
oxts2Mercartor(vector<vector<double>> &data)
{
	double scale = cos(data[0][0] * M_PI / 180.);
	double er = 6378137;

	// pose[i] contains the transformation which takes a
	// 3D point in the i'th frame and projects it into the oxts
	// coordinates of the first frame.
	vector<Matrix<double, 4, 4>> poses;
	Matrix<double, 4, 4> p0;

	for (int i = 0; i < data.size(); i++)
	{
		double x, y, z;
		x = scale * data[i][1] * M_PI * er / 180;
		y = scale * er * log(tan((90 + data[i][0]) * M_PI / 360));
		z = data[i][2];

		double rx = data[i][3]; // roll
		double ry = data[i][4]; // pitch
		double rz = data[i][5]; // heading

		Matrix<double, 3, 3> Rx, Ry, Rz, R;
		Matrix<double, 4, 4> p;

		Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
		Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
		Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
		R  = Rz * Ry * Rx;

		p << R(0, 0),  R(0, 1),  R(0, 2), x,
			R(1, 0),  R(1, 1),  R(1, 2), y,
			R(2, 0),  R(2, 1),  R(2, 2), z,
			0, 0, 0, 1;

		if (i == 0)
			p0 = p;

		p = p0.inverse() * p;
		poses.push_back(p);
	}

	return poses;
}


void
print_poses(vector<Matrix<double, 4, 4>> &poses)
{
	Matrix<double, 4, 4> p;

	for (int i = 0; i < poses.size(); i++)
	{
		p = poses[i];

		printf("%.2lf %.2lf %.2lf\n",
				p(0, 3) / p(3, 3),
				p(1, 3) / p(3, 3),
				p(2, 3) / p(3, 3));
	}
}


int
main()
{
	vector<double> times = load_timestamps();
	vector<vector<double>> data = load_oxts(times);
	vector<Matrix<double, 4, 4>> poses = oxts2Mercartor(data);
	print_poses(poses);

	return 0;
}


