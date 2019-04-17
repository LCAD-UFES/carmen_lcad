
#include <cstdio>
#include <string>
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_util.h>
#include <carmen/segmap_viewer.h>
#include <carmen/segmap_sensors.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;


void
save_data(NewCarmenDataset &dataset, DataSample *data_package, int sample_id)
{
	char name[256];
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	Mat raw = load_image(data_package);
	Mat img(480, 640, CV_8UC3);
	resize(raw, img, img.size());

	CarmenLidarLoader loader(data_package->velodyne_path.c_str(), data_package->n_laser_shots, dataset.intensity_calibration);
	load_as_pointcloud(&loader, cloud);

	sprintf(name, "calibration/bb3/img%04d.png", sample_id);
	printf("img path: %s\n", name);
	imwrite(name, img);

	sprintf(name, "calibration/velodyne/cloud%04d.txt", sample_id);
	printf("velodyne path: %s\n", name);

	FILE *f = fopen(name, "w");

	int n = 0;

	for (int i = 0; i < cloud->size(); i++)
		if (cloud->at(i).x != 0 || cloud->at(i).y != 0 || cloud->at(i).z != 0)
			n++;

	fprintf(f, "%d\n", n);

	for (int i = 0; i < cloud->size(); i++)
	{
		if (cloud->at(i).x != 0 || cloud->at(i).y != 0 || cloud->at(i).z != 0)
			fprintf(f, "%lf %lf %lf %d\n", cloud->at(i).x, cloud->at(i).y, cloud->at(i).z, cloud->at(i).r);
	}

	fclose(f);
}


vector<string>
log_list()
{
	vector<string> v;

	v.push_back("/media/filipe/Hitachi-Teste/log_aeroporto_vila_velha_20170726-2.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-mata-da-praia-20181130.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_aeroporto_vila_velha_20170726.txt");
	//v.push_back("/media/filipe/Hitachi-Teste/log_sao_paulo_brt_20170827-2.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_dante_michelini-20181116-pista-esquerda.txt");
	//v.push_back("/media/filipe/Hitachi-Teste/log_sao_paulo_brt_20170827.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_dante_michelini-20181116.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_volta_da_ufes-20180112-2.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-estacionamento-ambiental-20181208.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_volta_da_ufes-20180112.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_estacionamentos-20181130-test.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_volta_da_ufes-20180907-2.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log_estacionamentos-20181130.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-volta-da-ufes-20181206-estacionamento-test.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-jardim_da_penha-20181207-2.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-volta-da-ufes-20181206-honofre-test.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-jardim_da_penha-20181207.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-volta-da-ufes-20181206.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-jardim-da-penha-mapeamento-20181208.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-volta-da-ufes-20181207-estacionamento_ambiental.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-mata-da-praia-20181130-test.txt");
	v.push_back("/media/filipe/Hitachi-Teste/log-volta-da-ufes-noite-20181130.txt");

	return v;
}


int
main()
{
	Pose2d gps0;
	DataSample* data_package;
	vector<string> v = log_list();
	int sample_id = 0;

	for (int k = 0; k < v.size(); k++)
	{
		string odom_calib_path = default_odom_calib_path(v[k].c_str());

		//printf("log name: %s\n", v[k].c_str());
		//printf("odom calib name: %s\n", odom_calib_path.c_str());

		NewCarmenDataset dataset(v[k], odom_calib_path);

		int count = 0;
		gps0 = dataset[0]->gps;

		for (int n = 0; n < dataset.size(); n++)
		{
			data_package = dataset[n];

			if (fabs(data_package->v) < 1.)
				continue;

			if (count++ < 50)
				continue;

			count = 0;

			save_data(dataset, data_package, sample_id);
			sample_id++;
		}
	}
	printf("Ok\n");
	return 0;
}


