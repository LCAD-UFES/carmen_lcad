
#include <cstdio>
#include <string>
#include "segmap_dataset.h"
#include "segmap_util.h"
#include "segmap_viewer.h"
#include "segmap_sensors.h"
#include <pcl/visualization/pcl_visualizer.h>


using namespace std;
using namespace pcl;


int 
main(int argc, char **argv)
{
    //if (argc < 2)
        //exit(printf("Use %s <log-path>\n", argv[0]));

    vector<string> v;

    v.push_back("/media/filipe/Hitachi-Teste/log_aeroporto_vila_velha_20170726-2.txt");          
    v.push_back("/media/filipe/Hitachi-Teste/log-mata-da-praia-20181130.txt");
    v.push_back("/media/filipe/Hitachi-Teste/log_aeroporto_vila_velha_20170726.txt");            
    v.push_back("/media/filipe/Hitachi-Teste/log_sao_paulo_brt_20170827-2.txt");
    v.push_back("/media/filipe/Hitachi-Teste/log_dante_michelini-20181116-pista-esquerda.txt");  
    v.push_back("/media/filipe/Hitachi-Teste/log_sao_paulo_brt_20170827.txt");
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

    DataSample* data_package;
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    PointCloudViewer viewer;

    int sample_id;
    sample_id = 0;

    char name[128];

    for (int k = 0; k < v.size(); k++)
    {
        NewCarmenDataset dataset = 
            NewCarmenDataset((char*) v[k].c_str(),
                            (char*) ("/dados/data2/data_" + v[k] + "/odom_calib.txt").c_str(),
                            NewCarmenDataset::SYNC_BY_CAMERA);

        Pose2d dead_reckoning;
        double previous_time = 0;

        Pose2d gps0;

        int count = 0;

        while ((data_package = dataset.next_data_package()))
        {
            if (previous_time > 0.)
            {
                ackerman_motion_model(dead_reckoning, 
                    data_package->v, data_package->phi, 
                    data_package->odom_time - previous_time);
            }
            else
            {
                gps0 = data_package->gps;
            }

            if (fabs(data_package->v) < 1.)
                continue;

            if (count++ < 50)
                continue;
            
            count = 0;

            printf("gps: %lf %lf ", data_package->gps.x, data_package->gps.y);
            printf("odom: %lf %lf ", dead_reckoning.x + gps0.x, dead_reckoning.y + gps0.y);
            printf("gps time: %lf ", data_package->gps_time);
            printf("image time: %lf ", data_package->image_time);
            printf("velodyne time: %lf ", data_package->velodyne_time);
            printf("odom time: %lf ", data_package->odom_time);
            printf("xsens time: %lf\n", data_package->xsens_time);
            
            previous_time = data_package->odom_time;

            printf("Image path: %s velodyne path: %s\n",
                data_package->image_path.c_str(),
                data_package->velodyne_path.c_str()
            );

            Mat img = load_image(data_package);
            CarmenLidarLoader loader(data_package->velodyne_path.c_str(), data_package->n_laser_shots, dataset.intensity_calibration);
            load_as_pointcloud(&loader, cloud);
            
            sprintf(name, "calibration/bb3/img%04d.png", sample_id);
            imwrite(name, img);
            sprintf(name, "calibration/velodyne/cloud%04d.txt", sample_id);
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

            sample_id++;

            //viewer.show(img, "img", 320);
            //viewer.show(cloud);
            //viewer.loop();
            //viewer.clear();
        }
    }
    printf("Ok\n");
    return 0;
}
