
#include <cstdio>
#include "segmap_dataset.h"

int 
main()
{
    DataSample* data_package;
    NewCarmenDataset dataset = 
        NewCarmenDataset("/dados/log_estacionamentos-20181130-test.txt",
                         NewCarmenDataset::SYNC_BY_CAMERA);
    
    while (data_package = dataset.next_data_package())
    {
        printf("gps time: %lf\n", data_package->gps_time);
        printf("image time: %lf\n", data_package->image_time);
        printf("velodyne time: %lf\n", data_package->velodyne_time);
        printf("odom time: %lf\n", data_package->odom_time);
        printf("xsens time: %lf\n", data_package->xsens_time);
    }
    
    printf("Ok\n");
    return 0;
}