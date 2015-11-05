#include "driver.h"



int main()
{
	velodyne_driver::VelodyneDriver velodyne(2368, 8308, 360, 360);

	while(true)
	{

		if(velodyne.pollScan())
		{
			//velodyne.printVelodyneScan();
		}
		else
			break;
	}

	return 0;
}
