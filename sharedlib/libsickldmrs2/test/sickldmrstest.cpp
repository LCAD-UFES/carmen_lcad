#include <sickldmrs.h>
#include <vpTime.h>

int main()
{
	std::string ip = "192.168.0.1";
	//unsigned long int iter = 0;
	vpSickLDMRS laser;
	laser.setIpAddress(ip);
	laser.setPort(12002);
	laser.setup();

	vpLaserScan laserscan[4];
	for ( ; ; ) {
		//double t1 = vpTime::measureTimeMs();

		// Get the measured points in the four layers
		if (laser.measure(laserscan) == false)
			continue;

		// Measures time across iterations
		/*
		double t2 = vpTime::measureTimeMs();
		std::cout << "iter: " << ++iter << " time: " << t2 - t1 << " ms" << std::endl;
		*/

		// Prints all the measured points
		///*
		for (int layer=0; layer<4; layer++) {
			std::vector<vpScanPoint> pointsInLayer = laserscan[layer].getScanPoints();
			vpScanPoint p;

			for (unsigned int i=0; i < pointsInLayer.size(); i++) {
				std::cout << pointsInLayer[i] << std::endl;
			}
		}
		//*/
	}
}
