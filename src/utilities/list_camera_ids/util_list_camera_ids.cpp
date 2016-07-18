#include <stdio.h>
#include <flycapture/FlyCapture2.h>
#include <stdlib.h>
#include <unistd.h>
#include <dc1394/conversions.h>
#include <dc1394/register.h>
#include <flycapture/Utilities.h>

using namespace FlyCapture2;


void
handle_error(Error &error)
{
	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		exit(-1);
	}
}


int
main()
{
	Error error;
	BusManager busMgr;

	unsigned int numCameras;

	error = busMgr.GetNumOfCameras(&numCameras);

	if (error != PGRERROR_OK)
	{
		error.PrintErrorTrace();
		return -1;
	}

	printf( "Number of cameras detected: %u\n", numCameras );

	for (unsigned int i = 0; i < numCameras; i++)
	{
		PGRGuid guid;
		error = busMgr.GetCameraFromIndex(i, &guid);
		handle_error(error);

		printf("Camera %d: %u %u %u %u\n", i, guid.value[0], guid.value[1], guid.value[2], guid.value[3]);
	}

	return 0;
}
