#include <stdio.h>

#include <opencv2/opencv.hpp>
#include "ELAS/utils/IPM.h"

using namespace cv;
using namespace std;

/* Notes:
 * The way it is done, we assume the camera is aligned to the center of the car
 * 		if it is not true, this misalignment can be considered as a bias to the projection
 */

// define visible distance
#define VISIBLE_DISTANCE_HEIGHT 10  // in meters (from the beginning of visible region)
#define VISIBLE_DISTANCE_WIDTH 10   // in meters (with the camera on the center of this region)
#define PIXEL_TO_METER_X 0.025		// size, in meters, of each pixel
#define PIXEL_TO_METER_Y 0.025		// size, in meters, of each pixel
#define KNOWN_DISTANCE_HEIGHT 5		// in meters
#define KNOWN_DISTANCE_WIDTH 4.2	// in meters
#define DIST_VISIBLE_TO_KNOWN 3.8	// in meters

int main(int argc, char * argv[]) {

	argc = argc;
	printf("RUNNING: %s\n\n", argv[0]);

	// read calibration image
	Mat3b frame = imread("/dados/calibration/frame_1462194885.707623.png");
	if (frame.empty()) {
		printf("Image not found!");
		return EXIT_FAILURE;
	}
	imshow("Original frame", frame);

	// calibration points
	vector<Point2f> calibration_points = {
			Point2f(358.5, 623.5), 	// bottom-left
			Point2f(452.0, 567.0), 	// top-left
			Point2f(744.0, 563.0), 	// top-right
			Point2f(792.5, 619.5) 	// bottom-right
	};
	// extra two on the top
	// top-right: 501.0, 537.5
	// top-left: 720.5, 535.5

	// from meters to pixels
	Size dstSize = Size(VISIBLE_DISTANCE_WIDTH / PIXEL_TO_METER_X, VISIBLE_DISTANCE_HEIGHT / PIXEL_TO_METER_Y);
	const float shift_height = KNOWN_DISTANCE_HEIGHT / PIXEL_TO_METER_Y;
	const float shift_width = KNOWN_DISTANCE_WIDTH / PIXEL_TO_METER_X;
	const float dist_region_to_known = DIST_VISIBLE_TO_KNOWN / PIXEL_TO_METER_Y;
	Point2f base_point = Point2f((dstSize.width / 2.0) - (shift_width / 2.0), dstSize.height - dist_region_to_known - shift_height);
	vector<Point2f> dst_points = {
			base_point + Point2f(0, shift_height),
			base_point + Point2f(0,0),
			base_point + Point2f(shift_width, 0),
			base_point + Point2f(shift_width, shift_height)
	};

	// init IPM
	Size origSize = Size(frame.cols, frame.rows);
	IPM ipm = IPM(origSize, dstSize, calibration_points, dst_points);

	printf("Homography matrix:\n");
	cout << ipm.getH() << endl;

	// apply the homography
	Mat3b frameIPM;
	ipm.applyHomography(frame, frameIPM);
	imshow("IPM frame", frameIPM);

	// save matrix to file
	string fname = "data/elas_calibration.xml";
	printf("Saving data to %s... ", fname.c_str());
	FileStorage fs(fname, FileStorage::WRITE);
	fs << "origSize" << origSize;
	fs << "dstSize" << dstSize;
	fs << "calibration_points" << calibration_points;
	fs << "dst_points" << dst_points;
	fs << "scale_x" << PIXEL_TO_METER_X;
	fs << "scale_y" << PIXEL_TO_METER_Y;
	fs.release();
	printf("done!\n");

	waitKey(0); // prevent from closing without seeing the images
	printf("Exit!");
	return EXIT_SUCCESS;
}
