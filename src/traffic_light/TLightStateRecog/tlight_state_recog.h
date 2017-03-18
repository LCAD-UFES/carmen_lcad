
#ifndef TLIGHT_STATE_RECOG_H_
#define TLIGHT_STATE_RECOG_H_

#include <opencv/cv.h>

class TLightRecogInterface
{
public:

	static const int TRAFFIC_SIGN_HEIGHT = 40;
	static const int TRAFFIC_SIGN_WIDTH = 20;
	static const int NUM_CLASSES = 4;

	virtual ~TLightRecogInterface() {}

	/**
	 * 	red = 0,
	 * 	green = 1
	 * 	yellow = 2
	 * 	off = 3
	 */
	virtual int run(cv::Mat &m, cv::Rect &r) = 0;

	static char* get_complete_filename(std::string filename)
	{
		static char *carmen_home = getenv("CARMEN_HOME");
		static char *complete = NULL;

		if (complete == NULL)
			complete = new char[2048];

		if (carmen_home == NULL)
			sprintf(complete, "%s", filename.c_str());
		else
			sprintf(complete, "%s/data/tlight_trained_nets/%s", carmen_home, filename.c_str());

		return complete;
	}
};

#endif
