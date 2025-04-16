#include "camera_drivers.h"
#include "camera_drivers_process_image.hpp"


// FFmpeg library
av_ffmpeg *av_ffmpeg_params;


///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers																					 //
///////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int signo)
{
    if (signo == SIGINT)
	{
		if (av_ffmpeg_params)
		{
			if (av_ffmpeg_params->decframe) av_frame_free(&av_ffmpeg_params->decframe);
			if (av_ffmpeg_params->frame) av_frame_free(&av_ffmpeg_params->frame);
			if (av_ffmpeg_params->avctx) avcodec_free_context(&av_ffmpeg_params->avctx);
			if (av_ffmpeg_params->inctx) avformat_close_input(&av_ffmpeg_params->inctx);

			if (av_ffmpeg_params->framebuf) free(av_ffmpeg_params->framebuf);
			if (av_ffmpeg_params->vcap) delete(av_ffmpeg_params->vcap);
			free(av_ffmpeg_params);
		}
        printf("Signal %d received, exiting program ...\n", signo);
        exit(0);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations																		     //
///////////////////////////////////////////////////////////////////////////////////////////////


int
find_camera_model(char *smodel)
{
	if (strncmp(smodel, "rtsp", 4) == 0)
		return (ip_ffmpeg);
	return (usb_opencv);
}


int
main(int argc, char **argv)
{
	unsigned char *raw = NULL;
	int width, height, model, first_time = 1;

	signal(SIGINT, shutdown_module);

	if (argc < 2)
		exit(1);

	model = find_camera_model(argv[1]);

	if (first_time)
	{
		av_ffmpeg_params = (av_ffmpeg *)malloc(sizeof(av_ffmpeg));
		first_time = 0;
	}

	while (1)
	{
		switch (model)
		{
		case ip_opencv:
			raw = get_image_ip_camera_with_opencv(argv[1], 0, NULL, width, height);
			break;

		case ip_ffmpeg:
			raw = get_image_ip_camera_with_ffmmpeg(argv[1], 0, NULL, av_ffmpeg_params, width, height);
			break;

		case usb_opencv:
			raw = get_image_usb_camera_with_opencv(argv[1], 0, NULL, width, height);
			break;

		default:
			break;
		}

		if (!raw)
			continue;
		
		Mat cv_image = Mat(height, width, CV_8UC3, raw, 0);
		imshow("camera_check", cv_image);
    	waitKey(1);
	}
}
