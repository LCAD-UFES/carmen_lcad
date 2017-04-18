/*********************************************************
	---   Skeleton Module Application ---
 **********************************************************/

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/semantic_segmentation_interface.h>
#include <carmen/semantic_segmentation.h>
#include <opencv/cv.h>

using namespace cv;

SegNet *seg_net = NULL;

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
void 
publish_segmented_image(Mat &out, double bumblebee_time)
{
	IPC_RETURN_TYPE err;
	carmen_semantic_segmentation_image_message message;

	message.width = out.cols;
	message.height = out.rows;
	message.image_size = out.rows * out.cols * out.channels();
	message.image = out.data;
	message.timestamp = bumblebee_time;
	message.host = carmen_get_host();

	err = IPC_publishData(CARMEN_SEMANTIC_SEGMENTATION_IMAGE_NAME, &message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_SEMANTIC_SEGMENTATION_IMAGE_FMT);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void 
bumblebee_handler(carmen_bumblebee_basic_stereoimage_message *message)
{
	printf("Message received!\n");

	Mat m(Size(message->width, message->height), CV_8UC3, message->raw_right);
	Mat res(Size(message->width / 2, message->height / 2), CV_8UC3);

	resize(m, res, res.size());
	cvtColor(res.clone(), res, CV_RGB2BGR);

	Mat out = seg_net->Predict(res);

	imshow("bgr", res);
	imshow("segmented", seg_net->ClassesToColors(out));
	waitKey(1);

	publish_segmented_image(out, message->timestamp);
}


void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("semantic_segmentation: disconnected.\n");

		exit(0);
	}
}


int 
main(int argc, char **argv) 
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);

	carmen_semantic_segmentation_define_messages();
	seg_net = new SegNet("segnet_model_driving_webdemo.prototxt", "segnet_iter_30000_timo.caffemodel", "camvid12.png");

	carmen_bumblebee_basic_subscribe_stereoimage(8 , NULL, (carmen_handler_t) bumblebee_handler, CARMEN_SUBSCRIBE_LATEST);
	carmen_ipc_dispatch();
	return (0);
}
