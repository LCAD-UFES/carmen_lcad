#include "yolo_detector.hpp"

char **classes_names;
void *network_struct;
int camera_in_use[MAX_CAMERA_ID + 1];
#define SHOW_DETECTIONS 1


void shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		cvDestroyAllWindows();

		printf("Digit_Locomotive_Detector: Disconnected.\n");
		exit(0);
	}
}



/////////////////////PUBLISHERS////////////////

void
build_and_publish_detections(vector<bbox_t> &predictions, int cam_number)
{
	yolo_detector_message msg;
	msg.cam_id = cam_number;
	msg.qtd_bboxes = predictions.size();
	msg.bounding_boxes = (bboxes*) malloc(msg.qtd_bboxes*sizeof(bboxes));

	for(unsigned int i = 0; i < predictions.size(); i++)
	{
		msg.bounding_boxes[i].x = predictions[i].x;
		msg.bounding_boxes[i].y = predictions[i].y;
		msg.bounding_boxes[i].w = predictions[i].w;
		msg.bounding_boxes[i].h = predictions[i].h;
		msg.bounding_boxes[i].obj_id = predictions[i].obj_id;
		msg.bounding_boxes[i].prob = predictions[i].prob;
		msg.bounding_boxes[i].track_id = predictions[i].track_id;
	}

	yolo_detector_publish_yolo_detector_message(&msg);
}

///////////////////////////////////////////////


void
show_detections(Mat image, vector<bbox_t> predictions, double fps)
{
	cout<<"show_detections"<<endl;
	char info[128];

    cvtColor(image, image, COLOR_RGB2BGR);

	sprintf(info, "%dx%d", image.cols, image.rows);
    putText(image, info, Point(10, 15), FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);

    sprintf(info, "FPS %.2f", fps);
    putText(image, info, Point(10, 30), FONT_HERSHEY_PLAIN, 1, cvScalar(0, 255, 0), 1);
	
    for (unsigned int i = 0; i < predictions.size(); i++)
	{
		sprintf(info, "prob %.2f", predictions[i].prob);
		rectangle(image, Point(predictions[i].x, predictions[i].y), Point((predictions[i].x + predictions[i].w), (predictions[i].y + predictions[i].h)),
				Scalar(255, 0, 255), 4);
		putText(image, info, Point(predictions[i].x + 1, predictions[i].y - 3), FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
	}

	// resize(image, image, Size(image.cols * resize_factor, image.rows * resize_factor));
    imshow("Yolo_Detector", image);
    waitKey(1);
}


void
filter_predictions_of_interest(vector<bbox_t> &predictions)
{
	cout<<"filter_predictions_of_interest"<<endl;
	vector<bbox_t> filtered_predictions;

	for (unsigned int i = 0; i < predictions.size(); i++)
	{
		if (predictions[i].obj_id == 0)// ||  // person
		{
			filtered_predictions.push_back(predictions[i]);
		}
	}
	predictions = filtered_predictions;
}


void get_yolo_detections(Mat &roi, vector<bbox_t> &predictions_vector)
{
	cout<<"get_yolo_detections"<<endl;
	predictions_vector = run_YOLO(roi.data, 3, roi.cols, roi.rows, network_struct, classes_names, 0.5, 0.5);
	filter_predictions_of_interest(predictions_vector);
	cout<<"finished get_yolo_detections"<<endl;
}


Mat convert_message_image_to_opencv_image(bool *first_time, camera_image *image)
{
	cout<<"convert_message_image_to_opencv_image"<<endl;
    int crop_x, crop_y, crop_w, crop_h;
	Mat open_cv_image;
	if (*first_time)
	{
		crop_y = 0;
		crop_x = 0;
		crop_w = image->width;
		crop_h = image->height;
		*first_time = false;
	}
	open_cv_image = Mat(image->height, image->width, CV_8UC3, image->raw_data, 0); // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	Rect myROI(crop_x, crop_y, crop_w, crop_h);
	open_cv_image = open_cv_image(myROI);

	return open_cv_image;
}


////////////////////HANDLERS//////////////////

void
image_handler(int cam_number, camera_message *msg)
{
	printf("image_handler\n");
    vector<bbox_t> predictions_vector;
    Mat open_cv_image;
    bool first_time = true;
	static double start_time = 0.0;

    open_cv_image = convert_message_image_to_opencv_image(&first_time, &msg->images[0]);
    get_yolo_detections(open_cv_image, predictions_vector);
	double fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();
	if (SHOW_DETECTIONS)
		show_detections(open_cv_image, predictions_vector, fps);
	if(predictions_vector.size() > 0)
	{
		build_and_publish_detections(predictions_vector, cam_number);
	}
}



void image_handler1(camera_message *msg)
{
	int cam_number = 1;
		image_handler(cam_number, msg);
}

void image_handler2(camera_message *msg)
{
	int cam_number = 2;
		image_handler(cam_number, msg);
}

void image_handler3(camera_message *msg)
{
	int cam_number = 3;
		image_handler(cam_number, msg);
}

void image_handler4(camera_message *msg)
{
	int cam_number = 4;
		image_handler(cam_number, msg);
}

void image_handler5(camera_message *msg)
{
	int cam_number = 5;
		image_handler(cam_number, msg);
}

void image_handler6(camera_message *msg)
{
	int cam_number = 6;
		image_handler(cam_number, msg);
}

void image_handler7(camera_message *msg)
{
	int cam_number = 7;
		image_handler(cam_number, msg);
}

void image_handler8(camera_message *msg)
{
	int cam_number = 8;
		image_handler(cam_number, msg);
}

void image_handler9(camera_message *msg)
{
	int cam_number = 9;
		image_handler(cam_number, msg);
}

void image_handler10(camera_message *msg)
{
	int cam_number = 10;
		image_handler(cam_number, msg);
}

void image_handler11(camera_message *msg)
{
	int cam_number = 11;
		image_handler(cam_number, msg);
}

void image_handler12(camera_message *msg)
{
	int cam_number = 12;
		image_handler(cam_number, msg);
}

void image_handler13(camera_message *msg)
{
	int cam_number = 13;
		image_handler(cam_number, msg);
}

void image_handler14(camera_message *msg)
{
	int cam_number = 14;
		image_handler(cam_number, msg);
}

void image_handler15(camera_message *msg)
{
	int cam_number = 15;
		image_handler(cam_number, msg);
}

void image_handler16(camera_message *msg)
{
	int cam_number = 16;
		image_handler(cam_number, msg);
}

void image_handler17(camera_message *msg)
{
	int cam_number = 17;
		image_handler(cam_number, msg);
}

void image_handler18(camera_message *msg)
{
	int cam_number = 18;
		image_handler(cam_number, msg);
}

void image_handler19(camera_message *msg)
{
	int cam_number = 19;
		image_handler(cam_number, msg);
}

void image_handler20(camera_message *msg)
{
	int cam_number = 20;
		image_handler(cam_number, msg);
}

void (*image_handler_vector[])(camera_message *) =
	{
		NULL,
		image_handler1,
		image_handler2,
		image_handler3,
		image_handler4,
		image_handler5,
		image_handler6,
		image_handler7,
		image_handler8,
		image_handler9,
		image_handler10,
		image_handler11,
		image_handler12,
		image_handler13,
		image_handler14,
		image_handler15,
		image_handler16,
		image_handler17,
		image_handler18,
		image_handler19,
		image_handler20,
};
//////////////////////////////////////////////


void subscribe_messages()
{
	for (int i = 1; i <= MAX_CAMERA_ID; i++)
	{
		if (camera_in_use[i] > 0)
			camera_drivers_subscribe_message(i, NULL, (carmen_handler_t)(carmen_handler_t)image_handler_vector[i], CARMEN_SUBSCRIBE_LATEST);
	}
}


void
initialize_yolo()
{
	// initialize_sick_transformations(board_pose, camera_pose, bullbar_pose, sick_pose, &transformer_sick);

	char* carmen_home = getenv("CARMEN_HOME");
	char classes_names_path[1024];
	char yolo_cfg_path[1024];
	char yolo_weights_path[1024];

	sprintf(classes_names_path, "%s/sharedlib/darknet3/data/coco.names", carmen_home);
	sprintf(yolo_cfg_path, "%s/sharedlib/darknet3/cfg/yolov4.cfg", carmen_home);
	sprintf(yolo_weights_path, "%s/sharedlib/darknet3/yolov4.weights", carmen_home);

	classes_names = get_classes_names(classes_names_path);

	network_struct = load_yolo_network(yolo_cfg_path, yolo_weights_path, 1);
}


char **copy_argv(int argc, char *argv[])
{
	// calculate the contiguous argv buffer size
	int length = 0;
	size_t ptr_args = argc + 1;
	for (int i = 0; i < argc; i++)
	{
		length += (strlen(argv[i]) + 1);
	}
	char **new_argv = (char **)malloc((ptr_args) * sizeof(char *) + length);
	// copy argv into the contiguous buffer
	length = 0;
	for (int i = 0; i < argc; i++)
	{
		new_argv[i] = &(((char *)new_argv)[(ptr_args * sizeof(char *)) + length]);
		strcpy(new_argv[i], argv[i]);
		length += (strlen(argv[i]) + 1);
	}
	// insert NULL terminating ptr at the end of the ptr array
	new_argv[ptr_args - 1] = NULL;
	return (new_argv);
}


void
read_parameters(int argc, char **argv)
{
    char **new_argv = copy_argv(argc, argv);

	for (int i = 1; i < argc; i++)
	{
		char tmp[64];
		strcpy(tmp, new_argv[i]);
		string current_param = tmp; // = argv[i];
		int message_id = -999;
		if (current_param.find("intelbras") != string::npos)
		{
            string cam_id = current_param.substr(9,1);
			// cout<<cam_id<<endl;
            camera_in_use[stoi(cam_id)] = 1;
        }
    }
	cout<<"parameters were read"<<endl;
}

int main(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	read_parameters(argc, argv);

    // initialize_yolo();

	yolo_detector_define_messages();

    subscribe_messages();

	signal(SIGINT, shutdown_module);

	setlocale(LC_ALL, "C");

	carmen_ipc_dispatch();

	return 0;
}