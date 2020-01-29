#include <carmen/carmen.h>
#include <carmen/stereo_util.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/localize_ackerman_interface.h>

#include <opencv/highgui.h>
#include <opencv/cv.h>

#include <string.h>
#include <vector>

using namespace std;
using namespace cv;

typedef struct
{
	carmen_bumblebee_basic_stereoimage_message* image;
	carmen_pose_3D_t pose;
}carmen_image_and_pose_synchronized;

int camera;
stereo_util camera_data;
carmen_pose_3D_t camera_pose_g;
vector<carmen_image_and_pose_synchronized*> synchronized_image_and_pose_queue;
Mat K1, K2, D1, D2;
char camera_message_name[256];

void 
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		carmen_ipc_disconnect();
		printf("baseline_extension: disconnected.\n");
		exit(0);
	}
}


void
show_matrix(const char *label, Mat m)
{
	printf("%s\n", label);

	for (int i = 0; i < m.rows; i++)
	{
		printf("[\t");

		for (int j = 0; j < m.cols; j++)
		{
			printf("%lf\t", m.at<double>(i, j));
		}

		printf("]\n");
	}
}


void
rectify_images(carmen_image_and_pose_synchronized *image_1, carmen_image_and_pose_synchronized *image_2, Mat R, Mat T)
{
	int i, j;

	IplImage *ipl_image_1_l = cvCreateImage(cvSize(image_1->image->width, image_1->image->height), IPL_DEPTH_8U, 3);
	IplImage *ipl_image_2_l = cvCreateImage(cvSize(image_2->image->width, image_2->image->height), IPL_DEPTH_8U, 3);

	IplImage *ipl_image_1_r = cvCreateImage(cvSize(image_1->image->width, image_1->image->height), IPL_DEPTH_8U, 3);
	IplImage *ipl_image_2_r = cvCreateImage(cvSize(image_2->image->width, image_2->image->height), IPL_DEPTH_8U, 3);

	for (i = 0; i < image_1->image->height; i++)
	{
		for (j = 0; j < image_2->image->width; j++)
		{
			ipl_image_1_l->imageData[i * ipl_image_1_l->widthStep + 3 * j + 0] = image_1->image->raw_left[3 * i * image_1->image->width + 3 * j + 2];
			ipl_image_1_l->imageData[i * ipl_image_1_l->widthStep + 3 * j + 1] = image_1->image->raw_left[3 * i * image_1->image->width + 3 * j + 1];
			ipl_image_1_l->imageData[i * ipl_image_1_l->widthStep + 3 * j + 2] = image_1->image->raw_left[3 * i * image_1->image->width + 3 * j + 0];

			ipl_image_1_r->imageData[i * ipl_image_1_r->widthStep + 3 * j + 0] = image_1->image->raw_right[3 * i * image_1->image->width + 3 * j + 2];
			ipl_image_1_r->imageData[i * ipl_image_1_r->widthStep + 3 * j + 1] = image_1->image->raw_right[3 * i * image_1->image->width + 3 * j + 1];
			ipl_image_1_r->imageData[i * ipl_image_1_r->widthStep + 3 * j + 2] = image_1->image->raw_right[3 * i * image_1->image->width + 3 * j + 0];

			ipl_image_2_l->imageData[i * ipl_image_2_l->widthStep + 3 * j + 0] = image_2->image->raw_left[3 * i * image_2->image->width + 3 * j + 2];
			ipl_image_2_l->imageData[i * ipl_image_2_l->widthStep + 3 * j + 1] = image_2->image->raw_left[3 * i * image_2->image->width + 3 * j + 1];
			ipl_image_2_l->imageData[i * ipl_image_2_l->widthStep + 3 * j + 2] = image_2->image->raw_left[3 * i * image_2->image->width + 3 * j + 0];

			ipl_image_2_r->imageData[i * ipl_image_2_r->widthStep + 3 * j + 0] = image_2->image->raw_right[3 * i * image_2->image->width + 3 * j + 2];
			ipl_image_2_r->imageData[i * ipl_image_2_r->widthStep + 3 * j + 1] = image_2->image->raw_right[3 * i * image_2->image->width + 3 * j + 1];
			ipl_image_2_r->imageData[i * ipl_image_2_r->widthStep + 3 * j + 2] = image_2->image->raw_right[3 * i * image_2->image->width + 3 * j + 0];
		}
	}

	// DEBUG:

	double scale = 0.3;

	IplImage *ipl_image_1_l_res = cvCreateImage(cvSize(image_1->image->width * scale, image_1->image->height * scale), IPL_DEPTH_8U, 3);
	IplImage *ipl_image_2_l_res = cvCreateImage(cvSize(image_2->image->width * scale, image_2->image->height * scale), IPL_DEPTH_8U, 3);
	IplImage *ipl_image_1_r_res = cvCreateImage(cvSize(image_1->image->width * scale, image_1->image->height * scale), IPL_DEPTH_8U, 3);
	IplImage *ipl_image_2_r_res = cvCreateImage(cvSize(image_2->image->width * scale, image_2->image->height * scale), IPL_DEPTH_8U, 3);

	cvResize(ipl_image_1_l, ipl_image_1_l_res, CV_INTER_CUBIC);
	cvResize(ipl_image_2_l, ipl_image_2_l_res, CV_INTER_CUBIC);
	cvResize(ipl_image_1_l, ipl_image_1_r_res, CV_INTER_CUBIC);
	cvResize(ipl_image_2_l, ipl_image_2_r_res, CV_INTER_CUBIC);

	cvShowImage("image_1_l", ipl_image_1_l_res);
	cvShowImage("image_2_l", ipl_image_2_l_res);
	cvShowImage("image_1_r", ipl_image_1_r_res);
	cvShowImage("image_2_r", ipl_image_2_r_res);

	Mat left(ipl_image_1_l), right(ipl_image_2_r);

	// Matrix<double> RLeft = new Matrix<double>(3, 3);
	Mat RLeft(Size(3, 3), CV_64FC1);
	// Matrix<double> RRight = new Matrix<double>(3, 3);
	Mat RRight(Size(3, 3), CV_64FC1);
	// Matrix<double> PLeft = new Matrix<double>(3, 4);
	Mat PLeft(Size(3, 4), CV_64FC1);
	// Matrix<double> PRight = new Matrix<double>(3, 4);
	Mat PRight(Size(3, 4), CV_64FC1);
	// Matrix<double> ReprojectionMatrix = new Matrix<double>(4, 4);
	Mat ReprojectionMatrix(Size(4, 4), CV_64FC1);

	// Rectangle ROI = new Rectangle()
	Rect ROI;
	// Rectangle ROI2 = new Rectangle();
	Rect ROI2;
	// CvInvoke.cvStereoRectify(KLeft, KRight, DLeft, DRight, left.Size, R, T, RLeft, RRight, PLeft, PRight, ReprojectionMatrix, 0, 0, left.Size, ref ROI, ref ROI2);
	stereoRectify(K1, D1, K2, D2, Size(left.cols, left.rows), R, T, RLeft, RRight, PLeft, PRight, ReprojectionMatrix, 0, 0, Size(0, 0), &ROI, &ROI2);

	show_matrix("RLeft", RLeft);
	show_matrix("RRight", RRight);
	show_matrix("ReprojectionMatrix", ReprojectionMatrix);

	// Image<Gray, float> MapXLeft = new Image<Gray, float>(left.Size);
	Mat MapXLeft(left.size(), CV_32FC1);
	// Image<Gray, float> MapYLeft = new Image<Gray, float>(left.Size);
	Mat MapYLeft(left.size(), CV_32FC1);
	// Image<Gray, float> MapXRight = new Image<Gray, float>(left.Size);
	Mat MapXRight(right.size(), CV_32FC1);
	// Image<Gray, float> MapYRight = new Image<Gray, float>(right.Size);
	Mat MapYRight(right.size(), CV_32FC1);
	// CvInvoke.cvInitUndistortRectifyMap(KLeft, DLeft, RLeft, PLeft, MapXLeft, MapYLeft);
	initUndistortRectifyMap(K1, D1, RLeft, PLeft, Size(left.cols, left.rows), CV_32FC1, MapXLeft, MapYLeft);
	// CvInvoke.cvInitUndistortRectifyMap(KRight, DRight, RRight, PRight, MapXRight, MapYRight);
	initUndistortRectifyMap(K2, D2, RRight, PRight, Size(right.cols, right.rows), CV_32FC1, MapXRight, MapYRight);

	// Image<Bgr, Byte> leftRectified = left.Clone();
	Mat leftRectified(left);
	// Image<Bgr, Byte> rightRectified = right.Clone();
	Mat rightRectified(right);

	// CvInvoke.cvRemap(left, leftRectified, MapXLeft, MapYLeft, 0, new Emgu.CV.Structure.MCvScalar(0));
	remap(left, leftRectified, MapXLeft, MapYLeft, INTER_CUBIC, 0, Scalar(0, 0, 0));
	// CvInvoke.cvRemap(right, rightRectified, MapXRight, MapYRight, 0, new Emgu.CV.Structure.MCvScalar(0));
	remap(right, rightRectified, MapXRight, MapYRight, INTER_CUBIC, 0, Scalar(0, 0, 0));

	Mat leftRectified_res (Size(leftRectified.cols * scale, leftRectified.rows * scale), leftRectified.type());
	Mat rightRectified_res(Size(rightRectified.cols * scale, rightRectified.rows * scale), rightRectified.type());

	resize(leftRectified, leftRectified_res, Size(leftRectified.cols * scale, leftRectified.rows * scale), 0, 0, INTER_CUBIC);
	resize(rightRectified, rightRectified_res, Size(leftRectified.cols * scale, leftRectified.rows * scale), 0, 0, INTER_CUBIC);

	imshow("leftRectified", leftRectified_res);
	imshow("rightRectified", rightRectified_res);
	cvWaitKey(-1);

	cvReleaseImage(&ipl_image_1_l_res);
	cvReleaseImage(&ipl_image_1_r_res);
	cvReleaseImage(&ipl_image_2_l_res);
	cvReleaseImage(&ipl_image_2_r_res);

	cvReleaseImage(&ipl_image_1_l);
	cvReleaseImage(&ipl_image_1_r);
	cvReleaseImage(&ipl_image_2_l);
	cvReleaseImage(&ipl_image_2_r);
}


void
publish_rectified_images(carmen_image_and_pose_synchronized *image_1, carmen_image_and_pose_synchronized *image_2)
{
	// TODO
	(void) image_1;
	(void) image_2;
}


void
calulate_R_and_T_1(carmen_image_and_pose_synchronized *image_1, carmen_image_and_pose_synchronized *image_2, Mat &R, Mat &T)
{
	Mat R_vector(1, 3, CV_64FC1);
	R = Mat::eye(3, 3, CV_64FC1);
	T = Mat::zeros(3, 1, CV_64FC1);

	Mat R_1(3, 3, CV_64FC1);
	Mat R_2(3, 3, CV_64FC1);

	R_vector.at<double>(0, 0) = image_1->pose.orientation.pitch;
	R_vector.at<double>(0, 1) = image_1->pose.orientation.yaw;
	R_vector.at<double>(0, 2) = image_1->pose.orientation.roll;

	Rodrigues(R_vector, R_1);

	R_vector.at<double>(0, 0) = image_2->pose.orientation.pitch;
	R_vector.at<double>(0, 1) = image_2->pose.orientation.yaw;
	R_vector.at<double>(0, 2) = image_2->pose.orientation.roll;

	Rodrigues(R_vector, R_2);

	Mat T1 = Mat::zeros(4, 4, CV_64FC1);
	Mat T2 = Mat::zeros(4, 4, CV_64FC1);
	Mat T1_to_T2 = Mat::zeros(4, 4, CV_64FC1);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			T1.at<double>(i, j) = R_1.at<double>(i, j);
			T2.at<double>(i, j) = R_2.at<double>(i, j);
		}
	}

	T1.at<double>(0, 3) = -image_1->pose.position.y;
	T1.at<double>(1, 3) = image_1->pose.position.z;
	T1.at<double>(2, 3) = image_1->pose.position.x;
	T1.at<double>(3, 3) = 1.0;

	T2.at<double>(0, 3) = -image_2->pose.position.y;
	T2.at<double>(1, 3) = image_2->pose.position.z;
	T2.at<double>(2, 3) = image_2->pose.position.x;
	T2.at<double>(3, 3) = 1.0;

	T1_to_T2 = T2 * T1.inv();

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R.at<double>(i, j) = T1_to_T2.at<double>(i, j);
			R.at<double>(i, j) = T1_to_T2.at<double>(i, j);
		}

		T.at<double> (i, 0) = T1_to_T2.at<double>(i, 3);
	}
}


void
calulate_R_and_T(carmen_image_and_pose_synchronized *image_1, carmen_image_and_pose_synchronized *image_2, Mat &R, Mat &T)
{
	Mat R_vector(1, 3, CV_64FC1);

	R = Mat::eye(3, 3, CV_64FC1);
	T = Mat::zeros(3, 1, CV_64FC1);

	// TODO: sera que eu deveria somar o baseline aqui?
	T.at<double>(0, 0) = image_2->pose.position.x - image_1->pose.position.x;
	T.at<double>(0, 1) = image_2->pose.position.y - image_1->pose.position.y;
	T.at<double>(0, 2) = image_2->pose.position.z - image_1->pose.position.z;

	R_vector.at<double>(0, 0) = carmen_normalize_theta(image_2->pose.orientation.pitch - image_1->pose.orientation.pitch);
	R_vector.at<double>(0, 1) = carmen_normalize_theta(image_2->pose.orientation.yaw - image_1->pose.orientation.yaw);
	R_vector.at<double>(0, 2) = carmen_normalize_theta(image_2->pose.orientation.roll - image_1->pose.orientation.roll);

	Rodrigues(R_vector, R);
}


void
rectify_images_and_publish()
{
	int i;
	int n;
	carmen_image_and_pose_synchronized *image_1;
	carmen_image_and_pose_synchronized *image_2;
	Mat R(3, 3, CV_64FC1), T(1, 3, CV_64FC1);

	n = synchronized_image_and_pose_queue.size() - 1;

	for (i = 0; i < n; i += 2)
	{
		image_1 = synchronized_image_and_pose_queue[i];
		image_2 = synchronized_image_and_pose_queue[i + 1];

		calulate_R_and_T(image_1, image_2, R, T);

		rectify_images(image_1, image_2, R, T);
		publish_rectified_images(image_1, image_2);
	}
}


carmen_bumblebee_basic_stereoimage_message *
alloc_stereo_image_message(int image_size)
{
	carmen_bumblebee_basic_stereoimage_message *message = (carmen_bumblebee_basic_stereoimage_message *) calloc (sizeof(carmen_bumblebee_basic_stereoimage_message), 1);

	message->raw_left = (unsigned char *) calloc (image_size, sizeof(unsigned char));
	message->raw_right = (unsigned char *) calloc (image_size, sizeof(unsigned char));

	return message;
}


carmen_bumblebee_basic_stereoimage_message *
read_image(FILE *input_file_ptr)
{
	int i;
	int width;
	int height;
	int image_size;
	int isRectified;
	int intensity;
	double timestamp;

	carmen_bumblebee_basic_stereoimage_message *stereo_message;

	fscanf(input_file_ptr, "%d %d %d %d %lf ", &height, &width, &image_size, &isRectified, &timestamp);
	stereo_message = alloc_stereo_image_message(image_size);

	stereo_message->width = width;
	stereo_message->height = height;
	stereo_message->image_size = image_size;
	stereo_message->isRectified = isRectified;
	stereo_message->timestamp = timestamp;

	for (i = 0; i < stereo_message->image_size; i++)
	{
		fscanf(input_file_ptr, "%d ", &intensity);
		stereo_message->raw_left[i] = intensity;
	}

	for (i = 0; i < stereo_message->image_size; i++)
	{
		fscanf(input_file_ptr, "%d ", &intensity);
		stereo_message->raw_right[i] = intensity;
	}

	return stereo_message;
}


carmen_pose_3D_t
read_image_pose(FILE *input_file_ptr)
{
	carmen_pose_3D_t pose;

	fscanf(input_file_ptr, "%lf %lf %lf %lf %lf %lf\n",
			&(pose.position.x), &(pose.position.y), &(pose.position.z),
			&(pose.orientation.roll), &(pose.orientation.pitch), &(pose.orientation.yaw)
	);

	return pose;
}


void
enqueue_image_and_pose(carmen_bumblebee_basic_stereoimage_message *stereo_message, carmen_pose_3D_t pose)
{
	carmen_image_and_pose_synchronized *image_and_pose = (carmen_image_and_pose_synchronized *) calloc (sizeof(carmen_image_and_pose_synchronized), 1);

	image_and_pose->image = stereo_message;
	image_and_pose->pose = pose;

	synchronized_image_and_pose_queue.push_back(image_and_pose);
}


FILE *
open_file(char *filename)
{
	FILE *file_ptr = fopen(filename, "r");

	if (!file_ptr)
	{
		printf("Error: unable to open file to read '%s'\n", filename);
		exit(-1);
	}

	return file_ptr;
}

int first_time = 1;
carmen_bumblebee_basic_stereoimage_message *resized_stereo_message = NULL;
double timestamp_last_message_sent = 0;

void
publish_image(carmen_bumblebee_basic_stereoimage_message *stereo_message)
{
	double scale = 0.25;

	IplImage *limg = cvCreateImage(cvSize(stereo_message->width, stereo_message->height), IPL_DEPTH_8U, 3);
	IplImage *rimg = cvCreateImage(cvSize(stereo_message->width, stereo_message->height), IPL_DEPTH_8U, 3);
	IplImage *lres_img = cvCreateImage(cvSize(stereo_message->width * scale, stereo_message->height * scale), IPL_DEPTH_8U, 3);
	IplImage *rres_img = cvCreateImage(cvSize(stereo_message->width * scale, stereo_message->height * scale), IPL_DEPTH_8U, 3);

	for (int i = 0; i < stereo_message->height; i++)
	{
		for (int j = 0; j < stereo_message->width; j++)
		{
			limg->imageData[i * limg->widthStep + 3 * j + 0] = stereo_message->raw_left[3 * i * stereo_message->width + 3 * j + 2];
			limg->imageData[i * limg->widthStep + 3 * j + 1] = stereo_message->raw_left[3 * i * stereo_message->width + 3 * j + 1];
			limg->imageData[i * limg->widthStep + 3 * j + 2] = stereo_message->raw_left[3 * i * stereo_message->width + 3 * j + 0];

			rimg->imageData[i * rimg->widthStep + 3 * j + 0] = stereo_message->raw_right[3 * i * stereo_message->width + 3 * j + 2];
			rimg->imageData[i * rimg->widthStep + 3 * j + 1] = stereo_message->raw_right[3 * i * stereo_message->width + 3 * j + 1];
			rimg->imageData[i * rimg->widthStep + 3 * j + 2] = stereo_message->raw_right[3 * i * stereo_message->width + 3 * j + 0];
		}
	}

	cvResize(limg, lres_img, CV_INTER_CUBIC);
	cvResize(rimg, rres_img, CV_INTER_CUBIC);

	if (first_time)
	{
		resized_stereo_message = (carmen_bumblebee_basic_stereoimage_message *) calloc (sizeof(carmen_bumblebee_basic_stereoimage_message), 1);
		resized_stereo_message->raw_left = (unsigned char *) calloc (sizeof(unsigned char), 3 * stereo_message->width * scale * stereo_message->height * scale);
		resized_stereo_message->raw_right = (unsigned char *) calloc (sizeof(unsigned char), 3 * stereo_message->width * scale * stereo_message->height * scale);

		first_time = 0;
	}

	resized_stereo_message->height = stereo_message->height * scale;
	resized_stereo_message->width = stereo_message->width * scale;
	resized_stereo_message->image_size = 3 * stereo_message->height * scale * stereo_message->width * scale;
	resized_stereo_message->isRectified = 1;
	resized_stereo_message->timestamp = stereo_message->timestamp;
	resized_stereo_message->host = carmen_get_host();

	for (int i = 0; i < resized_stereo_message->height ; i++)
	{
		for (int j = 0; j < resized_stereo_message->width; j++)
		{
			resized_stereo_message->raw_left[3 * i * resized_stereo_message->width + 3 * j + 0] = lres_img->imageData[i * lres_img->widthStep + 3 * j + 2];
			resized_stereo_message->raw_left[3 * i * resized_stereo_message->width + 3 * j + 1] = lres_img->imageData[i * lres_img->widthStep + 3 * j + 1];
			resized_stereo_message->raw_left[3 * i * resized_stereo_message->width + 3 * j + 2] = lres_img->imageData[i * lres_img->widthStep + 3 * j + 0];

			resized_stereo_message->raw_right[3 * i * resized_stereo_message->width + 3 * j + 0] = rres_img->imageData[i * rres_img->widthStep + 3 * j + 2];
			resized_stereo_message->raw_right[3 * i * resized_stereo_message->width + 3 * j + 1] = rres_img->imageData[i * rres_img->widthStep + 3 * j + 1];
			resized_stereo_message->raw_right[3 * i * resized_stereo_message->width + 3 * j + 2] = rres_img->imageData[i * rres_img->widthStep + 3 * j + 0];
		}
	}

	// publish 10 stereo messages per second
	while (fabs(carmen_get_time() - timestamp_last_message_sent) < 1.0)
	{
		// sleep 10 ms
		usleep(10 * 1000);
	}

	timestamp_last_message_sent = carmen_get_time();

	printf("publish %lf\n", timestamp_last_message_sent);
	IPC_RETURN_TYPE err;
	err = IPC_publishData(camera_message_name, resized_stereo_message);
	carmen_test_ipc_exit(err, "Could not publish", camera_message_name);

	cvReleaseImage(&limg);
	cvReleaseImage(&rimg);
	cvReleaseImage(&lres_img);
	cvReleaseImage(&rres_img);
}


void
read_synchronization_file(char *filename)
{
	FILE *file_ptr = open_file(filename);

	carmen_bumblebee_basic_stereoimage_message *stereo_message;
	carmen_pose_3D_t pose;

//	printf("loading synch file...\n");

	while (!feof(file_ptr))
	{
		stereo_message = read_image(file_ptr);
		publish_image(stereo_message);

		free(stereo_message->raw_left);
		free(stereo_message->raw_right);
		free(stereo_message);

		//		pose = read_image_pose(file_ptr);
		//		enqueue_image_and_pose(stereo_message, pose);
	}

	printf("end\n");

	fclose(file_ptr);
}


void
initialize_camera_matrices(int camera_index, int argc, char **argv)
{
	int num_items;
	int width, height;

	char stereo_string[256];
	char camera_string[256];
	sprintf(camera_string, "%s%d", "camera", camera_index);
	sprintf(stereo_string, "%s%d", "stereo", camera_index);

	carmen_param_t param_list[] =
	{
			{stereo_string, (char *)"width",  			CARMEN_PARAM_INT, &width, 0, NULL},
			{stereo_string, (char *)"height",			CARMEN_PARAM_INT, &height, 0, NULL},
			{(char *) camera_string, (char *) "x",		CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.x), 0, NULL},
			{(char *) camera_string, (char *) "y",		CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.y), 0, NULL},
			{(char *) camera_string, (char *) "z", 		CARMEN_PARAM_DOUBLE, &(camera_pose_g.position.z), 0, NULL},
			{(char *) camera_string, (char *) "yaw", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.yaw), 0, NULL},
			{(char *) camera_string, (char *) "pitch", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.pitch), 0, NULL},
			{(char *) camera_string, (char *) "roll", 	CARMEN_PARAM_DOUBLE, &(camera_pose_g.orientation.roll), 0, NULL}
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	camera_data = get_stereo_instance(camera_index, width, height);

	K1 = Mat::zeros(3, 3, CV_64FC1);
	K2 = Mat::zeros(3, 3, CV_64FC1);
	D1 = Mat::zeros(1, 4, CV_64FC1);
	D2 = Mat::zeros(1, 4, CV_64FC1);

	K1.at<double>(0, 0) = K2.at<double>(0, 0) = camera_data.fx;
	K1.at<double>(1, 1) = K2.at<double>(1, 1) = camera_data.fy;
	K1.at<double>(0, 2) = K2.at<double>(0, 2) = camera_data.xc;
	K1.at<double>(1, 2) = K2.at<double>(1, 2) = camera_data.yc;
	K1.at<double>(2, 2) = K2.at<double>(2, 2) = 1;
}


void
carmen_baseline_extension_publish_initialize(int camera, int argc, char **argv)
{
	initialize_camera_matrices(camera, argc, argv);

	sprintf(camera_message_name, "%s%d", CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_NAME, camera);

	IPC_RETURN_TYPE err;
	err = IPC_defineMsg(camera_message_name, IPC_VARIABLE_LENGTH, CARMEN_BUMBLEBEE_BASIC_STEREOIMAGE_FMT);
	carmen_test_ipc_exit(err, "Could not define", camera_message_name);

	read_synchronization_file(argv[2]);
}


int 
parse_arguments(int argc, char **argv)
{
	if (argc < 3)
		return 0;

	camera = atoi(argv[1]);

	return 1;
}


void
show_usage_information_and_exit(int argc __attribute__ ((unused)), char **argv)
{
	printf("\n");
	printf("Use %s <camera-index> <synchronization-file>\n", argv[0]);
	printf("\n");

	exit(-1);
}


int
main(int argc, char **argv) 
{
	if (!parse_arguments(argc, argv))
		show_usage_information_and_exit(argc, argv);

	carmen_ipc_initialize(argc, argv);
	signal(SIGINT, shutdown_module);

	carmen_baseline_extension_publish_initialize(camera, argc, argv);
//	rectify_images_and_publish();

	return (0);
}


