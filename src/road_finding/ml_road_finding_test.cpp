#include "ml_road_finding.h"
#include "image_utils.h"
#include "time_profile.h"

vector<CvPoint> image_points_selected;

void mouse_handler(int event, int x, int y, int flags, void *param)
{
  switch(event)
  {
    case CV_EVENT_LBUTTONUP:
      IplImage *image = (IplImage*)param;
      CvPoint pt = cvPoint(cvRound(x), cvRound(y));
      image_points_selected.push_back(pt);

      int n_points = image_points_selected.size();

      if (n_points % 2 == 1) // first point of the current road line
      {
        cvCircle(image, pt, 5, CV_RGB(0,255,0), 1, CV_AA, 0);
      }
      else
      {
        cvCircle(image, pt, 5, CV_RGB(0,255,0), 1, CV_AA, 0);
        CvPoint pt0 = image_points_selected.at(n_points - 2); // get the previous line point to draw a line
        cvLine(image, pt0, pt, CV_RGB(0,255,0), 1, CV_AA, 0);
        cvRectangle(image, pt0, pt, CV_RGB(0,255,0), 1, CV_AA, 0);
      }
      cvShowImage("IMAGE", image);
      break;
  }
}

void test_mahalanobis()
{
  rgb_gaussian *g0 = init_rgb_gaussian(1);

  gsl_vector_set(g0->mean, 0, 119.0);
  gsl_vector_set(g0->mean, 1, 120.0);
  gsl_vector_set(g0->mean, 2, 119.0);

  gsl_matrix_set(g0->cov, 0, 0, 887.0);
  gsl_matrix_set(g0->cov, 0, 1, 869.0);
  gsl_matrix_set(g0->cov, 0, 2, 889.0);
  gsl_matrix_set(g0->cov, 1, 0, 869.0);
  gsl_matrix_set(g0->cov, 1, 1, 892.0);
  gsl_matrix_set(g0->cov, 1, 2, 890.0);
  gsl_matrix_set(g0->cov, 2, 0, 889.0);
  gsl_matrix_set(g0->cov, 2, 1, 890.0);
  gsl_matrix_set(g0->cov, 2, 2, 906.0);

  gsl_vector *x = gsl_vector_alloc(3);
  gsl_vector_set(x, 0, 149.0);
  gsl_vector_set(x, 1, 150.0);
  gsl_vector_set(x, 2, 149.0);

  print_matrix(g0->cov, stdout);

  double distance = gsl_mahalanobis_distance(x, g0->mean, g0->cov);
  printf("Mahalanobis Distance %f\n", distance);

  gsl_vector_free(x);
  release_gaussian(g0);
}

rgb_gaussian *get_gaussian_from_file(IplImage *image, const char *file_name)
{
  int n_samples;
  FILE *f = fopen(file_name, "r");
  fscanf(f, "%d\n", &n_samples);
  CvScalar *samples = (CvScalar*)malloc(n_samples * sizeof(CvScalar));

  IplImage *tmp = cvCloneImage(image);
  for (int i =0; i < n_samples; i++)
  {
    int x, y;
    fscanf(f, "%d\t%d\n", &x, &y);
    samples[i] = cvGet2D(image, y, x);

    cvSet2D(tmp, y, x, CV_RGB(255,0,0));
  }
  fclose(f);

  cvShowImage("TMP", tmp);
  cvWaitKey(-1);

  rgb_gaussian *gaussian = get_gaussian_in_samples(samples, n_samples);

  return gaussian;
}

void get_gaussians(char *filename)
{
  ml_road_finding road_finding_instance;
  FILE *f = fopen(filename, "r");

  cvNamedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
  cvNamedWindow("GRAY", CV_WINDOW_AUTOSIZE);
  int first_time = 1;

  while (!feof(f))
  {
    char imageName[256];
    fscanf(f, "%[^\n]\n", imageName);

    if (imageName[0] == '#')
      continue;

    IplImage *src_image = cvLoadImage(imageName, CV_LOAD_IMAGE_COLOR);
//    cvSetImageROI(src_image, cvRect(0, 0.4 * src_image->height, src_image->width, 0.6 * src_image->height));
//
//    IplImage *image = cvCreateImage(cvSize(src_image->width, 0.6 * src_image->height), src_image->depth, src_image->nChannels);
//    cvCopy(src_image, image);
//    cvResetImageROI(src_image);

    IplImage *image = cvCloneImage(src_image);

    if(first_time)
    {
    	init_ml_road_finding(&road_finding_instance, 10, image->widthStep, image->height);
    	first_time = 0;
    }

    IplImage *image_cpy = cvCloneImage(image);

    cvCvtColor(image, image, CV_BGR2HSV);
//    cvSmooth(image, image);

    cvShowImage("IMAGE", image_cpy);
    cvSetMouseCallback("IMAGE", mouse_handler, image_cpy);
    cvWaitKey(-1);

    int n_points_selected = image_points_selected.size();

    for (int i = 0; i < n_points_selected / 2; i++)
    {
      CvPoint A = image_points_selected.at(2 * i + 0);
      CvPoint B = image_points_selected.at(2 * i + 1);
      rgb_gaussian *gaussian = get_gaussian_at_rect(image, image_utils_get_rect(image, A, B), NULL);
      //rgb_gaussian *gaussian= get_gaussian_from_file(image, "/home/vazevedo/roboticaprobabilistica/code/carmen/src/road_finding/right_image14_points.txt");
      //rgb_gaussian *gaussian = get_gaussian_at_line(image, A, B);

      print_gaussian(gaussian);
      print_distances(&road_finding_instance, gaussian);

      add_gaussian(&road_finding_instance, gaussian);
    }

    struct timeval start, end, elapsed;
    gettimeofday(&start, NULL);

    fill_image_based_on_gaussians(&road_finding_instance, image, image, NULL);

    gettimeofday(&end, NULL);
    timevalSubtract(&elapsed, &end, &start);
    print_time(&elapsed, (char*)"Time to fill image", stdout);

    IplImage *gray = cvCreateImage(cvGetSize(image), image->depth, 1);
    cvCvtColor(image, gray, CV_BGR2GRAY);
    cvThreshold(gray, gray, 4, 255, CV_THRESH_BINARY_INV);

    cvShowImage("GRAY", gray);
    cvShowImage("IMAGE", image);
    cvWaitKey(-1);

    image_points_selected.clear();

    // release memory
    cvReleaseImage(&src_image);
    cvReleaseImage(&image);
    cvReleaseImage(&image_cpy);
    cvReleaseImage(&gray);
  }

  destroy_ml_road_finding(&road_finding_instance);
  cvDestroyWindow("IMAGE");
  cvDestroyWindow("GRAY");
  fclose(f);
}

int main()
{
  get_gaussians((char*)"/home/vazevedo/roboticaprobabilistica/code/carmen/src/road_finding/train_set.txt");
  //get_gaussians("train_set.txt");

  return 0;
}

