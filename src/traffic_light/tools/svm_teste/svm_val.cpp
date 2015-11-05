#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "dlib/svm.h"

using namespace std;
using namespace dlib;
using namespace cv;

int WIDTH, HEIGHT;

typedef matrix<double, 200, 1> sample_type;

// This is a typedef for the type of kernel we are going to use in this example.  In
// this case I have selected the radial basis kernel that can operate on our 2D
// sample_type objects
typedef radial_basis_kernel<sample_type> kernel_type;

static sample_type sample;

void
read_image_test_green();

/*
 * 
 */
int
main(int argc, char** argv)
{

    if (argc != 3)
    {
        return EXIT_FAILURE;
    }

    WIDTH = atoi(argv[1]);
    HEIGHT = atoi(argv[2]);

    typedef decision_function<kernel_type> dec_funct_type;
    typedef normalized_function<dec_funct_type> funct_type;

    // Here we are making an instance of the normalized_function object.  This object
    // provides a convenient way to store the vector normalization information along with
    // the decision function we are going to learn.  
    funct_type learned_function;


    ifstream fin("saved_function.dat", ios::binary);
    deserialize(learned_function, fin);

    read_image_test_green();
    cout << "The classifier output is " << learned_function(sample) << endl;

    return 0;
}

void
read_image_test_green()
{
    cv::Mat image;
    cv::Mat resized_image;

    image = imread("test/104.png", 1);
    resized_image.create(HEIGHT, WIDTH, CV_8UC3);

    resize(image, resized_image, Size(WIDTH, HEIGHT), 0, 0, CV_INTER_CUBIC);

    std::vector<Mat> channels;
    split(resized_image, channels);

    Mat aux;
    aux = ((channels.at(2) - channels.at(1)) + 255) / 2;

    for (int x = 0; x < resized_image.rows; x++)
    {
        for (int y = 0; y < resized_image.cols; y++)
        {
            sample(x * WIDTH + y) = (double) (aux.at<uchar>(x, y));
        }
    }
}

