#include <cstdio>
#include <iostream>
#include <fstream>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>

#include <boost/algorithm/string/trim.hpp>


using namespace std;
using namespace cv;

string ts_cascade_name = "data_g.xml";
CascadeClassifier ts_cascade;
string window_name = "Capture - Traffic Sign detection. Image: ";
RNG rng(12345);

string
detectAndDisplay(Mat frame, string image, char *verbose, string aux);

/*
 * 
 */
int
main(int argc, char** argv)
{

    if (argc != 4)
    {
        cerr << argv[0] << " test.txt 0 result.txt" << endl;
        return EXIT_FAILURE;
    }
    Mat frame;
    ifstream file;
    ofstream result_file;
    string image;
    char ikey = 0;
    result_file.open(argv[3]);

    cerr << "Testing" << endl;
    //-- 1. Load the cascades
    if (!ts_cascade.load(ts_cascade_name))
    {
        cerr << "--(!)Error loading" << endl;
        return -1;
    };

    //-- 2. Reading the file of images
    file.open(argv[1]);
    if (file.is_open())
    {
        char *verbos = argv[2];
        getline(file, image);
        boost::trim(image);
        //-- 3. Loop to detect the words
        while (!file.eof())
        {
            string aux = image;
//            image = "../database_g/" + image;
//            printf("%s\n", image.c_str());
            frame = cvLoadImageM(image.c_str(), 1);

            //-- 4. Apply the classifier to the frame
            if (!frame.empty())
            {
                string newline = detectAndDisplay(frame, image, verbos, aux);
                //cerr << newline << endl;
                result_file << newline;
            }
            else
            {
                cerr << " --(!) No image has informed -- Break!" << endl;
                return 1;
            }
            if (verbos[0] == '1')
            {
                //-- 5. Lookin the key for exit or continue
                ikey = waitKey(0);
                switch (ikey)
                {
                case 27:
                {
                    cvDestroyAllWindows();
                    return 0;
                }
                default:
                {
                }
                    cvDestroyAllWindows();
                }
            }
            getline(file, image);
            boost::trim(image);
        }
    }
    result_file.close();
}

string
detectAndDisplay(Mat frame, string image, char *verbose, string aux)
{
    char newline[100000] = "";

    vector<Rect> semaphores;
    Mat frame_gray;

    Rect ROI(cv::Point(0, 0), cv::Point(frame.cols, frame.rows));
    cv::Mat half_image;

    cv::Mat(frame, ROI).copyTo(half_image);

//    Vector<cv::Mat> channels;
//
//    channels.push_back( frame );
//    channels.push_back( Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC1) );
//    channels.push_back( Mat::zeros(cv::Size(frame.cols, frame.rows), CV_8UC1) );
//
//    cv::merge(channels, frame_gray);
//    cvtColor(half_image, frame_gray, CV_YCrCb2BGR);

    cvtColor(half_image, frame_gray, CV_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

//    half_image.convertTo(frame_gray, CV_8U);
    //-- Detect traffic lights  
    ts_cascade.detectMultiScale(frame_gray, semaphores, 1.05, 3, 0, Size(5, 20), Size(200, 400));

    for (size_t i = 0; i < semaphores.size(); i++)
    {
        CvPoint p1, p2;
        p1.x = semaphores[i].x;// + frame.cols/4;
        p1.y = semaphores[i].y;
        p2.x = p1.x + semaphores[i].width;
        p2.y = p1.y + semaphores[i].height;
        rectangle(frame, p1, p2, CV_RGB(255, 0, 0), 3, 8, 0);

        sprintf(newline, "%s%s %d %d %d %d\n", newline, aux.c_str(), p1.x, p1.y, p2.x, p2.y);
    }

    //cerr << "Image " << image << endl;
    //-- Show what you got
    if (verbose[0] == '1')
    {
        imshow(window_name + image, frame);
    }
	imwrite("result_db/" + aux, frame);
    string result = newline;
    //imwrite("image/" + image, frame);
    return result;
}
