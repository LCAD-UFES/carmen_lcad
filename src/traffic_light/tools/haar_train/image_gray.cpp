/* 
 * File:   image_gray.cpp
 * Author: tiago
 *
 * Created on December 5, 2013, 11:54 AM
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/contrib/detection_based_tracker.hpp>


using namespace std;
using namespace cv;

/*
 * 
 */
int
main(int argc, char** argv)
{

    if (argc != 2)
    {
        cout << "Usage " << argv[0] << " files.txt" << endl;
        return EXIT_FAILURE;
    }
    ifstream file;
    string line;
    Mat frame;

    file.open(argv[1]);
    if (file.is_open())
    {
        getline(file, line);
        //-- 3. Loop to detect the words
        while (!file.eof())
        {
            frame = cvLoadImageM(line.c_str(), 0);
            //-- 4. Apply the classifier to the frame
            if (!frame.empty())
            {               
                imwrite("../" + line, frame);
            }
            getline(file, line);
        }
    }
    return EXIT_SUCCESS;
}