/*******************view_gt***********************


 *************************************************/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <cstdio>
#include <sys/io.h>
#include <string>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>

using namespace std;
using namespace cv;

Mat image;
Mat image2;
int roi_x0 = 0;
int roi_y0 = 0;
int roi_x1 = 0;
int roi_y1 = 0;
int numOfRec = 0;
int startDraw = 0;
string window_name = "Generate Output <ESC> Exit";
string old_string = "";

string
IntToString(int num)
{
    ostringstream myStream; //creates an ostringstream object
    myStream << num << flush;
    /*
     * outputs the number into the string stream and then flushes
     * the buffer (makes sure the output is put into the stream)
     */
    return (myStream.str()); //returns the string form of the stringstream object
};

void
on_mouse(int event, int x, int y, int flag, void *param)
{
    flag = flag;
    param = param;
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        if (!startDraw)
        {
            roi_x0 = x;
            roi_y0 = y;
            startDraw = 1;
        }
        else
        {
            roi_x1 = x;
            roi_y1 = y;
            startDraw = 0;
        }
    }
    if (event == CV_EVENT_MOUSEMOVE && startDraw)
    {
        //redraw ROI selection
        image2 = image.clone();

        imshow(window_name, image2);
        image2.~Mat();
    }

}

int
main(int argc, char** argv)
{
    int iKey = 0;
    string input_file;
    string output_file;
    ifstream input;
    ofstream output;
    int num = 0;

    if (argc != 3)
    {
        cerr << argv[0] << " input_info.txt output_info.txt" << endl;
        return -1;
    }

    input_file = argv[1];
    output_file = argv[2];

    cerr << "Opening input_file with names of images" << endl;
    input.open(input_file.c_str());
    cerr << "Done." << endl;

    output.open(output_file.c_str());

    namedWindow(window_name, 1);

    if (input.is_open())
    {
        string line;

        getline(input, line);

        while (!input.eof())
        {
            cout << "Linha: " << line << endl;
            string s;
            vector<string> strings;
            istringstream iss(line);
            while (getline(iss, s, ' '))
            {
                cout << "Leitura: " << s << endl;
                strings.push_back(s);
            }
            numOfRec = 0;

            cerr << "Loading image :" << strings.at(0) << endl;

            image = imread(strings.at(0), 1);

            if (strings.at(0) != old_string)
            {
                old_string = strings.at(0);
                output << old_string << endl;
            }
            if (!image.empty())
            {
                cout << "x1 " << strings.at(1) << endl;
                int x1 = atoi(strings.at(1).c_str());
                cout << "y1 " << strings.at(2) << endl;
                int y1 = atoi(strings.at(2).c_str());
                cout << "x2 " << strings.at(3) << endl;
                int x2 = atoi(strings.at(3).c_str());
                cout << "y2 " << strings.at(4) << endl;
                int y2 = atoi(strings.at(4).c_str());

                Rect myROI(cvPoint(x1, y1), cvPoint(x2, y2));
                cv::Mat croppedImage;

                cv::Mat(image, myROI).copyTo(croppedImage);
                num++;
                ostringstream myStream;
                myStream << num << flush;
                imwrite("positives/image" + myStream.str() + ".png", croppedImage);

                rectangle(image, cvPoint(x1, y1), cvPoint(x2, y2), CV_RGB(255, 0, 255), 1);
                imshow(window_name, image);

                iKey = waitKey(1500);

                if (iKey == 27)
                {
                    image.release();
                    destroyWindow(window_name);
                    return EXIT_SUCCESS;
                }
            }
            getline(input, line);
            image.~Mat();
        }
    }
    else
    {
        cerr << "Failed to open: " << input_file << endl;
    }

    image.~Mat();
    image2.~Mat();
    input.close();
    output.close();
    destroyWindow(window_name);

    return EXIT_SUCCESS;
}
