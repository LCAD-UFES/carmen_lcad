/***************objectmarker.cpp******************

Objectmarker for marking the objects to be detected  from positive samples and then creating the 
description file for positive images.

compile this code and run with two arguments, first one the name of the descriptor file and the second one 
the address of the directory in which the positive images are located

while running this code, each image in the given directory will open up. Now mark the edges of the object using the mouse buttons
  then press then press "SPACE" to save the selected region, or any other key to discard it. Then use "B" to move to next image. the program automatically
  quits at the end. press ESC at anytime to quit.

 *the key B was chosen  to move to the next image because it is closer to SPACE key and nothing else.....

author: achu_wilson@rediffmail.com
 */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/legacy/legacy.hpp>


// for filelisting
#include <stdio.h>
#include <sys/io.h>
// for fileoutput
#include <string>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <sys/types.h>

using namespace std;
using namespace cv;

Mat image;
Mat image2;
//int start_roi=0;
int roi_x0 = 0;
int roi_y0 = 0;
int roi_x1 = 0;
int roi_y1 = 0;
int numOfRec = 0;
int startDraw = 0;
string window_name = "<SPACE>add, <B>save and <N>load next <ESC>exit";

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
        rectangle(image2, cvPoint(roi_x0, roi_y0), cvPoint(x, y), CV_RGB(255, 0, 255), 1);
        imshow(window_name, image2);
        image2.~Mat();
    }

}

int
main(int argc, char** argv)
{
    int iKey = 0;
    //string strPrefix;
    string strPostfix;
    string input_file;
    string output_file;
    ifstream input;
    ofstream output;
    
    if (argc != 3)
    {
        cerr << argv[0] << " output_info.txt input_info.txt" << endl;
        return -1;
    }

    input_file = argv[2];
    output_file = argv[1];

    cerr << "Opening input_file with names of images" << endl;
    input.open(input_file.c_str());
    cerr << "Done." << endl;

    namedWindow(window_name, 1);
    setMouseCallback(window_name, on_mouse, NULL);

    output.open(output_file.c_str(), ofstream::out | ofstream::app);

    if (output.is_open() && input.is_open())
    {
        string line;
        getline(input, line);
        while (!input.eof())
        {
            numOfRec = 0;

            strPostfix = "";

            cerr << "Loading image :" << line << endl;

            image = imread(line, 1);

            if (!image.empty())
            {
                //    work on current image
                iKey = -1;
                while ((iKey != 110 && iKey != 78))
                {
                    imshow(window_name, image);

                    // used cvWaitKey returns:                   
                    //    <ESC>=27      exit program
                    //    <Space>=32    add rectangle to current image
                    //    <B>=66        save added rectangles 
                    //    <N>=75        show next image
                    //  any other key clears rectangle drawing only
                    iKey = waitKey(0);

                    switch (iKey)
                    {

                    case 27:
                        image.release();
                        destroyWindow(window_name);
                        return EXIT_SUCCESS;
                    case 32:
                        numOfRec++;
                        cout << "Adicionou " << numOfRec << " " << roi_x0 << " " << roi_y0 << " " << roi_x1 << " " << roi_y1 << endl;
                        if (roi_x0 < roi_x1 && roi_y0 < roi_y1)
                        {

                            //printf("   %d. rect x=%d\ty=%d\twidth=%d\theight=%d\n", numOfRec, roi_x0, roi_y0, roi_x1 - roi_x0, roi_y1 - roi_y0);
                            cout << "   " << numOfRec << ". rect \t x= " << roi_x0 << "\t y = " << roi_y0 << "\t width = " << roi_x1 << "\theight = " << roi_y1 << endl;
                            // append rectangle coord to previous line content
                            strPostfix += " " + IntToString(roi_x0) + " " + IntToString(roi_y0) + " " + IntToString(roi_x1) + " " + IntToString(roi_y1);

                        }
                        else //(roi_x0>roi_x1 && roi_y0>roi_y1)
                        {
                            cout << " Size equals 0" << endl;
                            cout << "    " << numOfRec << ". rect \t x= " << roi_x1 << "\t y = " << roi_y1 << "\t width = " << roi_x0 << "\theight = " << roi_y0 << endl;
                            // append rectangle coord to previous line content
                            strPostfix += " " + IntToString(roi_x1) + " " + IntToString(roi_y1) + " " + IntToString(roi_x0 - roi_x1) + " " + IntToString(roi_y0 - roi_y1);
                        }
                        break;
                    case 66:
                    case 98:
                        if (numOfRec > 0)
                        {
                            if (!strPostfix.empty())
                            {
                                cout << "Saving " << line << strPostfix << endl;
                                output << line << strPostfix << endl;
                                strPostfix.clear();
                            }
                        }
                        break;
                    }
                }
                image.~Mat();                
            }

            
            getline(input, line);
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
