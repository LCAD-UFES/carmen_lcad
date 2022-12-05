#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>


using namespace std;
using namespace cv;

// board 6x1: XSize = 6, YSize = 1, interMarkerDistance = 60, pixSize = 210
// ./run <XSize> <YSize> <out.png> <out.yml> <factor_quality>
// ./run 6 1 out.png out.yml 10
int main(int argc, char **argv)
{
    try
    {
        int factor_quality = atoi(argv[5]);
        int XSize = atoi(argv[1]), 
            YSize = atoi(argv[2]);
        auto Dict = aruco::Dictionary::load("ARUCO_MIP_36h12");
        int pixSize = 150*factor_quality;
        int interMarkerDistance = 30*factor_quality;

        std::vector<int> ids = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
        ids.resize(XSize * YSize);

        aruco::MarkerMap BInfo =
            Dict.createMarkerMap(Size(XSize, YSize), pixSize, interMarkerDistance, ids, false);
        // create a printable image to save
        cv::Mat MarkerMapImage = BInfo.getImage();
        cv::copyMakeBorder(MarkerMapImage, MarkerMapImage, interMarkerDistance, interMarkerDistance+80*factor_quality, interMarkerDistance, interMarkerDistance, cv::BORDER_CONSTANT, 255);

        // save
        BInfo.saveToFile(argv[4]);
        cv::imwrite(argv[3], MarkerMapImage);
    }
    catch (std::exception &ex)
    {
        cout << ex.what() << endl;
    }
}
