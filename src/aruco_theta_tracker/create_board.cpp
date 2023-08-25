#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/aruco.h>


int main(int argc, char **argv)
{
    if (argc < 4)
    {
        printf("%s <x_size> <y_size> <OPTIONAL factor_quality> <out_file> <OPTIONAL comma_separated_ids>\n", argv[0]);
        return 1;
    }
    char *out_path;
    int x_size, y_size, factor_quality = 1;
    x_size = atoi(argv[1]); 
    y_size = atoi(argv[2]);
    if (argc >= 5)
    {
        factor_quality = atoi(argv[3]);
        out_path = argv[4];
    }
    else
    {
        out_path = argv[3];
    }

    aruco::Dictionary Dict = aruco::Dictionary::load("ARUCO_MIP_36h12"); // ARUCO_MIP_16h3 ARUCO_MIP_36h12
    int marker_size = 500*factor_quality;
    int inter_marker_distance = 100*factor_quality;
    std::vector<int> ids;

    if (argc == 6)
    {
        char *pch = strtok(argv[5], " ,.-");
        while (pch != NULL)
        {
            ids.push_back(atoi(pch));
            pch = strtok(NULL, " ,.-");
        }
    }
    else
    {
        for (int i = 1; i <= x_size * y_size; i++)
            ids.push_back(i);
    }
    ids.resize(x_size * y_size);

    aruco::MarkerMap B_info =
        Dict.createMarkerMap(cv::Size(x_size, y_size), marker_size, inter_marker_distance, ids, false);
    // create a printable image to save
    cv::Mat marker_map_image = B_info.getImage();
    // cv::copyMakeBorder(marker_map_image, marker_map_image, 0, 0, inter_marker_distance, 0, cv::BORDER_CONSTANT, 255);
    // cv::copyMakeBorder(marker_map_image, marker_map_image, 0, 0, inter_marker_distance/2, 0, cv::BORDER_CONSTANT, 0);

    char outfile[256];
    sprintf(outfile, "%s.yml", out_path);
    B_info.saveToFile(outfile);
    
    sprintf(outfile, "%s.png", out_path);
    cv::imwrite(outfile, marker_map_image);

    cv::threshold(marker_map_image, marker_map_image, 128, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat contour_output = marker_map_image.clone();
    cv::findContours(contour_output, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    sprintf(outfile, "%s.svg", out_path);
    FILE *fp = fopen(outfile, "w");
    fprintf(fp, "<svg width=\"%d\" height=\"%d\" xmlns=\"http://www.w3.org/2000/svg\">\n", marker_map_image.cols, marker_map_image.rows);
    fprintf(fp, "<rect width=\"100%%\" height=\"100%%\" fill=\"black\"/>");
    for(std::vector<cv::Point> c : contours)
    {
        fprintf(fp, "<path d=\"M");
        for(cv::Point p : c)
            fprintf(fp, " %d %d", p.x, p.y);
        fprintf(fp, "\" style=\"fill:white\"/>\n");
    }
    fprintf(fp, "</svg>");
    fclose(fp);

    cv::waitKey(0);
}

/*
./create_board 6 1 1 ~/Pictures/tags/front 1,2,3,4,5,6
./create_board 6 1 1 ~/Pictures/tags/left 7,8,9,10,11,12
./create_board 6 1 1 ~/Pictures/tags/right 13,14,15,16,17,18
*/