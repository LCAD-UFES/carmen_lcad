
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>


using namespace cv;
using namespace std;


int img_x = -1, img_y = -1;
float pcl_x = -1, pcl_y = -1, pcl_z = -1;
string velodyne_path = "";
string image_path = "";
int point_id;


class PointPair
{
public:
	int img_x, img_y;
	float pcl_x, pcl_y, pcl_z;
	string velodyne_path, image_path;
};


vector<PointPair> points;


void 
pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    point_id = event.getPointIndex();

    float x, y, z;
    std::cout << "[INOF] Point picking event occurred." << std::endl;

    if (event.getPointIndex() == -1)
    {
    	printf("point not found!\n");
    	pcl_x = -1;
    	pcl_y = -1;
    	pcl_z = -1;
    }

    event.getPoint(x, y, z);
    std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;

    pcl_x = x;
    pcl_y = y;
    pcl_z = z;
}


void 
CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  (event == EVENT_LBUTTONUP)
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        img_x = x;
        img_y = y;
    }
}


void
add_point()
{
	if (pcl_x == -1)
		return;

	PointPair p;

	p.img_x = img_x;
	p.img_y = img_y;
	p.pcl_x = pcl_x;
	p.pcl_y = pcl_y;
	p.pcl_z = pcl_z;
	p.velodyne_path = velodyne_path;
	p.image_path = image_path;

	points.push_back(p);

	img_x = -1;
	img_y = -1;
	pcl_x = -1;
	pcl_y = -1;
	pcl_z = -1;
}


void
draw_point_in_pcl(pcl::visualization::PCLVisualizer &viewer, double x, double y, double z, unsigned char r, unsigned char g, unsigned char b, string id)
{
	pcl::PointXYZ center;

	center.x = x;
	center.y = y;
	center.z = z;

	viewer.addSphere(center, .1, r, g, b, id);
}


void
draw_saved_points(pcl::visualization::PCLVisualizer &viewer, Mat &img)
{
	char name[64];

	for (int i = 0; i < points.size(); i++)
	{
		if (points[i].velodyne_path == velodyne_path && points[i].image_path == image_path)
		{
			sprintf(name, "sphere_%d", i);
			draw_point_in_pcl(viewer, points[i].pcl_x, points[i].pcl_y, points[i].pcl_z, 255, 255, 0, name);
			cv::circle(img, cv::Point(points[i].img_x, points[i].img_y), 5, Scalar(0, 255, 255), -1);
		}
	}
}


void
append_points()
{
	FILE *f = fopen("points.txt", "a");

	for (int i = 0; i < points.size(); i++)
	{
		fprintf(f, "%s %s %lf %lf %lf %d %d\n",
				points[i].velodyne_path.c_str(),
				points[i].image_path.c_str(),
				points[i].pcl_x,
				points[i].pcl_y,
				points[i].pcl_z,
				points[i].img_x,
				points[i].img_y
		);
	}

	fclose(f);
}


int 
main (int argc, char** argv)
{
	char name[64];
    int32_t num = 1000000;
    float *data = (float*) malloc(num * sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen ("velodyne_0000000000.bin", "rb");
    num = fread(data, sizeof(float), num, stream) / 4;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int32_t i=0; i < num; i++) 
    {
        px += 4; py += 4; pz += 4; pr += 4;

        if ((*px) > 0)
        {
            double r = (*pr);
            double b = 1. - (*pr);

            pcl::PointXYZRGB point;
            point.x = *px;
            point.y = *py;
            point.z = *pz;
            point.r = 255 * r;
            point.g = 0;
            point.b = 255 * b;
            cloud->push_back(point);
        }
    }

    fclose(stream);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // theta radians around X axis
    // transform.rotate(Eigen::AngleAxisf(-M_PI / 2., Eigen::Vector3f::UnitX()));
    // transform.rotate(Eigen::AngleAxisf(M_PI / 2., Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    viewer.setBackgroundColor(1., 1., 1.);
    viewer.addPointCloud(transformed_cloud, "cloud");
    viewer.registerPointPickingCallback(pointPickingEventOccurred, (void*) &viewer);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

    Mat img = imread("cam0_0000000000.png");
    namedWindow("img", 1);
    setMouseCallback("img", CallBackFunc, NULL);

    point_id = -1;

    while (1)
    {
        Mat img_view = img.clone();
        
        viewer.removeAllShapes();

        draw_saved_points(viewer, img_view);

        if (point_id != -1)
        {
        	printf("POINT: %d %lf %lf %lf\n", point_id,
        			transformed_cloud->at(point_id).x,
					transformed_cloud->at(point_id).y,
					transformed_cloud->at(point_id).z);
        }

        if (pcl_x != -1 && pcl_y != -1 && pcl_z != -1)
        {
        	sprintf(name, "sphere_%ld", points.size());
        	draw_point_in_pcl(viewer, pcl_x, pcl_y, pcl_z, 0, 255, 0, name);
        }

        if (img_x != -1 && img_y != -1)
            cv::circle(img_view, cv::Point(img_x, img_y), 5, Scalar(0, 255, 0), -1);

        imshow("img", img_view);
        viewer.spinOnce();
        char c = waitKey(10);

        if (c == 'a' || c == 'A')
        	add_point();

        if (c == 'g' || c == 'G')
        	append_points();
    }

    return 0;
}


