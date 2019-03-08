#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <carmen/segmap_util.h>

using namespace pcl;


int
main(int argc, char **argv)
{
    if (argc < 2)
        exit(printf("Use %s <graphslam-file>\n", argv[0]));

    char dummy[64];
    int n;
    double x, y, th;
    double x0 = 0., y0 = 0.;
    Matrix<double, 4, 4> pose;
    Eigen::Affine3f affine;

    FILE *f = safe_fopen(argv[1], "r");

    pcl::visualization::PCLVisualizer viewer("CloudViewer");
	viewer.setBackgroundColor(1, 1, 1);

    n = 0;

    while (!feof(f))
    {

        fscanf(f, "\n%s %lf %lf %lf %s %s\n", 
            dummy, &x, &y, &th, dummy, dummy);

        if (n == 0)
        {
            x0 = x;
            y0 = y;
            n = 1;
        }

        pose = pose3d_to_matrix(x - x0, y - y0, th);
	    affine = pose.cast<float>();
        viewer.addCoordinateSystem(1., affine);
    }

    viewer.spin();
    return 0;
}