#include "Scancontext.h"
#include <limits>

int main()
{
    const float cases[][3] = {{1, 0, 0}, {0, 1, 90}, {-1, 0, 180},
        {0, -1, 270}, {1, 1, 45}, {-1, 1, 135}, {-1, -1, 225},
        {1, -1, 315}, {0, 0, 0}, {-0.0f, -0.0f, 0}};
    for (const auto &sample : cases)
        if (std::abs(xy2theta(sample[0], sample[1]) - sample[2]) > 0.0001f)
            return 1;
    SCManager manager;
    pcl::PointCloud<SCPointType> cloud;
    SCPointType point;
    point.x = 2; point.y = 1; point.z = 3;
    cloud.push_back(point);
    auto expected = manager.makeScancontext(cloud);
    point.x = std::numeric_limits<float>::quiet_NaN();
    cloud.push_back(point);
    point.x = std::numeric_limits<float>::infinity();
    cloud.push_back(point);
    auto actual = manager.makeScancontext(cloud);
    if (!actual.allFinite() || !actual.isApprox(expected, 0.0))
        return 2;
    std::cout << "PASS angles, origin and nonfinite point rejection\n";
}
