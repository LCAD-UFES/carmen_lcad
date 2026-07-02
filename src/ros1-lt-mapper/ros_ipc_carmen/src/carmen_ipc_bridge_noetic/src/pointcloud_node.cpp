/*
 * pointcloud_node.cpp  (ROS1 / Noetic)
 *
 * Nó dedicado à conversão 3D do LiDAR:
 *   Subscreve: /velodyne_raw_ipc  (dados brutos do ipc_bridge_node)
 *   Publica:   /points_raw        (sensor_msgs/PointCloud2 para o LIO-SAM)
 *
 * Toda a trigonometria pesada fica aqui, isolada do loop IPC.
 * Mesma lógica da versão ROS2, só com a API roscpp.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <cmath>
#include <cstring>
#include <vector>

// ─── Elevações Hesai XT-32 ────────────────────────────────────────────────────
static const float HESAI_XT32_ELEVATIONS_DEG[32] = {
     15.0f,  14.0f,  13.0f,  12.0f,
     11.0f,  10.0f,   9.0f,   8.0f,
      7.0f,   6.0f,   5.0f,   4.0f,
      3.0f,   2.0f,   1.0f,   0.0f,
     -1.0f,  -2.0f,  -3.0f,  -4.0f,
     -5.0f,  -6.0f,  -7.0f,  -8.0f,
     -9.0f, -10.0f, -11.0f, -12.0f,
    -13.0f, -14.0f, -15.0f, -16.0f
};

static float COS_ELEV[32];
static float SIN_ELEV[32];
static bool g_elev_init = []() {
    for (int k = 0; k < 32; ++k) {
        float er = HESAI_XT32_ELEVATIONS_DEG[k] * static_cast<float>(M_PI) / 180.0f;
        COS_ELEV[k] = std::cos(er);
        SIN_ELEV[k] = std::sin(er);
    }
    return true;
}();

class PointcloudNode
{
public:
    PointcloudNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh)
    {
        pnh_.param<std::string>("laser_frame_id", laser_frame_id_, "velodyne");
        pnh_.param<double>("range_div", range_div_, 500.0);
        pnh_.param<double>("max_range", max_range_,  70.0);
        pnh_.param<double>("min_range", min_range_,   0.1);

        sub_ = nh_.subscribe("/velodyne_raw_ipc", 5,
            &PointcloudNode::on_raw_cloud, this);

        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_raw", 5);

        ROS_INFO(
            "pointcloud_node iniciado: range_div=%.0f max_range=%.1fm",
            range_div_, max_range_);
    }

private:
    // Mede o período real entre scans consecutivos (EMA), em vez de
    // assumir um Hz fixo — se adapta a qualquer taxa real do sensor.
    float update_scan_period(const ros::Time &stamp)
    {
        double stamp_sec = stamp.toSec();
        if (last_stamp_sec_ > 0.0) {
            double delta = stamp_sec - last_stamp_sec_;
            if (delta > 0.005 && delta < 0.5)  // ignora outlier (stall/primeiro frame)
                scan_period_ema_ = 0.9f * scan_period_ema_ + 0.1f * static_cast<float>(delta);
        }
        last_stamp_sec_ = stamp_sec;
        return scan_period_ema_;
    }

    void on_raw_cloud(const sensor_msgs::PointCloud2::ConstPtr &raw)
    {
        // Lê campos do frame bruto:
        // shot_idx(uint16,off=0) ring(uint8,off=2) distance(uint16,off=4)
        // intensity(uint8,off=6) cos_az(float,off=8) sin_az(float,off=12)
        // point_step = 16

        const uint32_t n   = raw->width;
        const uint32_t ps  = raw->point_step;
        if (n == 0 || ps < 16) return;

        // Saída: x(4) y(4) z(4) intensity(4) ring(2) pad(2) time(4) pad(8) = 32 bytes
        sensor_msgs::PointCloud2 cloud;
        cloud.header        = raw->header;
        cloud.height        = 1;
        cloud.is_bigendian  = false;
        cloud.is_dense      = true;
        cloud.point_step    = 32;

        cloud.fields.resize(6);
        cloud.fields[0].name="x";         cloud.fields[0].offset=0;  cloud.fields[0].datatype=7; cloud.fields[0].count=1;
        cloud.fields[1].name="y";         cloud.fields[1].offset=4;  cloud.fields[1].datatype=7; cloud.fields[1].count=1;
        cloud.fields[2].name="z";         cloud.fields[2].offset=8;  cloud.fields[2].datatype=7; cloud.fields[2].count=1;
        cloud.fields[3].name="intensity"; cloud.fields[3].offset=12; cloud.fields[3].datatype=7; cloud.fields[3].count=1;
        cloud.fields[4].name="ring";      cloud.fields[4].offset=16; cloud.fields[4].datatype=4; cloud.fields[4].count=1;
        cloud.fields[5].name="time";      cloud.fields[5].offset=20; cloud.fields[5].datatype=7; cloud.fields[5].count=1;

        // Pre-aloca máximo
        cloud.data.resize(n * cloud.point_step, 0);

        const float scan_period = update_scan_period(raw->header.stamp);
        const float inv_total = (n > 1) ? scan_period / static_cast<float>(n - 1) : 0.0f;

        uint8_t       *out = cloud.data.data();
        const uint8_t *in  = raw->data.data();
        uint32_t valid = 0;

        for (uint32_t idx = 0; idx < n; ++idx) {
            const uint8_t *p = in + idx * ps;

            uint16_t dist_raw; std::memcpy(&dist_raw, p + 4, 2);
            uint8_t  ring_raw; std::memcpy(&ring_raw, p + 2, 1);
            uint8_t  inten;    std::memcpy(&inten,    p + 6, 1);
            float    cos_az;   std::memcpy(&cos_az,   p + 8, 4);
            float    sin_az;   std::memcpy(&sin_az,   p + 12,4);

            float r = static_cast<float>(dist_raw) / static_cast<float>(range_div_);
            if (r < static_cast<float>(min_range_) || r > static_cast<float>(max_range_))
                continue;
            if (!std::isfinite(cos_az) || !std::isfinite(sin_az)) continue;

            int ring = static_cast<int>(ring_raw);
            if (ring < 0 || ring >= 32) continue;

            float cos_el = COS_ELEV[ring];
            float sin_el = SIN_ELEV[ring];

            float x = r * cos_el * cos_az;
            float y = r * cos_el * sin_az;
            float z = r * sin_el;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            float intensity = static_cast<float>(inten);
            uint16_t ring16 = static_cast<uint16_t>(ring);
            float t = static_cast<float>(idx) * inv_total - scan_period;

            std::memcpy(out + 0,  &x,         4);
            std::memcpy(out + 4,  &y,         4);
            std::memcpy(out + 8,  &z,         4);
            std::memcpy(out + 12, &intensity,  4);
            std::memcpy(out + 16, &ring16,     2);
            // bytes 18-19: padding (zero)
            std::memcpy(out + 20, &t,          4);
            // bytes 24-31: padding (zero)

            out += cloud.point_step;
            ++valid;
        }

        cloud.width    = valid;
        cloud.row_step = valid * cloud.point_step;
        cloud.data.resize(cloud.row_step);

        // [DBG] Hz de publicação (1x/seg)
        static uint64_t cnt = 0; static double t0 = 0.0;
        ++cnt;
        double now_s = ros::Time::now().toSec();
        if (t0 == 0.0) t0 = now_s;
        if (now_s - t0 >= 1.0) {
            ROS_INFO(
                "[DBG-CLOUD] Hz=%.1f  pontos_validos=%u  stamp=%.3f",
                static_cast<double>(cnt) / (now_s - t0), valid,
                cloud.header.stamp.toSec());
            cnt = 0; t0 = now_s;
        }

        pub_.publish(cloud);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    std::string laser_frame_id_;
    double range_div_;
    double max_range_;
    double min_range_;

    // Estimativa dinâmica do período de scan, medida pelo intervalo real
    // entre headers consecutivos (em vez de assumir 10Hz fixo).
    double last_stamp_sec_ = -1.0;
    float scan_period_ema_ = 0.055f;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    PointcloudNode node(nh, pnh);
    ros::spin();
    return 0;
}
