#include "carmen_ipc_bridge/platform.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/TimeReference.h>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include "carmen_ipc_bridge/carmen_time.hpp"
#include <csignal>
#include <sstream>
#include <stdexcept>
#include <limits>

// Module state: all application callbacks run on the main thread.
// LiDAR calibration and scan timestamp policy.
static int lidar_sensor_id = -1;
static double lidar_range_division_factor{0.0};
static double lidar_max_range{0.0};
static double lidar_time_between_shots{0.0};
static int lidar_shot_size = 0;

static bool use_exact_lidar_timestamp;

static int lidar_num_rays = 0;
static std::vector<float> ring_cos_el;
static std::vector<float> ring_sin_el;

// Raw channel index -> elevation-ranked ring in the ROS cloud.
static std::vector<int> ring_of_raw;

static std::vector<float> ring_cos_az_off;
static std::vector<float> ring_sin_az_off;
// Raw cloud and time publishers.
static ros::Publisher clock_pub;
static ros::Publisher points_raw_publisher;

static ros::Publisher scan_time_ref_pub;

// ROS frame names.
static std::string laser_frame_id;

// Timestamp diagnostics and graceful shutdown.
static double scan_period_ema_s = 0.05;
static volatile sig_atomic_t shutdown_requested = 0;
static const int ipc_wait_ms = 1;

///////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

// Select the exact end stamp or subtract the configured scan duration.
static int64_t
scan_start_stamp_ns(double carmen_ts_end, int num_shots)
{
    static double last_carmen_ts = 0.0;
    if (last_carmen_ts > 0.0)
    {
        double delta_s = carmen_ts_end - last_carmen_ts;
        if (delta_s > 0.005 && delta_s < 0.5)
            scan_period_ema_s = 0.9 * scan_period_ema_s + 0.1 * delta_s;
    }
    last_carmen_ts = carmen_ts_end;

    if (use_exact_lidar_timestamp)
    {
        return carmen_bridge::carmen_to_ros_ns(carmen_ts_end);
    }

    const double scan_duration =
        lidar_time_between_shots * num_shots;
    const double scan_start = carmen_ts_end - scan_duration;
    ROS_DEBUG_THROTTLE(5.0,
                       "scan stamp: end=%.3f duration=%.4f start=%.3f ema=%.4f",
                       carmen_ts_end, scan_duration, scan_start, scan_period_ema_s);
    return carmen_bridge::carmen_to_ros_ns(scan_start);
}

// Reject incomplete tables, non-finite angles and extra values.
static std::vector<double>
parse_lidar_table(const char *text, int count, const std::string &parameter)
{
    if (text == nullptr)
        throw std::runtime_error(parameter + " ausente");
    std::istringstream input(text);
    std::vector<double> values;
    for (int i = 0; i < count; ++i)
    {
        double value;
        if (!(input >> value) || !std::isfinite(value))
            throw std::runtime_error(parameter + " deve conter " + std::to_string(count) + " valores finitos");
        values.push_back(value);
    }
    input >> std::ws;
    if (!input.eof())
        throw std::runtime_error(parameter + " contem valores extras ou texto invalido");
    return values;
}

// Validate calibration before changing any lookup table.
static void
configure_lidar_angles(const carmen_lidar_config *cfg)
{
    if (cfg->shot_size <= 0)
        throw std::runtime_error("shot_size deve ser positivo");
    if (cfg->shot_size - 1 > std::numeric_limits<uint16_t>::max())
        throw std::runtime_error("shot_size excede o campo ring UINT16 da nuvem ROS");
    if (!cfg->vertical_angles || !cfg->horizontal_angles_deltas || !cfg->ray_order)
        throw std::runtime_error("vertical_angles, horizontal_angles e ray_order sao obrigatorios");
    std::vector<int> order(cfg->shot_size);
    std::vector<bool> seen(cfg->shot_size, false);
    for (int i = 0; i < cfg->shot_size; ++i)
    {
        int raw = cfg->ray_order[i];
        if (raw < 0 || raw >= cfg->shot_size || seen[raw])
            throw std::runtime_error("ray_order deve ser uma permutacao dos canais");
        if (!std::isfinite(cfg->vertical_angles[i]) || !std::isfinite(cfg->horizontal_angles_deltas[i]))
            throw std::runtime_error("angulos do LiDAR devem ser finitos");
        seen[raw] = true;
        order[i] = raw;
    }
    std::stable_sort(order.begin(), order.end(), [cfg](int a, int b)
    {
        return cfg->vertical_angles[a] < cfg->vertical_angles[b];
    });
    ring_cos_el.resize(cfg->shot_size);
    ring_sin_el.resize(cfg->shot_size);
    ring_cos_az_off.resize(cfg->shot_size);
    ring_sin_az_off.resize(cfg->shot_size);
    ring_of_raw.resize(cfg->shot_size);
    for (int i = 0; i < cfg->shot_size; ++i)
    {
        float elevation = static_cast<float>(cfg->vertical_angles[i]) * static_cast<float>(M_PI) / 180.0f;
        float azimuth = -static_cast<float>(cfg->horizontal_angles_deltas[i]) * static_cast<float>(M_PI) / 180.0f;
        ring_cos_el[i] = std::cos(elevation);
        ring_sin_el[i] = std::sin(elevation);
        ring_cos_az_off[i] = std::cos(azimuth);
        ring_sin_az_off[i] = std::sin(azimuth);
        ring_of_raw[order[i]] = i;
    }
    lidar_num_rays = cfg->shot_size;
}

// Preserve the 32-byte x/y/z/intensity/ring/time layout and float arithmetic.
static sensor_msgs::PointCloud2
create_pointcloud(size_t point_count, const ros::Time &stamp)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = laser_frame_id;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = 32;

    cloud.fields.resize(6);
    cloud.fields[0].name = "x";         cloud.fields[0].offset = 0;  cloud.fields[0].datatype = 7;
    cloud.fields[0].count = 1;
    cloud.fields[1].name = "y";         cloud.fields[1].offset = 4;  cloud.fields[1].datatype = 7;
    cloud.fields[1].count = 1;
    cloud.fields[2].name = "z";         cloud.fields[2].offset = 8;  cloud.fields[2].datatype = 7;
    cloud.fields[2].count = 1;
    cloud.fields[3].name = "intensity"; cloud.fields[3].offset = 12; cloud.fields[3].datatype = 7;
    cloud.fields[3].count = 1;
    cloud.fields[4].name = "ring";      cloud.fields[4].offset = 16; cloud.fields[4].datatype = 4;
    cloud.fields[4].count = 1;
    cloud.fields[5].name = "time";      cloud.fields[5].offset = 20; cloud.fields[5].datatype = 7;
    cloud.fields[5].count = 1;

    if (point_count > std::numeric_limits<uint32_t>::max() / cloud.point_step)
        throw std::runtime_error("nuvem excede o tamanho suportado por PointCloud2");
    cloud.data.resize(point_count * cloud.point_step, 0);
    return cloud;
}

// Convert one CARMEN return directly into the ROS data buffer.
static void
append_point(sensor_msgs::PointCloud2 &cloud, uint32_t distance, uint8_t raw_intensity,
             int channel, int shot, int shot_size, float cos_az, float sin_az)
{
    float range = static_cast<float>(distance) / static_cast<float>(lidar_range_division_factor);
    if (range > static_cast<float>(lidar_max_range) || channel < 0 || channel >= lidar_shot_size)
        return;

    float cos_el = ring_cos_el[channel];
    float sin_el = ring_sin_el[channel];
    float cos_az_off = ring_cos_az_off[channel];
    float sin_az_off = ring_sin_az_off[channel];
    float cos_az_pt = cos_az * cos_az_off - sin_az * sin_az_off;
    float sin_az_pt = sin_az * cos_az_off + cos_az * sin_az_off;
    float x = range * cos_el * cos_az_pt;
    float y = range * cos_el * sin_az_pt;
    float z = range * sin_el;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        return;

    float intensity = static_cast<float>(raw_intensity);
    uint16_t ring_id = static_cast<uint16_t>(ring_of_raw[channel]);
    const float shot_period = static_cast<float>(lidar_time_between_shots);
    const float time_per_point = shot_period / static_cast<float>(shot_size);
    float point_time = static_cast<float>(shot) * shot_period
                     + static_cast<float>(channel) * time_per_point;
    uint8_t *output_point = cloud.data.data() + static_cast<size_t>(cloud.width) * cloud.point_step;
    std::memcpy(output_point + 0, &x, 4);
    std::memcpy(output_point + 4, &y, 4);
    std::memcpy(output_point + 8, &z, 4);
    std::memcpy(output_point + 12, &intensity, 4);
    std::memcpy(output_point + 16, &ring_id, 2);
    std::memcpy(output_point + 20, &point_time, 4);
    ++cloud.width;
}

// Trim rejected returns and report the cloud rate.
static void
finish_pointcloud(sensor_msgs::PointCloud2 &cloud)
{
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(cloud.row_step);
    const ros::Time &stamp = cloud.header.stamp;
    static uint64_t scan_count = 0;
    static double log_start_time = 0.0;
    ++scan_count;
    const double now_s = carmen_bridge::use_sim_time()
        ? stamp.toSec() : ros::Time::now().toSec();
    if (log_start_time == 0.0)
        log_start_time = now_s;
    if (now_s - log_start_time >= 1.0)
    {
        ROS_INFO("[DBG-CLOUD] Hz=%.1f  pontos_validos=%u  stamp=%.3f",
                 static_cast<double>(scan_count) / (now_s - log_start_time), cloud.width, stamp.toSec());
        scan_count = 0;
        log_start_time = now_s;
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////
// Publishers

// Publish clock and CARMEN time reference before the cloud.
static void
publish_lidar_scan(sensor_msgs::PointCloud2 &cloud, double timestamp, const std::string &source)
{
    finish_pointcloud(cloud);
    const ros::Time &stamp = cloud.header.stamp;
    carmen_bridge::publish_clock(clock_pub, stamp);

    sensor_msgs::TimeReference tref;
    tref.header.stamp = stamp;
    tref.header.frame_id = laser_frame_id;
    tref.time_ref.fromSec(timestamp);
    tref.source = source;
    scan_time_ref_pub.publish(tref);
    points_raw_publisher.publish(cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers

// Convert fixed 32-ray scans with the original constant intensity.
static void
velodyne_partial_scan_handler(carmen_velodyne_partial_scan_message *msg)
{
    if (!msg || !msg->partial_scan)
        return;
    const int num_shots = msg->number_of_32_laser_shots;
    if (num_shots <= 0 || num_shots > 10000)
        return;

    try
    {
        const ros::Time stamp = carmen_bridge::carmen_ns_to_ros_time(
            scan_start_stamp_ns(msg->timestamp, num_shots));
        auto cloud = create_pointcloud(static_cast<size_t>(num_shots) * 32, stamp);
        for (int i = 0; i < num_shots; ++i)
        {
            float az_deg = msg->partial_scan[i].angle;
            float az_rad = -az_deg * static_cast<float>(M_PI) / 180.0f;
            float cos_az = std::cos(az_rad);
            float sin_az = std::sin(az_rad);
            for (int j = 0; j < 32; ++j)
                append_point(cloud, msg->partial_scan[i].distance[j], 100, j, i, 32, cos_az, sin_az);
        }
        publish_lidar_scan(cloud, msg->timestamp, "partial_scan");
    }
    catch (const std::exception &error)
    {
        ROS_ERROR("Erro ao converter partial_scan: %s", error.what());
    }
}

// Convert variable scans directly, preserving per-shot channel counts.
static void
velodyne_variable_scan_handler(carmen_velodyne_variable_scan_message *msg)
{
    if (!msg || !msg->partial_scan || msg->number_of_shots <= 0)
        return;
    const int num_shots = msg->number_of_shots;
    ROS_INFO_THROTTLE(1.0,
                      "[DBG-SCAN-1] CARMEN->pointcloud: num_shots=%d  ts_carmen=%.3f",
                      num_shots, msg->timestamp);
    try
    {
        size_t point_count = 0;
        for (int i = 0; i < num_shots; ++i)
        {
            const auto &shot = msg->partial_scan[i];
            if (shot.shot_size > 0 && shot.shot_size <= lidar_shot_size && shot.distance)
                point_count += shot.shot_size;
        }
        ROS_INFO_THROTTLE(1.0, "[DBG-SCAN-2] raw_pts=%zu  (shots=%d)", point_count, num_shots);
        const ros::Time stamp = carmen_bridge::carmen_ns_to_ros_time(
            scan_start_stamp_ns(msg->timestamp, num_shots));
        if (point_count == 0)
            return;
        auto cloud = create_pointcloud(point_count, stamp);
        for (int i = 0; i < num_shots; ++i)
        {
            const auto &shot = msg->partial_scan[i];
            if (shot.shot_size <= 0 || shot.shot_size > lidar_shot_size || !shot.distance)
                continue;
            float az_deg = shot.angle;
            float az_rad = -az_deg * static_cast<float>(M_PI) / 180.0f;
            float cos_az = std::cos(az_rad);
            float sin_az = std::sin(az_rad);
            for (int j = 0; j < shot.shot_size; ++j)
            {
                uint8_t intensity = shot.intensity ? static_cast<uint8_t>(shot.intensity[j]) : 0;
                append_point(cloud, shot.distance[j], intensity, j, i, shot.shot_size, cos_az, sin_az);
            }
        }
        publish_lidar_scan(cloud, msg->timestamp, "variable_scan:" + std::to_string(lidar_sensor_id));
    }
    catch (const std::exception &error)
    {
        ROS_ERROR("Erro ao converter variable_scan: %s", error.what());
    }
}

// Defer ROS and IPC cleanup to main.
static void
shutdown_module(int signal_number)
{
    shutdown_requested = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations

// Read ROS output settings and CARMEN LiDAR calibration.
static void
read_parameters(int argc, char **argv)
{
    // ROS parameters.
    ros::NodeHandle private_node("~");
    if (!private_node.getParam("laser_frame_id", laser_frame_id) || !(!laser_frame_id.empty()))
    {
        ROS_FATAL("Parametro ROS ~laser_frame_id ausente ou invalido.");
        ros::shutdown();
        return;
    }
    if (!private_node.getParam("lidar_sensor_id", lidar_sensor_id) || !(lidar_sensor_id >= 0))
    {
        ROS_FATAL("Parametro ROS ~lidar_sensor_id ausente ou invalido.");
        ros::shutdown();
        return;
    }
    if (!private_node.getParam("use_exact_lidar_timestamp", use_exact_lidar_timestamp) || !(true))
    {
        ROS_FATAL("Parametro ROS ~use_exact_lidar_timestamp ausente ou invalido.");
        ros::shutdown();
        return;
    }

    // CARMEN parameters: only fields consumed by this converter.
    std::string section = "lidar" + std::to_string(lidar_sensor_id);
    int range_division_factor = 0;
    char *vertical_text = nullptr, *horizontal_text = nullptr, *order_text = nullptr;
    carmen_param_t parameters[] = {
        {section.data(), (char *) "shot_size", CARMEN_PARAM_INT, &lidar_shot_size, 0, nullptr},
        {section.data(), (char *) "range_division_factor", CARMEN_PARAM_INT, &range_division_factor, 0, nullptr},
        {section.data(), (char *) "max_range", CARMEN_PARAM_DOUBLE, &lidar_max_range, 0, nullptr},
        {section.data(), (char *) "time_between_shots", CARMEN_PARAM_DOUBLE, &lidar_time_between_shots, 0, nullptr},
        {section.data(), (char *) "vertical_angles", CARMEN_PARAM_STRING, &vertical_text, 0, nullptr},
        {section.data(), (char *) "ray_order", CARMEN_PARAM_STRING, &order_text, 0, nullptr},
    };
    carmen_param_allow_unfound_variables(0);
    carmen_param_install_params(argc, argv, parameters, sizeof(parameters) / sizeof(parameters[0]));

    carmen_param_t optional_parameters[] = {
        {section.data(), (char *) "horizontal_angles", CARMEN_PARAM_STRING, &horizontal_text, 0, nullptr},
    };
    carmen_param_allow_unfound_variables(1);
    carmen_param_install_params(argc, argv, optional_parameters, 1);
    carmen_param_allow_unfound_variables(0);
    try
    {
        if (lidar_shot_size <= 0)
            throw std::runtime_error("shot_size deve ser positivo");
        if (range_division_factor <= 0)
            throw std::runtime_error("range_division_factor deve ser positivo");
        if (!std::isfinite(lidar_max_range) || lidar_max_range <= 0.0)
            throw std::runtime_error("max_range deve ser finito e positivo");
        if (!std::isfinite(lidar_time_between_shots) || lidar_time_between_shots <= 0.0)
            throw std::runtime_error("time_between_shots deve ser finito e positivo");
        auto vertical = parse_lidar_table(vertical_text, lidar_shot_size, section + "_vertical_angles");
        std::vector<double> horizontal(lidar_shot_size, 0.0);
        if (horizontal_text != nullptr)
            horizontal = parse_lidar_table(horizontal_text, lidar_shot_size, section + "_horizontal_angles");
        else
            ROS_WARN("%s_horizontal_angles nao informado; usando zero nos %d canais.",
                     section.c_str(), lidar_shot_size);
        auto order_values = parse_lidar_table(order_text, lidar_shot_size, section + "_ray_order");
        std::vector<int> order(lidar_shot_size);
        for (int i = 0; i < lidar_shot_size; ++i)
        {
            double value = order_values[i];
            if (value < 0 || value >= lidar_shot_size || std::floor(value) != value)
                throw std::runtime_error("ray_order deve conter indices inteiros validos");
            order[i] = static_cast<int>(value);
        }
        carmen_lidar_config config{};
        config.shot_size = lidar_shot_size;
        config.vertical_angles = vertical.data();
        config.horizontal_angles_deltas = horizontal.data();
        config.ray_order = order.data();
        configure_lidar_angles(&config);
        lidar_range_division_factor = range_division_factor;
    }
    catch (const std::exception &error)
    {
        ROS_FATAL("Configuracao %s invalida: %s", section.c_str(), error.what());
        ros::shutdown();
    }
    free(vertical_text);
    free(horizontal_text);
    free(order_text);
}

// Create raw cloud and time publishers.
static void
setup_publishers(ros::NodeHandle &node)
{
    clock_pub = node.advertise<rosgraph_msgs::Clock>("/clock", 10);
    points_raw_publisher = node.advertise<sensor_msgs::PointCloud2>("/points_raw", 5);
    scan_time_ref_pub = node.advertise<sensor_msgs::TimeReference>("/carmen/scan_time_reference", 20);
}

// Subscribe to both scan formats.
static void
subscribe_to_ipc_message()
{
    carmen_velodyne_subscribe_partial_scan_message(
        nullptr,
        reinterpret_cast<carmen_handler_t>(velodyne_partial_scan_handler),
        CARMEN_SUBSCRIBE_LATEST);
    carmen_velodyne_subscribe_variable_scan_message(
        nullptr,
        reinterpret_cast<carmen_handler_t>(velodyne_variable_scan_handler),
        CARMEN_SUBSCRIBE_LATEST, lidar_sensor_id);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "carmen_to_ros_lidar_raw", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    carmen_ipc_initialize(argc, argv);
    read_parameters(argc, argv);
    if (!ros::ok())
    {
        IPC_disconnect();
        return 1;
    }
    setup_publishers(node);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    subscribe_to_ipc_message();

    while (ros::ok() && !shutdown_requested)
    {
        IPC_listenWait(ipc_wait_ms);
    }
    IPC_disconnect();
    ros::shutdown();
    return 0;
}
