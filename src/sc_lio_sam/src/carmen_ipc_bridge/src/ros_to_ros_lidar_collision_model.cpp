#include "carmen_ipc_bridge/platform.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include "carmen_ipc_bridge/carmen_time.hpp"
#include <csignal>

struct trailer_model
{
    std::string uri;
    tf2::Vector3 scale;
    tf2::Vector3 bbox_center;
    double off_x{0.0}, off_y{0.0}, off_z{0.0};
    double roll{0.0}, pitch{0.0}, yaw{0.0};
    double d{0.0}, hitch_offset{0.0};
};

// Raw mesh parameters; geometry preparation performs no parameter reads.
struct mesh_parameters
{
    char *file = nullptr;
    double x = 0.0, y = 0.0, z = 0.0;
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    double size_x = 0.0, size_y = 0.0, size_z = 0.0;
    double d = 0.0, hitch_offset = 0.0;
};
static mesh_parameters car_parameters;
static std::vector<mesh_parameters> trailer_parameters;
static int num_trailers = 0;

// Collision parameters can arrive after startup; refresh only those on demand.
static void read_parameters(int argc, char **argv, bool collision_only = false);

// Module state: all application callbacks run on the main thread.
// LiDAR mounting pose used by the filter and static transforms.

static int lidar_sensor_id = -1;

static double lidar_x_relative_to_sb1{0.0}, lidar_y_relative_to_sb1{0.0}, lidar_z_relative_to_sb1{0.0};
static double lidar_roll_relative_to_sb1{0.0};
static double lidar_pitch_relative_to_sb1{0.0};
static double lidar_yaw_relative_to_sb1{0.0};

// ROS publishers and visualization transport.

static ros::Publisher points_no_vehicle_pub;

static ros::Publisher collision_markers_pub;
static ros::Publisher car_model_pub;

static std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster;

// ROS frame names.
static std::string laser_frame_id;
static std::string base_frame_id;
static std::string sensorboard_frame_id;
static std::string ground_frame_id;
static std::string map_ground_frame_id;
static std::string map_base_link_frame_id;

// Collision cylinders, ground transforms and articulation.
static carmen_collision_config_t *collision_cfg{nullptr};
static tf2::Transform ground_from_lidar;
static tf2::Transform ground_from_robot;
static tf2::Transform robot_from_lidar;
static bool collision_ready{false};
static double robot_wheel_radius{0.0};
static double sb1_x{0.0}, sb1_y{0.0}, sb1_z{0.0};
static double sb1_roll{0.0}, sb1_pitch{0.0}, sb1_yaw{0.0};
static double trailer_beta{0.0};
static double trailer_betas[MAX_NUM_TRAILERS] = {};
static bool trailer_engaged{false};
static double robot_collision_height{0.0};
static double semi_trailer_collision_height{0.0};
static double collision_safety_margin{0.0};
static bool collision_heights_loaded{false};
static bool static_tfs_sent{false};

// Viewer-compatible truck and trailer mesh state.
static bool publish_car_model{true};
static std::string car_model_file_override;
static std::string car_model_frame_id;
static std::string car_model_uri;
static visualization_msgs::Marker car_model_marker;
static bool car_model_ready{false};

static std::vector<trailer_model> trailer_models;
static ros::Publisher car_model_trailers_pub;
static ros::WallTime last_trailer_model_pub{ros::WallTime(0)};
// Graceful shutdown.
static volatile sig_atomic_t shutdown_requested = 0;
static const int ipc_wait_ms = 1;

///////////////////////////////////////////////////////////////////////////////////////////////
// Utilities

// Read geometry vertices only; ignore texture coordinates and normals.
static bool
obj_bounding_box(const std::string &path, double bb_min[3], double bb_max[3])
{
    FILE *f = fopen(path.c_str(), "r");
    if (!f)
        return false;

    bool first = true;
    char line[1024];
    while (fgets(line, sizeof(line), f))
    {
        if (line[0] != 'v' || (line[1] != ' ' && line[1] != '\t'))
            continue;
        double x, y, z;
        if (sscanf(line + 1, "%lf %lf %lf", &x, &y, &z) != 3)
            continue;
        const double v[3] = {x, y, z};
        for (int i = 0; i < 3; ++i)
        {
            if (first || v[i] < bb_min[i])
                bb_min[i] = v[i];
            if (first || v[i] > bb_max[i])
                bb_max[i] = v[i];
        }
        first = false;
    }
    fclose(f);
    return !first;
}

// Match viewer_3D unitization, scaling and relative paths under CARMEN_HOME/bin.
static bool
prepare_obj_model(const std::string &raw_file, double size_x, double size_y, double size_z,
                  std::string &uri, tf2::Vector3 &scale, tf2::Vector3 &center,
                  tf2::Vector3 &dims_m, std::string &err)
{
    std::string file = raw_file;
    if (!file.empty() && file[0] != '/')
    {
#ifdef MAKE_IN_CARMEn
        const char *carmen_home = getenv("CARMEN_HOME");
#else
        const char *carmen_home = getenv("ASTRO_HOME");
#endif
        std::string bin_dir = carmen_home ? (std::string(carmen_home) + "/bin") : std::string(".");
        file = bin_dir + "/" + file;
    }

    double bb_min[3] = {0, 0, 0}, bb_max[3] = {0, 0, 0};
    if (!obj_bounding_box(file, bb_min, bb_max))
    {
        err = "nao consegui abrir/medir '" + file + "'";
        return false;
    }

    if (size_x <= 0.0 || size_y <= 0.0 || size_z <= 0.0)
    {
        err = "size_x/y/z invalido";
        return false;
    }

    // Match glmUnitize: absolute extents, not max minus min.
    const double w = std::abs(bb_max[0]) + std::abs(bb_min[0]);
    const double h = std::abs(bb_max[1]) + std::abs(bb_min[1]);
    const double d = std::abs(bb_max[2]) + std::abs(bb_min[2]);
    const double unit_scale = 2.0 / std::max(w, std::max(h, d));

    scale.setValue(unit_scale * size_x / 2.0,
                   unit_scale * size_y / 2.0,
                   unit_scale * size_z / 2.0);
    center.setValue((bb_max[0] + bb_min[0]) / 2.0,
                    (bb_max[1] + bb_min[1]) / 2.0,
                    (bb_max[2] + bb_min[2]) / 2.0);
    dims_m.setValue((bb_max[0] - bb_min[0]) * scale.x(),
                    (bb_max[1] - bb_min[1]) * scale.y(),
                    (bb_max[2] - bb_min[2]) * scale.z());
    uri = "file://" + file;
    return true;
}

// Retry ROS collision-height parameters until both vehicle heights are available.
static bool
ensure_collision_heights()
{
    if (collision_heights_loaded)
        return true;
    read_parameters(0, nullptr, true);
    return collision_heights_loaded;
}

// Test ground-relative cylinders for the truck and the first engaged trailer.
static bool
point_on_vehicle(float x, float y, float z)
{
    if (!collision_cfg)
        return false;

    const tf2::Vector3 point_ground = ground_from_lidar * tf2::Vector3(x, y, z);
    const double point_x = point_ground.x();
    const double point_y = point_ground.y();
    const double height = point_ground.z();

    if (height >= 0.0 && height <= robot_collision_height)
    {
        for (int i = 0; i < collision_cfg->n_markers; ++i)
        {
            const double dx = point_x - collision_cfg->markers[i].x;
            const double dy = point_y - collision_cfg->markers[i].y;
            const double r = collision_cfg->markers[i].radius + collision_safety_margin;
            if (dx * dx + dy * dy <= r * r)
                return true;
        }
    }

    if (trailer_engaged && collision_cfg->semi_trailer_type >= 1 &&
        height >= 0.0 && height <= semi_trailer_collision_height)
    {
        const double beta = trailer_beta;
        const double d = collision_cfg->semi_trailer_d[0];
        const double hitch_offset = collision_cfg->semi_trailer_M[0];
        const double trailer_yaw = -beta;
        const double cb = std::cos(trailer_yaw), sb = std::sin(trailer_yaw);
        const int n = collision_cfg->n_semi_trailer_markers[0];
        const carmen_collision_marker_t *markers = collision_cfg->semi_trailer_markers[0];
        for (int i = 0; i < n; ++i)
        {
            const double cx = (markers[i].x - d) * cb - markers[i].y * sb - hitch_offset;
            const double cy = (markers[i].x - d) * sb + markers[i].y * cb;
            const double dx = point_x - cx;
            const double dy = point_y - cy;
            const double r = markers[i].radius + collision_safety_margin;
            if (dx * dx + dy * dy <= r * r)
                return true;
        }
    }

    return false;
}

// Copy surviving point records unchanged, preserving ring, intensity and timing.
static sensor_msgs::PointCloud2
filter_vehicle_points(const sensor_msgs::PointCloud2 &cloud, uint32_t &removed)
{
    const uint32_t point_step = cloud.point_step;
    const uint32_t point_count = cloud.width;
    sensor_msgs::PointCloud2 filtered_cloud;
    filtered_cloud.header = cloud.header;
    filtered_cloud.height = 1;
    filtered_cloud.fields = cloud.fields;
    filtered_cloud.is_bigendian = cloud.is_bigendian;
    filtered_cloud.point_step = point_step;
    filtered_cloud.is_dense = cloud.is_dense;
    filtered_cloud.data.resize(cloud.data.size());

    const uint8_t *input_data = cloud.data.data();
    uint8_t       *output_point = filtered_cloud.data.data();
    uint32_t kept = 0;
    removed = 0;

    for (uint32_t i = 0; i < point_count; ++i)
    {
        const uint8_t *input_point = input_data + static_cast<size_t>(i) * point_step;
        float x, y, z;
        std::memcpy(&x, input_point + 0, 4);
        std::memcpy(&y, input_point + 4, 4);
        std::memcpy(&z, input_point + 8, 4);

        if (point_on_vehicle(x, y, z))
        {
            ++removed;
            continue;
        }

        std::memcpy(output_point, input_point, point_step);
        output_point += point_step;
        ++kept;
    }

    filtered_cloud.width = kept;
    filtered_cloud.row_step = kept * point_step;
    filtered_cloud.data.resize(static_cast<size_t>(kept) * point_step);
    return filtered_cloud;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Publishers

// Republish the latched truck mesh when its configuration is ready.
static void
publish_car_model_marker()
{
    if (!car_model_ready)
        return;
    car_model_pub.publish(car_model_marker);
}

// Draw the articulated mesh chain, or delete its markers when disengaged.
static void
publish_trailer_model_markers()
{
    if (trailer_models.empty())
        return;

    visualization_msgs::MarkerArray markers_message;

    if (!trailer_engaged)
    {
        for (size_t i = 0; i < trailer_models.size(); ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = car_model_frame_id;
            marker.header.stamp = ros::Time(0);
            marker.ns = "car_model_trailer";
            marker.id = static_cast<int>(i);
            marker.action = visualization_msgs::Marker::DELETE;
            markers_message.markers.push_back(marker);
        }
        car_model_trailers_pub.publish(markers_message);
        return;
    }

    double beta[MAX_NUM_TRAILERS];
    for (int i = 0; i < MAX_NUM_TRAILERS; ++i)
        beta[i] = trailer_betas[i];

    const double car_x = trailer_models[0].off_x;
    const double car_y = trailer_models[0].off_y;
    const double car_z = trailer_models[0].off_z;

    const size_t n = trailer_models.size();
    std::vector<tf2::Vector3> hop(n);
    for (size_t i = 0; i < n; ++i)
    {
        const double cb = std::cos(beta[i]), sb = std::sin(beta[i]);
        const double prev_x = (i == 0) ? car_x : (-car_x * cb) + (car_y * sb) + car_x;
        const double prev_y = (i == 0) ? car_y : (car_x * sb) - (car_y * cb);

        const double st_x = prev_x - (trailer_models[i].d + trailer_models[i].hitch_offset * cb);
        const double st_y = prev_y + trailer_models[i].hitch_offset * sb;

        hop[i].setValue(st_x, -st_y, car_z);
    }

    for (size_t k = 0; k < n; ++k)
    {
        const trailer_model &model = trailer_models[k];

        tf2::Transform transform;
        transform.setIdentity();
        for (size_t i = 0; i <= k; ++i)
        {
            tf2::Quaternion qb;
            qb.setRPY(0.0, 0.0, -beta[i]);
            transform *= tf2::Transform(qb, tf2::Vector3(0, 0, 0));
            transform *= tf2::Transform(tf2::Quaternion(0, 0, 0, 1), hop[i]);
        }

        tf2::Quaternion q_yaw, q_pitch, q_roll;
        q_yaw.setRPY(0.0, 0.0, -model.yaw);
        q_pitch.setRPY(M_PI / 2.0 - model.pitch, 0.0, 0.0);
        q_roll.setRPY(0.0, 0.0, -model.roll);
        transform *= tf2::Transform(q_yaw * q_pitch * q_roll, tf2::Vector3(0, 0, 0));

        const tf2::Vector3 c_scaled(model.bbox_center.x() * model.scale.x(),
                                    model.bbox_center.y() * model.scale.y(),
                                    model.bbox_center.z() * model.scale.z());
        const tf2::Vector3 pos = transform.getOrigin() - transform.getBasis() * c_scaled;

        visualization_msgs::Marker marker;
        marker.header.frame_id = car_model_frame_id;
        marker.header.stamp = ros::Time(0);
        marker.ns = "car_model_trailer";
        marker.id = static_cast<int>(k);
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.mesh_resource = model.uri;
        marker.mesh_use_embedded_materials = true;
        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();
        marker.pose.orientation = tf2::toMsg(transform.getRotation());
        marker.scale.x = model.scale.x();
        marker.scale.y = model.scale.y();
        marker.scale.z = model.scale.z();
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 0.0;
        marker.lifetime = ros::Duration(0);
        marker.frame_locked = true;
        markers_message.markers.push_back(marker);
    }

    car_model_trailers_pub.publish(markers_message);
}

// Publish sensor, ground and inverse map transforms once per process.
static void
publish_static_transforms(const ros::Time &stamp)
{
    const ros::Time tf_time = carmen_bridge::tf_stamp(stamp);

    tf2::Quaternion q_tf_robot;
    q_tf_robot.setRPY(sb1_roll, sb1_pitch, sb1_yaw);
    geometry_msgs::TransformStamped tf_robot;
    tf_robot.header.stamp = tf_time;
    tf_robot.header.frame_id = base_frame_id;
    tf_robot.child_frame_id = sensorboard_frame_id;
    tf_robot.transform.translation.x = sb1_x;
    tf_robot.transform.translation.y = sb1_y;
    tf_robot.transform.translation.z = sb1_z;
    tf_robot.transform.rotation = tf2::toMsg(q_tf_robot);
    static_tf_broadcaster->sendTransform(tf_robot);

    geometry_msgs::TransformStamped tf_lidar;
    tf_lidar.header.stamp = tf_time;
    tf_lidar.header.frame_id = base_frame_id;
    tf_lidar.child_frame_id = laser_frame_id;
    tf_lidar.transform.translation.x = robot_from_lidar.getOrigin().x();
    tf_lidar.transform.translation.y = robot_from_lidar.getOrigin().y();
    tf_lidar.transform.translation.z = robot_from_lidar.getOrigin().z();
    tf_lidar.transform.rotation.x = robot_from_lidar.getRotation().x();
    tf_lidar.transform.rotation.y = robot_from_lidar.getRotation().y();
    tf_lidar.transform.rotation.z = robot_from_lidar.getRotation().z();
    tf_lidar.transform.rotation.w = robot_from_lidar.getRotation().w();
    static_tf_broadcaster->sendTransform(tf_lidar);

    tf2::Quaternion q_tf_gnd;
    q_tf_gnd.setRPY(0.0, 0.0, 0.0);
    geometry_msgs::TransformStamped tf_gnd;
    tf_gnd.header.stamp = tf_time;
    tf_gnd.header.frame_id = base_frame_id;
    tf_gnd.child_frame_id = ground_frame_id;
    tf_gnd.transform.translation.x = 0.0;
    tf_gnd.transform.translation.y = 0.0;
    tf_gnd.transform.translation.z = -robot_wheel_radius;
    tf_gnd.transform.rotation = tf2::toMsg(q_tf_gnd);
    static_tf_broadcaster->sendTransform(tf_gnd);

    const tf2::Transform lidar_from_robot = robot_from_lidar.inverse();
    geometry_msgs::TransformStamped tf_map_base;
    tf_map_base.header.stamp = tf_time;
    tf_map_base.header.frame_id = "map";
    tf_map_base.child_frame_id = map_base_link_frame_id;
    tf_map_base.transform.translation.x = lidar_from_robot.getOrigin().x();
    tf_map_base.transform.translation.y = lidar_from_robot.getOrigin().y();
    tf_map_base.transform.translation.z = lidar_from_robot.getOrigin().z();
    tf_map_base.transform.rotation.x = lidar_from_robot.getRotation().x();
    tf_map_base.transform.rotation.y = lidar_from_robot.getRotation().y();
    tf_map_base.transform.rotation.z = lidar_from_robot.getRotation().z();
    tf_map_base.transform.rotation.w = lidar_from_robot.getRotation().w();
    static_tf_broadcaster->sendTransform(tf_map_base);

    const tf2::Transform robot_from_ground = ground_from_robot.inverse();
    geometry_msgs::TransformStamped tf_base_gnd_map;
    tf_base_gnd_map.header.stamp = tf_time;
    tf_base_gnd_map.header.frame_id = map_base_link_frame_id;
    tf_base_gnd_map.child_frame_id = map_ground_frame_id;
    tf_base_gnd_map.transform.translation.x = robot_from_ground.getOrigin().x();
    tf_base_gnd_map.transform.translation.y = robot_from_ground.getOrigin().y();
    tf_base_gnd_map.transform.translation.z = robot_from_ground.getOrigin().z();
    tf_base_gnd_map.transform.rotation.x = robot_from_ground.getRotation().x();
    tf_base_gnd_map.transform.rotation.y = robot_from_ground.getRotation().y();
    tf_base_gnd_map.transform.rotation.z = robot_from_ground.getRotation().z();
    tf_base_gnd_map.transform.rotation.w = robot_from_ground.getRotation().w();
    static_tf_broadcaster->sendTransform(tf_base_gnd_map);
}

// Show the same cylinders used by the vehicle filter.
static void
publish_collision_markers(const ros::Time &stamp)
{
    if (stamp.isZero())
        return;
    if (!collision_ready || !ensure_collision_heights())
        return;

    visualization_msgs::MarkerArray markers_message;
    int id = 0;

    for (int i = 0; i < collision_cfg->n_markers; ++i)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = base_frame_id;
        marker.header.stamp = stamp;
        marker.ns = "collision_truck";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = collision_cfg->markers[i].x;
        marker.pose.position.y = collision_cfg->markers[i].y;
        marker.pose.position.z = -robot_wheel_radius + (robot_collision_height / 2.0);
        marker.pose.orientation.w = 1.0;

        double r = collision_cfg->markers[i].radius + collision_safety_margin;
        marker.scale.x = marker.scale.y = 2.0 * r;
        marker.scale.z = robot_collision_height;
        marker.color.r = 1.0; marker.color.a = 0.35;
        marker.lifetime = ros::Duration(0);
        markers_message.markers.push_back(marker);
    }

    if (trailer_engaged && collision_cfg->semi_trailer_type >= 1)
    {
        const double beta = trailer_beta;
        const double d = collision_cfg->semi_trailer_d[0];
        const double hitch_offset = collision_cfg->semi_trailer_M[0];
        const double trailer_yaw = -beta;
        const double cb = std::cos(trailer_yaw), sb = std::sin(trailer_yaw);
        const int n = collision_cfg->n_semi_trailer_markers[0];
        const carmen_collision_marker_t *markers = collision_cfg->semi_trailer_markers[0];
        for (int i = 0; i < n; ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = base_frame_id;
            marker.header.stamp = stamp;
            marker.ns = "collision_trailer";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = (markers[i].x - d) * cb - markers[i].y * sb - hitch_offset - sb1_x;
            marker.pose.position.y = (markers[i].x - d) * sb + markers[i].y * cb - sb1_y;
            marker.pose.position.z = -robot_wheel_radius + (semi_trailer_collision_height / 2.0);
            marker.pose.orientation.w = 1.0;

            double r = markers[i].radius + collision_safety_margin;
            marker.scale.x = marker.scale.y = 2.0 * r;
            marker.scale.z = semi_trailer_collision_height;
            marker.color.b = 1.0; marker.color.a = 0.35;
            marker.lifetime = ros::Duration(0);
            markers_message.markers.push_back(marker);
        }
    }

    collision_markers_pub.publish(markers_message);
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Handlers

// Update articulation and refresh trailer meshes at up to 20 Hz.
static void
update_trailer_pose(const double *betas, int n, bool engaged)
{
    for (int i = 0; i < MAX_NUM_TRAILERS; ++i)
        trailer_betas[i] = i < n ? betas[i] : 0.0;
    trailer_beta = n > 0 ? betas[0] : 0.0;
    trailer_engaged = engaged;

    if (!trailer_models.empty())
    {
        const ros::WallTime now = ros::WallTime::now();
        if ((now - last_trailer_model_pub).toSec() >= 0.05)
        {
            publish_trailer_model_markers();
            last_trailer_model_pub = now;
        }
    }
}

// Convert trailer headings into successive articulation angles.
static void
localize_ackerman_globalpos_handler(carmen_localize_ackerman_globalpos_message *msg)
{
    if (!msg)
        return;

    double betas[MAX_NUM_TRAILERS];
    double prev_theta = msg->globalpos.theta;
    for (int i = 0; i < MAX_NUM_TRAILERS; ++i)
    {
        betas[i] = convert_theta1_to_beta(prev_theta, msg->trailer_theta[i]);
        prev_theta = msg->trailer_theta[i];
    }
    update_trailer_pose(betas, MAX_NUM_TRAILERS, msg->semi_trailer_engaged != 0);
}

// Pass through until collision geometry is ready; otherwise remove vehicle points.
static void
points_raw_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    const ros::Time stamp = msg->header.stamp;
    if (!static_tfs_sent)
    {
        publish_static_transforms(stamp);
        static_tfs_sent = true;
    }
    static ros::Time last_marker_pub(0);
    const ros::Time ref_now = carmen_bridge::use_sim_time() ? stamp : ros::Time::now();

    if (ref_now < last_marker_pub)
    {
        ROS_WARN("Tempo andou pra tras (%.3f < %.3f) -- playback reiniciado? "
                 "Rearmando a publicacao dos markers de colisao.",
                 ref_now.toSec(), last_marker_pub.toSec());
        last_marker_pub = ros::Time(0);
    }

    if ((ref_now - last_marker_pub).toSec() >= 0.05)
    {
        publish_collision_markers(stamp);
        last_marker_pub = ref_now;
    }

    const uint32_t point_step = msg->point_step;
    const uint32_t point_count = msg->width;

    if (!collision_ready || !ensure_collision_heights() || point_step < 12 || point_count == 0)
    {
        points_no_vehicle_pub.publish(*msg);
        return;
    }

    uint32_t removed = 0;
    sensor_msgs::PointCloud2 filtered_cloud = filter_vehicle_points(*msg, removed);
    points_no_vehicle_pub.publish(filtered_cloud);

    static uint64_t scan_count = 0;
    static double log_start_time = 0.0;
    ++scan_count;
    const double now_s = carmen_bridge::use_sim_time()
        ? msg->header.stamp.toSec() : ros::Time::now().toSec();
    if (log_start_time == 0.0)
        log_start_time = now_s;
    if (now_s - log_start_time >= 1.0)
    {
        ROS_INFO("[DBG-COLLISION] Hz=%.1f  pontos=%u  removidos=%u  trailer_engaged=%d",
                 static_cast<double>(scan_count) / (now_s - log_start_time), point_count, removed,
                 static_cast<int>(trailer_engaged));
        scan_count = 0;
        log_start_time = now_s;
    }
}

// Refresh meshes using wall time, even when playback is paused.
static void
car_model_timer_handler(const ros::WallTimerEvent &event)
{
    publish_car_model_marker();
    publish_trailer_model_markers();
}

// Defer ROS and IPC cleanup to main.
static void
shutdown_module(int signal_number)
{
    shutdown_requested = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// Initializations

// Create the original cloud, clock, time-reference and visualization publishers.
static void
setup_publishers(ros::NodeHandle &node)
{
    static_tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    points_no_vehicle_pub = node.advertise<sensor_msgs::PointCloud2>("/points_raw_no_vehicle", 5);

    collision_markers_pub = node.advertise<visualization_msgs::MarkerArray>("/collision_model_markers", 1);

    car_model_pub = node.advertise<visualization_msgs::Marker>("/car_model_marker", 1, true);
    car_model_trailers_pub =
        node.advertise<visualization_msgs::MarkerArray>("/car_model_trailer_markers", 1, true);
}

// Compose LiDAR-to-ground transforms from the loaded vehicle geometry.
static void
setup_collision_filter()
{
    tf2::Quaternion q_sb;
    q_sb.setRPY(sb1_roll, sb1_pitch, sb1_yaw);
    tf2::Transform robot_from_board(q_sb, tf2::Vector3(sb1_x, sb1_y, sb1_z));

    tf2::Quaternion q_bl;
    q_bl.setRPY(lidar_roll_relative_to_sb1, lidar_pitch_relative_to_sb1, lidar_yaw_relative_to_sb1);
    tf2::Transform board_from_lidar(q_bl, tf2::Vector3(lidar_x_relative_to_sb1,
                                                       lidar_y_relative_to_sb1,
                                                       lidar_z_relative_to_sb1));

    robot_from_lidar = robot_from_board * board_from_lidar;

    tf2::Quaternion q_gnd;
    q_gnd.setRPY(0, 0, 0);
    ground_from_robot.setOrigin(tf2::Vector3(0, 0, robot_wheel_radius));
    ground_from_robot.setRotation(q_gnd);

    ground_from_lidar = ground_from_robot * robot_from_board * board_from_lidar;

    collision_cfg = carmen_collision_detection_get_global_collision_config();
    if (!collision_cfg)
    {
        ROS_ERROR("Nao foi possivel obter o modelo de colisao (collision_config). "
                  "Filtro de pontos sobre o veiculo ficara desativado (pass-through).");
        return;
    }

    collision_ready = true;
    ROS_INFO(
        "Filtro de colisao pronto: wheel_radius=%.3fm, board_1=(%.3f, %.3f, %.3f), "
        "n_markers_caminhao=%d, semi_trailer_type=%d",
        robot_wheel_radius, sb1_x, sb1_y, sb1_z,
        collision_cfg->n_markers, collision_cfg->semi_trailer_type);
}

// Prepare trailer meshes from the loaded model parameters.
static void
setup_trailer_models()
{
    if (num_trailers <= 0)
    {
        ROS_INFO("semi_trailer_initial_type=%d -- veiculo sem semi-reboque, "
                 "so' o modelo do cavalo sera' publicado.", num_trailers);
        return;
    }
    if (num_trailers > MAX_NUM_TRAILERS)
        num_trailers = MAX_NUM_TRAILERS;

    for (int id = 1; id <= num_trailers; ++id)
    {
        const mesh_parameters &parameters = trailer_parameters[id - 1];
        const auto file = parameters.file;
        const auto d = parameters.d;
        const auto hitch_offset = parameters.hitch_offset;
        const auto sx = parameters.size_x;
        const auto sy = parameters.size_y;
        const auto sz = parameters.size_z;
        const auto ox = parameters.x;
        const auto oy = parameters.y;
        const auto oz = parameters.z;
        const auto roll = parameters.roll;
        const auto pitch = parameters.pitch;
        const auto yaw = parameters.yaw;
        char sec_model[64];
        snprintf(sec_model, sizeof(sec_model), "semi_trailer_model%d", id);

        if (!file || file[0] == '\0')
        {
            ROS_WARN("%s_file_name nao encontrado -- semi-reboque %d nao sera' desenhado.",
                     sec_model, id);
            break;
        }

        trailer_model model;
        tf2::Vector3 dims_m;
        std::string err;
        if (!prepare_obj_model(file, sx, sy, sz, model.uri, model.scale, model.bbox_center, dims_m, err))
        {
            ROS_ERROR("Semi-reboque %d desativado: %s.", id, err.c_str());
            break;
        }

        model.off_x = ox; model.off_y = oy; model.off_z = oz;
        model.roll = roll; model.pitch = pitch; model.yaw = yaw;
        model.d = d; model.hitch_offset = hitch_offset;
        trailer_models.push_back(model);

        ROS_INFO("Semi-reboque %d pronto: %s | d=%.3f M=%.3f | offset=(%.2f, %.2f, %.2f) "
                 "yaw=%.3f | bbox final=(%.2f x %.2f x %.2f) m",
                 id, model.uri.c_str(), d, hitch_offset, ox, oy, oz, yaw,
                 dims_m.x(), dims_m.y(), dims_m.z());
    }

    publish_trailer_model_markers();
}

// Match viewer_3D mesh pose, scale and centering.
static void
setup_car_model()
{
    auto carmodel_file = car_parameters.file;
    auto cm_x = car_parameters.x;
    auto cm_y = car_parameters.y;
    auto cm_z = car_parameters.z;
    auto cm_roll = car_parameters.roll;
    auto cm_pitch = car_parameters.pitch;
    auto cm_yaw = car_parameters.yaw;
    auto cm_size_x = car_parameters.size_x;
    auto cm_size_y = car_parameters.size_y;
    auto cm_size_z = car_parameters.size_z;

    std::string file = car_model_file_override.empty()
                     ? (carmodel_file ? std::string(carmodel_file) : std::string())
                     : car_model_file_override;
    if (file.empty())
    {
        ROS_WARN("carmodel_file_name nao encontrado no param_daemon e nenhum "
                 "~car_model_file definido -- modelo 3D do robo desativado.");
        return;
    }

    if (cm_size_x <= 0.0 || cm_size_y <= 0.0 || cm_size_z <= 0.0)
    {
        ROS_WARN("carmodel_size_* invalido/ausente (%.3f, %.3f, %.3f) -- usando "
                 "as dimensoes cruas do .obj.", cm_size_x, cm_size_y, cm_size_z);
        cm_size_x = cm_size_y = cm_size_z = 2.0;
    }

    tf2::Vector3 scale, bbox_center, dims_m;
    std::string uri, err;
    if (!prepare_obj_model(file, cm_size_x, cm_size_y, cm_size_z,
                           uri, scale, bbox_center, dims_m, err))
    {
        ROS_ERROR("Modelo 3D do robo desativado: %s. (O RViz tambem precisa "
                  "enxergar esse caminho.)", err.c_str());
        return;
    }
    const double sx = scale.x(), sy = scale.y(), sz = scale.z();

    if (std::abs(cm_roll) > 1e-9 || std::abs(cm_pitch) > 1e-9)
    {
        ROS_WARN("carmodel_roll/pitch (%.4f, %.4f) sao ignorados -- o viewer_3D "
                 "tambem so' usa carmodel_yaw para o carro.", cm_roll, cm_pitch);
    }
    tf2::Quaternion q;
    q.setRPY(M_PI / 2.0, 0.0, -cm_yaw);
    const tf2::Matrix3x3 rotation(q);

    const tf2::Vector3 p0(cm_x, cm_y, cm_z + robot_wheel_radius);

    const tf2::Vector3 c_scaled(bbox_center.x() * sx,
                                bbox_center.y() * sy,
                                bbox_center.z() * sz);
    const tf2::Vector3 t = p0 - rotation * c_scaled;

    car_model_uri = uri;

    visualization_msgs::Marker &marker = car_model_marker;
    marker.header.frame_id = car_model_frame_id;
    marker.header.stamp = ros::Time(0);
    marker.ns = "car_model";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.mesh_resource = car_model_uri;
    marker.mesh_use_embedded_materials = true;
    marker.pose.position.x = t.x();
    marker.pose.position.y = t.y();
    marker.pose.position.z = t.z();
    marker.pose.orientation = tf2::toMsg(q);
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;

    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 0.0;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = true;

    car_model_ready = true;
    ROS_INFO("Modelo 3D do robo pronto: %s | escala=(%.6f, %.6f, %.6f) | "
             "pose no %s=(%.3f, %.3f, %.3f) yaw=%.3f rad | bbox final=(%.2f x %.2f x %.2f) m",
             car_model_uri.c_str(), sx, sy, sz, car_model_frame_id.c_str(),
             t.x(), t.y(), t.z(), cm_yaw,
             dims_m.x(), dims_m.y(), dims_m.z());

    publish_car_model_marker();
    setup_trailer_models();
}

// Read ROS settings and CARMEN sensor/vehicle parameters in one place.
static void
read_parameters(int argc, char **argv, bool collision_only)
{
    // ROS collision heights may be published after this node starts.
    if (!collision_heights_loaded)
    {
        double rh = 0.0, th = 0.0;
        if (ros::param::get("/lio_sam/robot_collision_height", rh) &&
            ros::param::get("/lio_sam/semi_trailer_collision_height", th) &&
            rh > 0.0 && th > 0.0)
        {
            robot_collision_height = rh;
            semi_trailer_collision_height = th;
            double margin = 0.0;
            ros::param::get("/lio_sam/collision_safety_margin", margin);
            collision_safety_margin = (margin > 0.0) ? margin : 0.0;
            collision_heights_loaded = true;
            ROS_INFO("Alturas de colisao: robot=%.2fm, semi_trailer=%.2fm, margem=%.2fm",
                     rh, th, collision_safety_margin);
        }
    }
    if (collision_only)
        return;

    // ROS parameters.
    ros::NodeHandle private_node("~");

    if (!private_node.getParam("laser_frame_id", laser_frame_id) || !(!laser_frame_id.empty()))
    {
        ROS_FATAL("Parametro ROS ~laser_frame_id ausente ou invalido.");
        ros::shutdown();
        return;
    }
    private_node.param<std::string>("base_frame_id", base_frame_id, "base_link");
    private_node.param<std::string>("sensorboard_frame_id", sensorboard_frame_id, "sensor_board_1");
    private_node.param<std::string>("ground_frame_id", ground_frame_id, "ground_link");
    private_node.param<std::string>("map_ground_frame_id_", map_ground_frame_id, "map_ground_link");
    private_node.param<std::string>("map_base_link_frame_id", map_base_link_frame_id, "map_base_link");
    if (!private_node.getParam("lidar_sensor_id", lidar_sensor_id) || !(lidar_sensor_id >= 0))
    {
        ROS_FATAL("Parametro ROS ~lidar_sensor_id ausente ou invalido.");
        ros::shutdown();
        return;
    }
    private_node.param<bool>("publish_car_model", publish_car_model, true);
    private_node.param<std::string>("car_model_file", car_model_file_override, "");
    private_node.param<std::string>("car_model_frame_id", car_model_frame_id, base_frame_id);

    // CARMEN LiDAR mounting pose and vehicle geometry.

    std::string section = "lidar" + std::to_string(lidar_sensor_id);
    carmen_param_t lidar_parameters[] = {
        {section.data(), (char *) "x", CARMEN_PARAM_DOUBLE, &lidar_x_relative_to_sb1, 0, nullptr},
        {section.data(), (char *) "y", CARMEN_PARAM_DOUBLE, &lidar_y_relative_to_sb1, 0, nullptr},
        {section.data(), (char *) "z", CARMEN_PARAM_DOUBLE, &lidar_z_relative_to_sb1, 0, nullptr},
        {section.data(), (char *) "roll", CARMEN_PARAM_DOUBLE, &lidar_roll_relative_to_sb1, 0, nullptr},
        {section.data(), (char *) "pitch", CARMEN_PARAM_DOUBLE, &lidar_pitch_relative_to_sb1, 0, nullptr},
        {section.data(), (char *) "yaw", CARMEN_PARAM_DOUBLE, &lidar_yaw_relative_to_sb1, 0, nullptr},
    };
    carmen_param_allow_unfound_variables(0);
    carmen_param_install_params(argc, argv, lidar_parameters,
                               sizeof(lidar_parameters) / sizeof(lidar_parameters[0]));
    for (const auto &parameter : lidar_parameters)
    {
        if (!std::isfinite(*static_cast<double *>(parameter.user_variable)))
        {
            ROS_FATAL("Parametro %s_%s deve ser finito.", section.c_str(), parameter.variable);
            ros::shutdown();
            return;
        }
    }

    {
        carmen_param_t param_list[] = {
            {(char *) "sensor_board_1", (char *) "x", CARMEN_PARAM_DOUBLE, &sb1_x, 0, NULL},
            {(char *) "sensor_board_1", (char *) "y", CARMEN_PARAM_DOUBLE, &sb1_y, 0, NULL},
            {(char *) "sensor_board_1", (char *) "z", CARMEN_PARAM_DOUBLE, &sb1_z, 0, NULL},
            {(char *) "sensor_board_1", (char *) "roll", CARMEN_PARAM_DOUBLE, &sb1_roll, 0, NULL},
            {(char *) "sensor_board_1", (char *) "pitch", CARMEN_PARAM_DOUBLE, &sb1_pitch, 0, NULL},
            {(char *) "sensor_board_1", (char *) "yaw", CARMEN_PARAM_DOUBLE, &sb1_yaw, 0, NULL},
            {(char *) "robot", (char *) "wheel_radius", CARMEN_PARAM_DOUBLE, &robot_wheel_radius, 0, NULL},
        };
        carmen_param_allow_unfound_variables(1);
        carmen_param_install_params(0, nullptr, param_list,
                                    sizeof(param_list) / sizeof(param_list[0]));
        carmen_param_allow_unfound_variables(0);
    }

    if (!publish_car_model)
        return;

    // CARMEN visualization models.
    {
        carmen_param_allow_unfound_variables(1);
        carmen_param_t param_list[] = {
            {(char *) "carmodel", (char *) "file_name", CARMEN_PARAM_STRING, &car_parameters.file, 0, NULL},
            {(char *) "carmodel", (char *) "size_x", CARMEN_PARAM_DOUBLE, &car_parameters.size_x, 0, NULL},
            {(char *) "carmodel", (char *) "size_y", CARMEN_PARAM_DOUBLE, &car_parameters.size_y, 0, NULL},
            {(char *) "carmodel", (char *) "size_z", CARMEN_PARAM_DOUBLE, &car_parameters.size_z, 0, NULL},
            {(char *) "carmodel", (char *) "x", CARMEN_PARAM_DOUBLE, &car_parameters.x, 0, NULL},
            {(char *) "carmodel", (char *) "y", CARMEN_PARAM_DOUBLE, &car_parameters.y, 0, NULL},
            {(char *) "carmodel", (char *) "z", CARMEN_PARAM_DOUBLE, &car_parameters.z, 0, NULL},
            {(char *) "carmodel", (char *) "roll", CARMEN_PARAM_DOUBLE, &car_parameters.roll, 0, NULL},
            {(char *) "carmodel", (char *) "pitch", CARMEN_PARAM_DOUBLE, &car_parameters.pitch, 0, NULL},
            {(char *) "carmodel", (char *) "yaw", CARMEN_PARAM_DOUBLE, &car_parameters.yaw, 0, NULL},
        };
        carmen_param_install_params(0, nullptr, param_list,
                                    sizeof(param_list) / sizeof(param_list[0]));
        carmen_param_allow_unfound_variables(0);
    }
    carmen_param_allow_unfound_variables(1);
    carmen_param_t n_list[] = {
        {(char *) "semi_trailer", (char *) "initial_type", CARMEN_PARAM_INT, &num_trailers, 0, NULL},
    };
    carmen_param_install_params(0, nullptr, n_list, 1);
    carmen_param_allow_unfound_variables(0);

    num_trailers = std::max(0, std::min(num_trailers, MAX_NUM_TRAILERS));
    trailer_parameters.resize(num_trailers);
    for (int id = 1; id <= num_trailers; ++id)
    {
        char sec_trailer[64], sec_model[64];
        snprintf(sec_trailer, sizeof(sec_trailer), "semi_trailer%d", id);
        snprintf(sec_model, sizeof(sec_model), "semi_trailer_model%d", id);

        mesh_parameters &parameters = trailer_parameters[id - 1];
        carmen_param_allow_unfound_variables(1);
        carmen_param_t p[] = {
            {sec_trailer, (char *) "d", CARMEN_PARAM_DOUBLE, &parameters.d, 0, NULL},
            {sec_trailer, (char *) "M", CARMEN_PARAM_DOUBLE, &parameters.hitch_offset, 0, NULL},
            {sec_model, (char *) "file_name", CARMEN_PARAM_STRING, &parameters.file, 0, NULL},
            {sec_model, (char *) "size_x", CARMEN_PARAM_DOUBLE, &parameters.size_x, 0, NULL},
            {sec_model, (char *) "size_y", CARMEN_PARAM_DOUBLE, &parameters.size_y, 0, NULL},
            {sec_model, (char *) "size_z", CARMEN_PARAM_DOUBLE, &parameters.size_z, 0, NULL},
            {sec_model, (char *) "x", CARMEN_PARAM_DOUBLE, &parameters.x, 0, NULL},
            {sec_model, (char *) "y", CARMEN_PARAM_DOUBLE, &parameters.y, 0, NULL},
            {sec_model, (char *) "z", CARMEN_PARAM_DOUBLE, &parameters.z, 0, NULL},
            {sec_model, (char *) "roll", CARMEN_PARAM_DOUBLE, &parameters.roll, 0, NULL},
            {sec_model, (char *) "pitch", CARMEN_PARAM_DOUBLE, &parameters.pitch, 0, NULL},
            {sec_model, (char *) "yaw", CARMEN_PARAM_DOUBLE, &parameters.yaw, 0, NULL},
        };
        carmen_param_install_params(0, nullptr, p, sizeof(p) / sizeof(p[0]));
        carmen_param_allow_unfound_variables(0);
    }
}

// Subscribe only to trailer articulation; clouds arrive through ROS.
static void
subscribe_to_ipc_message()
{
    carmen_localize_ackerman_subscribe_globalpos_message(
        nullptr,
        reinterpret_cast<carmen_handler_t>(localize_ackerman_globalpos_handler),
        CARMEN_SUBSCRIBE_LATEST);
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_to_ros_lidar_collision_model", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    setup_publishers(node);
    ros::Subscriber points_raw_subscriber = node.subscribe("/points_raw", 5, points_raw_handler);
    ros::WallTimer car_model_timer;
    if (publish_car_model)
        car_model_timer = node.createWallTimer(ros::WallDuration(1.0), car_model_timer_handler);
    if (!ros::ok())
    {
        IPC_disconnect();
        return 1;
    }
    
    read_parameters(argc, argv);
    carmen_ipc_initialize(argc, argv);
    signal(SIGINT, shutdown_module);
    signal(SIGTERM, shutdown_module);
    setup_collision_filter();
    if (publish_car_model)
        setup_car_model();
    subscribe_to_ipc_message();


    // ROS subscribers and wall timers share this thread with IPC handlers.
    while (ros::ok() && !shutdown_requested)
    {
        IPC_listenWait(ipc_wait_ms);
        ros::spinOnce();
    }
    IPC_disconnect();
    ros::shutdown();
    return 0;
}
