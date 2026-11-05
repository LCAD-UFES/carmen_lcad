#pragma once
#include "image_projection_support.hpp"
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/registration/icp.h>
#include <pcl/common/angles.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <fstream>
#include <iomanip>
#include <queue>
#include <thread>

using dbl = std::numeric_limits<double>;

// ROS1 accepts integer YAML values for floating-point parameters.
inline double mapping_numeric_parameter(const string &name, double fallback)
{
    const auto &overrides = node->get_node_parameters_interface()->get_parameter_overrides();
    const auto entry = overrides.find(name);
    if (entry != overrides.end() && entry->second.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        return node->declare_parameter<int64_t>(name, static_cast<int64_t>(fallback));
    return node->declare_parameter<double>(name, fallback);
}

class MappingParameters : public ParamServer
{
public:
    std::string gpsTopic;
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    bool gpsAnchorIgnoreCov;
    float priorYawNoise;
    float priorPosNoise;
    float gpsMinBaseline;
    float gpsFactorSpacing;
    float gpsAnchorMinBaseline;
    int ackermannYawLockKeyframes;
    float ackermannYawWeight;
    float ackermannYawMaxCorrDeg;
    float ackermannPosWeight;
    float ackermannPosMaxCorr;
    float ackermannRejectYawDeg;
    float ackermannRejectPosM;
    float ackermannRejectPosGrowth;
    float ackermannRejectYawGrowthDeg;
    float ackermannWeightRefHz;
    float ackermannCurvBiasDist;
    float poseCovThreshold;
    bool useGpu;
    std::string map_directory;
    bool localizationMode;
    float initialPoseX;
    float initialPoseY;
    float initialPoseZ;
    float initialPoseRoll;
    float initialPosePitch;
    float initialPoseYaw;
    bool posesEnable;
    std::string posesOutputFile;
    double posesStartTime;
    double posesOriginX;
    double posesOriginY;
    double posesOriginZ;
    double posesOriginYaw;
    std::string posesOriginMode;
    bool posesOriginLock;
    bool posesOriginRefine;
    int posesOriginMinSamples;
    double posesOriginMaxDt;
    double posesOriginMinBaseline;
    double posesOriginRelPosNoise;
    int posesSettleScans;
    double posesInterpolateMaxGap;
    double posesInterpolateMaxError;
    std::string posesCarmenStampTopic;
    std::string posesAckermannTopic;
    float imuRPYWeight;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize;
    float z_tollerance;
    float rotation_tollerance;
    int numberOfCores;
    double mappingProcessInterval;
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    bool loopClosureEnableFlag;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;
    MappingParameters()
    {
        gpsTopic = node->declare_parameter<std::string>("gpsTopic", "odometry/gps");
        useImuHeadingInitialization = node->declare_parameter<bool>("useImuHeadingInitialization", false);
        useGpsElevation = node->declare_parameter<bool>("useGpsElevation", false);
        gpsCovThreshold = mapping_numeric_parameter("gpsCovThreshold", 2.0);
        gpsAnchorIgnoreCov = node->declare_parameter<bool>("gpsAnchorIgnoreCov", false);
        priorYawNoise = mapping_numeric_parameter("priorYawNoise", 1e-2);
        priorPosNoise = mapping_numeric_parameter("priorPosNoise", 1e-4);
        gpsMinBaseline = mapping_numeric_parameter("gpsMinBaseline", 30.0);
        gpsFactorSpacing = mapping_numeric_parameter("gpsFactorSpacing", 5.0);
        gpsAnchorMinBaseline = mapping_numeric_parameter("gpsAnchorMinBaseline", 15.0);
        ackermannYawLockKeyframes = node->declare_parameter<int>("ackermannYawLockKeyframes", 10);
        ackermannYawWeight = mapping_numeric_parameter("ackermannYawWeight", 1.0);
        ackermannYawMaxCorrDeg = mapping_numeric_parameter("ackermannYawMaxCorrDeg", 0.0);
        ackermannPosWeight = mapping_numeric_parameter("ackermannPosWeight", 1.0);
        ackermannPosMaxCorr = mapping_numeric_parameter("ackermannPosMaxCorr", 0.0);
        ackermannRejectYawDeg = mapping_numeric_parameter("ackermannRejectYawDeg", 0.0);
        ackermannRejectPosM = mapping_numeric_parameter("ackermannRejectPosM", 0.0);
        ackermannRejectPosGrowth = mapping_numeric_parameter("ackermannRejectPosGrowth", 0.0);
        ackermannRejectYawGrowthDeg = mapping_numeric_parameter("ackermannRejectYawGrowthDeg", 0.0);
        ackermannWeightRefHz = mapping_numeric_parameter("ackermannWeightRefHz", 0.0);
        ackermannCurvBiasDist = mapping_numeric_parameter("ackermannCurvBiasDist", 0.0);
        poseCovThreshold = mapping_numeric_parameter("poseCovThreshold", 25.0);
        useGpu = node->declare_parameter<bool>("useGpu", false);
        map_directory = node->declare_parameter<std::string>("map_directory");
        localizationMode = node->declare_parameter<bool>("localizationMode", false);
        initialPoseX = mapping_numeric_parameter("initialPoseX", 0.0);
        initialPoseY = mapping_numeric_parameter("initialPoseY", 0.0);
        initialPoseZ = mapping_numeric_parameter("initialPoseZ", 0.0);
        initialPoseRoll = mapping_numeric_parameter("initialPoseRoll", 0.0);
        initialPosePitch = mapping_numeric_parameter("initialPosePitch", 0.0);
        initialPoseYaw = mapping_numeric_parameter("initialPoseYaw", 0.0);
        posesEnable = node->declare_parameter<bool>("posesEnable", true);
        posesOutputFile = node->declare_parameter<std::string>("posesOutputFile", "");
        posesStartTime = mapping_numeric_parameter("posesStartTime", 0.0);
        posesOriginX = mapping_numeric_parameter("posesOriginX", 0.0);
        posesOriginY = mapping_numeric_parameter("posesOriginY", 0.0);
        posesOriginZ = mapping_numeric_parameter("posesOriginZ", 0.0);
        posesOriginYaw = mapping_numeric_parameter("posesOriginYaw", 0.0);
        posesOriginMode = node->declare_parameter<std::string>("posesOriginMode", "manual");
        posesOriginLock = node->declare_parameter<bool>("posesOriginLock", false);
        posesOriginRefine = node->declare_parameter<bool>("posesOriginRefine", true);
        posesOriginMinSamples = node->declare_parameter<int>("posesOriginMinSamples", 20);
        posesOriginMaxDt = mapping_numeric_parameter("posesOriginMaxDt", 0.50);
        posesOriginMinBaseline = mapping_numeric_parameter("posesOriginMinBaseline", 1.0);
        posesOriginRelPosNoise = mapping_numeric_parameter("posesOriginRelPosNoise", 0.25);
        posesSettleScans = node->declare_parameter<int>("posesSettleScans", 20);
        posesInterpolateMaxGap = mapping_numeric_parameter("posesInterpolateMaxGap", 5.0);
        posesInterpolateMaxError = mapping_numeric_parameter("posesInterpolateMaxError", 0.10);
        posesCarmenStampTopic = node->declare_parameter<std::string>("posesCarmenStampTopic", "/carmen/scan_time_reference");
        posesAckermannTopic = node->declare_parameter<std::string>("posesAckermannTopic", "/ackermann/odom_raw");
        imuRPYWeight = mapping_numeric_parameter("imuRPYWeight", 0.01);
        edgeFeatureMinValidNum = node->declare_parameter<int>("edgeFeatureMinValidNum", 10);
        surfFeatureMinValidNum = node->declare_parameter<int>("surfFeatureMinValidNum", 100);
        mappingCornerLeafSize = mapping_numeric_parameter("mappingCornerLeafSize", 0.2);
        mappingSurfLeafSize = mapping_numeric_parameter("mappingSurfLeafSize", 0.2);
        z_tollerance = mapping_numeric_parameter("z_tollerance", FLT_MAX);
        rotation_tollerance = mapping_numeric_parameter("rotation_tollerance", FLT_MAX);
        numberOfCores = node->declare_parameter<int>("numberOfCores", 2);
        mappingProcessInterval = mapping_numeric_parameter("mappingProcessInterval", 0.15);
        surroundingkeyframeAddingDistThreshold = mapping_numeric_parameter("surroundingkeyframeAddingDistThreshold", 1.0);
        surroundingkeyframeAddingAngleThreshold = mapping_numeric_parameter("surroundingkeyframeAddingAngleThreshold", 0.2);
        surroundingKeyframeDensity = mapping_numeric_parameter("surroundingKeyframeDensity", 1.0);
        surroundingKeyframeSearchRadius = mapping_numeric_parameter("surroundingKeyframeSearchRadius", 50.0);
        loopClosureEnableFlag = node->declare_parameter<bool>("loopClosureEnableFlag", false);
        loopClosureFrequency = mapping_numeric_parameter("loopClosureFrequency", 1.0);
        surroundingKeyframeSize = node->declare_parameter<int>("surroundingKeyframeSize", 50);
        historyKeyframeSearchRadius = mapping_numeric_parameter("historyKeyframeSearchRadius", 10.0);
        historyKeyframeSearchTimeDiff = mapping_numeric_parameter("historyKeyframeSearchTimeDiff", 30.0);
        historyKeyframeSearchNum = node->declare_parameter<int>("historyKeyframeSearchNum", 25);
        historyKeyframeFitnessScore = mapping_numeric_parameter("historyKeyframeFitnessScore", 0.3);
        globalMapVisualizationSearchRadius = mapping_numeric_parameter("globalMapVisualizationSearchRadius", 1e3);
        globalMapVisualizationPoseDensity = mapping_numeric_parameter("globalMapVisualizationPoseDensity", 10.0);
        globalMapVisualizationLeafSize = mapping_numeric_parameter("globalMapVisualizationLeafSize", 1.0);
    }
};

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

// Le de volta um .scd salvo por saveSCD() (mesmo formato: um valor double por
// celula, delimitado por espaco, uma linha por linha da matriz).
Eigen::MatrixXd readSCD(std::string fileToOpen)
{
    std::vector<double> matrixEntries;
    std::ifstream matrixDataFile(fileToOpen);
    std::string matrixRowString;
    std::string matrixEntry;

    int matrixRowNumber = 0;
    while (getline(matrixDataFile, matrixRowString)) {
        std::stringstream matrixRowStringStream(matrixRowString);
        while (getline(matrixRowStringStream, matrixEntry, ' '))
            matrixEntries.push_back(stod(matrixEntry));
        matrixRowNumber++;
    }
    if (matrixRowNumber == 0 || matrixEntries.empty())
        return Eigen::MatrixXd();
    return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

inline int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time &stamp)
{
    return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}
inline rclcpp::Time time_from_seconds(double seconds)
{
    const auto sec = static_cast<int64_t>(floor(seconds));
    return rclcpp::Time(sec * 1000000000LL + llround((seconds - sec) * 1e9), RCL_ROS_TIME);
}
inline tf2::Transform lookup_transform(tf2_ros::Buffer &buffer,
    const string &target, const string &source, double timeout = 0.0)
{
    tf2::Transform result;
    tf2::fromMsg(buffer.lookupTransform(target, source, tf2::TimePointZero,
                                       tf2::durationFromSec(timeout)).transform, result);
    return result;
}
