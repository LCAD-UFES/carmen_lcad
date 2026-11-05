#include "image_projection_support.hpp"
#include "lio_sam/msg/cloud_info.hpp"

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<lio_sam::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

    // Quando true, o IMU deixa de ser obrigatório: o deskew rotacional (yaw) e a
    // atitude são derivados da odometria Ackermann (odometrySource == "ackermann").
    bool useAckermannOdom;

    // Print de 1 Hz com a taxa de scans (ver printRates()).
    rclcpp::TimerBase::SharedPtr rateTimer;
    RateCounter scansInRate;
    RateCounter scansOutRate;


public:
    ImageProjection():
    deskewFlag(0)
    {
        // Deskew/atitude via Ackermann apenas no modo "ackermann".
        // Em "imu" e "fusion" o IMU continua sendo usado para atitude/deskew.
        useAckermannOdom = (odometrySource == "ackermann");

        subImu        = node->create_subscription<sensor_msgs::msg::Imu>(imuTopic, 2000, std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1));
        subOdom       = node->create_subscription<nav_msgs::msg::Odometry>(odomTopic+"_incremental", 2000, std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1));
        subLaserCloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(pointCloudTopic, std::max(5, slamQueueSize), std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));

        pubExtractedCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = node->create_publisher<lio_sam::msg::CloudInfo>("lio_sam/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();

        rateTimer = node->create_wall_timer(std::chrono::seconds(1), std::bind(&ImageProjection::printRates, this));

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    // "recebidos" = nuvens que chegaram do LiDAR; "entregues" = cloud_info publicados
    // pro resto da cadeia (menor quando falta odometria/IMU pro deskew).
    void printRates()
    {
        const int raysTotal = (N_SCAN + downsampleRate - 1) / downsampleRate;
        const int raysUsed = (raysTotal + lidarJumpSize - 1) / lidarJumpSize;
        printf("[FREQ] scans do lidar: %.1f Hz recebidos, %.1f Hz entregues | jump_size=%d (%d de %d raios)\n",
               scansInRate.rate(), scansOutRate.rate(), lidarJumpSize, raysUsed, N_SCAN);
        fflush(stdout);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.start_ring_index.assign(N_SCAN, 0);
        cloudInfo.end_ring_index.assign(N_SCAN, 0);

        cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }

    ~ImageProjection(){}

    void imuHandler(sensor_msgs::msg::Imu::ConstSharedPtr imuMsg)
    {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);


    }

    void odometryHandler(nav_msgs::msg::Odometry::ConstSharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
    {
        scansInRate.tick();

        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();
        scansOutRate.tick();

        resetParameters();
    }

    bool cachePointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());

        cloudHeader = currentCloudMsg.header;
        timeScanCur = stamp_seconds(cloudHeader.stamp);

        cloudQueue.pop_front();

        pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);

        // get timestamp
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        // check dense flag
        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(node->get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                RCLCPP_ERROR(node->get_logger(), "Point cloud ring channel not available, please configure your point cloud data!");
                rclcpp::shutdown();
            }
        }

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                RCLCPP_WARN(node->get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // O IMU cobre todo o scan?
        bool imuCovers = !imuQueue.empty()
                      && stamp_seconds(imuQueue.front().header.stamp) <= timeScanCur
                      && stamp_seconds(imuQueue.back().header.stamp)  >= timeScanEnd;

        if (imuCovers)
        {
            // IMU disponível: caminho original (também usado como híbrido em
            // modo Ackermann quando há IMU montado no robô).
            imuDeskewInfo();
        }
        else if (useAckermannOdom)
        {
            // Sem IMU cobrindo o scan e em modo Ackermann: deriva o deskew
            // rotacional (yaw) e a atitude da odometria Ackermann. Assim o
            // sistema não descarta todos os scans por falta de IMU.
            ackermannDeskewInfo();
        }
        else
        {
            RCLCPP_DEBUG(node->get_logger(), "Waiting for IMU data ...");
            return false;
        }

        odomDeskewInfo();

        return true;
    }

    void ackermannDeskewInfo()
    {
        cloudInfo.imu_available = false;

        if (odomQueue.empty())
            return;

        // Inicializa com zero como fallback, mas será sobrescrito se houver odometria
        cloudInfo.imu_roll_init  = 0;
        cloudInfo.imu_pitch_init = 0;
        cloudInfo.imu_yaw_init   = 0;

        // Pega Roll, Pitch e Yaw absolutos no início do scan, vindos da odometria incremental Ackermann.
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            if (stamp_seconds(odomQueue[i].header.stamp) <= timeScanCur)
            {
                tf2::Quaternion q;
                tf2::fromMsg(odomQueue[i].pose.pose.orientation, q);
                double r, p, y;
                tf2::Matrix3x3(q).getRPY(r, p, y);
                
                // CORREÇÃO: Agora repassamos a atitude 3D completa, não apenas o Yaw!
                cloudInfo.imu_roll_init  = r;
                cloudInfo.imu_pitch_init = p;
                cloudInfo.imu_yaw_init   = y;
            }
            else
                break;
        }

        // Deskew rotacional integrando a velocidade angular convertida ao longo do scan.
        imuPointerCur = 0;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            double tcur = stamp_seconds(odomQueue[i].header.stamp);
            if (tcur < timeScanCur - 0.01)
                continue;
            if (tcur > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0)
            {
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = tcur;
                ++imuPointerCur;
                continue;
            }

            // CORREÇÃO: Pega as 3 velocidades angulares (agora no frame correto do Lidar)
            double ang_x = odomQueue[i].twist.twist.angular.x;
            double ang_y = odomQueue[i].twist.twist.angular.y;
            double ang_z = odomQueue[i].twist.twist.angular.z;
            
            double dt = tcur - imuTime[imuPointerCur-1];
            
            // Integra nos 3 eixos (Lida perfeitamente com sensores invertidos ou com pitch/roll)
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + ang_x * dt;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + ang_y * dt;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + ang_z * dt;
            
            imuTime[imuPointerCur] = tcur;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imu_available = false;

        while (!imuQueue.empty())
        {
            if (stamp_seconds(imuQueue.front().header.stamp) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = stamp_seconds(thisImuMsg.header.stamp);

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odom_available = false;

        while (!odomQueue.empty())
        {
            if (stamp_seconds(odomQueue.front().header.stamp) < timeScanCur - 0.015)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (stamp_seconds(odomQueue.front().header.stamp) > timeScanCur + 0.015)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (message_time(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll  = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw   = yaw;

        //printf("[ODOM_PICK] timeScanCur=%.3f startOdomStamp=%.3f dt=%.4f qsize=%d x=%.4f y=%.4f z=%.4f yaw=%.4f\n",
        //       timeScanCur, message_time(&startOdomMsg), timeScanCur - message_time(&startOdomMsg),
        //       (int)odomQueue.size(),
        //       startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y,
        //       startOdomMsg.pose.pose.position.z, yaw);
        fflush(stdout);

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (stamp_seconds(odomQueue.back().header.stamp) < timeScanEnd)
            return;

        nav_msgs::msg::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (message_time(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odom_available == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            // int rowIdn = (i % 64) + 1 ; // for MulRan dataset, Ouster OS1-64 .bin file,  giseop 

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            if ((rowIdn / downsampleRate) % lidarJumpSize != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.start_ring_index[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.point_col_ind[count] = j;
                    // save range info
                    cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count -1 - 5;
        }
    }
    
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("lio_sam_imageProjection");

    ImageProjection IP;
    
    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Image Projection Started.\033[0m");

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}