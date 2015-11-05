#include "voslam_rigid_transformation.h"

VoslamRigidTransformation::VoslamRigidTransformation(const double minValue,const double maxValue) {
	// TODO Auto-generated constructor stub
    minDepthValue=minValue;
    maxDepthValue=maxValue;
}

VoslamRigidTransformation::~VoslamRigidTransformation() {
	// TODO Auto-generated destructor stub
}

void VoslamRigidTransformation::setMinDepthValue(const double minValue)
{
    minDepthValue=minValue;
}

void VoslamRigidTransformation::setMaxDepthValue(const double maxValue)
{
    maxDepthValue=maxValue;
}

//Get 3D correspondences with valid data
void VoslamRigidTransformation::matchesWith3DValidData(
				const std::vector<carmen_vector_2D_t>& current_features,
				const std::vector<carmen_vector_2D_t>& previous_features,
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr1,
                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr2)
{
    static int point1X,point1Y,point2X,point2Y;
    static int pointIndex1,pointIndex2;
    static double depthPoint1,depthPoint2;

    for(int i=0;i<current_features.size();i++)
    {
        point1X=current_features[i].x;
        point1Y=current_features[i].y;

        point2X=previous_features[i].x;
        point2Y=previous_features[i].y;

        pointIndex1=640*point1Y+point1X;
        pointIndex2=640*point2Y+point2X;

        if(pointIndex1<=0 ||  //Check if the idexes are invalid
           pointIndex1>=pointCloudPtr1->points.size() ||
           pointIndex2<=0 ||
           pointIndex2>=pointCloudPtr2->points.size())
        {
            (*correspondences)[i].index_query=-1;
            (*correspondences)[i].index_match=-1;
            (*correspondences)[i].distance=0;
        }
        else
        {
            depthPoint1=pointCloudPtr1->points[pointIndex1].z;
            depthPoint2=pointCloudPtr2->points[pointIndex2].z;

            if((minDepthValue<=depthPoint1) && (depthPoint1<=maxDepthValue) && //The first observation has valid depth data
               (minDepthValue<=depthPoint2) && (depthPoint2<=maxDepthValue))   //The second observation has valid depth data
            {
                //Check for valid (x,y,z) values
                if(pcl_isfinite (pointCloudPtr1->points[pointIndex1].x) &&
                   pcl_isfinite (pointCloudPtr1->points[pointIndex1].y) &&
                   pcl_isfinite (pointCloudPtr1->points[pointIndex1].z) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].x) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].y) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].z))
                {
                    double distance = sqrt(pow(pointCloudPtr1->points[pointIndex1].x-
                                               pointCloudPtr2->points[pointIndex2].x,2)+
                                           pow(pointCloudPtr1->points[pointIndex1].y-
                                               pointCloudPtr2->points[pointIndex2].y,2)+
                                           pow(pointCloudPtr1->points[pointIndex1].z-
                                               pointCloudPtr2->points[pointIndex2].z,2));

                    (*correspondences)[i].index_query=pointIndex1;
                    (*correspondences)[i].index_match=pointIndex2;
                    (*correspondences)[i].distance=0;
                }
                else
                {
                    (*correspondences)[i].index_query=-1;
                    (*correspondences)[i].index_match=-1;
                    (*correspondences)[i].distance=0;

                }
            }
            else
            {
                (*correspondences)[i].index_query=-1;
                (*correspondences)[i].index_match=-1;
                (*correspondences)[i].distance=0;
            }
        }
    }

}


int VoslamRigidTransformation::estimateVisual3DRigidTransformation(
		const std::vector<carmen_vector_2D_t>& current_features,
		const std::vector<carmen_vector_2D_t>& previous_features,
//		const unsigned short* depth1,
//		const unsigned short* depth2,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr1,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloudPtr2,
        Eigen::Matrix<float, 4, 4>& H)
{
    correspondences.reset(new pcl::Correspondences);
    correspondences->clear();
    correspondences->resize(current_features.size());

    matchesWith3DValidData(current_features, previous_features,pointCloudPtr1,pointCloudPtr2);

    for(int i = 0; i < correspondences->size(); i++)
    {
    	printf("%d %d %f", (*correspondences)[i].index_query, (*correspondences)[i].index_query, (*correspondences)[i].distance);
    }

    corrsRejectorSAC.setInputCloud(pointCloudPtr1);
    corrsRejectorSAC.setInputTarget(pointCloudPtr2);
    corrsRejectorSAC.setInlierThreshold(0.05);
    corrsRejectorSAC.setMaximumIterations(500);
    corrsRejectorSAC.setInputCorrespondences(correspondences);
//    boost::shared_ptr<pcl::Correspondences> correspondencesRejSAC (new pcl::Correspondences);
//    corrsRejectorSAC.getCorrespondences(*correspondencesRejSAC);
    H = corrsRejectorSAC.getBestTransformation();


    //Return the number of RANSAC inliers
//    return correspondencesRejSAC->size();
}






