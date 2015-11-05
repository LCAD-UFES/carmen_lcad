#include "voslam_loop_detector.h"

VoslamLoopDetector::VoslamLoopDetector(int width, int height, double distance_to_loop_detection, int inliers_threshold) {

	this->rigid_transformation_extimator = new VoslamRigidTransformation(0.2, 20.0);

	this->matcher_params.nms_n                  = 3;
	this->matcher_params.nms_tau                = 50;
	this->matcher_params.match_binsize          = 50;
	this->matcher_params.match_radius           = 200;
	this->matcher_params.match_disp_tolerance   = 1;
	this->matcher_params.outlier_disp_tolerance = 3;
	this->matcher_params.outlier_flow_tolerance = 3;
	this->matcher_params.multi_stage            = 1;
	this->matcher_params.half_resolution        = 0;
	this->matcher_params.refinement             = 1;

	this->matcher = new Matcher(this->matcher_params);
	this->params.width = width;
	this->params.height = height;
	this->params.distanceForLoopDetection = distance_to_loop_detection;
	this->params.inliersThreshold = inliers_threshold;

	this->temporary_gray_image = (unsigned char *) calloc (this->params.width * this->params.height, sizeof(unsigned char));
	carmen_test_alloc(this->temporary_gray_image);
}

bool VoslamLoopDetector::CheckLoopDetection(VoslamKeyframes *keyframes, VoslamGeneralizedICP* icp)
{
	Eigen::Matrix<float, 4, 4> rigid_transformation_between_keyframes = Eigen::Matrix<float, 4, 4>::Identity();
	int number_of_inliers = 0;
	double distance_between_keyframes = 0.0;
	int keyframes_list_size = (*keyframes->getKeyframesList()).size();
	carmen_voslam_pointcloud_t last_keyframe = (*keyframes->getKeyframesList())[keyframes_list_size - 1];
	std::vector<carmen_vector_2D_t> last_keyframe_features;
	std::vector<carmen_vector_2D_t> to_check_keyframe_features;

	for(int i = 0; i < keyframes_list_size - 8; i++)
	{
		carmen_voslam_pointcloud_t to_check_keyframe = (*keyframes->getKeyframesList())[i];

		distance_between_keyframes = DistanceBetweenKeyframes(last_keyframe, to_check_keyframe);
		if(distance_between_keyframes < this->params.distanceForLoopDetection)
		{
			number_of_inliers = DoMatch(last_keyframe, to_check_keyframe);

			if(number_of_inliers >= this->params.inliersThreshold)
			{
				(*keyframes->getKeyframesList())[keyframes_list_size - 1].loop_partners_indexes.push_back(i);

//				for(int j = 0; j < matcher->getMatches().size(); j++)
//				{
//					carmen_vector_2D_t feature;
//
//					feature.x = matcher->getMatches()[j].u1c;
//					feature.y = matcher->getMatches()[j].v1c;
//					last_keyframe_features.push_back(feature);
//
//					feature.x = matcher->getMatches()[j].u1p;
//					feature.y = matcher->getMatches()[j].v1p;
//					to_check_keyframe_features.push_back(feature);
//				}


				Eigen::Matrix<float, 4, 4> guess = compute_pcl_transform_between_voslam_pointclouds(&last_keyframe, &to_check_keyframe);

//				this->rigid_transformation_extimator->estimateVisual3DRigidTransformation(last_keyframe_features, to_check_keyframe_features, keyframes->getKeyframesList()[keyframes_list_size - 1].pointcloud, keyframes->getKeyframesList()[i].pointcloud,rigid_transformation_between_keyframes);
				icp->runICP(&((*keyframes->getKeyframesList())[keyframes_list_size - 1]), &((*keyframes->getKeyframesList())[i]), &guess );
				//printf("last_index: %d, check_index: %d, dist: %f, inliers: %d\n", keyframes_list_size - 1, i, distance_between_keyframes, number_of_inliers);
				return true;
			}
		}
	}

	return false;
}

double VoslamLoopDetector::DistanceBetweenKeyframes(carmen_voslam_pointcloud_t last_keyframe, carmen_voslam_pointcloud_t to_check_keyframe)
{
	return sqrt((last_keyframe.pose.position[0] - to_check_keyframe.pose.position[0]) * (last_keyframe.pose.position[0] - to_check_keyframe.pose.position[0]) +
				(last_keyframe.pose.position[1] - to_check_keyframe.pose.position[1]) * (last_keyframe.pose.position[1] - to_check_keyframe.pose.position[1]) +
				(last_keyframe.pose.position[2] - to_check_keyframe.pose.position[2]) * (last_keyframe.pose.position[2] - to_check_keyframe.pose.position[2]));
}

int VoslamLoopDetector::DoMatch(carmen_voslam_pointcloud_t last_keyframe, carmen_voslam_pointcloud_t to_check_keyframe)
{
	int32_t dims[] = {this->params.width, this->params.height, 3 * this->params.width};
	this->matcher->pushBack(to_check_keyframe.image, 0, dims, false);
	this->matcher->pushBack(last_keyframe.image, 0, dims, false);

	this->matcher->matchFeatures(0);
	this->matches = this->matcher->getMatches();

	return this->GetNumberOfInliersFromMatcher(2000);
}

int VoslamLoopDetector::GetNumberOfInliersFromMatcher(int ransac_iterations)
{
	// get number of matches
	int32_t N = this->matches.size();
	if (N < this->params.inliersThreshold)
		return N;

	// normalize feature points and return on errors
	Matrix Tp,Tc;
	vector<Matcher::p_match> p_matched_normalized = this->matches;
	if (!NormalizeFeaturePoints(p_matched_normalized,Tp,Tc))
		return N;

	// initial RANSAC estimate of F
	Matrix F;
	this->inliers.clear();
	for (int32_t k = 0; k < ransac_iterations; k++) {

		// draw random sample set
		vector<int32_t> active = GetRandomSample(N,8);

		// estimate fundamental matrix and get inliers
		FundamentalMatrix(p_matched_normalized,active,F);
		vector<int32_t> inliers_curr = GetInliers(p_matched_normalized, F);

		// update model if we are better
		if (inliers_curr.size() > inliers.size())
			inliers = inliers_curr;
	}

	if (inliers.size() < this->params.inliersThreshold)
		return inliers.size();

	// refine F using all inliers
	FundamentalMatrix(p_matched_normalized, inliers, F);
	this->inliers = GetInliers(p_matched_normalized, F);

	return this->inliers.size();
}

vector<int32_t> VoslamLoopDetector::GetRandomSample(int32_t N,int32_t num) {

  // init sample and totalset
  vector<int32_t> sample;
  vector<int32_t> totalset;

  // create vector containing all indices
  for (int32_t i=0; i<N; i++)
    totalset.push_back(i);

  // add num indices to current sample
  sample.clear();
  for (int32_t i=0; i<num; i++) {
    int32_t j = rand()%totalset.size();
    sample.push_back(totalset[j]);
    totalset.erase(totalset.begin()+j);
  }

  // return sample
  return sample;
}

void VoslamLoopDetector::FundamentalMatrix (const vector<Matcher::p_match> &p_matched,const vector<int32_t> &active,Matrix &F) {

  // number of active p_matched
  int32_t N = active.size();

  // create constraint matrix A
  Matrix A(N,9);
  for (int32_t i=0; i<N; i++) {
    Matcher::p_match m = p_matched[active[i]];
    A.val[i][0] = m.u1c*m.u1p;
    A.val[i][1] = m.u1c*m.v1p;
    A.val[i][2] = m.u1c;
    A.val[i][3] = m.v1c*m.u1p;
    A.val[i][4] = m.v1c*m.v1p;
    A.val[i][5] = m.v1c;
    A.val[i][6] = m.u1p;
    A.val[i][7] = m.v1p;
    A.val[i][8] = 1;
  }

  // compute singular value decomposition of A
  Matrix U,W,V;
  A.svd(U,W,V);

  // extract fundamental matrix from the column of V corresponding to the smallest singular value
  F = Matrix::reshape(V.getMat(0,8,8,8),3,3);

  // enforce rank 2
  F.svd(U,W,V);
  W.val[2][0] = 0;
  F = U*Matrix::diag(W)*~V;
}

std::vector<int32_t> VoslamLoopDetector::GetInliers (vector<Matcher::p_match> &p_matched,Matrix &F) {

  // extract fundamental matrix
  double f00 = F.val[0][0]; double f01 = F.val[0][1]; double f02 = F.val[0][2];
  double f10 = F.val[1][0]; double f11 = F.val[1][1]; double f12 = F.val[1][2];
  double f20 = F.val[2][0]; double f21 = F.val[2][1]; double f22 = F.val[2][2];

  // loop variables
  double u1,v1,u2,v2;
  double x2tFx1;
  double Fx1u,Fx1v,Fx1w;
  double Ftx2u,Ftx2v;

  // vector with inliers
  vector<int32_t> inliers;

  // for all matches do
  for (int32_t i=0; i<(int32_t)p_matched.size(); i++) {

    // extract matches
    u1 = p_matched[i].u1p;
    v1 = p_matched[i].v1p;
    u2 = p_matched[i].u1c;
    v2 = p_matched[i].v1c;

    // F*x1
    Fx1u = f00*u1+f01*v1+f02;
    Fx1v = f10*u1+f11*v1+f12;
    Fx1w = f20*u1+f21*v1+f22;

    // F'*x2
    Ftx2u = f00*u2+f10*v2+f20;
    Ftx2v = f01*u2+f11*v2+f21;

    // x2'*F*x1
    x2tFx1 = u2*Fx1u+v2*Fx1v+Fx1w;

    // sampson distance
    double d = x2tFx1*x2tFx1 / (Fx1u*Fx1u+Fx1v*Fx1v+Ftx2u*Ftx2u+Ftx2v*Ftx2v);

    // check threshold
    if (fabs(d) < 0.00001)
    	inliers.push_back(i);
  }

  // return set of all inliers
  return inliers;
}

bool VoslamLoopDetector::NormalizeFeaturePoints(vector<Matcher::p_match> &p_matched,Matrix &Tp,Matrix &Tc) {

  // shift origins to centroids
  double cpu=0,cpv=0,ccu=0,ccv=0;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    cpu += it->u1p;
    cpv += it->v1p;
    ccu += it->u1c;
    ccv += it->v1c;
  }
  cpu /= (double)p_matched.size();
  cpv /= (double)p_matched.size();
  ccu /= (double)p_matched.size();
  ccv /= (double)p_matched.size();
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p -= cpu;
    it->v1p -= cpv;
    it->u1c -= ccu;
    it->v1c -= ccv;
  }

  // scale features such that mean distance from origin is sqrt(2)
  double sp=0,sc=0;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    sp += sqrt(it->u1p*it->u1p+it->v1p*it->v1p);
    sc += sqrt(it->u1c*it->u1c+it->v1c*it->v1c);
  }
  if (fabs(sp)<1e-10 || fabs(sc)<1e-10)
    return false;
  sp = sqrt(2.0)*(double)p_matched.size()/sp;
  sc = sqrt(2.0)*(double)p_matched.size()/sc;
  for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it!=p_matched.end(); it++) {
    it->u1p *= sp;
    it->v1p *= sp;
    it->u1c *= sc;
    it->v1c *= sc;
  }

  // compute corresponding transformation matrices
  double Tp_data[9] = {sp,0,-sp*cpu,0,sp,-sp*cpv,0,0,1};
  double Tc_data[9] = {sc,0,-sc*ccu,0,sc,-sc*ccv,0,0,1};
  Tp = Matrix(3,3,Tp_data);
  Tc = Matrix(3,3,Tc_data);

  // return true on success
  return true;
}

VoslamLoopDetector::~VoslamLoopDetector() {
	// TODO Auto-generated destructor stub
}
