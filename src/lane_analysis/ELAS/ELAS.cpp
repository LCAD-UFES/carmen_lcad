#include "ELAS.h"

using namespace std;
using namespace cv;

static ConfigXML * cfg;

// vars
static ELAS::pre_processed * out_pre_process;
static ELAS::feature_maps * out_feature_maps;
static ELAS::crosswalks * out_crosswalks;
static ELAS::road_signs * out_road_signs;
static ELAS::lane_position * out_lane_position;
static ELAS::raw_houghs * out_raw_houghs;
static ELAS::lane_change * out_lane_change;
static ELAS::lane_marking_type * out_lmt;
static ELAS::adjacent_lanes * out_adjacent_lanes;
static ELAS::raw_elas_message * out_raw_elas_message;

ELAS::raw_elas_message& ELAS::get_raw_message() { return *out_raw_elas_message; };

// run init procedures, if any
void ELAS::init(string & config_fname, string & config_xml_fname) {
	printf("\n******************************************\nEgo-Lane Analysis System\ninit() - ...\n\n");

	cfg = new ConfigXML();

	// TODO: load calibration params via config file
	printf("Loading config... ");
	loadConfig(config_fname, *cfg);
	printf("DONE!\n");
	loadDatasetConfig(config_xml_fname, *cfg);

	// overwrite display and verbose settings
	cfg->display = false;
	cfg->verbose = false;

	// TODO: calib correctly
	cfg->carPosition = Point2d(cfg->dataset.FrameSize.width / 2.0, cfg->dataset.FrameSize.height);
	cfg->carPositionIPM = cfg->ipm->applyHomography(Point2d(cfg->dataset.FrameSize.width / 2.0, (double)cfg->roi.height));

	printf("Config:\n\tDATASETS_DIR\t%s\n\tDATA_DIR\t%s\n", cfg->DATASETS_DIR.c_str(), cfg->DATA_DIR.c_str());

	// init lane estimation
	printf("Lane Estimation Module :: init()... ");
	lane_estimation_init(cfg);
	printf("DONE!\n");

	// init lmt detector
	printf("LMT Detector :: init()... ");
	lmt_classification_init(cfg);
	printf("DONE!\n");

	printf("init() - DONE!\n******************************************\n");

}

void ELAS::run(const Mat3b & original_frame) {

	out_pre_process = new pre_processed();
	out_feature_maps = new feature_maps();
	out_crosswalks = new crosswalks();
	out_road_signs = new road_signs();
	out_lane_position = new lane_position();
	out_raw_houghs = new raw_houghs();
	out_lane_change = new lane_change();
	out_lmt = new lane_marking_type();
	out_adjacent_lanes = new adjacent_lanes();
	out_raw_elas_message = new raw_elas_message();

	printf("\nELAS::run()\n");

	// pre-processing
	pre_processing(original_frame, cfg, out_pre_process);
	if (DISPLAY_PRE_PROCESSING) {
		imshow("pre-processing ROI", out_pre_process->grayFrameRoi);
		imshow("pre-processing", out_pre_process->grayFrameRoiIPM);
		imshow("mask IPM", out_pre_process->maskIPM);
		waitKey();
	}

	// feature map generation
	feature_map_generation(out_pre_process, cfg, out_feature_maps);
	if (DISPLAY_FEATURE_MAPS) {
		imshow("srf", out_feature_maps->map_srf);
		imshow("hdog ipm", out_feature_maps->map_hdog_ipm);
		imshow("vad ipm", out_feature_maps->map_vad_ipm);
		imshow("inb ipm", out_feature_maps->map_inb_ipm);
		waitKey();
	}

	// crosswalk detection
	crosswalk_detection(out_feature_maps, cfg, out_crosswalks);
	if (DISPLAY_CROSSWALK) {
		imshow("crosswalk map", out_crosswalks->mask);
		putText(out_crosswalks->mask, (out_crosswalks->status) ? "true" : "false", Point(10,10), FONT_HERSHEY_COMPLEX, 1, Scalar(255));
		if (out_crosswalks->status) printf("CROSSWALK FOUND! **************************** ");
	}

	// road signs detection and classification
	road_signs_classification(out_pre_process, out_feature_maps, NULL, cfg, out_road_signs);
	if (DISPLAY_ROAD_SIGNS) {
		printf("%d road signs found!\n", out_road_signs->n);
	}

	// road signs and crosswalk removal
	if (DISPLAY_MARKINGS_REMOVAL) imshow("map_srf before", out_feature_maps->map_srf);
	// pavement_markings_removal(out_feature_maps, out_crosswalks, out_road_signs, cfg);
	if (DISPLAY_MARKINGS_REMOVAL) {
		imshow("map_srf after", out_feature_maps->map_srf);
	}

	// MAIN MODULE: lane estimation process
	lane_position_estimation(out_pre_process, out_feature_maps, out_road_signs, cfg, out_lane_position, out_lane_change, out_raw_houghs);
	if (DISPLAY_LANE_POSITION) {
		if (out_lane_position->is_hough_only) printf("\tLane Estimation using Hough Only!\n");
		else printf("\tLane Estimation using Particle Filter!\n");

		Mat3b lane_position_img = out_pre_process->colorFrame.clone();

		// lane position
		for (Point p : out_lane_position->left) if (!(isnan(p.x) || isnan(p.y))) circle(lane_position_img, p, 2, BGR_RED, CV_FILLED);
		for (Point p : out_lane_position->right) if (!(isnan(p.x) || isnan(p.y))) circle(lane_position_img, p, 2, BGR_RED, CV_FILLED);

		imshow("DISPLAY_LANE_POSITION", lane_position_img);
	}

	// lane center deviation
	lane_center_deviation(out_lane_position, cfg);
	if (DISPLAY_LANE_CENTER_DEVIATION) {
		Mat3b lane_deviation_img = out_pre_process->colorFrame.clone();

		// lane center deviation
		printf("Lane Center Deviation: %.2f\n", out_lane_position->center_deviation);
		string lane_center_text = "Lane Center Deviation: " + to_string(out_lane_position->center_deviation);
		putText(lane_deviation_img, lane_center_text, Point(15,15), FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(0,0,255), 1, 8);

		imshow("DISPLAY_LANE_CENTER_DEVIATION", lane_deviation_img);
	}

	// lane markings type classification
	lmt_classification(out_pre_process, out_feature_maps, out_lane_position, out_lmt);
	if (DISPLAY_LANE_MARKINGS_TYPE) {
		printf("LMT: left(%d), right(%d)\n", out_lmt->left, out_lmt->right);

		const int padding = 15;
		Mat3b lmt_img = out_pre_process->colorFrame.clone();
		if (out_lmt->left != LMT::NONE && !std::isnan(out_lmt->left)) {
			Mat3b lmtLeftImg = LMTDetector::LMTtoIMG((LMT)out_lmt->left, cfg);
			Rect leftRoi = Rect(padding, padding, lmtLeftImg.cols, lmtLeftImg.rows);
			lmtLeftImg.copyTo(lmt_img(leftRoi));
		}
		if (out_lmt->right != LMT::NONE && !std::isnan(out_lmt->right)) {
			Mat3b lmtRightImg = LMTDetector::LMTtoIMG((LMT)out_lmt->right, cfg);
			Rect rightRoi = Rect(lmt_img.cols - padding - lmtRightImg.cols, padding, lmtRightImg.cols, lmtRightImg.rows);
			lmtRightImg.copyTo(lmt_img(rightRoi));
		}
		imshow("DISPLAY_LANE_MARKINGS_TYPE", lmt_img);
	}

	// adjacent lanes detection
	//adjacent_lanes_detection(out_feature_maps, out_lane_position, out_raw_houghs, out_lmt, cfg, out_adjacent_lanes);
	if (DISPLAY_ADJACENT_LANES) {
		printf("adjacent lanes: left(%d), right(%d)\n", out_adjacent_lanes->left, out_adjacent_lanes->right);
	}

	// construct raw message
	out_raw_elas_message->lane_position.left = out_lane_position->left;
	out_raw_elas_message->lane_position.right = out_lane_position->right;
	out_raw_elas_message->trustworthy_height = out_lane_position->trustworthy_height;
	out_raw_elas_message->lane_deviation = out_lane_position->center_deviation;

	if (out_lane_position->lane_base != NULL) {
		out_raw_elas_message->lane_base.width = out_lane_position->lane_base->largura;
		out_raw_elas_message->lane_base.point_bottom = Point2d(out_lane_position->lane_base->xBase, cfg->roi.height);
		out_raw_elas_message->lane_base.point_top = Point2d(out_lane_position->lane_base->xTopo, 0);
		out_raw_elas_message->lane_base.direction = out_raw_elas_message->lane_base.point_top - out_raw_elas_message->lane_base.point_bottom;
		out_raw_elas_message->isKalmanNull = true;
	} else {
		out_raw_elas_message->isKalmanNull = false;
	}

	out_raw_elas_message->adjacent_lanes.left = out_adjacent_lanes->left;
	out_raw_elas_message->adjacent_lanes.right = out_adjacent_lanes->right;

	out_raw_elas_message->lane_change = out_lane_change->status;

	out_raw_elas_message->lmt.left = out_lmt->left;
	out_raw_elas_message->lmt.right = out_lmt->right;

	out_raw_elas_message->car_position_x = cfg->carPositionIPM.x;
	out_raw_elas_message->isKalmanNull = out_lane_position->lane_base == NULL;

	// free memory
	finishRun();
}

void ELAS::display(const Mat3b& frame, raw_elas_message * message) {
	frame_viz viz_data = get_viz_data(message);
	render(viz_data, frame, cfg);
	waitKey(1);
}

void ELAS::setIPM(Size& _origSize, Size& _dstSize, vector<Point2f>& _origPoints, vector<Point2f>& _dstPoints) {

	// if image size used to generate the stored IPM is different than the image we are trying to read, a scaling must be performed
	if (!(_origSize.width == cfg->dataset.FrameSize.width && _origSize.height == cfg->dataset.FrameSize.height)) {

		// calculates the required scaling
		const double w_scale = cfg->dataset.FrameSize.width / (double)_origSize.width;
		const double h_scale = cfg->dataset.FrameSize.height / (double)_origSize.height;

		// change the original size
		_origSize = Size(cfg->dataset.FrameSize.width, cfg->dataset.FrameSize.height);

		// apply scaling to the original points
		for (unsigned int i = 0; i < _origPoints.size(); i++) {
			_origPoints[i] = Point2f(_origPoints[i].x * w_scale, _origPoints[i].y * h_scale);
		}
	} else {
		printf("Images used to generate stored IPM and the ones we are using have the same size! That's good, right?\n");
	}

	cfg->ipm = new IPM(_origSize, _dstSize, _origPoints, _dstPoints);
	cfg->roi = Rect(0,0,cfg->dataset.FrameSize.width, cfg->dataset.FrameSize.height);
}

frame_viz ELAS::get_viz_data(raw_elas_message * message) {
	frame_viz viz_data;

	Point2d _tl = Point2d(cfg->roi.tl().x, cfg->roi.tl().y);

	for(Point2d p : message->lane_position.left) viz_data.lane_position.left.push_back(cfg->ipm->applyHomography(p - _tl));
	for(Point2d p : message->lane_position.right) viz_data.lane_position.right.push_back(cfg->ipm->applyHomography(p -_tl));

	viz_data.lmt.left = message->lmt.left;
	viz_data.lmt.right = message->lmt.right;
	viz_data.adjacent_lanes.left = message->adjacent_lanes.left;
	viz_data.adjacent_lanes.right = message->adjacent_lanes.right;

	viz_data.lane_base.direction = message->lane_base.direction;
	viz_data.lane_base.point_bottom = cfg->ipm->applyHomographyInv(Point2d(message->lane_base.point_bottom.x, cfg->roi.height)) + Point2d(0, cfg->roi.y);
	viz_data.lane_base.point_top = cfg->ipm->applyHomographyInv(Point2d(message->lane_base.point_top.x, 0)) + Point2d(0, cfg->roi.y);
	viz_data.lane_base.width = message->lane_base.width;

	viz_data.lane_deviation = message->lane_deviation;
	viz_data.execution_time = 33; // not set: message->execution_time;
	viz_data.x_carro = message->car_position_x;
	viz_data.isKalmanNull = message->isKalmanNull;
	viz_data.trustworthy_height = message->trustworthy_height;
	viz_data.lane_change = (int)message->lane_change;

	viz_data.idx_frame = 0; // unused
	viz_data.frame_number = 0; // unused
	// empty: viz_data.symbols

	return viz_data;
}

ConfigXML * ELAS::getConfigXML() {
	return cfg;
}

// TODO: call this when module is terminated
void ELAS::finishProgram() {
	printf("ELAS::finishProgram()\n");
	delete cfg;
	finishRun();
}

void ELAS::finishRun() {
	printf("ELAS::finishRun()\n");
	delete out_pre_process;
	delete out_feature_maps;
	delete out_crosswalks;
	delete out_road_signs;
	delete out_lane_position;
	delete out_lane_change;
	delete out_lmt;
}
