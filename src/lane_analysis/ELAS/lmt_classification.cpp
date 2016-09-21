#include "lmt_classification.h"

using namespace std;
using namespace cv;

static LMTDetector * lmt_detector;

void ELAS::lmt_classification_init(ConfigXML * _cfg) {
	lmt_detector = new LMTDetector(_cfg);
}

// TODO: decouple from old implementation
void ELAS::lmt_classification(const pre_processed * _pre_processed, const feature_maps * _feature_maps, const lane_position * _lane_position, lane_marking_type * out) {
	vector<LMT> LMTs = lmt_detector->executar(_feature_maps->map_hdog_ipm, _pre_processed->colorFrame.clone(), _pre_processed->grayFrameRoiIPM, _lane_position->trustworthy_height, _lane_position->lane_base);

	out->left = (int)LMTs[0];
	out->right = (int)LMTs[1];
}
