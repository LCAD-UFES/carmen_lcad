#include "horizontal_signalisation.h"

using namespace std;
using namespace cv;

namespace ELAS {

void crosswalk_detection(const feature_maps * _feature_maps, ConfigXML * _cfg, crosswalks * out) {
	out->mask = SinalizacaoHorizontal::detectarFaixaDePedestre2(_feature_maps->map_hdog_ipm, _cfg->roi, _cfg->carPositionIPM);
	out->status = true;

	// if the crosswalk mask is empty, then there is not crosswalk
	if (out->mask.empty()) {
		out->mask = Mat1b(_feature_maps->map_hdog_ipm.size(), uchar(0));
		out->status = false;
	} else {
		out->status = true;
	}
}

void road_signs_classification(const pre_processed * _pre_processed, const feature_maps * _feature_maps, HoughDoMeio * hough_anterior, ConfigXML * _cfg, road_signs * out) {
	vector<viz_symbols> symbols;
	vector<int> roadSigns = SinalizacaoHorizontal::executar(_pre_processed->grayFrameRoiIPM, _feature_maps->map_hdog_ipm, _feature_maps->map_vad_ipm, _pre_processed->colorFrame, hough_anterior, _cfg, out->road_signs_blobs, symbols);
	out->n = 0;
	for (auto s : symbols) {
		road_sign r;
		r.id = s.id;
		r.region = s.region;
		out->signs.push_back(r);
		out->n++;
	}
}

void pavement_markings_removal(feature_maps * _feature_maps, crosswalks * _crosswalks, road_signs * _road_signs, ConfigXML * _cfg) {
	// crosswalk removal
	Mat1b inverse_crosswalk_mask = ~_crosswalks->mask;
	_feature_maps->map_srf = _feature_maps->map_srf & Helper::fillPerspectiva(Helper::toROI(inverse_crosswalk_mask, _cfg->ipm, INTER_NEAREST), _cfg->roi, Size(_cfg->dataset.FrameSize.width, _cfg->dataset.FrameSize.height), 255);
	_feature_maps->map_srf_ipm = _feature_maps->map_srf_ipm & inverse_crosswalk_mask;
	_feature_maps->map_hdog_ipm = _feature_maps->map_hdog_ipm & inverse_crosswalk_mask;
	_feature_maps->map_vad_ipm = _feature_maps->map_vad_ipm & inverse_crosswalk_mask;

	// road signs
	SinalizacaoHorizontal::removerSinalizacao(_feature_maps->map_srf, _road_signs->road_signs_blobs);

}

}
