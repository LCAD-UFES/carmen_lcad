#include "lane_estimation.h"
#include "Darknet.hpp" /*< Yolo V2 */


using namespace std;
using namespace cv;

Detector *darknet;

// aux vars
const static cv::Scalar esqCor = cv::Scalar(255, 0, 255);
const static cv::Scalar dirCor = cv::Scalar(255, 255, 0);
static vector<int> y_output;

// vars
static AnaliseDasHoughs * _houghs;
static KalmanState * kalmanState;
static KalmanFilter * KF;
static deque<HoughLine> esqBuffer, dirBuffer, esqBufferRejeitados, dirBufferRejeitados;
static bool first_pass = true;
static ParticleFilterHough * PF;
static ParticleHough * best_particle;

void ELAS::lane_estimation_init(ConfigXML * _cfg) {
	// TODO: remove this... this should be placed in a single function call instead of a class
	_houghs = new AnaliseDasHoughs(_cfg);
	std::string darknet_home = std::getenv("DARKNET_HOME"); /*< get environment variable pointing path of darknet*/
	if (darknet_home.empty())
		printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");
	std::string cfg_filename = darknet_home + "/cfg/yolo_voc_lane.cfg";
	std::string weight_filename = darknet_home + "/yolo_lane.weights";
	std::string voc_names = darknet_home + "/data/lane.names";

	darknet = new Detector(cfg_filename, weight_filename, 0);

	// select which points are to be outputted
	y_output = {
			_cfg->roi.y,
			_cfg->roi.y + (int)ceil((float)_cfg->roi.height / 32.0),
			_cfg->roi.y + (int)ceil((float)_cfg->roi.height / 16.0),
			_cfg->roi.y + (int)ceil((float)_cfg->roi.height / 8.0),
			_cfg->roi.y + (int)ceil((float)_cfg->roi.height / 4.0),
			_cfg->roi.y + (int)ceil((float)_cfg->roi.height / 2.0),
			_cfg->roi.y + _cfg->roi.height - 1
	};

	// init Kalman related vars
	const int kalman_m = 6, kalman_n = 3;
	KF = new KalmanFilter(kalman_m, kalman_n, 0, CV_64F);
	kalmanState = new KalmanState();
	Kalman::inicializa(KF, kalmanState, kalman_m, kalman_n);

	// particle filter
	double initial_lane_width = _cfg->dataset._IPM.tr - _cfg->dataset._IPM.tl;
	int initial_trustworthy_height = 0;
	PF = new ParticleFilterHough(NUM_PARTICLES, _cfg->carPositionIPM.x, initial_lane_width, initial_trustworthy_height, false, DISPLAY_PARTICLE_FILTER);
	best_particle = new ParticleHough(_cfg->carPositionIPM.x, initial_lane_width);
}

void ELAS::lane_position_estimation(const pre_processed * _pre_processed, const feature_maps * _feature_maps, const road_signs * _road_signs, ConfigXML * _cfg, lane_position * _out_lane_position, lane_change * _out_lane_change, raw_houghs * _out_raw_houghs) {
	// presume lane change is false
	_out_lane_change->status = false;
	_out_lane_position->is_hough_only = HOUGH_ONLY;

	Mat1b map_srf_skel = Helper::skeleton(_feature_maps->map_srf); // approx +1.5ms
	Mat src_image = (Mat) _pre_processed->colorFrame;
	std::vector<bbox_t> predictions = darknet->detect(src_image, 0.2, false);
    std::vector<bounding_box> bouding_boxes_list;
    for (const auto &box : predictions)
       {
           bounding_box bbox;

           bbox.pt1.x = box.x;

           bbox.pt2.x = box.x + box.w;

           if (box.x > src_image.cols / 2)
           {
               bbox.pt1.y = box.y;
               bbox.pt2.y = box.y + box.h;
            }else
            {
            	bbox.pt2.y = box.y;
                bbox.pt1.y = box.y + box.h;
            }

           bouding_boxes_list.push_back(bbox);
       }

	_out_raw_houghs->ego_lane = Houghs::getHoughs(bouding_boxes_list, map_srf_skel, _cfg->roi, _out_raw_houghs->adjacent_lanes);

	/* Display the selected houghs
	Mat3b im = _pre_processed->colorFrame.clone();
	for (auto h : _out_raw_houghs->ego_lane) h.draw(im, Scalar(0,0,255));
	imshow("houghs", im); /**/

	// generation of the combined map
	// Mat1d map_cmb = _feature_maps->map_srf_ipm & _feature_maps->map_inb_ipm; // old mapaIPM
	Mat1d map_cmb = _feature_maps->map_srf_ipm & _feature_maps->map_hdog_ipm; // old mapaIPM

	// TODO: separate into a different function call (maybe lane_measurement_generation)
	// lane measurement generation
	// TODO: adapt code to use "LaneMeasurement lane_measurement" instead of hough_X
	vector<HoughLine> houghs_X = { HoughLine::empty(), HoughLine::empty() };
	_houghs->setColorFrame(_pre_processed->colorFrame.clone());
	_houghs->executar2D(_feature_maps->map_srf_ipm, _out_raw_houghs->ego_lane, kalmanState->hough, map_cmb, houghs_X);

	// buffer mechanism
	const bool estimate_lane_width = buffer_mechanism(houghs_X, _cfg); // old estimarLargura

	// OLD: timer[Task::KALMAN].start();

	// transform the lane pair of Hough lines into a single one
	HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(houghs_X[0], houghs_X[1], _cfg);

	// detect lane changes
	double bottom_distance = kalmanState->_hough.xBase - rawMeasurement.xBase;
	if (abs(bottom_distance) > kalmanState->_hough.largura * 0.7) {
		if (!kalmanState->estaDesativado) {
			_out_lane_change->status = true;
			clear_buffers();
			Kalman::resetaKalman(KF, 6, 3, rawMeasurement.toKalman());
		}
	}

	// we would like to init kalman based on the raw measurement, instead of a random position
	if (first_pass) {
		Kalman::resetaKalman(KF, 6, 3, rawMeasurement.toKalman());
		first_pass = false;
	}

	// count the number of evidences under the houghs to see if there is a lane
	// TODO: i think it should be moved to inside Kalman estimation process
	kalmanState->nEvidencias = Houghs::contaEvidencias(houghs_X, map_cmb, _cfg);
	Kalman::estimar(KF, kalmanState, rawMeasurement, estimate_lane_width);
	_out_lane_position->lane_base = kalmanState->hough;

	// OLD: timer[Task::KALMAN].end();

	// TODO: move viz out of this module
	// viz_lane_measurement_generation(_pre_processed->colorFrame, houghs_X[0], houghs_X[1]);

	// OLD: timer[Task::PARTICLE_FILTER].start();

	// TODO: decouple particle filter from old implementation
	// [0, roi.rows], where [0] = trust everything and [roi.rows] = trust nothing
	int trustworthy_height = 0;

	// particle filter
	if (kalmanState->hough != NULL && !HOUGH_ONLY) {

		// estimate the trustworthy height based on the last virtual best or the hough
		// trustworthy_height = PF->atualizaAlturaConfiavel(best_particle, kalmanState->_hough, _feature_maps->map_vad_ipm, _road_signs->road_signs_blobs);

		// if the trustworthy area changes abruptly, particle filter might need to be reseted
		int m_rows = _feature_maps->map_vad_ipm.rows;
		if (PF->alturaConfiavel >= 3 * m_rows / 4 && trustworthy_height <= m_rows / 2) {
			PF->reset(kalmanState->_hough, false); // complete reset
		} else if (PF->alturaConfiavel >= 3 * m_rows / 8 && trustworthy_height <= 2 * m_rows / 8) {
			PF->reset(kalmanState->_hough, true); // parcial reset (only top part)
		}

		// update the trustworthy height of the particle filter
		// PF->alturaConfiavel = trustworthy_height;
		// _out_lane_position->trustworthy_height = PF->alturaConfiavel;

		// runs the particle filter
		*best_particle = PF->executar(kalmanState->_hough, map_cmb);
	}

	if (DISPLAY_PARTICLE_FILTER && !HOUGH_ONLY) {
		Mat particle_filter_display = _pre_processed->colorFrameRoiIPM.clone();
		PF->view(kalmanState->hough, _pre_processed->colorFrame.clone(), particle_filter_display, _cfg, false, trustworthy_height);
	}

	// OLD: timer[Task::PARTICLE_FILTER].end();

	// write the output
	if (HOUGH_ONLY) {
		
		vector<HoughLine> finalHoughs = kalmanState->_hough.toVetorDeHoughs(_cfg->roi, _cfg->ipm);
		for (unsigned int i = 0; i < y_output.size(); i++) {
			_out_lane_position->left.push_back(Point2d(finalHoughs[LANE_LEFT].getX(y_output[i]), y_output[i]));
			_out_lane_position->right.push_back(Point2d(finalHoughs[LANE_RIGHT].getX(y_output[i]), y_output[i]));
		}
	} else {
		for (unsigned int i = 0; i < y_output.size(); i++) {
			_out_lane_position->left.push_back(Point2d(best_particle->getPoint(y_output[i], kalmanState->_hough, _cfg, LANE_LEFT, true).x, y_output[i]));
			_out_lane_position->right.push_back(Point2d(best_particle->getPoint(y_output[i], kalmanState->_hough, _cfg, LANE_RIGHT, true).x, y_output[i]));
		}
	}

	// discard points above trustworthy height
	if (USE_TRUSTWORTHY_HEIGHT) {
		// transform trustworthy height from IPM to Perspective domain
		Point2d trustworthy_height_perspective = _cfg->ipm->applyHomographyInv(Point2d(_cfg->carPositionIPM.x, PF->alturaConfiavel));
		for (int i = (int)y_output.size() - 1; i >= 0; i--) {
			if (kalmanState->hough == NULL || trustworthy_height_perspective.y - 1 >= y_output[i] - _cfg->roi.y) {
				_out_lane_position->left[i] = Point2d(numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN());
				_out_lane_position->right[i] = Point2d(numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN());
			}
		}
	}
}

void ELAS::lane_center_deviation(lane_position * _lane_position, ConfigXML * _cfg) {
	const int pos_bottom = y_output.size() - 1;

	// if any of the bottom points are undefined, i am not able to calculate lane center deviation
	if (std::isnan(_lane_position->left[pos_bottom].x) || std::isnan(_lane_position->right[pos_bottom].x)) {
		_lane_position->center_deviation = numeric_limits<double>::quiet_NaN();
		return;
	}

	// calculate the estimated lane center
	const double lane_center_position = (_lane_position->left[pos_bottom].x + _lane_position->right[pos_bottom].x ) / 2.0;

	// otherwise, estimate lane center deviation
	// [-1,1] -> [car is positioned over the left lane marking, car is positioned over the right lane marking]
	double dist = (_cfg->carPositionIPM.x - _lane_position->lane_base->xBase);
	double lane_deviation = dist / (_lane_position->lane_base->largura / 2.0);

	// truncate, because I do not care how much after the limit it is
	if (lane_deviation < -1)	lane_deviation = -1;
	if (lane_deviation >  1)	lane_deviation = 1;

	_lane_position->center_deviation = lane_deviation;
}

void ELAS::clear_buffers() {
	esqBuffer.clear();
	esqBufferRejeitados.clear();
	dirBuffer.clear();
	dirBufferRejeitados.clear();
}

bool ELAS::buffer_mechanism(vector<HoughLine> & houghs_X, ConfigXML * _cfg) {
	// adiciono as houghs em potenciais para o buffer
	bool esqH = false, dirH = false; // flag para descartar as houghs
	if (!houghs_X[0].isEmpty()) {
		if (esqBuffer.size() < BUFFER_HOUGHS_SIZE || Houghs::validaHough(houghs_X[0], esqBuffer, _cfg)) {
			esqBufferRejeitados.clear();
			Helper::pushBuffer(esqBuffer, houghs_X[0], BUFFER_HOUGHS_SIZE);
		} else {
			Helper::pushBuffer(esqBufferRejeitados, houghs_X[0], BUFFER_HOUGHS_SIZE);
			esqH = true;
		}
	}

	if (!houghs_X[1].isEmpty()) {
		if (dirBuffer.size() < BUFFER_HOUGHS_SIZE || Houghs::validaHough(houghs_X[1], dirBuffer, _cfg)) {
			dirBufferRejeitados.clear();
			Helper::pushBuffer(dirBuffer, houghs_X[1], BUFFER_HOUGHS_SIZE);
		} else {
			Helper::pushBuffer(dirBufferRejeitados, houghs_X[1], BUFFER_HOUGHS_SIZE);
			dirH = true;
		}
	}

	bool esqCheio = esqBufferRejeitados.size() == BUFFER_HOUGHS_SIZE;
	bool dirCheio = dirBufferRejeitados.size() == BUFFER_HOUGHS_SIZE;

	if (esqCheio || dirCheio) {

		bool resetarKalman = false;

		// se somente um estiver cheio
		// swap desse buffer
		// zerar os outros
		if (esqCheio) {
			HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(esqBufferRejeitados.back(), dirBuffer.back(), _cfg);
			if (rawMeasurement.largura > BUFFER_MIN_LENGTH) {
				esqBuffer = esqBufferRejeitados;
				resetarKalman = true;
			}
			esqBufferRejeitados.clear();
		}

		if (dirCheio) {
			HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(esqBuffer.back(), dirBufferRejeitados.back(), _cfg);
			if (rawMeasurement.largura > BUFFER_MIN_LENGTH) {
				double mediaPosicao, desvioPosicao;
				Houghs::posicaoBufferStatistics(dirBufferRejeitados, mediaPosicao, desvioPosicao, _cfg);
				Houghs::posicaoBufferStatistics(dirBuffer, mediaPosicao, desvioPosicao, _cfg);
				if (desvioPosicao < 5) {
					dirBuffer = dirBufferRejeitados;
					resetarKalman = true;
				}
			}
			dirBufferRejeitados.clear();
		}

		if (resetarKalman) {
			Houghs::validaEmptyHoughs(houghs_X, *kalmanState, _cfg);
			HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(houghs_X[0], houghs_X[1], _cfg);
			// TODO: particle filter -> houghPF.reset(rawMeasurement.toKalman(), false);
			Kalman::resetaKalman(KF, 6, 3, rawMeasurement.toKalman());
		}
	}

	if (esqH) houghs_X[0] = HoughLine::empty();
	if (dirH) houghs_X[1] = HoughLine::empty();

	const bool estimarLargura = (!houghs_X[0].isEmpty() && !houghs_X[1].isEmpty());

	Houghs::validaEmptyHoughs(houghs_X, *kalmanState, _cfg);

	return estimarLargura;

}

void ELAS::viz_lane_measurement_generation(const Mat3b & _colorFrame, HoughLine &esq, HoughLine &dir) {
	Mat3b viz_image = _colorFrame.clone();

	// visualizar as houghs finais
	Scalar preto = Scalar(0, 0, 0);
	if (!esq.isEmpty()) esq.draw(viz_image, esqCor);
	if (!dir.isEmpty()) dir.draw(viz_image, dirCor);
	if (!esq.isEmpty()) esq.draw(viz_image, preto, 1);
	if (!dir.isEmpty()) dir.draw(viz_image, preto, 1);

	imshow("lane_measurement_generation", viz_image);
	waitKey();
}
