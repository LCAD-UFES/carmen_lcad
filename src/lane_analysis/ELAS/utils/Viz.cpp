#include "Viz.h"

void frame_viz::set_lane_deviation(double x_carro, double x_base, double lane_width) {
	double dist = (x_base - x_carro);
	lane_deviation = dist / (lane_width / 2.0);

	if (lane_deviation < -1)	lane_deviation = -1;
	if (lane_deviation >  1)	lane_deviation = 1;
}

void frame_viz::set_lane_base(double x_bottom, double x_top, double lane_width, const Rect &roi, IPM *ipm) {
	lane_base.point_bottom = ipm->applyHomographyInv(Point2d(x_bottom, roi.height)) + Point2d(0, roi.y);
	lane_base.point_top = ipm->applyHomographyInv(Point2d(x_top, 0)) + Point2d(0, roi.y);
	lane_base.width = lane_width;
	lane_base.direction = (lane_base.point_top - lane_base.point_bottom) * (1 / cv::norm(lane_base.point_top - lane_base.point_bottom));
}

void render(frame_viz &data, const Mat3b &color_frame, ConfigXML *config) {

	Mat3b img = color_frame.clone();
	const Size symbol_size = Size(32, 32);
	const int padding = 15;
	const int box_height = 20;

	// if kalman is null, there is nothing to show
	if (data.isKalmanNull) {

		// mostra faixa de pedestre, se tiver
		for (unsigned int i = 0; i < data.symbols.size(); i++) {
			if (data.symbols[i].id != ROAD_SIGN::FAIXA_PEDESTRE) continue;
			Mat3b img_symbol = SinalizacaoHorizontal::toIMG(ROAD_SIGN::FAIXA_PEDESTRE);
			resize(img_symbol, img_symbol, symbol_size, INTER_NEAREST);
			Rect symbol_destination = Rect(img.cols / 2 - symbol_size.width / 2, (2 + i) * padding + box_height + i*symbol_size.height, symbol_size.width, symbol_size.height);
			img_symbol.copyTo(img(symbol_destination));
			break;
		}

		if (data.frame_number % 16 == 0) {
			string fname = "C:/Users/berriel/Desktop/video/frames/aaaaa/VIZ/lane_viz_" + to_string(data.frame_number / 16) + ".png";
			//imwrite(fname, img);
		}

		imshow("ELAS - Ego-Lane Analysis System", img);
		return;
	}

#pragma region Text Output

	if (!color_frame.empty()) {
		cout << "--------------------------------------------------" << endl;
		cout << "Dados de saida (Frame #" << data.frame_number << "): " << endl;
		cout << "- Lane Position: left(" << data.lane_position.left.size() << "), right(" << data.lane_position.right.size() << ")" << endl;
		cout << "- Lane Base: position(" << data.lane_base.point_bottom << "), direction(" << data.lane_base.direction << "), width(" << data.lane_base.width << ")" << endl;
		cout << "- Lane Deviation: " << data.lane_deviation * 100 << "%" << endl;
		cout << "- Lane Change: " << data.lane_change << endl;
		cout << "- LMT: left(" << data.lmt.left << "), right(" << data.lmt.right << ")" << endl;
		cout << "- Adjacent Lanes: left(" << data.adjacent_lanes.left << "), right(" << data.adjacent_lanes.right << ")" << endl;
		cout << "- Execution Time: " << data.execution_time << " (" << (1000.0 / data.execution_time) << " fps)" << endl;
		cout << "- Symbols: " << data.symbols.size();
		if (data.symbols.size() > 0) {
			cout << " -> [ ";
			for (auto s : data.symbols) cout << SinalizacaoHorizontal::toText(s.id) << " ";
			cout << "]";
		}
		cout << endl;
		cout << "--------------------------------------------------" << endl;
	} else {
		img = Mat3b(color_frame.size(), Vec3b(0, 0, 0)); // fundo preto
	}

#pragma endregion
	
#pragma region Lane Marking Types - LMT

	// Lane Marking Types - LMT
	Mat3b imgLeftLMT = LMTDetector::LMTtoIMG((LMT)data.lmt.left, config);
	Mat3b imgRightLMT = LMTDetector::LMTtoIMG((LMT)data.lmt.right, config);

	Rect esqRoi = Rect(padding, padding, imgLeftLMT.cols, imgLeftLMT.rows);
	Rect dirRoi = Rect(img.cols - padding - imgRightLMT.cols, padding, imgRightLMT.cols, imgRightLMT.rows);

	imgLeftLMT.copyTo(img(esqRoi));
	imgRightLMT.copyTo(img(dirRoi));

#pragma endregion
	
#pragma region Lane Deviation

	// Lane Deviation
	const double larguraImagem = img.cols;
	const double larguraIndicador = img.cols / 2.0;
	const double max_size = larguraIndicador / 2.0;

	Scalar colorDeviation = BGR_GREEN;
	if (abs(data.lane_deviation) > 0.4 && abs(data.lane_deviation) < 0.75) colorDeviation = Scalar(0, 127, 255);
	if (abs(data.lane_deviation) >= 0.75) colorDeviation = BGR_RED;

	if (data.lane_deviation < 0) {
		Rect indicador = Rect((int)(larguraImagem / 2.0), padding, (int)(max_size * abs(data.lane_deviation)), box_height);
		rectangle(img, indicador, colorDeviation, -1);
	} else {
		int tam = (int)(max_size * abs(data.lane_deviation));
		Rect indicador = Rect((int)((larguraImagem / 2.0) - tam), padding, tam, box_height);
		rectangle(img, indicador, colorDeviation, -1);
	}
	Rect bordaIndicador = Rect((int)((larguraImagem / 2.0) - max_size), padding, (int)larguraIndicador, box_height);
	rectangle(img, bordaIndicador, Scalar(127, 127, 127));

#pragma endregion
	
#pragma region Execution Time

	// Execution Time
	// TODO: � necess�rio? se for, pensar na cor, posicionamento e colcoar um rect com fundo s�lido atr�s
	// string str_time = to_string((int)round(1000.0 / data.execution_time)) + " fps";
	// putText(img, str_time, Point(padding, img.rows - padding), FONT_HERSHEY_COMPLEX_SMALL, 0.5, BGR_BLUE);

#pragma endregion
	
#pragma region Lane Position

	// Lane Position
	Scalar colorFill = BGR_GREEN, colorStroke = Scalar(44, 120, 34);
	
	Mat3b polygonPosition = img.clone();
	vector<Point> pointsRight, pointsLeft;
	for (auto p : data.lane_position.left) if (p.y > data.trustworthy_height) pointsLeft.push_back(config->ipm->applyHomographyInv(p) + Point2d(0, config->roi.y)); // inicia os pontos add os da esquerda
	for (auto p : data.lane_position.right) if (p.y > data.trustworthy_height) pointsRight.push_back(config->ipm->applyHomographyInv(p) + Point2d(0, config->roi.y)); // transforma os da direita

	vector<Point> pointsRightReverse = pointsRight;
	reverse(pointsRightReverse.begin(), pointsRightReverse.end()); // inverte o vetor para ter a ordem desejada pelo fillPolygon

	vector<Point> allPoints = pointsLeft;
	allPoints.insert(allPoints.end(), pointsRightReverse.begin(), pointsRightReverse.end()); // junta os dois vetores de pontos

	fillConvexPoly(polygonPosition, allPoints, colorFill); // exibe o poligono
	// aplica a transparencia
	const double alpha = 0.7;
	addWeighted(polygonPosition, 1 - alpha, img, alpha, 0.0, img);
	// mostra os pontos
	for (auto p : data.lane_position.left) if (p.y > data.trustworthy_height) circle(img, config->ipm->applyHomographyInv(p) + Point2d(0, config->roi.y), 1, colorStroke);
	for (auto p : data.lane_position.right)  if (p.y > data.trustworthy_height) circle(img, config->ipm->applyHomographyInv(p) + Point2d(0, config->roi.y), 1, colorStroke);

#pragma endregion
	
#pragma region Adjacent Lanes

	Mat3b imgAdj = img.clone();
	const double alphaAdj = 0.85;
	const int shift = (int)data.lane_base.width;

	map<int, Scalar> adjColor;
	adjColor[-1] = Scalar(0, 204, 255); // adj = -1
	adjColor[0] = BGR_RED; // adj = 0
	adjColor[1] = BGR_GREEN; // adj = 1

	vector<Point> pointsAdjLeft, pointsAdjRight;
	for (auto p : data.lane_position.left) if (p.y > data.trustworthy_height) pointsAdjLeft.push_back(config->ipm->applyHomographyInv(p - Point2d(shift, 0)) + Point2d(0, config->roi.y)); // inicia os pontos add os da esquerda
	for (auto p : data.lane_position.right) if (p.y > data.trustworthy_height) pointsAdjRight.push_back(config->ipm->applyHomographyInv(p + Point2d(shift, 0)) + Point2d(0, config->roi.y)); // transforma os da direita

	// left
	if (data.adjacent_lanes.left != 0) {
		vector<Point> pointsLeftReverse = pointsLeft;
		reverse(pointsLeftReverse.begin(), pointsLeftReverse.end());
		vector<Point> allPointsAdjLeft = pointsAdjLeft;
		allPointsAdjLeft.insert(allPointsAdjLeft.end(), pointsLeftReverse.begin(), pointsLeftReverse.end());
		fillConvexPoly(imgAdj, allPointsAdjLeft, adjColor[data.adjacent_lanes.left]);
	}

	// right
	if (data.adjacent_lanes.right != 0) {
		vector<Point> allPointsAdjRight = pointsAdjRight;
		allPointsAdjRight.insert(allPointsAdjRight.end(), pointsRightReverse.begin(), pointsRightReverse.end());
		fillConvexPoly(imgAdj, allPointsAdjRight, adjColor[data.adjacent_lanes.right]);
	}

	addWeighted(imgAdj, 1 - alphaAdj, img, alphaAdj, 0, img);

#pragma endregion
	
#pragma region Road Signs and Crosswalks

	// mostra a regi�o do simbolo no ch�o
	Mat3b imgRoadSigns = img.clone();
	const double alphaSymbols = 0.8;
	for (auto s : data.symbols) if (s.id != ROAD_SIGN::FAIXA_PEDESTRE) fillConvexPoly(imgRoadSigns, s.region, BGR_BLUE);
	addWeighted(imgRoadSigns, 1 - alphaSymbols, img, alphaSymbols, 0, img);

	// mostra o �cone dos simbolos detectados
	for (unsigned int i = 0; i < data.symbols.size(); i++) {

		// carrega a img do simbolo e ajusta o tamanho
		Mat3b img_symbol = SinalizacaoHorizontal::toIMG(data.symbols[i].id);
		resize(img_symbol, img_symbol, symbol_size, INTER_NEAREST);

		Rect symbol_destination;
		if (data.symbols[i].id == ROAD_SIGN::FAIXA_PEDESTRE)
			symbol_destination = Rect(img.cols / 2 - symbol_size.width / 2, (2 + i) * padding + box_height + i*symbol_size.height, symbol_size.width, symbol_size.height);
		else
			symbol_destination = Rect(data.symbols[i].region[0].x, data.symbols[i].region[0].y - img_symbol.rows, img_symbol.cols, img_symbol.rows);
		img_symbol.copyTo(img(symbol_destination));
	}

#pragma endregion
	
#pragma region Lane Base

	// Lane Base
	Scalar colorLaneBase = Scalar(255,255,255);
	Point base_right = config->ipm->applyHomographyInv(data.lane_position.right.back()) + Point2d(0, config->roi.y);
	Point base_left = config->ipm->applyHomographyInv(data.lane_position.left.back()) + Point2d(0, config->roi.y);
	Point direction_middle = (data.lane_base.point_bottom + data.lane_base.point_top) * 0.5;
	Point base_center = (base_right + base_left) * 0.5;

	// int idx_mid_right = (int)(data.lane_position.right.size() - (data.lane_position.right.size() / 8));
	// int idx_mid_left = (int)(data.lane_position.left.size() - (data.lane_position.left.size() / 8));
	int idx_mid_right = (int)(data.lane_position.right.size() - (data.lane_position.right.size() / 2));
	int idx_mid_left = (int)(data.lane_position.left.size() - (data.lane_position.left.size() / 2));
	Point mid_right = config->ipm->applyHomographyInv(data.lane_position.right[idx_mid_right]) + Point2d(0, config->roi.y);
	Point mid_left = config->ipm->applyHomographyInv(data.lane_position.left[idx_mid_left]) + Point2d(0, config->roi.y);

	arrowedLine(img, base_center, direction_middle, colorLaneBase, 2);
	line(img, base_right, base_left, colorLaneBase, 2);
	line(img, base_right, mid_right, colorLaneBase, 2);
	line(img, base_left, mid_left, colorLaneBase, 2);
	circle(img, base_center, 6, colorLaneBase, CV_FILLED);

#pragma endregion

	imshow("ELAS - Ego-Lane Analysis System", img);

	/*
	int rec_start = 0, rec_end = 3600;
	if (data.frame_number > rec_start && data.frame_number < rec_end + 1) {
		string fname = "C:/Users/berriel/Desktop/video/frames/aaa/VIZ/lane_viz_" + to_string(data.idx_frame) + ".png";
		//imwrite(fname, img);
	}

	if (data.frame_number % 16 == 0) {
		string fname = "C:/Users/berriel/Desktop/video/frames/aaaaa/VIZ/lane_viz_" + to_string(data.frame_number / 16) + ".png";
		//imwrite(fname, img);
	}

	string fname = "/dados/berriel/datasets/carmen/reta-out/" + to_string(data.idx_frame) + ".png";
	imwrite(fname, img);
	/**/
}
