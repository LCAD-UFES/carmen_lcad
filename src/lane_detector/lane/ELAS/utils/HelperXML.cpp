#include "HelperXML.h"

using namespace std;
using namespace tinyxml2;

bool loadDatasetConfig(const string &filepath, ConfigXML &outConfigXML) {

	cout << "Carregando ConfigXML... ";
	
	// se houver problemas para abrir o arquivo, retorna false
	XMLDocument doc;
	XMLError error = doc.LoadFile(filepath.c_str());
	if (error != XML_NO_ERROR) {
		cout << "erro!" << endl;
		return false;
	}

	// pega as configs do m�todo
	XMLElement* configTag = doc.FirstChildElement("config");
	outConfigXML.verbose = (strcmp(configTag->FirstChildElement("verbose")->GetText(), "true") == 0);
	outConfigXML.display = (strcmp(configTag->FirstChildElement("display_images")->GetText(), "true") == 0);
	outConfigXML.numParticles = configTag->FirstChildElement("particle_filter")->IntAttribute("num");

	// pega as configs do dataset
	XMLElement* dataset = doc.FirstChildElement()->FirstChildElement("dataset");
	outConfigXML.dataset.id = dataset->Attribute("id");
	outConfigXML.dataset.name = dataset->Attribute("name");
	outConfigXML.dataset.path = dataset->Attribute("path");
	outConfigXML.dataset.FrameSequence.start = dataset->FirstChildElement("frame_sequence")->IntAttribute("start");
	outConfigXML.dataset.FrameSequence.end = dataset->FirstChildElement("frame_sequence")->IntAttribute("end");
	outConfigXML.dataset.FrameSize.width = dataset->FirstChildElement("frame_size")->IntAttribute("width");
	outConfigXML.dataset.FrameSize.height = dataset->FirstChildElement("frame_size")->IntAttribute("height");
	outConfigXML.dataset._ROI.x = dataset->FirstChildElement("region_of_interest")->IntAttribute("x");
	outConfigXML.dataset._ROI.y = dataset->FirstChildElement("region_of_interest")->IntAttribute("y");
	outConfigXML.dataset._ROI.width = dataset->FirstChildElement("region_of_interest")->IntAttribute("width");
	outConfigXML.dataset._ROI.height = dataset->FirstChildElement("region_of_interest")->IntAttribute("height");
	outConfigXML.dataset._IPM.tr = dataset->FirstChildElement("ipm_points")->IntAttribute("top_right");
	outConfigXML.dataset._IPM.tl = dataset->FirstChildElement("ipm_points")->IntAttribute("top_left");
	outConfigXML.dataset._IPM.br = dataset->FirstChildElement("ipm_points")->IntAttribute("bottom_right");
	outConfigXML.dataset._IPM.bl = dataset->FirstChildElement("ipm_points")->IntAttribute("bottom_left");

	// Constr�i o ipm e a roi
	setROI(outConfigXML);
	setIPM(outConfigXML);

	cout << "pronto!" << endl;

	return true;
}

bool loadConfig(const string &fname, ConfigXML &_config) {

	// open the config file
	ifstream config_file(fname, std::ifstream::in);

	// read each param of a line and assign into config property
	string type, content;
	while (config_file >> type >> content) {
		if (type == "DATASETS_DIR") _config.DATASETS_DIR = content;
		else if (type == "DATA_DIR") _config.DATA_DIR = content;
		else return false; // if there is any unknown param, return false
	}

	// return true if all params in the file were properly loaded
	return true;
}

void setIPM(ConfigXML &_config) {

	// os 4 pontos da imagem de origem
	vector<Point2f> origPoints;
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tl), 0));
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tr), 0));
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.br), static_cast<float>(_config.dataset._ROI.height)));
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.bl), static_cast<float>(_config.dataset._ROI.height)));

	// os 4 pontos da imagem de destino
	vector<Point2f> dstPoints;
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tl), 0));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tr), 0));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tr), static_cast<float>(_config.dataset._ROI.height)));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tl), static_cast<float>(_config.dataset._ROI.height)));

	// calcula o vp
	Point2f tl, tr, br, bl;
	tl = Point2f(static_cast<float>(_config.dataset._IPM.tl), static_cast<float>(_config.dataset._ROI.y));
	tr = Point2f(static_cast<float>(_config.dataset._IPM.tr), static_cast<float>(_config.dataset._ROI.y));
	br = Point2f(static_cast<float>(_config.dataset._IPM.br), static_cast<float>(_config.dataset._ROI.y + _config.dataset._ROI.height));
	bl = Point2f(static_cast<float>(_config.dataset._IPM.bl), static_cast<float>(_config.dataset._ROI.y + _config.dataset._ROI.height));
	lineIntersection(bl, tl, br, tr, _config.dataset._IPM.vp);

	// objeto IPM
	_config.ipm = new IPM(Size(_config.roi.size().width, _config.roi.size().height), Size(_config.roi.size().width, _config.roi.size().height), origPoints, dstPoints);
}

// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
//http://stackoverflow.com/a/7448287/4228275
bool lineIntersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2, Point2f &r) {
	Point2f x = o2 - o1;
	Point2f d1 = p1 - o1;
	Point2f d2 = p2 - o2;

	float cross = d1.x*d2.y - d1.y*d2.x;
	if (abs(cross) < /*EPS*/1e-8) return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = o1 + d1 * t1;
	return true;
}

void setROI(ConfigXML &_config) {
	_config.roi = Rect(_config.dataset._ROI.x, _config.dataset._ROI.y, _config.dataset._ROI.width, _config.dataset._ROI.height);
}

IPM * AtualizaIPMLivre(ConfigXML &_config, Point2f vp, bool verbose) {

	double tempoInicio = static_cast<double>(getTickCount());

	// calcula equa��o da reta da faixa da esquerda
	Point2f bottom_left = Point2f(static_cast<float>(_config.dataset._IPM.bl), static_cast<float>(_config.dataset._ROI.y + _config.dataset._ROI.height));
	float esquerda_delta_x = vp.x - bottom_left.x;
	float esquerda_delta_y = vp.y - bottom_left.y;
	float esquerda_slope = esquerda_delta_y / esquerda_delta_x;
	float esquerda_intercept = bottom_left.y - esquerda_slope * bottom_left.x;

	// calcula equa��o da reta da faixa da esquerda
	Point2f bottom_right = Point2f(static_cast<float>(_config.dataset._IPM.br), static_cast<float>(_config.dataset._ROI.y + _config.dataset._ROI.height));
	float direita_delta_x = vp.x - bottom_right.x;
	float direita_delta_y = vp.y - bottom_right.y;
	float direita_slope = direita_delta_y / direita_delta_x;
	float direita_intercept = bottom_right.y - direita_slope * bottom_right.x;

	// calcula os novos pontos superiores
	float novo_tl = (_config.dataset._ROI.y - esquerda_intercept) / esquerda_slope;
	float novo_tr = (_config.dataset._ROI.y - direita_intercept) / direita_slope;

	// os 4 pontos da imagem de origem
	vector<Point2f> origPoints;
	origPoints.push_back(Point2f(static_cast<float>(novo_tl), 0));
	origPoints.push_back(Point2f(static_cast<float>(novo_tr), 0));
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.br), static_cast<float>(_config.dataset._ROI.height)));
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.bl), static_cast<float>(_config.dataset._ROI.height)));

	// os 4 pontos da imagem de destino
	vector<Point2f> dstPoints;
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tl), 0));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tr), 0));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tr), static_cast<float>(_config.dataset._ROI.height)));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tl), static_cast<float>(_config.dataset._ROI.height)));

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- atualiza ipm livre: " << tempoExecutando << " ms" << endl;

	// objeto IPM
	return new IPM(Size(_config.roi.size().width, _config.roi.size().height), Size(_config.roi.size().width, _config.roi.size().height), origPoints, dstPoints);
}

IPM * AtualizaIPMRestricaoGaussiana2D(ConfigXML &_config, Point2f vp, bool verbose) {

	double tempoInicio = static_cast<double>(getTickCount());

	float sigma_x = 0 / 3; // 0px na horizontal
	float sigma_y = 20 / 3; // 40px na vertical

	Point2f vetorPeso = Point2f(vp.x - _config.dataset._IPM.vp.x, vp.y - _config.dataset._IPM.vp.y);
	float xPeso = (sigma_x == 0) ? 0 : (float)getGaussianValue(vetorPeso.x, sigma_x);
	float yPeso = (sigma_y == 0) ? 0 : (float)getGaussianValue(vetorPeso.y, sigma_y);
	Point2f novoVP = Point2f(_config.dataset._IPM.vp.x + vetorPeso.x * xPeso, _config.dataset._IPM.vp.y + vetorPeso.y * yPeso);

	// calcula equa��o da reta da faiya da esquerda
	Point2f bottom_left = Point2f(static_cast<float>(_config.dataset._IPM.bl), static_cast<float>(_config.dataset._ROI.y + _config.dataset._ROI.height));
	float esquerda_delta_x = novoVP.x - bottom_left.x;
	float esquerda_delta_y = novoVP.y - bottom_left.y;
	float esquerda_slope = esquerda_delta_y / esquerda_delta_x;
	float esquerda_intercept = bottom_left.y - esquerda_slope * bottom_left.x;

	// calcula equa��o da reta da faixa da esquerda
	Point2f bottom_right = Point2f(static_cast<float>(_config.dataset._IPM.br), static_cast<float>(_config.dataset._ROI.y + _config.dataset._ROI.height));
	float direita_delta_x = novoVP.x - bottom_right.x;
	float direita_delta_y = novoVP.y - bottom_right.y;
	float direita_slope = direita_delta_y / direita_delta_x;
	float direita_intercept = bottom_right.y - direita_slope * bottom_right.x;

	// calcula os novos pontos superiores
	float novo_tl = (_config.dataset._ROI.y - esquerda_intercept) / esquerda_slope;
	float novo_tr = (_config.dataset._ROI.y - direita_intercept) / direita_slope;

	// os 4 pontos da imagem de origem
	vector<Point2f> origPoints;
	origPoints.push_back(Point2f(static_cast<float>(novo_tl), 0));
	origPoints.push_back(Point2f(static_cast<float>(novo_tr), 0));
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.br), static_cast<float>(_config.dataset._ROI.height)));
	origPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.bl), static_cast<float>(_config.dataset._ROI.height)));

	// os 4 pontos da imagem de destino
	vector<Point2f> dstPoints;
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tl), 0));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tr), 0));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tr), static_cast<float>(_config.dataset._ROI.height)));
	dstPoints.push_back(Point2f(static_cast<float>(_config.dataset._IPM.tl), static_cast<float>(_config.dataset._ROI.height)));

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- atualiza ipm restrito: " << tempoExecutando << " ms" << endl;

	// objeto IPM
	return new IPM(Size(_config.roi.size().width, _config.roi.size().height), Size(_config.roi.size().width, _config.roi.size().height), origPoints, dstPoints);
}

OutputXML createOutputXML(string _id, Size _frameSize, int _nFrames) {
	OutputXML xml;
	xml.id = _id;
	xml.frameSize = _frameSize;
	xml.nFrames = _nFrames;
	return xml;
}

bool appendFrameData(OutputXML *xml, double laneCenter, vector<double> posLeft, vector<double> posRight, int multipleLeft, int multipleRight, LMT lmtLeft, LMT lmtRight, int laneChange, vector<int> &roadSigns, double time) {
	try {
		xml->laneCenter.push_back(laneCenter);
		xml->position.left.push_back(posLeft);
		xml->position.right.push_back(posRight);
		xml->multipleLanes.left.push_back(multipleLeft);
		xml->multipleLanes.right.push_back(multipleRight);
		xml->lmt.left.push_back(lmtLeft);
		xml->lmt.right.push_back(lmtRight);
		xml->laneChange.push_back(laneChange);
		xml->roadSigns.push_back(roadSigns);
		xml->time.push_back(time);
		return true;
	} catch (Exception e) {
		cout << e.what() << endl;
		return false;
	}
}

bool saveOutputXML(OutputXML *xml, string path) {
	cout << "Salvando..." << endl;
	XMLDocument doc;

	// root node
	XMLNode * testInput = doc.NewElement("testinput");

	// cabe�alho
	XMLElement * dataset = doc.NewElement("dataset");
	dataset->SetAttribute("id", xml->id.c_str());
	testInput->InsertFirstChild(dataset);

	// tempos
	XMLElement * times = doc.NewElement("execution_time");
	for (auto _t : xml->all_times) {
		// ignora esse elemento
		if (_t.first == Task::ALL) continue;

		// calcula a media e desvio padrao
		double m, d;
		Helper::mediaDesvioDouble(_t.second, m, d);

		// adiciona no xml
		XMLElement * time = doc.NewElement("time");
		time->SetAttribute("name", Task::getName(_t.first).c_str());
		time->SetAttribute("mean", m);
		time->SetAttribute("stddev", d);
		times->InsertEndChild(time);
	}
	testInput->InsertEndChild(times);
	
	// frames
	XMLElement * frames = doc.NewElement("frames");
	frames->SetAttribute("n", xml->nFrames);
	for (int i = 0; i < (int)xml->frameNumber.size(); i++) {
		XMLElement * frame = doc.NewElement("frame");

		frame->SetAttribute("id", xml->frameNumber[i]);
		frame->SetAttribute("laneCenter", xml->laneCenter[i]);
		frame->SetAttribute("laneChange", xml->laneChange[i]);
		frame->SetAttribute("laneLeft", xml->multipleLanes.left[i]);
		frame->SetAttribute("laneRight", xml->multipleLanes.right[i]);
		frame->SetAttribute("lmtLeft", xml->lmt.left[i]);
		frame->SetAttribute("lmtRight", xml->lmt.right[i]);
		string roadSigns = "";
		for (unsigned int j = 0; j < xml->roadSigns[i].size(); j++) {
			roadSigns += to_string(xml->roadSigns[i][j]);
			if (j != xml->roadSigns[i].size() - 1) roadSigns += ";";
		}
		frame->SetAttribute("roadSigns", roadSigns.c_str());
		frame->SetAttribute("time", xml->time[i]);

		XMLElement * position = doc.NewElement("position");
		XMLElement * posLeft = doc.NewElement("left");
		XMLElement * p1left = doc.NewElement("p1");
		XMLElement * p2left = doc.NewElement("p2");
		XMLElement * p3left = doc.NewElement("p3");
		XMLElement * p4left = doc.NewElement("p4");
		if (std::isnan(xml->position.left[i][0])) p1left->SetText("nan");
		else p1left->SetText(xml->position.left[i][0]);
		if (std::isnan(xml->position.left[i][1])) p2left->SetText("nan");
		else p2left->SetText(xml->position.left[i][1]);
		if (std::isnan(xml->position.left[i][2])) p3left->SetText("nan");
		else p3left->SetText(xml->position.left[i][2]);
		if (std::isnan(xml->position.left[i][3])) p4left->SetText("nan");
		else p4left->SetText(xml->position.left[i][3]);

		XMLElement * posRight = doc.NewElement("right");
		XMLElement * p1right = doc.NewElement("p1");
		XMLElement * p2right = doc.NewElement("p2");
		XMLElement * p3right = doc.NewElement("p3");
		XMLElement * p4right = doc.NewElement("p4");
		if (std::isnan(xml->position.right[i][0])) p1right->SetText("nan");
		else p1right->SetText(xml->position.right[i][0]);
		if (std::isnan(xml->position.right[i][1])) p2right->SetText("nan");
		else p2right->SetText(xml->position.right[i][1]);
		if (std::isnan(xml->position.right[i][2])) p3right->SetText("nan");
		else p3right->SetText(xml->position.right[i][2]);
		if (std::isnan(xml->position.right[i][3])) p4right->SetText("nan");
		else p4right->SetText(xml->position.right[i][3]);

		// insert the elements
		posLeft->InsertEndChild(p1left);
		posLeft->InsertEndChild(p2left);
		posLeft->InsertEndChild(p3left);
		posLeft->InsertEndChild(p4left);

		posRight->InsertEndChild(p1right);
		posRight->InsertEndChild(p2right);
		posRight->InsertEndChild(p3right);
		posRight->InsertEndChild(p4right);

		position->InsertFirstChild(posLeft);
		position->InsertEndChild(posRight);

		frame->InsertFirstChild(position);

		frames->InsertEndChild(frame);
	}
	testInput->InsertEndChild(frames);
	doc.InsertFirstChild(testInput);

	// print to console:
	// XMLPrinter printer;
	// doc.Print(&printer);
	// cout << printer.CStr() << endl;
	
	doc.SaveFile(path.c_str());
	return true;
}
