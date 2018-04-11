#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include "utils/common.h"
#include "utils/Helper.h"
#include "utils/HelperXML.h"
#include "utils/Viz.h"
#include "PreProcessamento.h"
#include "nieto.h"
#include "Kalman.h"
#include "MapaDeEvidencias.h"
#include "FiltroDeParticulas.h"
#include "FiltroDeParticulasHough.h"
#include "AnaliseDasHoughs.h"
#include "KalmanHoughs.h"
#include "Mapas.h"
#include "Houghs.h"
#include "LMTDetector.h"
#include "SinalizacaoHorizontal.h"
#include "FaixasAdjacentes.h"

#define NUM_PARTICLES 400
#define HOUGH_ONLY false
#define USAR_ALTURA true
#define IMG_QUALITY 1

using namespace std;
using namespace cv;

ConfigXML *config;

// #define TEST_VIDEO		"C:/Users/berriel/datasets/rod2.MP4"
// #define TEST_VIDEO_XML	"C:/Users/berriel/datasets/video-teste/ROD2-VIDEO.xml"

int sair(string mensagem = "", int retorno = EXIT_FAILURE);
void esperaTecla(int * _delay, bool * _keep_running);
string get_now();
void create_batch_eval(vector<string> &runs, string start_time, string dir);
int count_lines(string filename);


int main(int argc, char * argv[]) {

	// le o caminho para o XML com as configuracoes
	if (argc != 3) return sair("Argumentos invalidos!");

	const double dNaN = numeric_limits<double>::quiet_NaN();

	// start time
	string init_time = get_now();
	vector<string> runs;

	// abre o arquivo para processar o lote
	string batch_fname = argv[1];
	ifstream batch_file(batch_fname);
	int n_datasets = count_lines(batch_fname);
	cout << to_string(n_datasets) << " serao processados..." << endl;

	// pega cada dataset do batch para processar
	string dataset_name;
	while (getline(batch_file, dataset_name)) {
	
#pragma region Init
		map<int, TimeMeasurement> timer;
		map<int, vector<double> > all_times;

#ifdef TEST_VIDEO
		VideoCapture cap;
		cap.open(TEST_VIDEO);
		caminho_xml_configuracoes = TEST_VIDEO_XML;
#endif

		// set global config
		config = new ConfigXML();
		config->numParticles = NUM_PARTICLES;

		// load general data from config file
		if (!loadConfig(argv[2], *config)) return sair("Error loading general config file!");

		string DATASET_DIR = config->DATASETS_DIR + dataset_name;
		string caminho_xml_configuracoes = DATASET_DIR + "config.xml";
		cout << caminho_xml_configuracoes << endl;

		// load dataset specific config file
		if (!loadDatasetConfig(caminho_xml_configuracoes, *config)) return sair("Error loading dataset config file!");

		// in case of a batch run (multiple datasets), we do not want outputs
		if (n_datasets > 1) {
			config->verbose = false;
			config->display = false;
		}

		// inicializa o XML de sa�da
		int nframes = config->dataset.FrameSequence.end - config->dataset.FrameSequence.start + 1;
		Size frameSize = Size(config->dataset.FrameSize.width, config->dataset.FrameSize.height);
		OutputXML xml = createOutputXML(config->dataset.id, frameSize, nframes);
		vector<int> saida_y = {
			config->roi.y,
			config->roi.y + (int)ceil((float)config->roi.height / 4.0),
			config->roi.y + (int)ceil((float)config->roi.height / 2.0),
			config->roi.y + config->roi.height - 1
		};

		// Determina a posicao do carro = centro da imagem
		Point posicaoCarro = Point2d(config->dataset.FrameSize.width / 2.0, config->dataset.FrameSize.height);
		Point posicaoCarroIPM = config->ipm->applyHomography(Point2d(config->dataset.FrameSize.width / 2.0, (double)config->roi.height));
		Size ipmSize = config->roi.size();

		// instancia os objetos necess�rios na execu��o
		PreProcessamento preProcessamento(config->roi, config->ipm, config->verbose, false);
		AnaliseDasHoughs _houghs(config);
		KalmanHoughs kalmanHoughs = KalmanHoughs(config, config->verbose, config->display);

		// buffers para as posi��es das houghs medidas
		deque<HoughLine> esqBuffer, dirBuffer, esqBufferRejeitados, dirBufferRejeitados;
		deque<HoughDoMeio> bufferHoughs;
		vector<int> esqAdjBuffer, dirAdjBuffer;
		int esqAdjAnterior = -1, dirAdjAnterior = -1;

		// Filtro de Kalman
		KalmanFilter KF = KalmanFilter(6, 3, 0, CV_64F);
		KalmanState estadoKalman;
		Kalman::inicializa(&KF, &estadoKalman, 6, 3);

		// Filtro de Part�culas
		double larguraPistaInicial = config->dataset._IPM.tr - config->dataset._IPM.tl;
		ParticleFilterHough houghPF(NUM_PARTICLES, posicaoCarroIPM.x, larguraPistaInicial, 0, config->verbose, config->display);
		ParticleHough melhorParticulaHough(posicaoCarroIPM.x, larguraPistaInicial);

		// Tipo de Faixa
		LMTDetector tipoFaixa = LMTDetector();

		// Sinaliza��o horizontal
		SinalizacaoHorizontal::loadTemplates();

		// inicia o loop pelas imagens
		vector<double> tempos;
		deque<Mat> bufferMascara, bufferTemporalBlur;
		bool KEEP_RUNNING = true;
		int delay = (n_datasets > 1) ? 1 : 0;
		int frameNumber = config->dataset.FrameSequence.start;
#pragma endregion
		cout << endl << "Executando " << config->dataset.id << ":" << endl;
		
		int frame_idx = 0;
		//while (frameNumber <= 30000 && KEEP_RUNNING)
		while (frameNumber <= config->dataset.FrameSequence.end && KEEP_RUNNING)
		{
			timer[Task::ALL].start();

			// init values
			int laneChange = 0;
			LMT lmtLeft = LMT::NONE, lmtRight = LMT::NONE;
			int multipleLeft = 0, multipleRight = 0;
			// vector<double> posLeft = { dNaN, dNaN, dNaN, dNaN }, posRight = { dNaN, dNaN, dNaN, dNaN };
			vector<double> posLeft = { -1, -1, -1, -1 }, posRight = { -1, -1, -1, -1 };
			double laneCenter = -1;
			vector<int> roadSigns;

			frame_viz viz;
			viz.frame_number = frameNumber;
			viz.x_carro = posicaoCarro.x;

			// mostra qual frame vai ser processado
			// se for um batch, mostra de 100 em 100
			// senao, mostra todos, caso verbose seja true
			if ((n_datasets > 1 && frameNumber % 100 == 0) || (config->verbose))
				cout << endl << "Frame #" << frameNumber << endl;

			// l� o frame
			Mat3b colorFrame = imread(DATASET_DIR + "/" + "images/lane_" + to_string(frameNumber) + ".png");
			
			// simulate quality loss
			if (IMG_QUALITY != 1) {
				Size originalSize = colorFrame.size();
				resize(colorFrame, colorFrame, Size(0, 0), IMG_QUALITY, IMG_QUALITY, INTER_NEAREST);
				resize(colorFrame, colorFrame, originalSize, 0, 0, INTER_LINEAR);
			}

#ifdef TEST_VIDEO
			Mat3b colorFrameInit;
			cap >> colorFrameInit;

			Mat3b colorFrameFullHD = colorFrameInit(Rect(321,0,1440,1080));
			resize(colorFrameFullHD, colorFrame, Size(640, 480));
#endif

#pragma region Pr�-processamento
			Mat1b grayFrame = Helper::converteParaEscalaDeCinza(colorFrame);
			// Realize o Pre-Processamento (color -> [gray + roi + ipm])
			Mat1b grayFrameRoi; // guarda a ROI em escala de cinza
			Mat1b grayFrameRoiIPM = preProcessamento.executar(colorFrame, grayFrameRoi);
			Mat3b colorFrameRoiIPM = preProcessamento.executar(colorFrame);
			Mat1b mascaraIPM = preProcessamento.getMascaraIPM(-8);
#pragma endregion

			// Monta as imagens usadas para exibi��o
			Mat displayFrameRoiIPM, displayFramePerspectiva;
			cvtColor(grayFrameRoiIPM, displayFrameRoiIPM, CV_GRAY2BGR);
			displayFramePerspectiva = colorFrame.clone();
			
#pragma region Gera��o dos Mapas
			timer[Task::FEATURE_MAPS].start();
			// mapa 1: filtro do nieto
			Mat1b mapa1 = Mapas::getMapa1(grayFrame, 10, 100, 30);
			Mat1b mapa1ipm = Helper::toIPM(mapa1, config->roi, config->ipm, INTER_NEAREST);

			// mapa 2: diferen�a de gaussianas
			Mat1b mapa2ipm = Mapas::getMapa2(grayFrameRoiIPM, mascaraIPM);

			// mapa 4: sobel em y para pegar os carros e obst�culos a frente do ve�culo
			Mat mapa4ipm = Mapas::getMapa4(grayFrameRoiIPM, mascaraIPM);

			// mapa5: lane markings pelo desvio padr�o
			Mat mapa5ipm = Mapas::getMapa5(grayFrameRoi, mapa1, config);

			// mapa resultante
			Mat1d mapaIPM = Mapas::getMapaResultante(mapa1ipm, mapa2ipm, mapa5ipm);
			timer[Task::FEATURE_MAPS].end();

			if (CONFIG_DISPLAY) {
				imshow("mapa1ipm", mapa1ipm);
				imshow("mapa2ipm", mapa2ipm);
				imshow("mapa4ipm", mapa4ipm);
				imshow("mapa5ipm", mapa5ipm);
				imshow("mapaIPM", mapaIPM);
			}
#pragma endregion
			
#pragma region Faixa de Pedestre
			timer[Task::CROSSWALK].start();
			// faixa de pedestre
			Mat1b mapaFaixaPedestreIPM = SinalizacaoHorizontal::detectarFaixaDePedestre2(mapa2ipm, config->roi, posicaoCarroIPM);

			timer[Task::CROSSWALK].end();

			// remove a faixa de pedestre dos mapas
			if (!mapaFaixaPedestreIPM.empty()) {
				if (config->verbose) cout << "faixa de pedestre!" << endl;
				// faixa de pedestre detectada!
				if (roadSigns.size() == 1 && roadSigns[0] == 0) roadSigns[0] = ROAD_SIGN::FAIXA_PEDESTRE;

				// remove dos mapas
				mapa1 = mapa1 & Helper::fillPerspectiva(Helper::toROI(~mapaFaixaPedestreIPM, config->ipm, INTER_NEAREST), config->roi, frameSize, 255);
				mapa1ipm = mapa1ipm & ~mapaFaixaPedestreIPM;
				mapa2ipm = mapa2ipm & ~mapaFaixaPedestreIPM;
				mapa4ipm = mapa4ipm & ~mapaFaixaPedestreIPM;

				// visualiza��o
				viz_symbols symbol_crosswalk;
				symbol_crosswalk.id = ROAD_SIGN::FAIXA_PEDESTRE;
				symbol_crosswalk.region = {Point(-1, -1)};
				viz.symbols.push_back(symbol_crosswalk);
			}

#pragma endregion

#pragma region Sinaliza��o Horizontal
			timer[Task::ROAD_SIGNS].start();
			// sinaliza��o horizontal
			vector<SinalizacaoHorizontal::Blob> roadSignsBlobs;
			roadSigns = SinalizacaoHorizontal::executar(grayFrameRoiIPM, mapa2ipm, mapa4ipm, colorFrame, estadoKalman.hough, config, roadSignsBlobs, viz.symbols);
			timer[Task::ROAD_SIGNS].end();
#pragma endregion

#pragma region Pavement Markings Removal
			timer[Task::SIGNS_REMOVAL].start();
			SinalizacaoHorizontal::removerSinalizacao(mapa1, roadSignsBlobs); // remove as sinaliza��es do mapa
			timer[Task::SIGNS_REMOVAL].end();
#pragma endregion

#pragma region Candidate Generation
			// INICIO DAS HOUGHS - BAGUN�A!
			timer[Task::CANDIDATES_GENERATION].start();

			// pega as houghs no mapa 1
			Mat1b mapa1skel = Helper::skeleton(mapa1); // approx +1.5ms
			vector<HoughLine> houghsAdjacents;
			vector<HoughLine> houghs = Houghs::getHoughs(mapa1skel, config->roi, houghsAdjacents);

			// an�lise das houghs
			vector<HoughLine> houghs_X = { HoughLine::empty(), HoughLine::empty() };
			_houghs.setColorFrame(colorFrame.clone());
			_houghs.executar2D(mapa1ipm, houghs, estadoKalman.hough, mapaIPM, houghs_X);

			// adiciono as houghs em potenciais para o buffer
			bool esqH = false, dirH = false; // flag para descartar as houghs
			if (!houghs_X[0].isEmpty()) {
				if (esqBuffer.size() < TAMANHO_BUFFER_HOUGHS || Houghs::validaHough(houghs_X[0], esqBuffer, config)) {
					esqBufferRejeitados.clear();
					Helper::pushBuffer(esqBuffer, houghs_X[0], TAMANHO_BUFFER_HOUGHS);
				} else {
					Helper::pushBuffer(esqBufferRejeitados, houghs_X[0], TAMANHO_BUFFER_HOUGHS);
					esqH = true;
				}
			}

			if (!houghs_X[1].isEmpty()) {
				if (dirBuffer.size() < TAMANHO_BUFFER_HOUGHS || Houghs::validaHough(houghs_X[1], dirBuffer, config)) {
					dirBufferRejeitados.clear();
					Helper::pushBuffer(dirBuffer, houghs_X[1], TAMANHO_BUFFER_HOUGHS);
				} else {
					Helper::pushBuffer(dirBufferRejeitados, houghs_X[1], TAMANHO_BUFFER_HOUGHS);
					dirH = true;
				}
			}

			bool esqCheio = esqBufferRejeitados.size() == TAMANHO_BUFFER_HOUGHS;
			bool dirCheio = dirBufferRejeitados.size() == TAMANHO_BUFFER_HOUGHS;

			const int larguraMinima = 50;
			if (esqCheio || dirCheio) {

				bool resetarKalman = false;

				// se somente um estiver cheio
				// swap desse buffer
				// zerar os outros
				if (esqCheio) {
					HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(esqBufferRejeitados.back(), dirBuffer.back(), config);
					if (rawMeasurement.largura > larguraMinima) {
						esqBuffer = esqBufferRejeitados;
						resetarKalman = true;
					}
					esqBufferRejeitados.clear();
				}

				if (dirCheio) {
					HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(esqBuffer.back(), dirBufferRejeitados.back(), config);
					if (rawMeasurement.largura > larguraMinima) {
						dirBuffer = dirBufferRejeitados;
						resetarKalman = true;
					}
					dirBufferRejeitados.clear();
				}

				if (resetarKalman) {
					Houghs::validaEmptyHoughs(houghs_X, estadoKalman, config);
					HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(houghs_X[0], houghs_X[1], config);
					houghPF.reset(rawMeasurement.toKalman(), false);
					Kalman::resetaKalman(&KF, 6, 3, rawMeasurement.toKalman());
				}
			}

			if (esqH) houghs_X[0] = HoughLine::empty();
			if (dirH) houghs_X[1] = HoughLine::empty();

			const bool estimarLargura = (!houghs_X[0].isEmpty() && !houghs_X[1].isEmpty());

			Houghs::validaEmptyHoughs(houghs_X, estadoKalman, config);
			timer[Task::CANDIDATES_GENERATION].end();
#pragma endregion

#pragma region Kalman
			timer[Task::KALMAN].start();
			
			// valida se as houghs encontradas s�o suficientes. sen�o, se poss�vel, estime
			HoughDoMeio rawMeasurement = HoughLine::getKalmanMeasurement(houghs_X[0], houghs_X[1], config);

			// troca de faixa
			double distanciaBase = estadoKalman._hough.xBase - rawMeasurement.xBase;
			if (abs(distanciaBase) > estadoKalman._hough.largura * 0.7) if (!estadoKalman.estaDesativado) laneChange = true;

			// se for o primeiro frame
			if (frameNumber == config->dataset.FrameSequence.start) Kalman::resetaKalman(&KF, 6, 3, rawMeasurement.toKalman());

			// conta o n�mero de evid�ncias embaixo das houghs para ver se h� lane
			estadoKalman.nEvidencias = Houghs::contaEvidencias(houghs_X, mapaIPM, config);
			Kalman::estimar(&KF, &estadoKalman, rawMeasurement, estimarLargura);
			timer[Task::KALMAN].end();
			
			if (CONFIG_DISPLAY) {
				// mostra o indicador de desvio do centro
				AnaliseDasHoughs::mostrarDesvioDoCentro(displayFramePerspectiva, rawMeasurement, posicaoCarroIPM.x);

				houghs_X[0].draw(_houghs.colorFrame, Scalar(0, 255, 0));
				houghs_X[1].draw(_houghs.colorFrame, Scalar(0, 255, 255));
				// imshow("Analise das Houghs", _houghs.colorFrame);/**/
				
				kalmanHoughs.view(estadoKalman.hough, displayFramePerspectiva, displayFrameRoiIPM, Scalar(255, 0, 0));
			}
			// FIM DAS HOUGHS  
#pragma endregion

#pragma region Filtro de Part�culas
			timer[Task::PARTICLE_FILTER].start();
			// filtro de part�culas
			int alturaConfiavel = 0; // [0, mapa.rows], onde [0] = tudo confi�vel e [mapa.rows] = nada confi�vel
			if (estadoKalman.hough != NULL && !HOUGH_ONLY) {
				// calcula a altura confi�vel: usando a �ltima virtual best
				alturaConfiavel = houghPF.atualizaAlturaConfiavel(&melhorParticulaHough, estadoKalman._hough, mapa4ipm, roadSignsBlobs);

				// condi��es para resetar o filtro de part�culas
				if (houghPF.alturaConfiavel >= 3 * mapaIPM.rows / 4 && alturaConfiavel <= mapaIPM.rows / 2) {
					houghPF.reset(estadoKalman._hough, false); // resetar o filtro todo
				} else if (houghPF.alturaConfiavel >= 3 * mapaIPM.rows / 8 && alturaConfiavel <= 2 * mapaIPM.rows / 8) {
					houghPF.reset(estadoKalman._hough, true); // resetar o filtro parcialmente (s� o topo)
				}

				// atualiza a altura confi�vel do filtro
				houghPF.alturaConfiavel = alturaConfiavel;

				// executa o filtro de part�culas
				melhorParticulaHough = houghPF.executar(estadoKalman._hough, mapaIPM);
			}
			timer[Task::PARTICLE_FILTER].end();
			if (CONFIG_DISPLAY && !HOUGH_ONLY) houghPF.view(estadoKalman.hough, colorFrame.clone(), displayFrameRoiIPM, config, false, alturaConfiavel);
#pragma endregion

#pragma region Lane Markings Type (LMT)
			// detec��o inicial do tipo da faixa
			timer[Task::LMT].start();
			vector<LMT> LMTs = tipoFaixa.executar(mapa5ipm, displayFramePerspectiva, grayFrameRoiIPM, alturaConfiavel, estadoKalman);
			timer[Task::LMT].end();
			lmtLeft = LMTs[0];
			lmtRight = LMTs[1];

#pragma endregion

#pragma region Faixas Adjacentes
			timer[Task::ADJACENT_LANES].start();
			// faixas adjacentes
			// vector<int> multipleLanes = faixasAdjacentes(mapa2ipm, grayFrameRoiIPM, colorFrameRoiIPM, estadoKalman.hough, melhorParticulaHough, lmtLeft, lmtRight, maxAdjacentes);
			vector<int> multipleLanes = faixasAdjacentesHoughs(mapa1ipm, houghs, houghsAdjacents, estadoKalman.hough, lmtLeft, lmtRight, config);

			// diz que o atual (estimado) � o correto
			multipleLeft = multipleLanes[0];
			multipleRight = multipleLanes[1];

			// rejeita transi��es tempor�rias
			const int adjTamSeq = 3;											// tamanho m�nio da sequ�ncia

			if (esqAdjAnterior == -1) esqAdjAnterior = multipleLanes[0];		// primeira vez
			else if (multipleLanes[0] == esqAdjAnterior) esqAdjBuffer.clear();	// o atual � igual ao anterior, garanta que o buffer est� limpo
			else if (multipleLanes[0] != esqAdjAnterior) {						// se o atual � diferente do anterior
				if (esqAdjBuffer.size() == adjTamSeq) esqAdjBuffer.clear();		// se o buffer estiver cheio:
				else {															// sen�o,
					esqAdjBuffer.push_back(multipleLanes[0]);					//		adiciona o atual no buffer
					multipleLeft = esqAdjAnterior;								//		e diz que o correto � o anterior
				}
			}

			if (dirAdjAnterior == -1) dirAdjAnterior = multipleLanes[1];		// primeira vez
			else if (multipleLanes[1] == dirAdjAnterior) dirAdjBuffer.clear();	// o atual � igual ao anterior, garanta que o buffer est� limpo
			else if (multipleLanes[1] != dirAdjAnterior) {						// se o atual � diferente do anterior
				if (dirAdjBuffer.size() == adjTamSeq) dirAdjBuffer.clear();		// se o buffer estiver cheio:
				else {															// sen�o,
					dirAdjBuffer.push_back(multipleLanes[1]);					//		adiciona o atual no buffer
					multipleRight = dirAdjAnterior;								//		e diz que o correto � o anterior
				}
			}

			esqAdjAnterior = multipleLeft;
			dirAdjAnterior = multipleRight;
			timer[Task::ADJACENT_LANES].end();

			if (CONFIG_VERBOSE) {
				cout << "multiple left: " << multipleLeft << endl;
				cout << "multiple right: " << multipleRight << endl;
				Mat1b ileft = Mat1b(Size(50, 50), abs(multipleLeft * 255));
				Mat1b iRight = Mat1b(Size(50, 50), abs(multipleRight * 255));
				imshow("ileft", ileft);
				imshow("iRight", iRight);
			}
#pragma endregion

#pragma region Sa�da do Lane Estimation (posicionamento das faixas)
			if (!HOUGH_ONLY) {
				// preenche o XML de sa�da com os pontos da melhor part�cula
				for (unsigned int i = 0; i < saida_y.size(); i++) {
					posLeft[i] = melhorParticulaHough.getPoint(saida_y[i], estadoKalman._hough, config, LANE_LEFT, true).x;
					posRight[i] = melhorParticulaHough.getPoint(saida_y[i], estadoKalman._hough, config, LANE_RIGHT, true).x;
				}
			} else {
				// preenche o XML de sa�da com os pontos, por enquanto, usando as houghs
				vector<HoughLine> houghsFinais = estadoKalman._hough.toVetorDeHoughs(config->roi, config->ipm);
				for (unsigned int i = 0; i < saida_y.size(); i++) {
					posLeft[i] = houghsFinais[LANE_LEFT].getX(saida_y[i]);
					posRight[i] = houghsFinais[LANE_RIGHT].getX(saida_y[i]);
				}
			}

			// recusa os pontos que estao acima da altura confiavel
			if (USAR_ALTURA) {
				for (int i = (int)saida_y.size() - 2; i >= 0; i--) {
					if (houghPF.alturaConfiavel > saida_y[i + 1] - config->roi.y) {
						posLeft[i] = numeric_limits<double>::quiet_NaN();
						posRight[i] = numeric_limits<double>::quiet_NaN();
					}
				}
			}
#pragma endregion

#pragma region Calcula o centro da pista estimada
			timer[Task::LANE_CENTER_DEVIATION].start();

			// lane center calculado no P4
			if (posLeft[3] != -1 && posRight[3] != -1) laneCenter = (posLeft[3] + posRight[3]) / 2.0;
			// TODO: if (estadoKalman.hough == NULL) laneCenter = dnan

			timer[Task::LANE_CENTER_DEVIATION].end();
#pragma endregion

			// Fim do loop
			xml.frameNumber.push_back(frameNumber);
			frameNumber++;
			// if (frameNumber % 3 == 0) frameNumber++;
			timer[Task::ALL].end();
			if (config->verbose) cout << "- time: " << timer[Task::ALL].duration() << "ms" << endl;
			// displayTimePerformance(timer);
			if (frameNumber % 100 == 0) cout << frameNumber << "... ";
			esperaTecla(&delay, &KEEP_RUNNING);

			// salva os dados no XML de sa�da
			if (roadSigns.size() == 0) roadSigns.push_back(0);
			double total_time = getTotalTime(timer);
			appendFrameData(&xml, laneCenter, posLeft, posRight, multipleLeft, multipleRight, lmtLeft, lmtRight, laneChange, roadSigns, total_time);

			// guarda os valores para calcular a media e desvio de cada tarefas
			for (auto _t : timer) all_times[_t.first].push_back(_t.second.duration());

#pragma region Visualiza��o
			
			viz.execution_time = total_time;
			viz.isKalmanNull = estadoKalman.hough == NULL;
			if (!viz.isKalmanNull) {
				viz.set_lane_deviation(posicaoCarroIPM.x, estadoKalman.hough->xBase, rawMeasurement.largura);
				viz.set_lane_base(estadoKalman.hough->xBase, estadoKalman.hough->xTopo, estadoKalman.hough->largura, config->roi, config->ipm);
				viz.adjacent_lanes.left = multipleLeft;
				viz.adjacent_lanes.right = multipleRight;
				viz.lane_change = laneChange;
				viz.lmt.left = lmtLeft;
				viz.lmt.right = lmtRight;
				viz.lane_position.left = melhorParticulaHough.getSplinePoints(estadoKalman._hough, config->roi.height, LANE_LEFT);
				viz.lane_position.right = melhorParticulaHough.getSplinePoints(estadoKalman._hough, config->roi.height, LANE_RIGHT);
				viz.trustworthy_height = alturaConfiavel;
			}

			render(viz, colorFrame, config);

#pragma endregion

		}

		cout << endl;

		// guarda os valores no XML para salvar as medias e desvios
		xml.all_times = all_times;

		// mostra as medias e desvios de performance
		displayTimePerformance(all_times);

#pragma region Salva o XML de sa�da
		// salva o XML de sa�da
		if (frameNumber >= config->dataset.FrameSequence.end) {
			cout << "Salvando XML..." << endl;

			string xml_fname = DATASET_DIR + "/runs/" + config->dataset.id + "_" + get_now() + ".xml";
			saveOutputXML(&xml, xml_fname);
			cout << "XML salvo: " << xml_fname << endl;

			runs.push_back(xml_fname);
		}
#pragma endregion
	}

	// cria o arquivo batch_eval_{timestamp}.txt
	if (runs.size() > 0)
		create_batch_eval(runs, init_time, config->DATASETS_DIR + "/batch_eval/");
	
	cout << "Inicio: " << init_time << endl;
	cout << "Fim: " << get_now() << endl;

	// termina a execuccao do programa
	return sair("Programa executado com sucesso!", EXIT_SUCCESS);
}

int sair(string mensagem, int retorno) {
	cout << endl << mensagem << endl;
	cout << "Saindo..." << endl << endl;
	return retorno;
}

void esperaTecla(int * _delay, bool * _keep_running) {
	// (sair: ESQ ou q | pausar e continuar: s)
	char key = waitKey(static_cast<int>((*_delay)));
	if (key == 'q' || key == 'Q' || 27 == (int)key) (*_keep_running) = false;
	else if (key == 's' || key == 'S') (*_delay) = 0;
	else (*_delay) = 1;
}

string get_now() {

#ifdef _WIN32
	auto t = time(nullptr);
	auto tm = *localtime(&t);
	stringstream str_now;
	str_now << put_time(&tm, "%Y%m%d%H%M%S");
	return str_now.str();
#elif __linux__
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer,80,"%Y%m%d%H%M%S",timeinfo);
	string str(buffer);

	return str;
#else

#endif
}

void create_batch_eval(vector<string> &runs, string start_time, string dir) {
	// cria o arquivo
	ofstream file(dir + "batch_eval_" + start_time + ".txt");

	// escreve cada linha no arquivo de saida
	for (unsigned int i = 0; i<runs.size(); i++)
		file << runs[i] << endl;

	// fecha o arquivo
	file.close();
}

// from: http://stackoverflow.com/questions/3482064/counting-the-number-of-lines-in-a-text-file
int count_lines(string filename) {
	int number_of_lines = 0;
	string line;
	ifstream myfile(filename);

	while (getline(myfile, line))
		++number_of_lines;
	
	return number_of_lines;
}
