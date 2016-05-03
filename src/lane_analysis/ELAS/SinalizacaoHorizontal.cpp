#include "SinalizacaoHorizontal.h"

using namespace std;
using namespace cv;

void SinalizacaoHorizontal::loadTemplates() {
	string templateDirectory = "/dados/berriel/MEGA/projects/lane-research/data/images/templates/";

	templates.push_back(TemplateSinalizacao(ROAD_SIGN::FRENTE, imread(templateDirectory + "seta-01-bin.png", IMREAD_GRAYSCALE)));
	templates.push_back(TemplateSinalizacao(ROAD_SIGN::FRENTE, imread(templateDirectory + "seta-02-bin.png", IMREAD_GRAYSCALE)));
	templates.push_back(TemplateSinalizacao(ROAD_SIGN::FRENTE_DIREITA, imread(templateDirectory + "seta-03-bin.png", IMREAD_GRAYSCALE)));
	templates.push_back(TemplateSinalizacao(ROAD_SIGN::FRENTE_ESQUERDA, imread(templateDirectory + "seta-04-bin.png", IMREAD_GRAYSCALE)));
	templates.push_back(TemplateSinalizacao(ROAD_SIGN::DIREITA, imread(templateDirectory + "seta-05-bin.png", IMREAD_GRAYSCALE)));
	templates.push_back(TemplateSinalizacao(ROAD_SIGN::ESQUERDA, imread(templateDirectory + "seta-06-bin.png", IMREAD_GRAYSCALE)));
	templates.push_back(TemplateSinalizacao(ROAD_SIGN::VOLTA_ESQUERDA, imread(templateDirectory + "seta-07-bin.png", IMREAD_GRAYSCALE)));
	templates.push_back(TemplateSinalizacao(ROAD_SIGN::VOLTA_DIREITA, imread(templateDirectory + "seta-08-bin.png", IMREAD_GRAYSCALE)));
}

Mat3b SinalizacaoHorizontal::toIMG(int _id) {
	string templateDirectory = "/dados/berriel/MEGA/projects/lane-research/data/images/templates/";
	switch (_id) {
		case ROAD_SIGN::FRENTE: return imread(templateDirectory + "seta-01-bin.png");
		case ROAD_SIGN::FRENTE_CURTA: return imread(templateDirectory + "seta-02-bin.png");
		case ROAD_SIGN::FRENTE_DIREITA: return imread(templateDirectory + "seta-03-bin.png");
		case ROAD_SIGN::FRENTE_ESQUERDA: return imread(templateDirectory + "seta-04-bin.png");
		case ROAD_SIGN::DIREITA: return imread(templateDirectory + "seta-05-bin.png");
		case ROAD_SIGN::ESQUERDA: return imread(templateDirectory + "seta-06-bin.png");
		case ROAD_SIGN::VOLTA_ESQUERDA: return imread(templateDirectory + "seta-07-bin.png");
		case ROAD_SIGN::VOLTA_DIREITA: return imread(templateDirectory + "seta-08-bin.png");
		case ROAD_SIGN::BARRA_CONTENCAO: return imread(templateDirectory + "stop-line.png");
		case ROAD_SIGN::FAIXA_PEDESTRE: return imread(templateDirectory + "crosswalk.png");
		default: return imread(templateDirectory + "unknown.png");
	}
}

void SinalizacaoHorizontal::templateMatching(const Mat1b &inGrayRoiIPM, const Mat1b &mapaBinario, const KalmanState &kalman) {

	if (kalman.hough != NULL) {

		const int tamanhoBusca = (int)kalman._hough.largura * 1;
		const int metadeLargura = (int)(kalman._hough.largura / 2.0);

		// pega regi�o entre as houghs
		Mat1b regiaoBusca = Mat1b(Size(tamanhoBusca, inGrayRoiIPM.rows), uchar(0));
		Mat1b regiaoBuscaBin = Mat1b(Size(tamanhoBusca, inGrayRoiIPM.rows), uchar(0));
		vector<Point2d> esqHough = (*(kalman.hough)).getHoughPoints(inGrayRoiIPM.rows, LANE_LEFT);

		for (int p = 0; p < esqHough.size(); p++) { // para cada ponto da spline
			for (int i = 0; i < tamanhoBusca; i++) { // para cada pixel do espa�o de busca
				regiaoBusca.at<uchar>(p, i) = inGrayRoiIPM.at<uchar>((int)esqHough[p].y, (int)esqHough[p].x + metadeLargura - int(tamanhoBusca/2.0) + i);
				regiaoBuscaBin.at<uchar>(p, i) = mapaBinario.at<uchar>((int)esqHough[p].y, (int)esqHough[p].x + metadeLargura - int(tamanhoBusca / 2.0) + i);
			}
		}

		// template matching
		string templateDirectory = "/dados/berriel/MEGA/projects/lane-research/data/images/templates/";
		
		Mat1b faixaPedestre = imread(templateDirectory + "faixa-pedestre.png", IMREAD_GRAYSCALE);
		Mat1b seta = imread(templateDirectory + "seta-frente.png", IMREAD_GRAYSCALE);
		Mat1b carro = imread(templateDirectory + "carro.png", IMREAD_GRAYSCALE);

		vector<Mat1b> templs = { seta, faixaPedestre, carro };

		for (Mat1b templ : templs) {
			// Create the result matrix
			int result_cols = regiaoBusca.cols - templ.cols + 1;
			int result_rows = regiaoBusca.rows - templ.rows + 1;
			Mat resultado;
			resultado.create(result_rows, result_cols, CV_32FC1);

			matchTemplate(regiaoBusca, templ, resultado, TM_CCORR_NORMED);
			normalize(resultado, resultado, 0, 1, NORM_MINMAX, -1, Mat());

			double minVal; double maxVal; Point minLoc; Point maxLoc;
			minMaxLoc(resultado, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
			Point matchLoc = maxLoc;

			Rect templArea = Rect(matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows));
			double percMatch = ((double)countNonZero(regiaoBuscaBin(templArea)) / double(templArea.width * templArea.height));
			double percTempl = ((double)countNonZero(seta == 255) / double(seta.cols * seta.rows));

			if (percMatch > percTempl * 0.5 && percTempl < percTempl + percTempl * 0.5)
				rectangle(regiaoBusca, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);

			imshow("SinalizacaoHorizontal", regiaoBusca);
			imshow("SinalizacaoHorizontal Binario", regiaoBuscaBin);
		}
	}
}

// from: http://opencv-code.com/tutorials/fast-template-matching-with-image-pyramid/
void SinalizacaoHorizontal::fastMatchTemplate(Mat &srca, Mat &srcb, Mat &dst, int maxlevel)
{
	vector<Mat> refs, tpls, results;

	// Build Gaussian pyramid
	buildPyramid(srca, refs, maxlevel);
	buildPyramid(srcb, tpls, maxlevel);

	Mat ref, tpl, res;

	// Process each level
	for (int level = maxlevel; level >= 0; level--)
	{
		ref = refs[level];
		tpl = tpls[level];
		res = Mat::zeros(ref.size() + cv::Size(1, 1) - tpl.size(), CV_32FC1);

		if (level == maxlevel)
		{
			// On the smallest level, just perform regular template matching
			cv::matchTemplate(ref, tpl, res, CV_TM_CCORR_NORMED);
		} else {
			// On the next layers, template matching is performed on pre-defined 
			// ROI areas.  We define the ROI using the template matching result 
			// from the previous layer.

			Mat mask;
			pyrUp(results.back(), mask);

			Mat mask8u;
			mask.convertTo(mask8u, CV_8U);

			// Find matches from previous layer
			vector<vector<Point> > contours;
			findContours(mask8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

			// Use the contours to define region of interest and 
			// perform template matching on the areas
			for (int i = 0; i < contours.size(); i++)
			{
				Rect r = boundingRect(contours[i]);
				matchTemplate(
					ref(r + (tpl.size() - cv::Size(1, 1))),
					tpl,
					res(r),
					CV_TM_CCORR_NORMED
					);
			}
		}

		// Only keep good matches
		threshold(res, res, 0.9, 1., CV_THRESH_TOZERO);
		results.push_back(res);
	}

	res.copyTo(dst);
}

vector<int> SinalizacaoHorizontal::executar(const Mat1b &inGrayRoiIPM, const Mat1b &mapa2ipm, const Mat1b &mapa4ipm, const Mat3b &framePerspectiva, const HoughDoMeio * kalman_hough,
	ConfigXML *config, vector<Blob> &outBlobs, vector<viz_symbols> &symbols) {
	
	vector<int> roadSigns;
	Mat displaySinalizacao = framePerspectiva.clone();
	if (kalman_hough != NULL) {

		const int tamanhoBusca = (int)(kalman_hough->largura * 0.8);
		const int metadeLargura = (int)(kalman_hough->largura / 2.0);

		// mapa: DoG ou sobel-horizontal
		Mat1b mapaResultante;
		Mat1b mapa2ipm_median;
		Mat1b mapa4ipm_median;
		medianBlur(mapa2ipm, mapa2ipm_median, 3);
		medianBlur(mapa4ipm, mapa4ipm_median, 3);
		bitwise_or(mapa2ipm_median, mapa4ipm_median, mapaResultante);

		// imshow("mapa sinalizacao horizontal", mapaResultante);

		// pega a regi�o entre as houghs
		vector<Point2d> esqHough = kalman_hough->getHoughPoints(inGrayRoiIPM.rows, LANE_LEFT);
		vector<Point2d> dirHough = kalman_hough->getHoughPoints(inGrayRoiIPM.rows, LANE_RIGHT);
		Mat1b regiaoBuscaBin = Mat1b(Size(tamanhoBusca, inGrayRoiIPM.rows), uchar(0));
		Mat1b regiaoBuscaGray = Mat1b(Size(tamanhoBusca, inGrayRoiIPM.rows), uchar(0));
		for (int p = 0; p < esqHough.size(); p++) { // para cada ponto da spline
			for (int i = 0; i < tamanhoBusca; i++) { // para cada pixel do espa�o de busca
				regiaoBuscaBin.at<uchar>(p, i) = mapaResultante.at<uchar>((int)esqHough[p].y, (int)esqHough[p].x + metadeLargura - int(tamanhoBusca / 2.0) + i);
				regiaoBuscaGray.at<uchar>(p, i) = inGrayRoiIPM.at<uchar>((int)esqHough[p].y, (int)esqHough[p].x + metadeLargura - int(tamanhoBusca / 2.0) + i);
			}
		}

		// morph close
		Mat regiaoBuscaBinDilated = Helper::morphDilate(regiaoBuscaBin, 3);
		Mat regiaoBuscaBinEroded = Helper::morphErode(regiaoBuscaBin, 3);

		// m�dia do que n�o est� no mapa resultante (provavelmente, asfalto)
		Mat1b inversoRegiaoBusca = ~regiaoBuscaBinDilated;
		inversoRegiaoBusca(Rect(0, 0, inversoRegiaoBusca.cols, inversoRegiaoBusca.rows / 2)).setTo(0);
		Scalar mediaAsfalto, desvioPadraoAsfalto;
		meanStdDev(regiaoBuscaGray, mediaAsfalto, desvioPadraoAsfalto, inversoRegiaoBusca);
		
		vector<Blob> blobs = scan(regiaoBuscaBin, regiaoBuscaGray, mediaAsfalto[0] + desvioPadraoAsfalto[0], 0.8);
		vector<Blob> trimBlobs = trim(blobs);

		// salvar blobs
		for (unsigned int i = 0; i < trimBlobs.size(); i++) {
			Scalar cor = BGR_RED;
			// s� faz reconhecimento do que � sinaliza��o
			if (trimBlobs[i].tipo == BlobTipo::NAO_SINALIZACAO) continue;

			// n�o quero reconhecer faixa de conten��o pr�ximo ao limite da imagem
			double proporcao = (trimBlobs[i].regiao.width / (double)trimBlobs[i].regiao.height);
			if (proporcao > 2.3 && abs(trimBlobs[i].regiao.y + trimBlobs[i].regiao.height - config->roi.height) > 5) {
				roadSigns.push_back(ROAD_SIGN::BARRA_CONTENCAO);
				cor = BGR_BLUE;
				trimBlobs[i].templateId = ROAD_SIGN::BARRA_CONTENCAO;
			} else {
				// faz o template match
				TemplateSinalizacao sinalizacao = matchTemplates(trimBlobs[i].gray);
				// guarda o id do s�mbolo reconhecido
				roadSigns.push_back(sinalizacao.id);
				trimBlobs[i].templateId = sinalizacao.id;

				if (config->display) {
					// converte o simbolo de GRAY para BGR para mostrar o simbolo que foi reconhecido
					Mat sinalizacaoCor;
					cvtColor(sinalizacao.imagem, sinalizacaoCor, CV_GRAY2BGR);
					sinalizacaoCor.copyTo(displaySinalizacao(Rect(15, 15, sinalizacaoCor.cols, sinalizacaoCor.rows)));
				}
			}
			
			if (config->display) {
				imshow("template", trimBlobs[i].binario);
				// guarda os pontos tanto na IPM quanto da perspectiva
				setPoints(trimBlobs[i], esqHough, dirHough, config);
				outBlobs.push_back(trimBlobs[i]);
				fillConvexPoly(displaySinalizacao, trimBlobs[i].pointsPerspectiva, cor);

				// visualiza��o
				viz_symbols symbol;
				symbol.id = trimBlobs[i].templateId;
				symbol.region = trimBlobs[i].pointsPerspectiva;
				symbols.push_back(symbol);
			}
		}
	}

	// mostra a imagem com a sinalizacao
	if (config->display) {
		double alpha = 0.5;
		addWeighted(displaySinalizacao, alpha, framePerspectiva, 1 - alpha, 0.0, displaySinalizacao);
		imshow("display sinalizacao", displaySinalizacao);
	}

	// se roadSigns estiver vazio, eh pq nao ha sinalizacao horizontal
	if (roadSigns.size() == 0) roadSigns.push_back(0);

	// retorna o vetor com as sinalizacoes identificadas
	return roadSigns;
}

// pega um mapa bin�rio, passa uma scan line (ou regi�o) no centro do mapa e retorna os blobs encontrados
vector<SinalizacaoHorizontal::Blob> SinalizacaoHorizontal::scan(const Mat1b &mapa, const Mat1b &mapaGray, const double thresholdAsfalto, const double tamanhoPercentual) {
	
	vector<Blob> blobs;
	const int alturaScan = 5;
	const int tamanho = (int)(mapa.cols * tamanhoPercentual);
	const int xInicio = (int)(mapa.cols / 2.0 - tamanho / 2.0);

	// gera um histograma da quantidade de evid�ncias
	Mat1d histograma = Mat1d(Size(1, mapa.rows), double(0));
	for (int i = 0; i < mapa.rows - alturaScan; i++) {
		Rect areaScan = Rect(xInicio, i, tamanho, alturaScan);
		histograma.at<double>(i) = countNonZero(mapa(areaScan));
	}

	// verifica segmentos acima de 5 evid�ncias
	int inicio = -1, fim = -1;
	const int thresEvidencias = 5;
	const int thresTamanhoMinimo = 10;
	for (int i = 0; i < histograma.rows; i++) {
		double v = histograma.at<double>(i);
		if (v > thresEvidencias) { // continuidade
			if (inicio == -1) inicio = i; // inicio
		} else if ((v <= thresEvidencias || i == histograma.rows -1) && inicio != -1) { // fim
			fim = i;
			int tamanho = fim - inicio;
			if (tamanho >= thresTamanhoMinimo) {
				// se o blob estiver na �rea de interesse (metade inferior da regi�o de busca), ele ser� retornado
				if (inicio > mapa.rows / 2) {
					const int posicaoFinalBlob = (inicio - alturaScan) + (tamanho + 2 * alturaScan);
					Rect areaBlob = Rect(0, ((inicio - alturaScan) < 0) ? 0 : (inicio - alturaScan), mapa.cols, (posicaoFinalBlob >= histograma.rows) ? (histograma.rows - (inicio - alturaScan) - 1) : (tamanho + 2 * alturaScan));

					// pega a m�dia do que est� no mapa bin�rio
					Scalar media, desvio;
					Mat blobEroded = Helper::morphErode(mapa(areaBlob).clone(), 3);
					Mat1b blobGray = mapaGray(areaBlob).clone();
					meanStdDev(blobGray, media, desvio, blobEroded);

					// aplica o threshold estat�stico
					Blob b;
					b.binario = mapa(areaBlob).clone();
					b.gray = blobGray;
					b.regiao = areaBlob;
					b.tipo = (media[0] <= thresholdAsfalto) ? BlobTipo::NAO_SINALIZACAO : BlobTipo::SINALIZACAO;
					b.templateId = -1;

					blobs.push_back(b);
				}
			}
			inicio = -1;
			fim = -1;
		} else {
			inicio = -1;
			fim = -1;
		}
	}

	return blobs;
}

vector<SinalizacaoHorizontal::Blob> SinalizacaoHorizontal::trim(const vector<Blob> &blobs) {
	
	vector<Blob> novosBlobs;

	// para cada blob, fazer um trim
	for (unsigned int i = 0; i < blobs.size(); i++) {

		if (blobs[i].tipo == BlobTipo::NAO_SINALIZACAO) continue;

		bool achouInicio, achouFim;
		const int thresEvidencias = 0;

		// histograma colunas
		Mat1d histogramaColunas = Mat1d(Size(blobs[i].binario.cols, 1), uchar(0));
		reduce(blobs[i].binario, histogramaColunas, 0, CV_REDUCE_SUM, CV_64F); // colunas
		histogramaColunas /= 255;

		// TRIM: colunas
		// come�ando do meio, verificar os limites superiores e inferiores
		const int metadeColunasBlob = histogramaColunas.cols / 2;
		int colunaInicio = -1, colunaFim = -1;
		achouInicio = false, achouFim = false;
		for (int j = 0; j < metadeColunasBlob; j++) {
			if (metadeColunasBlob - j >= 0) {
				double vInicio = histogramaColunas.at<double>(metadeColunasBlob - j);
				if (!achouInicio && vInicio > thresEvidencias) colunaInicio = metadeColunasBlob - j - 1;
				else achouInicio = true;
			}

			if (metadeColunasBlob + j < histogramaColunas.cols) {
				double vFim = histogramaColunas.at<double>(metadeColunasBlob + j);
				if (!achouFim && vFim > thresEvidencias) colunaFim = metadeColunasBlob + j + 1;
				else achouFim = true;
			}
			if (achouInicio && achouFim) break;
		}

		// TRIM: linhas
		// come�ando do meio, verificar os limites superiores e inferiores
		Mat1d histogramaLinhas = Mat1d(Size(blobs[i].binario.rows, 1), double(0));
		reduce(blobs[i].binario, histogramaLinhas, 1, CV_REDUCE_SUM, CV_64F); // colunas
		histogramaLinhas /= 255;
		int linhaInicio = -1, linhaFim = -1;
		for (int j = 0; j < blobs[i].binario.rows; j++) {
			linhaInicio = j;
			if (histogramaLinhas.at<double>(j) > thresEvidencias) break;
		}
		for (int j = blobs[i].binario.rows-1; j >= 0; j--) {
			linhaFim = j;
			if (histogramaLinhas.at<double>(j) > thresEvidencias) break;
		}

		// se for v�lido
		if (colunaInicio != -1 && colunaFim != -1) {
			Rect novaAreaBlob = Rect(colunaInicio, linhaInicio, colunaFim - colunaInicio, linhaFim-linhaInicio);
			Blob b;
			b.binario = blobs[i].binario(novaAreaBlob);
			b.gray = blobs[i].gray(novaAreaBlob);
			b.regiao = Rect(colunaInicio, blobs[i].regiao.y, colunaFim - colunaInicio, blobs[i].regiao.height);
			b.tipo = blobs[i].tipo;
			b.templateId = blobs[i].templateId;
			novosBlobs.push_back(b);
		}
	}

	return novosBlobs;

}

SinalizacaoHorizontal::TemplateSinalizacao SinalizacaoHorizontal::matchTemplates(Mat1b &sample, const double threshold, int flag) {
	
	// redimensiona as amostras para 32x32
	Mat1b resizedSample;
	resize(sample, resizedSample, Size(32, 32));
	normalize(resizedSample, resizedSample, 0, 255, NORM_MINMAX);

	// faz o match com os templates
	Mat1f methodHistograma = Mat1f(1, int(templates.size()), float(0));
	for (unsigned int i = 0; i < templates.size(); i++) {
		Mat1f result = Mat1f(1, 1);
		// flags allowed:
		// - CV_TM_CCORR{_NORMED}
		// - CV_TM_CCOEFF{_NORMED}
		// - CV_TM_SQDIFF{_NORMED}
		matchTemplate(resizedSample, templates[i].imagem, result, flag);
		methodHistograma.at<float>(i) = result.at<float>(0);
	}

	if (false) {
		cout << "template-matching histogram: " << methodHistograma << endl;
		Scalar mean, std; double minHist, maxHist;
		meanStdDev(methodHistograma, mean, std);
		minMaxIdx(methodHistograma, &minHist, &maxHist);
		cout << "mean: " << mean[0] << ", std: " << std[0] << ", min-max: " << maxHist - minHist << ", min: " << minHist << ", max: " << maxHist << endl;
	}

	// pega o maior
	double min, max;
	Point minIdx, maxIdx, outIdx;
	if (flag == CV_TM_CCORR_NORMED) {
		minMaxLoc(methodHistograma, &min, &max, &minIdx, &maxIdx);
		outIdx = maxIdx;
	} else if (flag == CV_TM_SQDIFF_NORMED) {
		double min, max;
		minMaxLoc(methodHistograma, &min, &max, &minIdx, &maxIdx);
		outIdx = minIdx;
	}

	return (max > threshold) ? templates[outIdx.x] : TemplateSinalizacao(ROAD_SIGN::NONE, imread("/dados/berriel/MEGA/projects/lane-research/data/images/templates/none.png", IMREAD_GRAYSCALE));
}

bool SinalizacaoHorizontal::eFaixaDePedestre(const Blob &_blob) {
	// TODO
	// tentativas:
	// - BBPP + PPBB kernels
	// - BPB + PBP kernels
	// - canny

	return false;
}

void SinalizacaoHorizontal::setPoints(Blob &_blob, const vector<Point2d> &esqHough, const vector<Point2d> &dirHough, ConfigXML *config) {
	// desenha o poligono na img em perspectiva
	// pontos na IPM
	Point2d tlRegiaoIPM = esqHough[_blob.regiao.y];
	Point2d blRegiaoIPM = esqHough[_blob.regiao.y + _blob.regiao.height];
	Point2d trRegiaoIPM = dirHough[_blob.regiao.y];
	Point2d brRegiaoIPM = dirHough[_blob.regiao.y + _blob.regiao.height];

	// converte para perspectiva
	Point tlBlobPerspectiva = config->ipm->applyHomographyInv(tlRegiaoIPM);
	Point blBlobPerspectiva = config->ipm->applyHomographyInv(blRegiaoIPM);
	Point trBlobPerspectiva = config->ipm->applyHomographyInv(trRegiaoIPM);
	Point brBlobPerspectiva = config->ipm->applyHomographyInv(brRegiaoIPM);

	// adiciona o shift da ROI
	Point roi_tl = config->roi.tl();
	tlBlobPerspectiva += roi_tl;
	blBlobPerspectiva += roi_tl;
	trBlobPerspectiva += roi_tl;
	brBlobPerspectiva += roi_tl;

	vector<Point> points, pointsIPM;
	points.reserve(4);
	pointsIPM.reserve(4);

	_blob.pointsPerspectiva = { tlBlobPerspectiva, trBlobPerspectiva, brBlobPerspectiva, blBlobPerspectiva };
	_blob.pointsIPM = { tlRegiaoIPM, trRegiaoIPM, brRegiaoIPM, blRegiaoIPM };
}

void SinalizacaoHorizontal::removerSinalizacao(Mat1b &mapa, vector<Blob> &blobs, int erodeSize, int dilateSize) {
	for (Blob _blob : blobs) {

		// aplica um 'erode lateral' = erodeSize
		vector<Point> _p = {
			_blob.pointsPerspectiva[0] + Point(erodeSize, 0),	// top-left
			_blob.pointsPerspectiva[1] + Point(-erodeSize, 0),	// top-right
			_blob.pointsPerspectiva[2] + Point(-erodeSize, dilateSize),	// bottom-right
			_blob.pointsPerspectiva[3] + Point(erodeSize, dilateSize),	// bottom-left
		};

		// modifica os pixels dessa area para zero
		fillConvexPoly(mapa, _p, Scalar(0, 0, 0));
	}
}

void SinalizacaoHorizontal::detectarFaixaDePedestre(const Mat1b &mapa, const Rect &roi) {
	
	Mat1b mapaEval = mapa.clone();
	imshow("mapaEval", mapaEval);
	mapaEval = Helper::morphDilate(mapaEval, 3);
	mapaEval = Helper::morphDilate(mapaEval, 3);
	mapaEval = Helper::morphErode(mapaEval, 3);
	mapaEval = Helper::morphErode(mapaEval, 3);

	Mat1b mapaClone = mapaEval.clone();
	mapaClone = Helper::morphErode(mapaClone, 3);
	mapaClone = Helper::morphErode(mapaClone, 3);
	mapaClone = Helper::morphErode(mapaClone, 3);

	Mat3b mapaOutput;
	cvtColor(mapaClone, mapaOutput, CV_GRAY2BGR);
	
	vector<vector<Point> > lineSegments;
	vector<Vec4i> lines;

	const int houghThreshold = 1;
	const int houghMinLineLength = 10;
	const int houghMaxLineGap = 3;

	Mat1b maskHough = Mat1b::zeros(mapaClone.size());
	Rect areaInteresseFaixa = Rect(mapaClone.cols / 2 - 50, mapaClone.rows / 2, 100, 30);
	Mat1b interesseFaixa = mapaClone(areaInteresseFaixa);
	maskHough(areaInteresseFaixa) = 255;

	imshow("mapaClone AND", mapaClone & maskHough);

	cv::HoughLinesP(mapaClone & maskHough, lines, 1, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
	cout << "n houghs: " << lines.size() << endl;

	// guarda as houghs (line segments)
	for (size_t i = 0; i<lines.size(); i++)	{
		Point pt1, pt2;
		pt1.x = lines[i][0];
		pt1.y = lines[i][1];
		pt2.x = lines[i][2];
		pt2.y = lines[i][3];

		// Store into vector of pairs of Points for msac
		lineSegments.push_back({pt1, pt2});
	}

	// transforma os line segments nas houghs
	vector<HoughLine> houghs = HoughLine::getVetorDeHoughs({ lineSegments }, roi.height, 0);

	// monta o histograma 2d
	Mat1d histograma2D = montarHistograma2D(houghs, mapaClone, false);

	Mat1b GrayHistograma2D;
	histograma2D.convertTo(GrayHistograma2D, CV_8UC3, 255);

	Mat3b ColorHistograma2D;
	cvtColor(GrayHistograma2D, ColorHistograma2D, CV_GRAY2BGR);
	
	// pega o angulo dominante
	const int regiaoAngulo = 9;
	int anguloDominante = getAnguloDominante(histograma2D, regiaoAngulo);
	
	int distancia = -1;
	int nSegmentosBranco = 0, nSegmentosPreto = 0;
	const int thresholdTamanho = 45;

	// se h� um angulo dominante
	if (anguloDominante != -1) {

		// apaga as houghs com angulos distantes do angulo dominante
		for (auto it = houghs.begin(); it != houghs.end();) {
			if (abs((*it).getAngulo() - anguloDominante) > regiaoAngulo) it = houghs.erase(it);
			else ++it;
		}

		Mat1b mapaHoughs = Mat1b::zeros(mapaClone.size());
		for (size_t i = 0; i < houghs.size(); i++) line(mapaHoughs, houghs[i]._p1, houghs[i]._p2, Scalar(255), 1);
		

		//mapaHoughs = Helper::morphDilate(mapaHoughs, 15);
		//bitwise_and(mapaHoughs, mapaEval, mapaHoughs);

		Mat1b mapaHoughsRotacionado;
		Helper::rotate2D(mapaHoughs, mapaHoughsRotacionado, (float)anguloDominante - 90, INTER_NEAREST);

		// rotaciona a imagem
		Mat1b mapaRotacionado;
		Helper::rotate2D(mapaEval, mapaRotacionado, (float)anguloDominante - 90, INTER_NEAREST);
		
		// monta o histograma
		Mat1d histogramaFaixa;
		cv::reduce(mapaHoughsRotacionado, histogramaFaixa, 1, CV_REDUCE_SUM);
		histogramaFaixa = histogramaFaixa * 1 / 255.0;

		// pega o m�ximo do histograma
		double min, max; int minIdx, maxIdx;
		minMaxIdx(histogramaFaixa, &min, &max, &minIdx, &maxIdx);

		// pega a regi�o candidata a conter uma faixa de pedestre
		int inicio = (maxIdx - regiaoAngulo < 0) ? 0 : maxIdx - regiaoAngulo;
		int tamanhoRegiaoFaixa = (2 * regiaoAngulo + 1);
		tamanhoRegiaoFaixa = (inicio + tamanhoRegiaoFaixa >= histogramaFaixa.rows) ? histogramaFaixa.rows - inicio - 1 : tamanhoRegiaoFaixa;
		Rect areaFaixa = Rect(0, inicio, mapaRotacionado.cols, tamanhoRegiaoFaixa);
		Mat1b candidatoFaixa = mapaRotacionado(areaFaixa);
		Mat1b houghsFaixa = mapaHoughsRotacionado(areaFaixa);

		// verifica a distancia entre o primeiro e o �ltimo pixels branco
		cv::reduce(candidatoFaixa, candidatoFaixa, 0, CV_REDUCE_MAX);
		cv::reduce(houghsFaixa, houghsFaixa, 0, CV_REDUCE_MAX);

		nSegmentosBranco = Helper::contarSegmentosBranco(candidatoFaixa, 2, 10);
		nSegmentosPreto = Helper::contarSegmentosPreto(candidatoFaixa, 2, 10);
		cout << "n segmentos branco: " << nSegmentosBranco << endl;
		cout << "n segmentos preto: " << nSegmentosPreto << endl;

		int primeiro = -1, ultimo = -1;
		for (int i = 0; i < houghsFaixa.cols; ++i) {
			if (houghsFaixa.at<uchar>(i) == 255) {
				primeiro = i;
				break;
			}
		}

		for (int i = houghsFaixa.cols - 1; i >= 0; --i) {
			if (houghsFaixa.at<uchar>(i) == 255) {
				ultimo = i;
				break;
			}
		}

		distancia = ultimo - primeiro;
		cout << "distancia: " << distancia << endl;

		if (distancia > 0) {
			Rect areaFaixaDelimitada = Rect(primeiro, inicio, ultimo - primeiro + 1, tamanhoRegiaoFaixa);
			Mat1b candidatoFaixaDelimitada = interesseFaixa.clone();
			imshow("candidatoFaixaDelimitada", candidatoFaixaDelimitada);

			// threshold inicial, s� para constar
			if (distancia > thresholdTamanho && nSegmentosBranco >= 3) {

				// assert
				// Mat1d kernel = (Mat1d(Size(5, 1)) << -1, 1, -1, 1, -1);
				// Mat1d image = (Mat1d(Size(5, 1)) << 1, -1, 1, -1, 1);

				Mat1b sampleImageUchar; Mat1d sampleImage;
				cv::reduce(candidatoFaixaDelimitada, sampleImageUchar, 0, CV_REDUCE_MAX);
				sampleImageUchar.convertTo(sampleImage, CV_64FC1, 1 / 255.0);

				// ideia: convolu��o com o oposto deveria ter o m�ximo no caso de periodicidade, que � o esperado para faixas de pedestre
				Mat1d kernel = ~sampleImageUchar;
				cv::normalize(kernel, kernel, -1.0, 1.0, NORM_MINMAX); // range(-1.0, 1.0)

				// Mat1d image;
				// candidatoFaixaDelimitada.convertTo(image, CV_64FC1, 1/255.0);
				cv::normalize(sampleImage, sampleImage, -1.0, 1.0, NORM_MINMAX); // range(-1.0, 1.0)
				// TODO: essa normaliza��o n�o deve ser feita assim
				
				// Mat1d resultImage;
				// Helper::filter2Dsoma(sampleImage, kernel, resultImage);
				
				Mat1d resultImage;
				filter2D(sampleImage, resultImage, CV_64FC1, kernel, Point(-1, -1), 0.0, BORDER_CONSTANT);

				// resultImage = abs(resultImage);

				// normalize(resultImage, resultImage, 0.0, 1.0, NORM_MINMAX);
				imshow("resultImage", resultImage);

				if (false) {
					cout << "kernel: " << kernel << endl;
					cout << "sampleImage: " << sampleImage << endl;
					cout << "resultImage: " << resultImage << endl;
				}

				Helper::visualizarHistograma(resultImage, "result image histograma");

				double min, max;
				minMaxIdx(resultImage, &min, &max);

				cout << "cols: " << resultImage.cols << endl;
				cout << "soma: " << sum(resultImage)[0] / (double)resultImage.cols << endl;
				cout << "max: " << max << endl;

			}
		}
		
		
		// visualiza��es
		rectangle(ColorHistograma2D, Rect(0, anguloDominante - regiaoAngulo, ColorHistograma2D.cols, 2 * regiaoAngulo + 1), Scalar(0, 0, 255)); // regi�o do angulo dominante
		Helper::visualizarHistograma(histogramaFaixa, "imgHist", 150, true); // histograma dos pixels projetado
		
		imshow("candidatoFaixa", mapaRotacionado(areaFaixa));
		
		rectangle(mapaRotacionado, areaFaixa, Scalar(255)); // area da faixa de pedestre
		imshow("mapaRonacionado", mapaRotacionado);
	}
	
	imshow("Sinalizacao::Histograma2D", ColorHistograma2D);
	// mostra as houghs
	for (size_t i = 0; i < houghs.size(); i++) line(mapaOutput, houghs[i]._p1, houghs[i]._p2, Scalar(0, 0, 255), 1);
	imshow("detectarFaixaDePedestre", mapaOutput);

	if (distancia > thresholdTamanho && nSegmentosBranco >= 3) waitKey();
}

Mat1d SinalizacaoHorizontal::montarHistograma2D(const vector<HoughLine> &_houghs, const Mat1b &mapa, bool normalizar) {

	int _binsize_posicao = 3;
	int _binsize_angulo = 1;

	// inicia histograma com zeros
	Mat1d histograma2d = Mat1d(Size(mapa.cols, 360), double(0));
	Mat1d histograma2dImg = Mat1d(Size(mapa.cols, 360), double(0));

	Mat3b mapaClone;
	cvtColor(mapa, mapaClone, CV_GRAY2BGR);

	// calcula o histograma
	for (HoughLine h : _houghs) {
		if (h.p1.x < 0 || h.p1.x >= mapa.cols) continue;

		int y = (int)Helper::roundMultiple(h.getAngulo(), (double)_binsize_angulo);
		int x = (int)h.p1.x;
		x -= x % _binsize_posicao;

		histograma2d.at<double>(Point(x, y)) += 1;
		
		// mostra as houghs progressivamente
		circle(histograma2dImg, Point(x, y), 1, Scalar(255), CV_FILLED);
		line(mapaClone, h._p1, h._p2, Scalar(0, 0, 255));
	}

	if (normalizar) cv::normalize(histograma2d, histograma2d, 0.0, 1.0, NORM_MINMAX);
	
	return histograma2d;
}

int SinalizacaoHorizontal::getAnguloDominante(const Mat1d &histograma2d, int gaussianBlurSize) {

	Mat1d _histograma2d, histogramaAngulos;

	// aplica um blur
	_histograma2d = histograma2d.clone();
	
	// reduz o histograma a uma �nica coluna, acumulando (somando) as linhas
	cv::reduce(_histograma2d, histogramaAngulos, 1, CV_REDUCE_SUM);

	// acumula as contagens pr�ximas
	Mat1d kernel = Mat1d::ones(Size(1, gaussianBlurSize));
	filter2D(histogramaAngulos, histogramaAngulos, CV_64F, kernel);

	// verifica qual � o angulo dominante
	int anguloDominante = -1;
	double maxVal = 0;
	for (int i = 0; i < histogramaAngulos.rows; ++i) {
		if (histogramaAngulos.at<double>(i) > maxVal) {
			anguloDominante = i;
			maxVal = histogramaAngulos.at<double>(i);
		}
	}

	// esse � o n�mero minimo de houghs para aceitar como angulo dominante
	const int minValue = 3;
	if (maxVal <= minValue) return -1;

	return anguloDominante;

}

Mat1b SinalizacaoHorizontal::detectarFaixaDePedestre2(const Mat1b &mapa, const Rect &roi, const Point &posicaoCarroIPM) {

	// 1. close: para deixar as faixas sem 'buracos' -> gera MAPA_CLOSE
	// 2. erode: para afinar e pegar menos houghs -> gera MAPA_ERODE
	// 3. define a regi�o de interesse para a faixa de pedestre (ROI_FAIXA)
	// 4. calcula a matrix de rota��o que normaliza a imagem para que a regi�o de busca seja sempre a mesma
	// 5. rotaciona a imagem em rela��o ao angulo da hough anterior (matrix de 3.), se n�o tiver, n�o rotaciona
	//		pode ser feito depois para acelerar o teste do m�todo
	// 6. pega as houghs na ROI_FAIXA do MAPA_ERODE -> gera MAPA_HOUGHS
	// 7. pega o angulo dominante das houghs -> gera ANGULO_DOMINANTE
	// 8. rotaciona a ROI_FAIXA (ou o MAPA_CLOSE? ou o MAPA_HOUGHS?) pelo ANGULO_DOMINANTE -> gera REGIAO_FINAL
	// 9. monta o kernel da convolu��o com um NOT do OR da REGIAO_FINAL (reduce to single row with MAX) -> gera REGIAO_FINAL_OR e KERNEL
	// 10. convolui REGIAO_FINAL_OR usando KERNEL e pega os m�ximos das duas metades da imagem resultante (o MIN da imagem ser� no meio) -> gera RESULT e SOMA_MAX
	// 11. verifica se SOMA_MAX est� acima de um threshold (definir threshold deterministicamente)

	// PASSO 1
	Mat1b MAPA_CLOSE = mapa.clone();
	MAPA_CLOSE = Helper::morphDilate(MAPA_CLOSE, 3);
	MAPA_CLOSE = Helper::morphDilate(MAPA_CLOSE, 3);
	MAPA_CLOSE = Helper::morphErode(MAPA_CLOSE, 3);
	MAPA_CLOSE = Helper::morphErode(MAPA_CLOSE, 3);

	// PASSO 2
	Mat1b MAPA_ERODE = MAPA_CLOSE.clone();
	MAPA_ERODE = Helper::morphErode(MAPA_ERODE, 3);
	MAPA_ERODE = Helper::morphErode(MAPA_ERODE, 3);
	MAPA_ERODE = Helper::morphErode(MAPA_ERODE, 3);

	// PASSO 3
	// obs: tive que pegar a posi��o do carro, pois o centro da imagem na IPM nem sempre est� alinhado ao centro do carro
	int ROI_FAIXA_LARGURA = 100, ROI_FAIXA_ALTURA = MAPA_ERODE.rows / 2;
	Rect ROI_FAIXA = Rect(posicaoCarroIPM.x - (ROI_FAIXA_LARGURA / 2), MAPA_CLOSE.rows / 2, ROI_FAIXA_LARGURA, ROI_FAIXA_ALTURA);
	Mat1b MASK_ROI_FAIXA = Mat1b::zeros(MAPA_ERODE.size());
	MASK_ROI_FAIXA(ROI_FAIXA).setTo(255);
	Point CENTRO_ROI_FAIXA = Point(ROI_FAIXA.x + ROI_FAIXA.width / 2, ROI_FAIXA.y + ROI_FAIXA.height / 2);

	// PASSO 4
	// PASSO 5

	// PASSO 6
	Mat1b MAPA_ERODE_MASKED = MAPA_ERODE & MASK_ROI_FAIXA;
	vector<HoughLine> houghs = getHoughs(MAPA_ERODE_MASKED);
	// cout << "n houghs: " << houghs.size() << endl;
	if (houghs.size() > 0) {

		// PASSO 7
		Mat1d histograma2D = montarHistograma2D(houghs, MAPA_ERODE, false);
		int ANGULO_DOMINANTE = getAnguloDominante(histograma2D, 9);
		// cout << "AnguloDominante: " << ANGULO_DOMINANTE << endl;
		if (ANGULO_DOMINANTE != -1) {

			// PASSO 8
			Mat H_ROTACAO_HOUGH = getRotationMatrix2D(CENTRO_ROI_FAIXA, ANGULO_DOMINANTE - 90, 1.0);
			Mat1b MAPA_CLOSE_ROTACIONADO;
			warpAffine(Helper::morphDilate(MAPA_ERODE_MASKED, 6), MAPA_CLOSE_ROTACIONADO, H_ROTACAO_HOUGH, MAPA_CLOSE.size(), INTER_NEAREST);
			Mat1b REGIAO_FINAL = MAPA_CLOSE_ROTACIONADO(ROI_FAIXA).clone();

			// PASSO 9
			Mat1b REGIAO_FINAL_UCHAR;  Mat1d REGIAO_FINAL_OR;
			cv::reduce(REGIAO_FINAL, REGIAO_FINAL_UCHAR, 0, CV_REDUCE_MAX);
			REGIAO_FINAL_UCHAR.convertTo(REGIAO_FINAL_OR, CV_64FC1, 1 / 255.0);
			REGIAO_FINAL_OR.setTo(-1, REGIAO_FINAL_OR == 0); // normaliza��o

			Mat1d KERNEL = ~REGIAO_FINAL_UCHAR;
			KERNEL.setTo(-1, KERNEL == 0); // normaliza��o
			KERNEL.setTo(1, KERNEL == 255); // normaliza��o
			
			// PASSO 10
			Mat1d RESULTADO_CONVOLUCAO;
			filter2D(REGIAO_FINAL_OR, RESULTADO_CONVOLUCAO, -1, KERNEL);
			Mat1b RESULTADO_DIR, RESULTADO_ESQ;
			RESULTADO_ESQ = RESULTADO_CONVOLUCAO(Rect(0, 0, RESULTADO_CONVOLUCAO.cols / 2, RESULTADO_CONVOLUCAO.rows));
			RESULTADO_DIR = RESULTADO_CONVOLUCAO(Rect(RESULTADO_CONVOLUCAO.cols / 2, 0, RESULTADO_CONVOLUCAO.cols / 2, RESULTADO_CONVOLUCAO.rows));

			double esqMax, esqMin, dirMax, dirMin;
			minMaxIdx(RESULTADO_ESQ, &esqMin, &esqMax);
			minMaxIdx(RESULTADO_DIR, &dirMin, &dirMax);

			double SOMA_MAX = esqMax + dirMax;

			// PASSO 11
			if (SOMA_MAX > 0) {

				// cout << "Faixa de Pedestre!" << endl;
				// imshow("MAPA_ERODE_MASKED", Helper::morphDilate(MAPA_ERODE_MASKED, 12));

				// PASSO 12
				return Helper::morphDilate(MAPA_ERODE_MASKED, 12);

				// waitKey();
			}
		}
	}
	return Mat();
}

vector<HoughLine> SinalizacaoHorizontal::getHoughs(const Mat1b &mapa) {
	vector<vector<Point> > lineSegments;
	vector<Vec4i> lines;

	const int houghThreshold = 1;
	const int houghMinLineLength = 10;
	const int houghMaxLineGap = 3;

	cv::HoughLinesP(mapa, lines, 1, CV_PI / 180, houghThreshold, houghMinLineLength, houghMaxLineGap);
	// cout << "n houghs: " << lines.size() << endl;

	// guarda as houghs (line segments)
	for (size_t i = 0; i<lines.size(); i++)	{
		Point pt1, pt2;
		pt1.x = lines[i][0];
		pt1.y = lines[i][1];
		pt2.x = lines[i][2];
		pt2.y = lines[i][3];

		// Store into vector of pairs of Points for msac
		lineSegments.push_back({ pt1, pt2 });
	}

	// transforma os line segments nas houghs
	vector<HoughLine> houghs = HoughLine::getVetorDeHoughs({ lineSegments }, mapa.rows, 0);

	return houghs;
}
