#include "FiltroDeParticulasHough.h"

using namespace std;
using namespace cv;


// =================================
// PARTICLE - HOUGH
// =================================
ParticleHough::ParticleHough(double posicaoCarroIPM, double larguraInicial) {
	weight = 0;
	points.assign(2, posicaoCarroIPM);
	laneWidth = larguraInicial;
}

ParticleHough::ParticleHough(const ParticleHough &p) {
	weight = p.weight;
	points = p.points;
	laneWidth = p.laneWidth;
}

ParticleHough::~ParticleHough() { }

bool ParticleHough::operator<(const ParticleHough &p) const { return (weight > p.weight); }

ParticleHough ParticleHough::getRandom(double xBase, double xMean, double xStd, double larguraBase) {
	ParticleHough randomParticle(xBase, larguraBase);
	randomParticle.points[0] = (xBase + randomGaussian(xMean, xStd));
	randomParticle.points[1] = (xBase + randomGaussian(xMean, xStd));
	randomParticle.laneWidth = larguraBase;
	return randomParticle;
}

vector<Point2d> ParticleHough::getSplinePoints(const HoughDoMeio &hough, int roiHeight, const int laneSide, const double factor) const {
	
	// define o shift, com base no lado desejado
	double shift;
	switch (laneSide) {
		case LANE_LEFT: shift = -1; break;
		case LANE_RIGHT: shift = 1; break;
		case LANE_CENTER: shift = 0; break;
	}

	// calcula os pontos da part�cula com a hough atual
	// ponto inferior => determinado pela hough
	Point2d inferior = Point2d(hough.xBase, roiHeight);
	inferior.x += factor * shift * (hough.largura / 2.0);
	// ponto medio => largura definida por interpola��o do superior e inferior
	Point2d medio = Point2d(points[0], (double)roiHeight / 2.0);
	medio.x += factor * shift * ((hough.largura + laneWidth) / 2.0) / 2.0;
	// ponto superior => determinado pela part�cula
	Point2d superior = Point2d(points[1], 0);
	superior.x += factor * shift * laneWidth / 2.0;
	vector<Point2d> pontosParticula = { superior, medio, inferior };

	// pega todos os pontos que pertencem spline representada pela part�cula
	vector<Point2d> pontosSpline;
	if (pontosParticula.size() >= 2) {
		std::vector<double> x, y;
		for (Point2d p : pontosParticula) {
			y.push_back(p.x);
			x.push_back(p.y);
		}
		tk::spline s;
		s.set_points(x, y);
		for (double i = 0; i < roiHeight; i++) {
			pontosSpline.push_back(Point2d(s(i), i));
		}
	}

	return pontosSpline;
}

tk::spline ParticleHough::getSplineFunc(const HoughDoMeio &hough, int roiHeight, const int laneSide) {

	// define o shift, com base no lado desejado
	double shift;
	switch (laneSide) {
	case LANE_LEFT: shift = -1; break;
	case LANE_RIGHT: shift = 1; break;
	case LANE_CENTER: shift = 0; break;
	}

	// calcula os pontos da part�cula com a hough atual
	// ponto inferior => determinado pela hough
	Point2d inferior = Point2d(hough.xBase, roiHeight);
	inferior.x += shift * hough.largura / 2.0;
	// ponto medio => largura definida por interpola��o do superior e inferior
	Point2d medio = Point2d(points[0], (double)roiHeight / 2.0);
	medio.x += shift * ((hough.largura + laneWidth) / 2.0) / 2.0;
	// ponto superior => determinado pela part�cula
	Point2d superior = Point2d(points[1], 0);
	superior.x += shift * laneWidth / 2.0;
	vector<Point2d> pontosParticula = { superior, medio, inferior };

	// pega todos os pontos que pertencem spline representada pela part�cula
	vector<Point2d> pontosSpline;
	if (pontosParticula.size() >= 2) {
		std::vector<double> x, y;
		for (Point2d p : pontosParticula) {
			y.push_back(p.x);
			x.push_back(p.y);
		}
		tk::spline s;
		s.set_points(x, y);
		return s;
	}

	return tk::spline();
}

// TODO: remove splineFunc calculation from within this function and pre-compute it
Point2d ParticleHough::getPoint(int _y, const HoughDoMeio &hough, ConfigXML * _config, int laneSide, bool onPerspective) {
	if (onPerspective) {
		double roi_y = _y - _config->roi.y;
		double ipm_y = _config->ipm->applyHomography(Point2d(_config->roi.x, roi_y)).y; // these calls are not good either, since they can be pre-computed at the beginning

		tk::spline fSpline = this->getSplineFunc(hough, _config->roi.height, laneSide); // not good for performance

		double ipm_x = fSpline(ipm_y);
		Point2d perspective_p = _config->ipm->applyHomographyInv(Point2d(ipm_x, ipm_y));
		perspective_p.y += _config->roi.y;

		return perspective_p;

	} else {
		// this implementation was not intended to have this part, 
		// but it was made in order to show that it is a possibility
		return Point2d();
	}
}

// =================================
// PARTICLE FILTER
// =================================
ParticleFilterHough::ParticleFilterHough(int n, double posicaoCarroIPM, double larguraInicial, int alturaConfiavelInicial, bool _verbose, bool _display) {
	verbose = _verbose;
	display = _display;
	particles.clear();
	for (int i = 0; i < n; i++) particles.push_back(ParticleHough());
	setRandom(n, posicaoCarroIPM, larguraInicial); // inicializa com part�culas aleat�rias
	alturaConfiavel = alturaConfiavelInicial;
}

void ParticleFilterHough::setRandom(int n, double posicaoCarroIPM, double larguraInicial) {
	for (unsigned int i = 0; i < particles.size(); i++)	{
		particles[i] = ParticleHough::getRandom(posicaoCarroIPM, 0, desvioPadraoPoints, larguraInicial);
	}
}

ParticleHough ParticleFilterHough::executar(const HoughDoMeio &hough, const Mat &mapaProbabilistico) {
	
	double tempoInicio = static_cast<double>(getTickCount()); // inicializa o contador de tempo

	predict(hough);
	correct(calculaErrosConfianca(hough, mapaProbabilistico, this->alturaConfiavel));

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	double tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- filtro de particulas com hough: " << tempoExecutando << " ms" << endl;

	return getBestParticle();
}

void ParticleFilterHough::predict(const HoughDoMeio &hough) {
	if (disabled) return;
	ParticleHough lastParticle, predictedParticle;

	// predi��o considerando a dire��o da hough
	for (unsigned int i = 0; i < particles.size(); i++) {
		lastParticle = particles[i]; // guarda a �ltima part�cula

		// distancia da part�cula para a hough
		double hough_xMeio = (hough.xBase + hough.xTopo) / 2.0;
		double d = abs(hough_xMeio - lastParticle.points[0]);
		// pega um ponto aleat�rio entre a hough e a part�cula, seguindo uma distribui��o normal
		double r = randomGaussian(0, d / 3);
		// pega o 'vetor' desse ponto para a hough
		double v = d - r;
		// calcula a media de onde o novo ponto ser� selecionado aleatoriamente
		int fator = ((hough_xMeio - lastParticle.points[0]) >= 0) ? 1 : -1;
		double xReferenciaMeio = lastParticle.points[0] + fator * v;
		// seleciona um n�mero aleat�rio em torno do x de refer�ncia com desvio padr�o fixo
		double novo_xMeio = randomGaussian(xReferenciaMeio, desvioPadraoPoints);
		// atualiza o ponto do meio
		predictedParticle.points[0] = novo_xMeio;

		// calcula o ponto de cima com a mesma diferen�a da base para o meio, por�m com desvioPadrao (maior?!)
		double dx = novo_xMeio - hough_xMeio;
		double xReferenciaTopo = lastParticle.points[1] + dx;
		double novo_xTopo = randomGaussian(xReferenciaTopo, desvioPadraoPoints);
		
		// atualiza o ponto superior
		predictedParticle.points[1] = novo_xTopo;

		//predictedParticle.points[1] = lastParticle.points[1] + randomGaussian(0, desvioPadraoPoints); // atualiza o ponto superior
		predictedParticle.laneWidth = lastParticle.laneWidth + randomGaussian(0, desvioPadraoLargura); // atualiza a largura da pista superior
		
		particles[i] = predictedParticle;
	}

	// predi��o sem considerar a dire��o da hough
	/*
	for (unsigned int i = 0; i < particles.size(); i++) {
		lastParticle = particles[i]; // guarda a �ltima part�cula
		predictedParticle.points[0] = lastParticle.points[0] + randomGaussian(0, desvioPadraoPoints); // atualiza o ponto do meio
		predictedParticle.points[1] = lastParticle.points[1] + randomGaussian(0, desvioPadraoPoints); // atualiza o ponto superior
		predictedParticle.laneWidth = lastParticle.laneWidth + randomGaussian(0, desvioPadraoLargura); // atualiza a largura da pista superior
		particles[i] = predictedParticle;
	}
	*/
}

void ParticleFilterHough::correct(vector<double> erros) {
	if (disabled) return;
	
	const double std = 1 / 3.0;
	// define os pesos
	for (unsigned int i = 0; i < particles.size(); i++) {
		// e^-(erro^2/variance)
		particles[i].weight = erros[i]; // = std::exp(-((erros[i] * erros[i]) / ((std*std))));
	}
	
	normalizaPesos(); // normalize
	resample(); // resample
}

void ParticleFilterHough::reset(const HoughDoMeio &hough, bool parcial) {

	// atualiza todas as part�culas seguindo a hough
	ParticleHough predictedParticle;
	for (unsigned int i = 0; i < particles.size(); i++) {
		if (parcial) {
			// ponto do meio
			predictedParticle.points[0] = (hough.xBase + hough.xTopo) / 2.0;
			predictedParticle.points[0] += randomGaussian(0, desvioPadraoPoints);
		}
		// ponto do topo
		predictedParticle.points[1] = hough.xTopo;
		predictedParticle.points[1] += randomGaussian(0, desvioPadraoPoints);
		// largura da pista
		predictedParticle.laneWidth = hough.largura; // atualiza a largura da pista superior
		particles[i] = predictedParticle;
	}

	if (verbose) {
		cout << " Reset " << ((parcial) ? "parcial" : "total") << " do Filtro de Particulas" << endl;
	}

}

// normaliza, tal que a soma dos pesos = 1
void ParticleFilterHough::normalizaPesos() {
	double sum = 0;
	for (unsigned int i = 0; i < particles.size(); i++) sum += particles[i].weight;
	for (unsigned int i = 0; i < particles.size(); i++) particles[i].weight /= sum;
}

// this method implements the low_variance_sampler presented by Thrun (see Probabilistic Robotics, p. 110)
void ParticleFilterHough::resample() {
	vector<ParticleHough> resampled_particles;

	srand(static_cast<unsigned int>(time(NULL)));

	unsigned int i = 0;
	unsigned int num_particles = static_cast<unsigned int>(particles.size());

	double M_inv = 1.0 / ((double)num_particles);
	double r = M_inv * ((double)rand() / (double)RAND_MAX);
	double c = particles[0].weight;

	for (unsigned int m = 0; m < num_particles; m++)	{
		double U = r + m * M_inv;

		while (U > c) {
			i += 1;
			c += particles[i].weight;
		}

		if (i > particles.size()) i = rand() % particles.size();

		resampled_particles.push_back(ParticleHough(particles[i]));
	}

	particles = resampled_particles;
}

ParticleHough ParticleFilterHough::getBestParticle() {
	ParticleHough virtualBest;
	double sumWeights = 0; // deveria ser = 1, ap�s normaliza��o

	// acumula
	for (unsigned int i = 0; i < particles.size(); i++) {
		virtualBest.points[0] += particles[i].points[0] * particles[i].weight;
		virtualBest.points[1] += particles[i].points[1] * particles[i].weight;
		virtualBest.laneWidth += particles[i].laneWidth * particles[i].weight;
		sumWeights += particles[i].weight;
	}

	// tira m�dia
	virtualBest.points[0] /= sumWeights;
	virtualBest.points[1] /= sumWeights;
	virtualBest.laneWidth /= sumWeights;

	return virtualBest;
}
vector<ParticleHough> ParticleFilterHough::getBestParticles(int number) {
	if (number >= 0) {
		std::sort(particles.begin(), particles.end());
		vector<ParticleHough> bestParticles(particles.begin(), particles.begin() + number);
		return bestParticles;
	} else {
		return vector<ParticleHough>();
	}
}
void ParticleFilterHough::getParticulasMaiorMenorPeso(ParticleHough &maiorPeso, ParticleHough &menorPeso) {
	// ordena as part�culas pelo peso
	std::sort(particles.begin(), particles.end());

	maiorPeso = particles[0];
	menorPeso = particles[particles.size() - 1];
}
vector<ParticleHough> ParticleFilterHough::getParticles() { return particles; }

vector<double> ParticleFilterHough::calculaErros(const HoughDoMeio &hough, const Mat &mapaProbabilistico, const double confidenceHeight) {
	vector<double> output;
	const int altura = mapaProbabilistico.rows;
	const int largura = mapaProbabilistico.cols;

	// para cada part�cula
	for (unsigned int i = 0; i < particles.size(); i++) {

		// get the points that belongs to the spline represented by the particle for each side
		vector<Point2d> esqSpline = particles[i].getSplinePoints(hough, altura, LANE_LEFT);
		vector<Point2d> dirSpline = particles[i].getSplinePoints(hough, altura, LANE_RIGHT);

		double errosPorLado[2];
		double erroFinal;

		for (int lado : {LANE_LEFT, LANE_RIGHT}) {

			// pega os pontos da spline correta, baseado no lado que est� sendo avaliado
			vector<Point2d> pontosSpline = (lado == LANE_LEFT) ? esqSpline : dirSpline;

			// inicializa os contadores
			double total = 0;

			// percorre a spline at� a altura confi�vel
			for (unsigned int p = 0; p < confidenceHeight; p++) {
				int p_y = (int)pontosSpline[p].y;
				int p_x = (int)pontosSpline[p].x;
				// adiciona o valor do pixel, se estiver dentro do mapa, no total
				if (p_x >= 0 && p_x < largura && p_y >= 0 && p_y < altura) {
					total += mapaProbabilistico.at<double>(p_y, p_x);
				}
			}
			int j = (lado == LANE_LEFT) ? 0 : 1;
			errosPorLado[j] = total / pontosSpline.size();
		}

		// tira a m�dia do erro de cada lado
		double x = errosPorLado[LANE_LEFT];
		double y = errosPorLado[LANE_RIGHT];
		erroFinal = x*y + (1 - x*y)*((x + y) / 2);
		output.push_back(erroFinal);
	}
	return output;
}

vector<double> ParticleFilterHough::calculaErrosConfianca(const HoughDoMeio &hough, const Mat &mapaProbabilistico, const double confidenceHeight) {
	vector<double> output;
	const int altura = mapaProbabilistico.rows;
	const int largura = mapaProbabilistico.cols;
	Mat mapa = mapaProbabilistico.clone();
	Rect areaImagem = Rect(Point(), mapaProbabilistico.size());

	cv::normalize(mapaProbabilistico, mapaProbabilistico, 0.0, 1.0, NORM_MINMAX);

	// para cada part�cula
	for (unsigned int i = 0; i < particles.size(); i++) {

		// pega os pontos que pertence a spline representada pela part�cula de cada lado
		vector<Point2d> esqSpline = particles[i].getSplinePoints(hough, altura, LANE_LEFT);
		vector<Point2d> dirSpline = particles[i].getSplinePoints(hough, altura, LANE_RIGHT);

		double errosPorLado[2];
		double erroFinal;

		for (int lado : {LANE_LEFT, LANE_RIGHT}) {

			// pega os pontos da spline correta, baseado no lado que est� sendo avaliado
			vector<Point2d> pontosSpline = (lado == LANE_LEFT) ? esqSpline : dirSpline;

			// inicializa os contadores
			double total = 0;

			// percorre a spline at� a altura confi�vel
			for (int p = mapaProbabilistico.rows - 1; p > confidenceHeight; p--) {
				int p_y = (int)pontosSpline[p].y;
				int p_x = (int)pontosSpline[p].x;
				// adiciona o valor do pixel, se estiver dentro do mapa, no total
				if (p_x >= 0 && p_x < largura && p_y >= 0 && p_y < altura) {
					total += mapaProbabilistico.at<double>(p_y, p_x);
				}
			}
			int j = (lado == LANE_LEFT) ? 0 : 1;
			int n = mapaProbabilistico.rows - (int)confidenceHeight;

			errosPorLado[j] = total / n;
		}

		// tira a m�dia do erro de cada lado
		double x = errosPorLado[LANE_LEFT];
		double y = errosPorLado[LANE_RIGHT];
		erroFinal = x*y + (1 - x*y)*((x + y) / 2);

		// guarda os pixels entre as splines
		double larguraMaximaD = (particles[i].laneWidth > hough.largura) ? particles[i].laneWidth : hough.largura;
		double larguraMinimaD = (particles[i].laneWidth > hough.largura) ? hough.largura : particles[i].laneWidth;
		int larguraMaxima = (int)larguraMaximaD;
		int larguraMinima = (int)larguraMinimaD;

		// monta a imagem
		int total = 0;
		const int tamanhoBusca = 15;
		const int shift = 0;
		double razaoVariacaoLargura = ((hough.largura - particles[i].laneWidth) / mapaProbabilistico.rows);
		for (int p = 0; p < mapaProbabilistico.rows; p++) {
			// esquerda
			for (int i = 0 + shift; i < tamanhoBusca + shift; i++) {
				Point paraAvaliar = Point((int)esqSpline[p].x + i, (int)esqSpline[p].y);
				if (areaImagem.contains(paraAvaliar)) {
					if (dirSpline[p].y > alturaConfiavel) total += (int)mapaProbabilistico.at<double>(paraAvaliar);
				}
			}
			// direita
			for (int i = 0 + shift; i < tamanhoBusca + shift; i++) {
				Point paraAvaliar = Point((int)dirSpline[p].x - i, (int)dirSpline[p].y);
				if (areaImagem.contains(paraAvaliar)) {
					if (dirSpline[p].y > alturaConfiavel) total += (int)mapaProbabilistico.at<double>(paraAvaliar);
				}
			}
		}
		double percentualEvidencias = (double)total / (double)(2 * tamanhoBusca * (mapaProbabilistico.rows - alturaConfiavel));
		
		const double std1 = 1 / 3.0;
		const double std2 = 1 / 12.0;
		double erroAnterior = 1 - erroFinal;
		erroAnterior = std::exp(-((erroAnterior*erroAnterior) / ((std1*std1))));
		percentualEvidencias = std::exp(-((percentualEvidencias*percentualEvidencias) / ((std2*std2))));
		
		erroFinal = erroAnterior * percentualEvidencias;
		
		output.push_back(erroFinal);
	}

	return output;
}

int ParticleFilterHough::atualizaAlturaConfiavel(ParticleHough *ultimaVirtualBest, const HoughDoMeio &hough, const Mat1b &mapaBinario, const vector<SinalizacaoHorizontal::Blob> &roadSigns) {
	
	// histograma para visualizar o n�mero de evid�ncias ao longo
	Mat1d histograma = Mat1d(Size(mapaBinario.rows, 1), double(0));
	
	int _alturaConfiavel = -1;
	if (ultimaVirtualBest == NULL) return _alturaConfiavel; // n�o atualiza
	
	const int altura = mapaBinario.rows;
	const int largura = mapaBinario.cols;
	Rect areaImagem = Rect(Point(), mapaBinario.size());

	const int linhasParaChecar = 3;
	const int step = 1; // ir de 'step em 'step'
	const int primeiro = 1; // come�ar no 'primeiro'
	const int dist = (int)(hough.largura * 0.5); // distancia a ser percorrida (50% da largura) na perpedicular
	const int thres = (int)(hough.largura * 0.3) * (int)(linhasParaChecar * 0.5); // threshold se � obst�culo ou n�o
	
	// pega a spline do meio
	vector<Point2d> meioSpline = (*ultimaVirtualBest).getSplinePoints(hough, altura, LANE_CENTER);
	// calcula a vetor de orienta��o m�dio
	for (unsigned int p = (unsigned int)(meioSpline.size() - step - 1); p > linhasParaChecar; p -= step) {
		// pega os pontos
		Point2d p_antes = meioSpline[p - step];
		Point2d p_atual = meioSpline[p];
		Point2d p_proxi = meioSpline[p + step];
		Point p_atual_2i = p_atual;

		// calcula orienta��o entre o atual->antes e atual->proxi
		Point2d v_antes = p_atual - p_antes;
		v_antes *= 1.0 / cv::norm(v_antes);
		Point2d v_proxi = p_proxi - p_atual;
		v_proxi *= 1.0 / cv::norm(v_proxi);

		// calcula a orienta��o m�dia
		Point2d orientacao = (v_antes + v_proxi) * 0.5;

		// pega o perpendicular
		Point2d perpendicular = Point2d(orientacao.y, -orientacao.x);

		// percorre 'dist' na dire��o perpendicular
		double total = 0;
		for (int lado : {LANE_LEFT, LANE_RIGHT}) {
			int fator = (lado == LANE_LEFT) ? -1 : 1;
			Scalar cor = (lado == LANE_LEFT) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
			for (int i = 0; i < dist / 2; i++) {
				Point paraAvaliarBase = p_atual + fator * (i + 1) * perpendicular;
				for (int linhaAcima = 0; linhaAcima < linhasParaChecar; linhaAcima++) {
					Point orientacao_2i = orientacao;
					Point paraAvaliar = paraAvaliarBase + linhaAcima * orientacao_2i;
					if (paraAvaliar == p_atual_2i) continue; // caso o avan�o n�o seja suficiente para chegar em outro pixel
					if (!areaImagem.contains(paraAvaliar)) continue;
					if ((int)mapaBinario.at<uchar>(paraAvaliar) != 0) total++;
				}
			}
		}

		// se for o n�mero de evid�ncia for maior que o thres, a altura foi encontrada
		if (total > thres && _alturaConfiavel == -1) {
			_alturaConfiavel = p;
			// se estiver na altura de um road sign j� detectado, descartar e procurar um pr�ximo
			for (auto r : roadSigns) if (!(r.templateId == ROAD_SIGN::BARRA_CONTENCAO) && (int)(p - r.regiao.y) < r.regiao.height && (int)(p - r.regiao.y) > 0) _alturaConfiavel = -1;
		}
		if (display) histograma.at<double>(p) = total;
	}

	if (display) {
		Mat3b imgHistograma = Mat3b(Size(mapaBinario.rows, 100), Vec3b(0, 0, 0));
		for (int i = 1; i < imgHistograma.cols; i++) {
			line(imgHistograma, Point(i - 1, (int)(imgHistograma.rows - histograma.at<double>(i - 1))), Point(i, (int)(imgHistograma.rows - histograma.at<double>(i))), Scalar(255, 255, 255));
		}
		line(imgHistograma, Point(0, imgHistograma.rows - thres), Point(imgHistograma.cols, imgHistograma.rows - thres), Scalar(0, 0, 255));
		line(imgHistograma, Point(_alturaConfiavel, 0), Point(_alturaConfiavel, imgHistograma.rows), Scalar(0, 0, 255));
		// imshow("Histograma Altura Confiavel", imgHistograma);
	}
	
	_alturaConfiavel = (_alturaConfiavel == -1) ? 0 : _alturaConfiavel;
	return _alturaConfiavel;
}

int ParticleFilterHough::atualizaAlturaConfiavel(const HoughDoMeio &hough, const Mat &mapa) {

	// histograma para visualizar o n�mero de evid�ncias ao longo
	Mat1d histograma = Mat1d(Size(mapa.rows, 1), double(0));
	Mat3b img = Mat3b(mapa.size(), Vec3b(0,0,0));

	int _alturaConfiavel = mapa.rows;
	const int altura = mapa.rows;
	const int largura = mapa.cols;
	Rect areaImagem = Rect(0, 0, mapa.cols, mapa.rows);

	const int step = 1; // ir de 'step em 'step'
	const int primeiro = 1; // come�ar no 'primeiro'
	const int dist = (int)(hough.largura * 0.5); // distancia a ser percorrida (50% da largura) na perpedicular
	const int thres = (int)(hough.largura * 0.3);; // threshold se � obst�culo ou n�o

	// pega o vetor dire��o da hough com 1 em y
	Point pBase = Point((int)hough.xBase, altura);
	Point pMeio = Point((int)(hough.xBase + hough.xTopo) / 2, altura / 2);
	Point pTopo = Point((int)hough.xTopo, 0);
	
	// calcula orienta��o entre o atual->antes e atual->proxi
	Point2d v_antes = pMeio - pBase;
	v_antes *= 1.0 / cv::norm(v_antes);
	Point2d v_proxi = pTopo - pMeio;
	v_proxi *= 1.0 / cv::norm(v_proxi);

	// calcula a orienta��o m�dia
	Point2d orientacao = (v_antes + v_proxi) * 0.5;

	// pega o perpendicular
	Point2d perpendicular = Point2d(orientacao.y, -orientacao.x);
	
	// calcula a vetor de orienta��o m�dio
	for (unsigned int p = (unsigned int)(altura - step - 1); p > primeiro; p -= step) {
		int dy = altura - p;
		Point2d p_atual = Point2d(pBase.x + (orientacao * dy).x, p);
		Point p_atual_2i = p_atual;
		// percorre 'dist' na dire��o perpendicular
		double total = 0;
		for (int lado : {LANE_LEFT, LANE_RIGHT}) {
			int fator = (lado == LANE_LEFT) ? -1 : 1;
			Scalar cor = (lado == LANE_LEFT) ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
			for (int i = 0; i < dist / 2; i++) {
				Point paraAvaliar = p_atual + fator * (i + 1) * perpendicular;
				if (paraAvaliar == p_atual_2i) continue; // caso o avan�o n�o seja suficiente para chegar em outro pixel
				if (!areaImagem.contains(paraAvaliar)) continue; // caso o pixel esteja fora da imagem
				if (mapa.at<double>(paraAvaliar) != 0) total++;
			}
		}
		// se for o n�mero de evid�ncia for maior que o thres, a altura foi encontrada
		if (total > thres) break;
		_alturaConfiavel = p;
		if (display) histograma.at<double>(p) = total;
	}

	if (display) {
		Mat3b imgHistograma = Mat3b(Size(mapa.rows, dist), Vec3b(0, 0, 0));
		for (int i = 1; i < imgHistograma.cols; i++) {
			line(imgHistograma, Point(i - 1, (int)(imgHistograma.rows - histograma.at<double>(i - 1))), Point(i, (int)(imgHistograma.rows - histograma.at<double>(i))), Scalar(255, 255, 255));
		}
		line(imgHistograma, Point(0, imgHistograma.rows - thres), Point(imgHistograma.cols, imgHistograma.rows - thres), Scalar(0, 0, 255));
		line(imgHistograma, Point(_alturaConfiavel, 0), Point(_alturaConfiavel, imgHistograma.rows), Scalar(0, 0, 255));
		imshow("hist", imgHistograma);
	}

	return _alturaConfiavel;
}

void ParticleFilterHough::view(const HoughDoMeio *hough, const Mat &imagemPerspectiva, Mat &displayFrameRoiIPM, ConfigXML *_config, bool onlyPoints, int alturaConfiavel, const vector<Scalar> &cores) {

	Mat imagemComposta = imagemPerspectiva.clone();
	ParticleHough maior, menor;
	alturaConfiavel = (alturaConfiavel == -1) ? this->alturaConfiavel : alturaConfiavel;

	if (hough != NULL) {
		// ============================================
		// Imagem em IPM
		// ============================================
		getParticulasMaiorMenorPeso(maior, menor);

		mostrarParticulas({ menor }, *hough, this->alturaConfiavel, displayFrameRoiIPM, Scalar(0, 0, 255), true);
		mostrarParticulas({ maior }, *hough, this->alturaConfiavel, displayFrameRoiIPM, Scalar(0, 255, 0), false);
		mostrarParticulas({ getBestParticle() }, *hough, this->alturaConfiavel, displayFrameRoiIPM, Scalar(255, 0, 0), false);

		// ============================================
		// Imagem em PERSPECTIVA
		// ============================================

		//vector<ParticleHough> particulas = { getBestParticle(), maior, menor };
		vector<ParticleHough> particulas = { getBestParticle() };

		for (unsigned int n = 0; n < particulas.size(); n++) {

			vector<Point2d> splineLeftIPM = particulas[n].getSplinePoints(*hough, _config->roi.height, LANE_LEFT);
			vector<Point2d> splineRightIPM = particulas[n].getSplinePoints(*hough, _config->roi.height, LANE_RIGHT);

			vector<Point2d> splineLeft = _config->ipm->applyHomographyInv(splineLeftIPM);
			vector<Point2d> splineRight = _config->ipm->applyHomographyInv(splineRightIPM);

			if (!onlyPoints) {

				for (int i = (int)splineLeft.size() - 2; i > alturaConfiavel; i--) {
					Point p1 = Point((int)splineLeft[i].x, (int)splineLeft[i].y + _config->roi.y);
					Point p2 = Point((int)splineLeft[i + 1].x, (int)splineLeft[i + 1].y + _config->roi.y);
					line(imagemComposta, p1, p2, cores[n], 1);
				}

				for (int i = (int)splineRight.size() - 2; i > alturaConfiavel; i--) {
					Point p1 = Point((int)splineRight[i].x, (int)splineRight[i].y + _config->roi.y);
					Point p2 = Point((int)splineRight[i + 1].x, (int)splineRight[i + 1].y + _config->roi.y);
					line(imagemComposta, p1, p2, cores[n], 1);
				}

				// linhas de uma lane para outra
				for (int i = (int)splineLeft.size() - 2; i > alturaConfiavel; i -= 10) {
					Point p1 = Point((int)splineLeft[i].x, (int)splineLeft[i].y + _config->roi.y);
					Point p2 = Point((int)splineRight[i + 1].x, (int)splineRight[i + 1].y + _config->roi.y);
					line(imagemComposta, p1, p2, cores[n], 1);
				}

			} else {

				for (int i = (int)splineLeft.size() - 1; i > alturaConfiavel; i--) {
					Point p = Point((int)splineLeft[i].x, (int)splineLeft[i].y + _config->roi.y);
					circle(imagemComposta, p, 0, cores[n], -1);
				}

				for (int i = (int)splineRight.size() - 1; i > alturaConfiavel; i--) {
					Point p = Point((int)splineRight[i].x, (int)splineRight[i].y + _config->roi.y);
					circle(imagemComposta, p, 0, cores[n], -1);
				}
			}
		}
	}

	Rect areaIPM = Rect(0, 0, displayFrameRoiIPM.cols, displayFrameRoiIPM.rows);
	displayFrameRoiIPM.copyTo(imagemComposta(areaIPM));

	if (display) {
		imshow("Filtro de Part�culas Hough", imagemComposta);
		//moveWindow("Filtro de Part�culas Hough", 10, 10);
	}
	if (verbose && hough != NULL) {
		cout << "  - maior peso: " << maior.weight << endl;
		cout << "  - menor peso: " << menor.weight << endl;
	}
}

// =================================
// OUTROS
// =================================
double randomGaussian(double mean, double std) {
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm; // can't let u == 0
	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * CV_PI * v);
	return mean + std * z;
}
double randomUniform(double min, double max) {
	return min + (rand() / (double)RAND_MAX) * (max - min);
}

void mostrarParticulas(const vector<ParticleHough> &particulas, const HoughDoMeio &hough, int alturaConfiavel, Mat &imagem, const Scalar cor, const bool intercalar) {
	for (ParticleHough p : particulas) {
		vector<Point2d> spline_points_left = p.getSplinePoints(hough, imagem.rows, LANE_LEFT);
		vector<Point2d> spline_points_right = p.getSplinePoints(hough, imagem.rows, LANE_RIGHT);
		for (int i = (int)spline_points_left.size() - 1; i > alturaConfiavel; i--) {
			if (intercalar && (i % 10) < 5) {
				circle(imagem, spline_points_left[i], 0, cor, -1);
				circle(imagem, spline_points_right[i], 0, cor, -1);
			} else if (!intercalar && (i % 10) >= 5) {
				circle(imagem, spline_points_left[i], 0, cor, -1);
				circle(imagem, spline_points_right[i], 0, cor, -1);
			}
		}
	}
}
