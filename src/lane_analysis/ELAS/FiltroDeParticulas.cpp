#include "FiltroDeParticulas.h"

using namespace std;
using namespace cv;

double carmen_gaussian_random(double mean, double std) {
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm;                  /**//* can't let u == 0 */
	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * CV_PI * v);
	return mean + std * z;
}

double carmen_uniform_random(double min, double max) {
	return min + (rand() / (double)RAND_MAX) * (max - min);
}

// =================================
// PARTICLE :: IMPLEMENTATION
// =================================

Particle::Particle(double centroImagem, double larguraPistaInicial) {
	weight = 0;
	spline_points.assign(3, centroImagem);
	lane_width.assign(2, larguraPistaInicial);
}

Particle::Particle(const Particle &p) {
	weight = p.weight;
	spline_points = p.spline_points;
	lane_width = p.lane_width;
}

Particle::~Particle() { }

bool Particle::operator<(const Particle &p) const { return (weight > p.weight); }

// Gera uma part�cula aleat�ria
Particle Particle::geraParticulaAleatoria(double mean, double std, double base_x, double initial_lane_width) {
	Particle random_particle;

	for (unsigned int i = 0; i < random_particle.spline_points.size(); i++){
		random_particle.spline_points[i] = (base_x + carmen_gaussian_random(mean, std));
	}

	for (unsigned int i = 0; i < random_particle.lane_width.size(); i++) {
		random_particle.lane_width[i] = initial_lane_width;
	}

	return random_particle;
}

// =================================
// PARTICLE FILTER :: IMPLEMENTATION
// =================================

ParticleFilter::ParticleFilter() { }
ParticleFilter::ParticleFilter(int num_particles, double centroImagem, double larguraPistaInicial, bool _verbose, bool _display) {
	posicaoCarro = centroImagem;
	larguraPistaPadrao = larguraPistaInicial;
	verbose = _verbose;
	display = _display;
	
	for (int i = 0; i < num_particles; i++) {
		particles.push_back(Particle());
	}

	geraParticulasAleatorias();
}
ParticleFilter::~ParticleFilter() { }

// gera um conjunto de part�culas aleat�rias
void ParticleFilter::geraParticulasAleatorias() {
	for (unsigned int i = 0; i < particles.size(); i++)	{
		particles[i] = Particle::geraParticulaAleatoria(0, desvioPadraoSplinePoints, posicaoCarro, larguraPistaPadrao);
	}
}

Particle ParticleFilter::executar(Mat &mapaDeEvidencia, int funcaoErro, int funcaoPeso) {
	// inicializa o contador de tempo
	double tempoInicio = static_cast<double>(getTickCount());

	// predi��o
	this->predict();
	// calcula o erro das particulas
	vector<double> erros = calculaErros(particles, mapaDeEvidencia, getBestParticle(), funcaoErro);
	// corre��o
	this->correct(erros, funcaoPeso);

	// calcula o tempo de execu��o
	double tempoFim = static_cast<double>(getTickCount());
	tempoExecutando = ((tempoFim - tempoInicio) / getTickFrequency()) * 1000;

	// exibe as sa�das definidas (texto e/ou imagem)
	if (verbose) cout << "- filtro de particulas: " << tempoExecutando << " ms" << endl;

	// retorna a melhor part�cula
	return getBestParticle();
}

// predict the next position of the particles by moving them
void ParticleFilter::predict() {
	if (disabled) return;

	Particle last_particle, predicted_particle;

	double MAX_LANE_WIDTH = larguraPistaPadrao * 1.2;
	double MIN_LANE_WIDTH = larguraPistaPadrao * 0.8;

	for (unsigned int i = 0; i < particles.size(); i++) {

		// store last particle
		last_particle = particles[i];

		// atualiza os valores dos pontos da spline
		/*
		for (unsigned int j = 0; j < last_particle.spline_points.size(); j++) {
		predicted_particle.spline_points[j] = last_particle.spline_points[j] + carmen_gaussian_random(0, std_dev_points);
		}
		*/

		// atualiza os lane_width
		for (unsigned int j = 0; j < last_particle.lane_width.size(); j++) {

			double diff = (last_particle.lane_width[j] - larguraPistaPadrao);
			double factor = std::exp(-((diff*diff) / ((larguraPistaPadrao * 0.1) * (larguraPistaPadrao * 0.1))));
			predicted_particle.lane_width[j] = last_particle.lane_width[j] + carmen_gaussian_random(0, factor * desvioPadraoLaneWidth);

			/*
			do {
			predicted_particle.lane_width[j] = last_particle.lane_width[j] + carmen_gaussian_random(0, std_dev_lane_width);
			} while (!(predicted_particle.lane_width[j] > MIN_LANE_WIDTH && predicted_particle.lane_width[j] < MAX_LANE_WIDTH));
			*/
		}

		// TODO: coloca iterativo
		double diff, factor;
		// prende o ponto de baixo dentro de uma gaussiana
		diff = (last_particle.spline_points[2] - posicaoCarro);
		factor = std::exp(-((diff*diff) / (40 * 40)));
		predicted_particle.spline_points[2] = last_particle.spline_points[2] + carmen_gaussian_random(0, factor * desvioPadraoSplinePoints);

		// prende o ponto do meio dentro de uma gaussiana
		diff = (last_particle.spline_points[1] - posicaoCarro);
		factor = std::exp(-((diff*diff) / (50 * 50)));
		predicted_particle.spline_points[1] = last_particle.spline_points[1] + carmen_gaussian_random(0, factor * desvioPadraoSplinePoints);

		// prende o ponto de cima dentro de uma gaussiana
		diff = (last_particle.spline_points[0] - posicaoCarro);
		factor = std::exp(-((diff*diff) / (60 * 60)));
		predicted_particle.spline_points[0] = last_particle.spline_points[0] + carmen_gaussian_random(0, factor * desvioPadraoSplinePoints);

		// update actual particles to their predicted
		particles[i] = predicted_particle;
	}
}

// com base no erro de cada part�cula: atribui um peso, normaliza e faz o resample
void ParticleFilter::correct(vector<double> error_vector, int weight_function) {
	if (disabled) return;

	// calcula os pesos das part�culas
	if (weight_function == WEIGHT_ALBERTO) {
		// calculate the standard deviation: http://stackoverflow.com/a/12405793/4228275
		double sum = std::accumulate(std::begin(error_vector), std::end(error_vector), 0.0);
		double m = sum / error_vector.size();
		double accum = 0.0;
		std::for_each(std::begin(error_vector), std::end(error_vector), [&](const double d) {
			accum += (d - m) * (d - m);
		});
		double stdev = sqrt(accum / (error_vector.size() - 1));

		// assign weights
		for (unsigned int i = 0; i < particles.size(); i++) particles[i].weight = calculate_particle_probability_alberto(error_vector[i], stdev);
	} else if (weight_function == WEIGHT_INVERSE_ERROR) {
		// assign weights
		for (unsigned int i = 0; i < particles.size(); i++) particles[i].weight = calculate_particle_probability(error_vector[i]);
	} else if (weight_function == WEIGHT_ERROR) {
		// assign weights
		for (unsigned int i = 0; i < particles.size(); i++) particles[i].weight = calculate_particle_probability_error(error_vector[i]);
	}

	// normalize
	normalize_particle_weights();

	// resample
	resample();
}

// the particle probability is the weight, that is inversely proportional to its error
double ParticleFilter::calculate_particle_probability(double error) {
	return (1 / (double)(1 + std::exp(2 * error)));
}

// peso igual ao erro
double ParticleFilter::calculate_particle_probability_error(double error) { return error; }

// the particle probability is the weight, that is inversely proportional to its error
double ParticleFilter::calculate_particle_probability_alberto(double error, double std_dev) {
	// e^-(erro^2/variance)
	return std::exp(-((error*error) / (std_dev*std_dev)));
}

// normalize to get sum of weight equal to one (sum_weights = 1)
void ParticleFilter::normalize_particle_weights() {
	double particle_summed_weight = 0;

	for (unsigned int i = 0; i < particles.size(); i++) {
		particle_summed_weight += particles[i].weight;
	}

	for (unsigned int i = 0; i < particles.size(); i++) {
		particles[i].weight /= particle_summed_weight;
	}
}

// this method implements the low_variance_sampler presented by Thrun (see Probabilistic Robotics, p. 110)
void ParticleFilter::resample() {
	vector<Particle> resampled_particles;

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

		if (i > particles.size()) {
			i = rand() % particles.size();
		}

		resampled_particles.push_back(Particle(particles[i]));
	}

	particles = resampled_particles;
}

// Pega a "melhor part�cula" que � a m�dia ponderada de todas as part�culas
Particle ParticleFilter::getBestParticle() {
	Particle virtual_best;
	double sum_particles_weight = 0;

	// accumulate
	for (unsigned int i = 0; i < particles.size(); i++) {
		for (unsigned int j = 0; j < particles[i].spline_points.size(); j++) {
			virtual_best.spline_points[j] += particles[i].spline_points[j] * particles[i].weight;
		}
		for (unsigned int j = 0; j < particles[i].lane_width.size(); j++) {
			virtual_best.lane_width[j] += particles[i].lane_width[j] * particles[i].weight;
		}
		sum_particles_weight += particles[i].weight;
	}

	// average
	for (unsigned int j = 0; j < virtual_best.spline_points.size(); j++) {
		virtual_best.spline_points[j] /= sum_particles_weight;
	}
	for (unsigned int j = 0; j < virtual_best.lane_width.size(); j++) {
		virtual_best.lane_width[j] /= sum_particles_weight;
	}

	return virtual_best;
}

vector<Particle> ParticleFilter::getBestParticles(int number) {
	if (number >= 0) {
		std::sort(particles.begin(), particles.end());
		vector<Particle> best_particles(particles.begin(), particles.begin() + number);
		return best_particles;
	} else {
		return vector<Particle>();
	}
}

vector<Particle> ParticleFilter::getParticles() { return particles; }

void ParticleFilter::getParticulasMaiorMenorPeso(Particle &maiorPeso, Particle &menorPeso) {
	
	// ordena as part�culas pelo peso
	std::sort(particles.begin(), particles.end());

	maiorPeso = particles[0];
	menorPeso = particles[particles.size() - 1];

}

void ParticleFilter::view(const Mat &imagemPerspectiva, Mat &displayFrameRoiIPM, const ConfigXML &_config, bool onlyPoints, const vector<Scalar> &cores) {
	
	// ============================================
	// Imagem em IPM
	// ============================================
	Particle maior, menor;
	getParticulasMaiorMenorPeso(maior, menor);

	mostrarParticulas({ getBestParticle() }, displayFrameRoiIPM, Scalar(255, 0, 0), false);
	mostrarParticulas({ menor }, displayFrameRoiIPM, Scalar(0, 0, 255), true);
	mostrarParticulas({ maior }, displayFrameRoiIPM, Scalar(0, 255, 0), false);

	// ============================================
	// Imagem em PERSPECTIVA
	// ============================================
	Mat imagemComposta = imagemPerspectiva.clone();

	vector<Particle> particulas = { getBestParticle(), maior, menor };

	for (unsigned int n = 0; n < particulas.size(); n++) {

		vector<Point2d> splineLeftIPM = getSplinePoints(particulas[n], _config.roi.height, LANE_LEFT);
		vector<Point2d> splineRightIPM = getSplinePoints(particulas[n], _config.roi.height, LANE_RIGHT);

		vector<Point2d> splineLeft = _config.ipm->applyHomographyInv(splineLeftIPM);
		vector<Point2d> splineRight = _config.ipm->applyHomographyInv(splineRightIPM);

		if (!onlyPoints) {

			for (int i = 0; i < splineLeft.size() - 1; i++) {
				Point p1 = Point((int)splineLeft[i].x, (int)splineLeft[i].y + _config.roi.y);
				Point p2 = Point((int)splineLeft[i + 1].x, (int)splineLeft[i + 1].y + _config.roi.y);
				line(imagemComposta, p1, p2, cores[n], 1);
			}

			for (int i = 0; i < splineRight.size() - 1; i++) {
				Point p1 = Point((int)splineRight[i].x, (int)splineRight[i].y + _config.roi.y);
				Point p2 = Point((int)splineRight[i + 1].x, (int)splineRight[i + 1].y + _config.roi.y);
				line(imagemComposta, p1, p2, cores[n], 1);
			}

		} else {

			for (Point2d p : splineLeft) {
				circle(imagemComposta, Point((int)p.x, (int)p.y + _config.roi.y), 0, cores[n], -1);
			}

			for (Point2d p : splineRight) {
				circle(imagemComposta, Point((int)p.x, (int)p.y + _config.roi.y), 0, cores[n], -1);
			}
		}
	}

	Rect areaIPM = Rect(0, 0, displayFrameRoiIPM.cols, displayFrameRoiIPM.rows);
	displayFrameRoiIPM.copyTo(imagemComposta(areaIPM));

	if (display) imshow("Filtro de Part�culas", imagemComposta);
	if (verbose) {
		cout << "  - maior peso: " << maior.weight << endl;
		cout << "  - menor peso: " << menor.weight << endl;
	}
}

// =================================
// FUN��ES DE C�LCULO DE ERRO
// =================================

vector<double> calculaErros(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula, int flag) {
	switch (flag) {
	case ERRO_POR_REGIOES: return erroPorRegioes(particulas, mapaDeEvidencia, ultimaMelhorParticula); break;
	case ERRO_PONDERADO_PELO_DESVIO_PADRAO: return erroPonderadoPeloDesvioPadrao(particulas, mapaDeEvidencia, ultimaMelhorParticula); break;
	case ERRO_PROXIMO_AO_CARRO_DIFERENCIADO: return erroProximoAoCarroDiferenciado(particulas, mapaDeEvidencia, ultimaMelhorParticula); break;
	case ERRO_TOTAL_EMBAIXO_SPLINE: return erroTotalEmbaixoDaSpline(particulas, mapaDeEvidencia, ultimaMelhorParticula); break;
	default: return erroPorRegioes(particulas, mapaDeEvidencia, ultimaMelhorParticula); break;
	}
}

vector<double> erroPorRegioes(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula) {
	vector<double> output;

	// masks
	Mat left_mask = Mat::zeros(mapaDeEvidencia.size(), mapaDeEvidencia.type());
	Mat right_mask = Mat::zeros(mapaDeEvidencia.size(), mapaDeEvidencia.type());
	rectangle(left_mask, Rect(0, 0, mapaDeEvidencia.cols / 2, mapaDeEvidencia.rows), 255, -1);
	rectangle(right_mask, Rect(mapaDeEvidencia.cols / 2, 0, mapaDeEvidencia.cols, mapaDeEvidencia.rows), 255, -1);

	Mat evidence_map_left, evidence_map_right;

	mapaDeEvidencia.copyTo(evidence_map_right, right_mask);
	mapaDeEvidencia.copyTo(evidence_map_left, left_mask);

	// cost of finding an evidence in the opposite direction of the intended lane side
	double opposite_direction_weight = 2;
	const int NUM_REGIONS = 2;

	vector<Point2d> best_spline_points = getSplinePoints(ultimaMelhorParticula, mapaDeEvidencia.rows, LANE_CENTER);
	double percentual_escolhido = 0.9; // 1/x = % escolhidos

	for (unsigned int ii = 0; ii < particulas.size(); ii++) {

		// get the points that belongs to the spline represented by the particle for each side
		vector<Point2d> spline_points_left = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_LEFT);
		vector<Point2d> spline_points_right = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_RIGHT);

		vector<double> distances_frame_left, distances_frame_right;

		map<int, map<int, vector<double> > > regions; // regions[side][num_region] = distances;

		// para cada lado
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}){
			// escolhe a spline e o evidence_map apropriados
			vector<Point2d> spline_points = (lane_side == LANE_LEFT) ? spline_points_left : spline_points_right;
			Mat side_evidence_map = (lane_side == LANE_LEFT) ? evidence_map_left : evidence_map_right;
			int n_spline = (int)spline_points.size();
			int side_factor = (lane_side == LANE_LEFT) ? -1 : 1;
			map<int, vector<double> > side;
			// para cada regi�o
			for (int r = 1; r <= NUM_REGIONS; r++){
				vector<double> region;
				// para cada ponto da spline
				for (int p = (n_spline / NUM_REGIONS) * (r - 1); p < (n_spline / NUM_REGIONS) * r; p++) {
					double min_distance = numeric_limits<double>::max();
					Point point_selected;
					int point_idx;
					// TODO: interpolar para ver a largura da faixa no ponto p da spline
					int width_in_p = static_cast<int>(particulas[ii].lane_width[2] * 2);
					// percorre a distancia calculada na dire��o do lado da pista em quest�o
					for (int d = 0; d < width_in_p; d++) {
						int p_y = (int)spline_points[p].y;
						//int p_x = (int)(mapaDeEvidencia.cols / 2) + (d * side_factor);
						int p_x = (int)best_spline_points[p].x + (d * side_factor);
						// se o ponto for branco no mapa de evidencia
						if (side_evidence_map.at<uchar>(p_y, p_x) == 255) {
							// calcula a distancia do ponto da spline para o ponto no evidence_map
							double temp_dist = abs(spline_points[p].x - (p_x));
							if (spline_points[p].x - (p_x) < 0 && side_factor == -1) {
								temp_dist *= 2;
							} else if (spline_points[p].x - (p_x) > 0 && side_factor == 1) {
								temp_dist *= 2;
							}
							// se a distancia for menor que � menor encontrada at� o momento, atualize
							if (temp_dist < min_distance) {
								min_distance = temp_dist;
								point_idx = d;
							}
						}
					}
					if (min_distance != numeric_limits<double>::max()) {
						int p_y = (int)spline_points[p].y;
						int p_x = (int)(mapaDeEvidencia.cols / 2) + (point_idx * side_factor);
						point_selected = Point(p_x, p_y);
						region.push_back(min_distance);
					}
				}
				side[r] = region;
			}
			regions[lane_side] = side;
		}

		// agrupa os x% melhores de cada lado
		std::map<int, vector<double> > distances;
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}) {
			vector<double> distances_side;

			// calcula o espa�o para o vetor final
			int vector_size = 0;
			for (int r = 1; r <= NUM_REGIONS; r++) {
				vector_size += (int)regions[lane_side][r].size();
			}
			// pr�-aloca mem�ria
			distances_side.reserve(vector_size);

			// concatena os x% menores de cada regi�o no vetor final do lado em quest�o
			for (int r = 1; r <= NUM_REGIONS; r++) {
				std::sort(regions[lane_side][r].begin(), regions[lane_side][r].end());
				// calcula quantos elementos ser�o mantidos
				int size_subvector = static_cast<int>(floor(regions[lane_side][r].size() * percentual_escolhido));
				distances_side.insert(distances_side.end(), regions[lane_side][r].begin(), regions[lane_side][r].begin() + size_subvector);
			}
			distances[lane_side] = distances_side;
		}

		// calcula a distance m�dia de cada lado
		std::map<int, double> mean_distances;
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}) {
			double mean_distance = numeric_limits<double>::max();
			if (distances[lane_side].size() != 0) {
				mean_distance = std::accumulate(distances[lane_side].begin(), distances[lane_side].end(), 0.0);
			}
			mean_distances[lane_side] = mean_distance;
		}

		// calcula a distancia m�dia final com algumas pondera��es
		double distance_total;
		if (mean_distances[LANE_LEFT] == numeric_limits<double>::max() && mean_distances[LANE_RIGHT] != numeric_limits<double>::max()) {
			distance_total = mean_distances[LANE_RIGHT] / distances[LANE_RIGHT].size();
		} else if (mean_distances[LANE_LEFT] != numeric_limits<double>::max() && mean_distances[LANE_RIGHT] == numeric_limits<double>::max()) {
			distance_total = mean_distances[LANE_LEFT] / distances[LANE_LEFT].size();
		} else {
			distance_total = (mean_distances[LANE_LEFT] + mean_distances[LANE_RIGHT]) / (distances[LANE_LEFT].size() + distances[LANE_RIGHT].size());
		}

		output.push_back(distance_total);

	}

	return output;
}

vector<double> erroPonderadoPeloDesvioPadrao(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula) {
	vector<double> output;

	// masks
	Mat left_mask = Mat::zeros(mapaDeEvidencia.size(), mapaDeEvidencia.type());
	Mat right_mask = Mat::zeros(mapaDeEvidencia.size(), mapaDeEvidencia.type());
	rectangle(left_mask, Rect(0, 0, mapaDeEvidencia.cols / 2, mapaDeEvidencia.rows), 255, -1);
	rectangle(right_mask, Rect(mapaDeEvidencia.cols / 2, 0, mapaDeEvidencia.cols, mapaDeEvidencia.rows), 255, -1);

	Mat evidence_map_left, evidence_map_right;

	mapaDeEvidencia.copyTo(evidence_map_right, right_mask);
	mapaDeEvidencia.copyTo(evidence_map_left, left_mask);

	// cost of finding an evidence in the opposite direction of the intended lane side
	double opposite_direction_weight = 2;

	vector<Point2d> best_spline_points = getSplinePoints(ultimaMelhorParticula, mapaDeEvidencia.rows, LANE_CENTER);

	for (unsigned int ii = 0; ii < particulas.size(); ii++) {

		// get the points that belongs to the spline represented by the particle for each side
		vector<Point2d> spline_points_left = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_LEFT);
		vector<Point2d> spline_points_right = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_RIGHT);

		vector<double> distances_frame_left, distances_frame_right;

		map<int, vector<double> > errors; // regions[side][num_region] = distance_error;

		// para cada lado
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}){
			// escolhe a spline e o evidence_map apropriados
			vector<Point2d> spline_points = (lane_side == LANE_LEFT) ? spline_points_left : spline_points_right;
			Mat side_evidence_map = (lane_side == LANE_LEFT) ? evidence_map_left : evidence_map_right;
			int n_spline = (int)spline_points.size();
			int side_factor = (lane_side == LANE_LEFT) ? -1 : 1;
			vector<double> side;
			// para cada ponto da spline
			for (int p = 0; p < n_spline; p++) {
				double min_distance = numeric_limits<double>::max();
				Point point_selected;
				int point_idx;
				// TODO: interpolar para ver a largura da faixa no ponto p da spline
				int width_in_p = static_cast<int>(particulas[ii].lane_width[2] * 2);
				// percorre a distancia calculada na dire��o do lado da pista em quest�o
				for (int d = 0; d < width_in_p; d++) {
					int p_y = (int)spline_points[p].y;
					//int p_x = (int)(mapaDeEvidencia.cols / 2) + (d * side_factor);
					int p_x = (int)best_spline_points[p].x + (d * side_factor);
					// se o ponto for branco no mapa de evidencia
					if (side_evidence_map.at<uchar>(p_y, p_x) == 255) {
						// calcula a distancia do ponto da spline para o ponto no evidence_map
						double temp_dist = abs(spline_points[p].x - (p_x));
						// se a distancia for menor que � menor encontrada at� o momento, atualize
						if (temp_dist < min_distance) {
							min_distance = temp_dist;
							point_idx = d;
						}
					}
				}
				if (min_distance != numeric_limits<double>::max()) {
					int p_y = (int)spline_points[p].y;
					int p_x = (int)(mapaDeEvidencia.cols / 2) + (point_idx * side_factor);
					point_selected = Point(p_x, p_y);

					side.push_back(min_distance);
				}
			}
			errors[lane_side] = side;
		}

		// calcula a m�dia
		map<int, double> mean;
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}) {
			double sum = 0;
			for (unsigned int i = 0; i < errors[lane_side].size(); i++)
				sum += errors[lane_side][i];
			mean[lane_side] = (sum / errors[lane_side].size());
		}

		// calcula o desvio padrao
		map<int, double> std_dev;
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}) {
			double temp = 0;
			for (unsigned int i = 0; i < errors[lane_side].size(); i++)
				temp += (errors[lane_side][i] - mean[lane_side]) * (errors[lane_side][i] - mean[lane_side]);
			std_dev[lane_side] = sqrt(temp / errors[lane_side].size());
		}

		// calcula o erro
		static const double inv_sqrt_2pi = 0.3989422804014327;
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}) {
			for (unsigned int i = 0; i < errors[lane_side].size(); i++) {
				double a = (errors[lane_side][i] - 0) / std_dev[lane_side];
				double number = inv_sqrt_2pi / std_dev[lane_side] * std::exp(-0.5f * a * a);
				errors[lane_side][i] = (number / (inv_sqrt_2pi / std_dev[lane_side])); // normalizado [0,1]
				// cout << "error: " << errors[lane_side][i] << ", gaussian: " << (number / (inv_sqrt_2pi / std_dev[lane_side])) << endl;
			}
		}

		// TODO: IS THAT USEFUL?

		// calcula a distance m�dia de cada lado
		std::map<int, double> mean_distances;
		for (int lane_side : {LANE_LEFT, LANE_RIGHT}) {
			double mean_distance = numeric_limits<double>::max();
			if (errors[lane_side].size() != 0) {
				mean_distance = std::accumulate(errors[lane_side].begin(), errors[lane_side].end(), 0.0);
			}
			mean_distances[lane_side] = mean_distance;
		}

		// calcula a distancia m�dia final com algumas pondera��es
		double distance_total;
		if (mean_distances[LANE_LEFT] == numeric_limits<double>::max() && mean_distances[LANE_RIGHT] != numeric_limits<double>::max()) {
			distance_total = mean_distances[LANE_RIGHT] / errors[LANE_RIGHT].size();
		} else if (mean_distances[LANE_LEFT] != numeric_limits<double>::max() && mean_distances[LANE_RIGHT] == numeric_limits<double>::max()) {
			distance_total = mean_distances[LANE_LEFT] / errors[LANE_LEFT].size();
		} else {
			distance_total = (mean_distances[LANE_LEFT] + mean_distances[LANE_RIGHT]) / (errors[LANE_LEFT].size() + errors[LANE_RIGHT].size());
		}

		output.push_back(distance_total);

	}

	return output;
}

vector<double> erroProximoAoCarroDiferenciado(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula) {

	vector<double> output;

	const double opposite_direction_weight = 4;

	struct regiao { int inicio, fim; double percentualDescarte; };
	regiao r1, r2, r3;
	// localiza��o das regi�es
	r1.inicio = 0; r1.fim = int(0.5 * mapaDeEvidencia.rows); // inicio = 0
	r2.inicio = r1.fim + 1; r2.fim = r2.inicio + int(0.25 * mapaDeEvidencia.rows);
	r3.inicio = r2.fim + 1; r3.fim = int(mapaDeEvidencia.rows); // fim = # of rows
	// percentual de descarte de cada regi�o
	r1.percentualDescarte = 0.5;
	r2.percentualDescarte = 0.5;
	r3.percentualDescarte = 0.15;
	vector<regiao> regions = { r1, r2, r3 };

	vector<Point2d> best_spline_points = getSplinePoints(ultimaMelhorParticula, mapaDeEvidencia.rows, LANE_CENTER);

	// para cada part�cula
	for (unsigned int ii = 0; ii < particulas.size(); ii++) {

		// get the points that belongs to the spline represented by the particle for each side
		vector<Point2d> spline_points_left = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_LEFT);
		vector<Point2d> spline_points_right = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_RIGHT);

		map<int, map<int, vector<double> > > errosPorRegioes;

		// para cada regi�o
		for (unsigned int numRegiao = 0; numRegiao < regions.size(); numRegiao++) {
			// para cada lado
			for (int lado : {LANE_LEFT, LANE_RIGHT}) {
				// pega os pontos da spline correta, baseado no lado que est� sendo avaliado
				vector<Point2d> spline_points = (lado == LANE_LEFT) ? spline_points_left : spline_points_right;
				int side_factor = (lado == LANE_LEFT) ? -1 : 1;
				int larguraBuscaLateral = static_cast<int>(particulas[ii].lane_width[2] * 2);
				// para cada ponto da regi�o
				for (int p = regions[numRegiao].inicio; p < regions[numRegiao].fim; p++) {
					// pega a menor distancia
					double menorDistancia = numeric_limits<double>::max();
					// para cada ponto da busca lateral
					for (int d = 0; d < larguraBuscaLateral; d++) {
						int p_y = (int)spline_points[p].y;
						int p_x = (int)best_spline_points[p].x + (d * side_factor);
						// se o ponto for branco no mapa de evidencia
						if (mapaDeEvidencia.at<uchar>(p_y, p_x) == 255) {
							// calcula a distancia do ponto da spline para o ponto no evidence_map
							double temp_dist = abs(spline_points[p].x - (p_x));
							if ((spline_points[p].x - (p_x) < 0 && side_factor == -1) || (spline_points[p].x - (p_x) > 0 && side_factor == 1)) {
								temp_dist *= opposite_direction_weight;
							}
							// se a distancia for menor que � menor encontrada at� o momento, atualize
							if (temp_dist < menorDistancia) menorDistancia = temp_dist;
						}
					}
					// s� guarda a menor distancia se alguma foi encontrada
					if (menorDistancia != numeric_limits<double>::max()) {
						errosPorRegioes[lado][numRegiao].push_back(menorDistancia);
					}
				}
			}
		}

		// agrupa os x% melhores de cada lado
		std::map<int, vector<double> > distancias;
		for (int lado : {LANE_LEFT, LANE_RIGHT}) {
			vector<double> distanciasLado;

			// calcula o espa�o para o vetor final
			int vector_size = 0;
			for (unsigned int numRegiao = 0; numRegiao < regions.size(); numRegiao++) {
				vector_size += (int)errosPorRegioes[lado][numRegiao].size();
			}
			// pr�-aloca mem�ria
			distanciasLado.reserve(vector_size);

			// concatena os x% menores de cada regi�o no vetor final do lado em quest�o
			for (unsigned int numRegiao = 0; numRegiao < regions.size(); numRegiao++) {
				std::sort(errosPorRegioes[lado][numRegiao].begin(), errosPorRegioes[lado][numRegiao].end());
				// calcula quantos elementos ser�o mantidos
				int size_subvector = static_cast<int>(floor(errosPorRegioes[lado][numRegiao].size() * (1 - regions[numRegiao].percentualDescarte)));
				distanciasLado.insert(distanciasLado.end(), errosPorRegioes[lado][numRegiao].begin(), errosPorRegioes[lado][numRegiao].begin() + size_subvector);
			}
			distancias[lado] = distanciasLado;
		}

		// calcula a dist�ncia m�dia de cada lado
		std::map<int, double> mediaDasDistancias;
		for (int lado : {LANE_LEFT, LANE_RIGHT}) {
			double distanciaMedia = numeric_limits<double>::max();
			if (distancias[lado].size() != 0) {
				distanciaMedia = std::accumulate(distancias[lado].begin(), distancias[lado].end(), 0.0);
			}
			mediaDasDistancias[lado] = distanciaMedia;
		}

		// calcula a distancia m�dia final com algumas pondera��es
		double distanciaPonderada;
		if (mediaDasDistancias[LANE_LEFT] == numeric_limits<double>::max() && mediaDasDistancias[LANE_RIGHT] != numeric_limits<double>::max()) {
			distanciaPonderada = mediaDasDistancias[LANE_RIGHT] / distancias[LANE_RIGHT].size();
		} else if (mediaDasDistancias[LANE_LEFT] != numeric_limits<double>::max() && mediaDasDistancias[LANE_RIGHT] == numeric_limits<double>::max()) {
			distanciaPonderada = mediaDasDistancias[LANE_LEFT] / distancias[LANE_LEFT].size();
		} else {
			distanciaPonderada = (mediaDasDistancias[LANE_LEFT] + mediaDasDistancias[LANE_RIGHT]) / (distancias[LANE_LEFT].size() + distancias[LANE_RIGHT].size());
		}
		output.push_back(distanciaPonderada);
	}
	return output;
}

vector<double> erroTotalEmbaixoDaSpline(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula) {

	vector<double> output;

	// para cada part�cula
	for (unsigned int ii = 0; ii < particulas.size(); ii++) {

		// get the points that belongs to the spline represented by the particle for each side
		vector<Point2d> spline_points_left = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_LEFT);
		vector<Point2d> spline_points_right = getSplinePoints(particulas[ii], mapaDeEvidencia.rows, LANE_RIGHT);

		map<int, double> errosPorLado;
		double erroFinal;

		for (int lado : {LANE_LEFT, LANE_RIGHT}) {

			// pega os pontos da spline correta, baseado no lado que est� sendo avaliado
			vector<Point2d> spline_points = (lado == LANE_LEFT) ? spline_points_left : spline_points_right;

			// inicializa os contadores
			double total = 0;
			double qtd = 0;

			// percore a spline
			//Mat mapaFaixaInterna = (lado == LANE_LEFT) ? mapaDeEvidencia.left : mapaDeEvidencia.right; // TODO: MUDAR NOME
			for (unsigned int p = 0; p < spline_points.size(); p++) {
				int p_y = (int)spline_points[p].y;
				int p_x = (int)spline_points[p].x;
				// se o ponto for branco no mapa de evidencia, adiciona no total de erro
				if (p_x >= 0 && p_x < mapaDeEvidencia.cols && p_y >= 0 && p_y < mapaDeEvidencia.rows) {
					total += mapaDeEvidencia.at<double>(p_y, p_x);
				}
				qtd++;
			}
			errosPorLado[lado] = total / spline_points.size();
		}

		// tira a m�dia do erro de cada lado
		double x = errosPorLado[LANE_LEFT];
		double y = errosPorLado[LANE_RIGHT];
		erroFinal = x*y + (1 - x*y)*((x + y) / 2);
		output.push_back(erroFinal);
	}
	return output;
}

// =================================
// OUTROS
// =================================

vector<Point2d> getSplinePoints(const Particle &particula, const int _height, const int laneSide) {
	// pega todos os pontos que s�o representados pela part�cula
	vector<Point2d> particle_points;
	double step = _height / (double)(particula.spline_points.size() - 1);
	for (unsigned int i = 0; i < particula.spline_points.size(); i++) {
		double shift;
		switch (laneSide) {
		case LANE_LEFT: shift = -1; break;
		case LANE_RIGHT: shift = 1; break;
		case LANE_CENTER: shift = 0; break;
		}
		double lane_width;
		if (i == 0) lane_width = particula.lane_width[0];
		else if (i == 1) lane_width = (particula.lane_width[0] + particula.lane_width[1]) / 2;
		else if (i == 2) lane_width = particula.lane_width[1];
		particle_points.push_back(Point2d(particula.spline_points[i] + (shift * (lane_width / 2)), static_cast<int>(step * i)));
	}

	// pega todos os pontos que pertencem spline representada pela part�cula
	vector<Point2d> spline_points;
	if (particle_points.size() >= 2) {
		std::vector<double> x, y;
		for (Point2d p : particle_points) {
			y.push_back(p.x);
			x.push_back(p.y);
		}
		tk::spline s;
		s.set_points(x, y);
		for (double i = 0; i < _height; i++) {
			spline_points.push_back(Point2d(s(i), i));
		}
	}

	return spline_points;
}

void mostrarParticulas(const vector<Particle> &particulas, Mat &imagem, const Scalar cor, const bool intercalar) {
	for (Particle p : particulas) {
		vector<Point2d> spline_points_left = getSplinePoints(p, imagem.rows, LANE_LEFT);
		vector<Point2d> spline_points_right = getSplinePoints(p, imagem.rows, LANE_RIGHT);
		for (unsigned int i = 0; i < spline_points_left.size(); i++) {
			if (intercalar && (i % 10) < 5) {
				circle(imagem, spline_points_left[i], 0, cor, -1);
				circle(imagem, spline_points_right[i], 0, cor, -1);
			} else if (!intercalar && (i % 10) >= 5) {
				circle(imagem, spline_points_left[i], 0, cor, -1);
				circle(imagem, spline_points_right[i], 0, cor, -1);
			}
			/* desenha as linhas verdes, que representam a largura da faixa
			if (i == 10 || i == spline_points_left.size() / 2 || i == spline_points_left.size() - 10) {
				line(imagem, spline_points_left[i], spline_points_right[i], Scalar(0, 255, 0), 2);
			}
			*/
		}
	}
}
