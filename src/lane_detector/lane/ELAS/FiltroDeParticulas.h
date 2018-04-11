#ifndef __FILTRO_DE_PARTICULAS_H
#define __FILTRO_DE_PARTICULAS_H

#include <opencv2/opencv.hpp>
#include "utils/HelperXML.h"
#include "utils/common.h"
#include "utils/spline.h"

using namespace std;
using namespace cv;

#include <cmath>
#include <vector>
#include <ctime> // time
#include <algorithm> // std::sort
#include <numeric> // std::accumulate
#include <iostream>

double gaussian_random(double mean, double std);
double carmen_uniform_random(double min, double max);

enum {
	WEIGHT_INVERSE_ERROR = 1,
	WEIGHT_ERROR = 2,
	WEIGHT_ALBERTO = 10
};

enum {
	ERRO_POR_REGIOES = 0,
	ERRO_PONDERADO_PELO_DESVIO_PADRAO = 1,
	ERRO_PROXIMO_AO_CARRO_DIFERENCIADO = 2,
	ERRO_TOTAL_EMBAIXO_SPLINE = 3
};

class Particle {
	public:
		double weight;
		vector<double> spline_points;
		vector<double> lane_width;
		inline bool operator<(const Particle &p1) const;

		Particle(const Particle &p);
		Particle(double centroImagem = 0, double larguraPistaInicial = 0);
		~Particle();

		static Particle geraParticulaAleatoria(double mean, double std, double base_x, double initial_lane_width);
};

class ParticleFilter {
	private:
		vector<Particle> particles;
		bool verbose;
		bool display;
		double tempoExecutando;

		void geraParticulasAleatorias();
		void resample();
		void normalize_particle_weights();
		double calculate_particle_probability(double error);
		double calculate_particle_probability_alberto(double error, double std_dev);
		double calculate_particle_probability_error(double error);

		void predict();
		void correct(std::vector<double> error_vector, int weight_function);

	public:
		bool disabled = false;
		double desvioPadraoSplinePoints = 5;	// => 15.0 / 3
		double desvioPadraoLaneWidth = 1.6666;	// => 05.0 / 3
		double posicaoCarro;
		double larguraPistaPadrao;

		ParticleFilter();
		ParticleFilter(int num_particles, double centroImagem, double larguraPistaInicial, bool _verbose = false, bool _display = false);
		~ParticleFilter();
		
		Particle executar(Mat &mapaDeEvidencia, int funcaoErro = ERRO_TOTAL_EMBAIXO_SPLINE, int funcaoPeso = WEIGHT_ERROR);
		void view(const Mat &imagemPerspectiva, Mat &displayFrameRoiIPM, const ConfigXML &_config, bool onlyPoints = false, const vector<Scalar> &cores = { Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255) });

		Particle getBestParticle();
		vector<Particle> getBestParticles(int number);
		void getParticulasMaiorMenorPeso(Particle &maiorPeso, Particle &menorPeso);
		vector<Particle> getParticles();
};

vector<double> calculaErros(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula, int flag = ERRO_POR_REGIOES);
vector<double> erroPorRegioes(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula);
vector<double> erroPonderadoPeloDesvioPadrao(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula);
vector<double> erroProximoAoCarroDiferenciado(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula);
vector<double> erroTotalEmbaixoDaSpline(const vector<Particle> &particulas, Mat &mapaDeEvidencia, const Particle &ultimaMelhorParticula);
vector<Point2d> getSplinePoints(const Particle &particula, const int _height, const int laneSide);
void mostrarParticulas(const vector<Particle> &particulas, Mat &imagem, const cv::Scalar cor = cv::Scalar(255, 0, 0), const bool intercalar = true);

#endif // __FILTRO_DE_PARTICULAS_H
