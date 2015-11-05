/*
 * spectogram.c
 *
 *  Created on: 29/05/2012
 *      Author: filipe
 */

#include "spectogram.h"
#include "wav_decoder.h"
#include "fftw3_util.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

extern wav_format_chunk file_format;

unsigned int min_frequency = 1; // != 0
unsigned int max_frequency = 20000;

unsigned int chunk_size = 128;
unsigned int num_frequencies = 64; // (chunk_size / 2)

// linhas = frequencia
unsigned int num_lines_img = 64; // (chunk_size / 2)
// colunas = tempo
unsigned int num_cols_img = 1074;

IplImage *spectogram;

fftw_complex *fftw_input;
fftw_complex *fftw_output;

/**
 * Funcao para fazer mudancas de escala
 *
 * @param x0 valor minimo da escala 1
 * @param x1 valor maximo da escala 1
 * @param y0 valor minimo da escala 2 (relativo ao ponto x0)
 * @param y1 valor maximo da escala 2 (relativo ao ponto x1)
 * @param x valor que queremos converter da escala 1 para a escala 2
 * @return double valor do y (relativo a x) na escala 2
 */
double change_scale (double x0, double x1, double y0, double y1, double x)
{
	double delta_y = (y1 - y0);
	double delta_x = (x1 - x0);
	double var_x = (x - x0);

	double y = (delta_y / delta_x) * var_x + y0;

	return y;
}

CvScalar convert_to_color (unsigned int x)
{
	if (x < 0)
		exit(printf("error: x < 0 while creating color\n"));

	if (x > 255)
		// exit(printf("error: x = %f while creating color\n", x));
		x = 255;

// os testes abaixo sao para criar uma escala de cores melhor
// para visualizar
//
//	if (x < 255)
//		return cvScalar ((unsigned char)x,0,0,0);
//	else if (x < (2 * 255))
//		return cvScalar (255, (unsigned char) x - 255,0,0);
//	else
//		return cvScalar (255, 255, (unsigned char) x - 2 * 255, 0);

	return cvScalar (x,x,x,0);
}

void spectogram_alloc (void)
{
	unsigned int i, j;

	spectogram = cvCreateImage(cvSize(num_cols_img, num_lines_img), IPL_DEPTH_8U, 3);

	// inicializa a imagem com zero
	for(i = 0; i < num_lines_img; i++)
		for(j = 0; j < num_cols_img; j++)
			cvSet2D(spectogram, i, j, cvScalar(0,0,0,0));

	fftw_input = (fftw_complex*) fftw_malloc ((chunk_size) * sizeof(fftw_complex));
	fftw_output = (fftw_complex*) fftw_malloc ((chunk_size) * sizeof(fftw_complex));
}

void spectogram_create_fft_input (unsigned int start)
{
	unsigned int i;

	for(i = 0; i < chunk_size; i++)
	{
		fftw_input [i][0] = wav_get_sample(start + i, 0);
		fftw_input [i][1] = 0;
	}
}

void spectogram_apply_fft (void)
{
	fftw_fourier_transform(fftw_input, fftw_output, chunk_size);
}

double spectogram_get_wave_magnitude (fftw_complex x)
{
	double magnitude = sqrt(pow (x[0], 2) + pow (x[1], 2));
	return magnitude;
}

void spectogram_shift_on_time ()
{
	int i, j;

	for (i = spectogram->width - 1; i > 0; i--)
	{
		for (j = 0; j < spectogram->height; j++)
		{
			cvSet2D(spectogram, j, i, cvGet2D (spectogram, j, i - 1));
		}
	}

	// zera a primeira coluna
	for (i = 0; i < spectogram->height; i++)
	{
		cvSet2D(spectogram, i, 0, cvScalar(0, 0, 0, 0));
	}
}

void test_shift ()
{
	IplImage *input = cvLoadImage ("bola.jpg", CV_LOAD_IMAGE_ANYCOLOR);
	spectogram = cvCreateImage (cvSize (640, 480), IPL_DEPTH_8U, 3);

	cvCopy(input, spectogram, NULL);

	int i;
	for(i = 0; i < 320; i++)
		spectogram_shift_on_time();

	cvShowImage ("spectogram", spectogram);
	cvShowImage ("input", input);

	cvWaitKey (-1);
	exit(0);
}

void spectogram_update_with_fft (void)
{
	unsigned int i, j;

//	printf("min_frequency: %d max_frequency: %d\n", min_frequency, max_frequency);
//	printf("chunk_size: %d num_frequencies: %d num_lines_img: %d num_cols_img: %d\n",
//			chunk_size,
//			num_frequencies,
//			num_lines_img,
//			num_cols_img);

	double max_freq = file_format.sample_rate / 2.0;
	double var_freq = max_freq / (double) num_frequencies;

	// calcula o numero de linhas que o intervalo que estamos interessados
	// ira ocupar na imagem final
	double subfreq_lines = ((max_frequency - min_frequency) / (max_freq)) * num_lines_img;

	// calcula o numero de linhas que deverao ser repetidas de forma
	// que a area ocupada pela frequencia de interesse ocupe a maior
	// parte da imagem
	unsigned int num_lines_per_freq = (num_lines_img / subfreq_lines) + 1;

	if (num_lines_per_freq < 1)
		num_lines_per_freq = 1;

	// printf ("subfreq_lines: %f num_lines_per_freq: %d\n", subfreq_lines, num_lines_per_freq);
	// exit(0);

	double min_magnitude_freq = 999999999;
	double max_magnitude_freq = 0;
	double avg_magnitude_freq = 0;
	unsigned int avg_magnitude_n = 0;

	// obtem os valores maximos e minimos
	// de amplitude no intervalo de frequencia
	// que estamos interessados
	for(i = 0; i < num_frequencies; i++)
	{
		double freq_true_value = var_freq * i;
		double m = spectogram_get_wave_magnitude(fftw_output[i]);

		if (freq_true_value > min_frequency && freq_true_value < max_frequency)
		{
			if (m < min_magnitude_freq)
				min_magnitude_freq = m;
			if (m > max_magnitude_freq)
				max_magnitude_freq = m;

			avg_magnitude_freq += m;
			avg_magnitude_n ++;
		}
	}

	avg_magnitude_freq /= (double) avg_magnitude_n;

	// printf ("min_magnitude_freq: %f\n", min_magnitude_freq);
	// printf ("max_magnitude_freq: %f\n", max_magnitude_freq);
	// printf ("avg_magnitude_freq: %f\n", avg_magnitude_freq);
	// printf ("avg_magnitude_n: %d de %d\n", avg_magnitude_n, num_frequencies);

	// desenha o espectograma
	for (i = 0; i < num_frequencies; i++)
	{
		double freq_true_value = var_freq * i;
		double freq_magnitude = spectogram_get_wave_magnitude(fftw_output[i]);

		if (freq_true_value > min_frequency && freq_true_value < max_frequency)
		{
			//unsigned int p =
			//	(512 * (freq_true_value - min_frequency)) / (max_frequency - min_frequency);

			// p sera a linha relativa a uma frequencia no
			// espectograma
			double p = 0;

			// precisamos mudar da escala de frequencia para
			// a escala da altura da imagem
			p = change_scale (min_frequency, max_frequency, 0, num_lines_img, freq_true_value);

			if (p > num_lines_img)
				p = num_lines_img;

			// printf ("%.2f -> %d\n", freq_true_value, p);

			// v sera a cor da magnitude da frequencia
			// na imagem
			double v = 0;

			// precisamos mudar a escala de magnitude de frequencia
			// para a escala de cor da imagem
			v = change_scale(min_magnitude_freq, avg_magnitude_freq, 0, 128, freq_magnitude);

			if (v > 255)
				v = 255;

			// printf ("\tfreq_magnitude: %f v: %d\n", freq_magnitude, v);

			for(j = 0; j < num_lines_per_freq; j++)
			{
				// desenhamos na linha (spectogram->height - p) para que o zero
				// fique em baixo na imagem (o zero da opencv eh em cima)
				int p_img = spectogram->height - p - j;

				// a ultima frequencia de interesse estara na posicao 0 e
				// nesse caso nao deve ser repetida
				if (p_img >= 0)
				{
					cvSet2D(spectogram, p_img, 0, convert_to_color(v));
				}
			}
		}
	}
}

void spectogram_show ()
{
	cvShowImage("spectogram", spectogram);
	cvWaitKey(-1);

	cvDestroyAllWindows();
}

void spectogram_create (unsigned int start)
{
	// merge current chunk and overlapping
	spectogram_create_fft_input (start);
	// compute fft
	spectogram_apply_fft ();
	// deslocar o espectograma para direita
	spectogram_shift_on_time ();
	// use fft to update spectogram
	spectogram_update_with_fft ();
	// show spectogram
}

void spectogram_release (void)
{
	cvReleaseImage(&spectogram);

	fftw_free (fftw_input);
	fftw_free (fftw_output);
}

void spectogram_write_to_file (char *filename)
{
	cvSaveImage(filename, spectogram, 0);
}
