/*********************************************************
 * Compatibilidade com a API C do OpenCV 4
 * (migração para Ubuntu 26.04 — ver doc/migracao_ubuntu26/)
 *
 * O OpenCV 4 removeu do binário algumas funções da API C que boa parte do código antigo do
 * CARMEN ainda usa (cvLoadImage/cvSaveImage), e trocou o CV_RGB() por uma versão C++ que
 * devolve cv::Scalar — incompatível com as funções da API C (cvLine, cvRectangle, ...).
 *
 * Este header reimplementa esse punhado de coisas em cima da API C++ (cv::imread/cv::imwrite),
 * para que o código que ainda trabalha com IplImage continue funcionando sem ser reescrito.
 * Em OpenCV <= 3 ele não faz nada, então continua compilando em Ubuntu antigo.
 *
 * Só pode ser incluído de código compilado como C++ (arquivos .cpp, ou .c em módulos com
 * CC = g++ no Makefile) — ele usa cv::Mat internamente.
 *********************************************************/

#ifndef CARMEN_OPENCV_C_COMPAT_H
#define CARMEN_OPENCV_C_COMPAT_H

#include <opencv2/core/version.hpp>

#if CV_MAJOR_VERSION >= 4

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

/* cvLoadImage(): devolve um IplImage recém-alocado (cvCloneImage), porque o chamador
   costuma guardar/retornar o ponteiro e liberá-lo depois com cvReleaseImage().
   Devolve NULL quando não consegue ler o arquivo, igual à original. */
static inline IplImage *
cvLoadImage(const char *filename, int flags = cv::IMREAD_COLOR)
{
	cv::Mat mat = cv::imread(filename, flags);
	if (mat.empty())
		return (NULL);

	IplImage ipl = cvIplImage(mat);
	return (cvCloneImage(&ipl));
}

/* cvSaveImage(): a gravação é síncrona, então basta uma view sobre o buffer do IplImage
   (cvarrToMat não copia). Devolve 1/0 como a original. O terceiro argumento (params) da
   API C não é repassado — nenhum ponto do CARMEN o usa (todos passam 0/NULL). */
static inline int
cvSaveImage(const char *filename, const CvArr *image, const int *params = 0)
{
	(void) params;
	return (cv::imwrite(filename, cv::cvarrToMat(const_cast<CvArr *>(image))) ? 1 : 0);
}

/* CV_RGB(): no OpenCV 4 o que sobra é a macro do imgproc.hpp, que devolve cv::Scalar e não
   converte para o CvScalar que a API C espera. Volta a devolver CvScalar (mesma ordem BGR). */
#undef CV_RGB
#define CV_RGB(r, g, b)  cvScalar((b), (g), (r), 0)

#endif /* CV_MAJOR_VERSION >= 4 */

#endif /* CARMEN_OPENCV_C_COMPAT_H */
