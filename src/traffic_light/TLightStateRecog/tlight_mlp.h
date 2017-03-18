
#ifndef TLIGHT_MLP_H_
#define TLIGHT_MLP_H_

#include <carmen/tlight_state_recog.h>
#include <dlib/dnn.h>
#include <dlib/data_io.h>
#include <cstdlib>

using namespace dlib;

class TLightMlp : public TLightRecogInterface
{
public:

	TLightMlp()
	{
		deserialize(get_complete_filename("tlight_dlib.net")) >> net_;
	}

	virtual int run(cv::Mat &m, cv::Rect &r)
	{
		// o clone eh necessario porque ao obter o roi de uma imagem img, vc recebe apenas
		// um ponteiro para o inicio do roi, mas os dados continuam sendo da imagem original.
		// ao tentar acessar a 2a linha do roi, por exemplo, estaremos acessando as colunas
		// que vem depois do roi na imagem original.
		cv::Mat croped = m(r).clone();
		cv::Mat res(cv::Size(TRAFFIC_SIGN_WIDTH, TRAFFIC_SIGN_HEIGHT), CV_8UC3);
		cv::resize(croped, res, res.size());
		std::vector<matrix<unsigned char>> input(1);
		int input_size = res.rows * res.cols * res.channels();
		input[0].set_size(input_size, 1);
		memcpy(&(input[0](0, 0)), res.data, input_size * sizeof(unsigned char));
		return net_(input)[0];
	}

protected:


	typedef loss_multiclass_log< fc<NUM_CLASSES, relu< fc<32, input< matrix<unsigned char>>>>>> net_type;
	net_type net_;
};



#endif
