#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>

#include "camera_drivers.h"
#include "camera_drivers_process_image.hpp"

int VERBOSE = 0;



void callback(void *avcl, int level, const char *fmt, va_list vl)
{
	(void) avcl;
    if (level > AV_LOG_ERROR)
        return;

    vprintf(fmt, vl);
    
}


int initialize_ffmpeg(av_ffmpeg *afp, char* ip_address)
{
    int ret;
    av_log_set_callback(&callback);

    // open input file context
    AVFormatContext *inctx = nullptr;
    ret = avformat_open_input(&inctx, ip_address, nullptr, nullptr);
    if (ret < 0)
    {
        std::cout << "avforamt_open_input(\"" << ip_address << "\"): ret=" << ret << std::endl;
		carmen_die("please check the host, and the user:password!\n");
        return 2;
    }
    // retrive input stream information
    ret = avformat_find_stream_info(inctx, nullptr);
    if (ret < 0)
    {
        std::cout << "avformat_find_stream_info: ret=" << ret << std::endl;
        return 2;
    }
    // find primary video stream
    AVCodec *vcodec = nullptr;
    ret = av_find_best_stream(inctx, AVMEDIA_TYPE_VIDEO, -1, -1, &vcodec, 0);
    if (ret < 0)
    {
        std::cout << "av_find_best_stream: ret=" << ret << std::endl;
        return 2;
    }
    const int vstrm_idx = ret;
    AVStream *vstrm = inctx->streams[vstrm_idx];

    if (vstrm->codecpar->codec_id == AV_CODEC_ID_MJPEG)
    {
        std::cout << "not stable with mjpeg codec!\nopening with OpenCV" << std::endl;
        afp->use_with_opencv = 1;
        std::string _infile(ip_address);
        afp->vcap = new cv::VideoCapture();

        if(afp->vcap->open(_infile))
        {
            cv::Mat _image;
	        if (afp->vcap->read(_image))
            {
                afp->dst_width = _image.cols;
        		afp->dst_height = _image.rows;
				afp->ret = 1;

                return 3;
            }
        }
        return 2;
    }

    // open video decoder context
    AVCodec *pCodec = avcodec_find_decoder(vstrm->codecpar->codec_id);
    AVCodecContext *avctx = avcodec_alloc_context3(pCodec);
    avcodec_parameters_to_context(avctx, vstrm->codecpar);

    vstrm->time_base = avctx->time_base;

    ret = avcodec_open2(avctx, vcodec, nullptr);
    if (ret < 0)
    {
        std::cout << "avcodec_open2: ret=" << ret << std::endl;
        return 2;
    }

    // print input video stream informataion
    if (VERBOSE)
    {
        std::cout
                << "format: " << inctx->iformat->name << "\n"
                << "vcodec: " << vcodec->name << "\n"
                << "size:   " << avctx->width << 'x' << avctx->height << "\n"
                << "fps:    " << av_q2d(avctx->framerate) << " [fps]\n"
                << "length: " << av_rescale_q(vstrm->duration, vstrm->time_base, {1, 1000}) / 1000. << " [sec]\n"
                << "pixfmt: " << av_get_pix_fmt_name(avctx->pix_fmt) << "\n"
                << "frame:  " << vstrm->nb_frames << "\n"
                << std::flush;
    }

    // initialize sample scaler
    const int dst_width = avctx->width;
    const int dst_height = avctx->height;
    const AVPixelFormat dst_pix_fmt = AV_PIX_FMT_BGR24;

    // avoid warning
    // https://stackoverflow.com/questions/23067722/swscaler-warning-deprecated-pixel-format-used
    AVPixelFormat src_pix_fmt;
    switch (avctx->pix_fmt) {
    case AV_PIX_FMT_YUVJ420P :
        src_pix_fmt = AV_PIX_FMT_YUV420P;
        break;
    case AV_PIX_FMT_YUVJ422P  :
        src_pix_fmt = AV_PIX_FMT_YUV422P;
        break;
    case AV_PIX_FMT_YUVJ444P   :
        src_pix_fmt = AV_PIX_FMT_YUV444P;
        break;
    case AV_PIX_FMT_YUVJ440P :
        src_pix_fmt = AV_PIX_FMT_YUV440P;
        break;
    default:
        src_pix_fmt = avctx->pix_fmt;
        break;
    }

    SwsContext *swsctx = sws_getCachedContext(
        nullptr, avctx->width, avctx->height, src_pix_fmt,
        dst_width, dst_height, dst_pix_fmt, SWS_BICUBIC, nullptr, nullptr, nullptr);
    if (!swsctx)
    {
        std::cout << "sws_getCachedContext" << std::endl;
        return 2;
    }
    if (VERBOSE)
        std::cout << "output: " << dst_width << 'x' << dst_height << ',' << av_get_pix_fmt_name(dst_pix_fmt) << std::endl;

    // allocate frame buffer for output
    AVFrame *frame = av_frame_alloc();
    std::vector<uint8_t> *framebuf = new std::vector<uint8_t>(av_image_get_buffer_size(dst_pix_fmt, dst_width, dst_height, 1));
    av_image_fill_arrays(frame->data, frame->linesize, framebuf->data(), dst_pix_fmt, dst_width, dst_height, 1);

    afp->inctx = inctx;
    afp->vstrm_idx = vstrm_idx;
    afp->vstrm = vstrm;
    afp->avctx = avctx;
    afp->swsctx = swsctx;
    afp->frame = frame;
    afp->framebuf = framebuf;

    // decoding loop
    afp->decframe = av_frame_alloc();
    afp->nb_frames = 0;
    afp->end_of_stream = false;
    afp->dst_width = afp->avctx->width;
    afp->dst_height = afp->avctx->height;
    afp->use_with_opencv = 0;

    return ret;
}


unsigned char *get_ffmpeg_image(av_ffmpeg *afp)
{
    if (afp->use_with_opencv)
    {
        cv::Mat _image;
        if (afp->vcap->read(_image))
		{
			afp->dst_width = _image.cols;
			afp->dst_height = _image.rows;

			return _image.data;
		}
		afp->dst_width = -1;
		return nullptr;		
    }

    int ret;

    if (!afp->end_of_stream)
    {
        // read packet from input file
        ret = av_read_frame(afp->inctx, &afp->pkt);
        if (ret < 0 && ret != AVERROR_EOF)
        {
            std::cout << "Nothing to read!" << std::endl;
            std::cout << "av_read_frame: ret=" << ret << std::endl;
            return nullptr;
        }
        if (ret == 0 && afp->pkt.stream_index != afp->vstrm_idx)
        {
            av_packet_unref(&afp->pkt);
            return nullptr;
        }
        afp->end_of_stream = (ret == AVERROR_EOF);
    }
    if (afp->end_of_stream)
    {
        // null packet for bumping process
        av_init_packet(&afp->pkt);
        afp->pkt.data = nullptr;
        afp->pkt.size = 0;
    }
    // decode video frame
    ret = avcodec_receive_frame(afp->avctx, afp->decframe);
    ret = avcodec_send_packet(afp->avctx, &afp->pkt);
    if (ret < 0)
    {
        std::cout << "avcodec_send_packet: ret=" << ret << std::endl;
        av_packet_unref(&afp->pkt);
        return nullptr;
    }
    // convert frame to OpenCV matrix
    sws_scale(afp->swsctx, afp->decframe->data, afp->decframe->linesize, 0, afp->decframe->height, afp->frame->data, afp->frame->linesize);

    ++afp->nb_frames;

    av_packet_unref(&afp->pkt);

    if (VERBOSE)
	{
		std::cout << afp->nb_frames << " | " << afp->frame->linesize[0] << '\r' << std::flush; // dump progress
	}

    return afp->framebuf->data();
}


VideoCapture
open_video_stream(char *infile)
{
	int ret;
	VideoCapture vcap;
	ret = !vcap.open(infile);
	while (ret)
	{
		printf("\nhost: %s\n", infile);
        carmen_die("please check the host, and the user:password!\n");
	}

	return (vcap);
}

void initialize_udp_h264(cv::VideoCapture& cap, std::string& pipeline, char* ip_address, char* port, char* network_interface)
{
    std::ostringstream pipeline_stream;
    pipeline_stream
        << "udpsrc address=" << ip_address
        << " port=" << port
        << " multicast-iface=" << network_interface << " "
        << "! application/x-rtp, media=video, encoding-name=H264 "
        << "! rtph264depay "
        << "! h264parse "
        << "! avdec_h264 "
        << "! videoconvert "
        << "! appsink";

    pipeline = pipeline_stream.str();

    cap = cv::VideoCapture(pipeline, cv::CAP_GSTREAMER);
}

bool reconnect(cv::VideoCapture& cap, const std::string& pipeline, int max_retries = 10) {
    for (int i = 0; i < max_retries; ++i) {
        std::cout << "Tentando reconectar... tentativa " << (i + 1) << std::endl;
        cap.open(pipeline, cv::CAP_GSTREAMER);
        if (cap.isOpened()) {
            std::cout << "Reconexão bem-sucedida!" << std::endl;
            return true;
        }
    }
    std::cerr << "Falha ao reconectar após " << max_retries << " tentativas." << std::endl;
    return false;
}

unsigned char*
get_image_udp_rtp(cv::VideoCapture& cap, char* ip_address, char* port, char* network_interface, int &width, int &height)
{
	unsigned char* raw;
    cv::Mat frame;
    std::string pipeline;

    if (!cap.isOpened()) 
    {
        initialize_udp_h264(cap, pipeline, ip_address, port, network_interface);
    }
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "Perda de conexão." << std::endl;
        cap.release();
        reconnect(cap, pipeline);
    }
    width = frame.cols;
    height = frame.rows;
    raw = frame.data;

	return raw;
}

unsigned char*
get_image_usb_camera_with_opencv(char* camera_model, int undistort, char* camera_name, int &width, int &height)
{
	unsigned char* raw;
    static int crop = 0;
	static bool first_time = true;
    static Mat MapX, MapY, image;
    static Rect myROI;
	static VideoCapture vcap;
    static vector<tCrop> x_y_vector;

	if (first_time)
	{
		vcap = open_video_stream(camera_model);
		if (!vcap.open(camera_model))
			exit(1);

        if (undistort && vcap.read(image))
            load_undistort(camera_name, 1, image.cols, image.rows, 1, crop, x_y_vector, MapX, MapY);

		first_time = false;
	}
	while (!vcap.read(image))
	{
		printf("\nconnection lost!%s\n\n", raw);
		sleep(3);
		vcap = open_video_stream(camera_model);
	}

	width = image.cols;
	height = image.rows;

	if (width*height <= 0)
		return nullptr;

    if (undistort)
        remap(image, image, MapX, MapY, INTER_LINEAR);

    raw = image.data;
	return raw;
}


unsigned char*
get_image_ip_camera_with_opencv(char* ip_address, int undistort, char* camera_name, int &width, int &height)
{
	return (get_image_usb_camera_with_opencv(ip_address, undistort, camera_name, width, height));
}


unsigned char*
get_image_ip_camera_with_ffmmpeg(char* ip_address, int undistort, char* camera_name, av_ffmpeg *afp, int &width, int &height)
{
	unsigned char* raw;
	static bool first_time = true;

	if (first_time)
	{
		initialize_ffmpeg(afp, ip_address);
		raw = get_ffmpeg_image(afp);
		while (!raw)
			exit(1);

		afp->ret = 1;
		first_time = false;
	}
    if (afp->use_with_opencv)
    {
        return get_image_usb_camera_with_opencv(ip_address, undistort, camera_name, width, height);
    }

	while (afp->ret < 0)
	{
		printf("\nconnection lost!\n\n");
		sleep(3);
		initialize_ffmpeg(afp, ip_address);
	}

	raw = get_ffmpeg_image(afp);

	width = afp->dst_width;
	height = afp->dst_height;

	if (width*height <= 0)
		return nullptr;

    return raw;
}
