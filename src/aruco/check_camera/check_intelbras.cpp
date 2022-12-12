#include <iostream>
#include <vector>
// FFmpeg
extern "C"
{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

typedef struct tAv_ffmpeg
{
    AVFormatContext *inctx;
    int vstrm_idx;
    AVStream *vstrm;
    AVCodecContext *avctx;
    SwsContext *swsctx;
    AVFrame *frame;
    std::vector<uint8_t> *framebuf;
    AVFrame *decframe;
    unsigned nb_frames;
    bool end_of_stream;
    AVPacket pkt;

    int dst_width;
    int dst_height;

    int ret;
} av_ffmpeg;


void callback(void *avcl, int level, const char *fmt, va_list vl)
{
	(void) avcl;
    if (level > AV_LOG_ERROR)
        return;

    vprintf(fmt, vl);
    
}


int init(av_ffmpeg *afp, char *ip_adress)
{
    int ret;
    av_log_set_callback(&callback);

    // open input file context
    AVFormatContext *inctx = nullptr;
    ret = avformat_open_input(&inctx, ip_adress, nullptr, nullptr);
    if (ret < 0)
    {
        std::cout << "avforamt_open_input(\"" << ip_adress << "\"): ret=" << ret << std::endl;
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
        std::cout << "Not stable with mjpeg codec!" << std::endl;
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

    std::cout
            << "format: " << inctx->iformat->name << "\n"
            << "vcodec: " << vcodec->name << "\n"
            << "size:   " << avctx->width << 'x' << avctx->height << "\n"
            << "fps:    " << av_q2d(avctx->framerate) << " [fps]\n"
            << "length: " << av_rescale_q(vstrm->duration, vstrm->time_base, {1, 1000}) / 1000. << " [sec]\n"
            << "pixfmt: " << av_get_pix_fmt_name(avctx->pix_fmt) << "\n"
            << "frame:  " << vstrm->nb_frames << "\n"
            << std::flush;

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

    return ret;
}


unsigned char *get_image(av_ffmpeg *afp)
{
    int ret;
    const int dst_width = afp->avctx->width;
    const int dst_height = afp->avctx->height;

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

    cv::Mat image(dst_height, dst_width, CV_8UC3, afp->framebuf->data(), afp->frame->linesize[0]);

    av_packet_unref(&afp->pkt);

	// std::cout << afp->nb_frames << " | " << afp->frame->linesize[0] << '\r' << std::flush; // dump progress

	// cv::cvtColor(image, image, CV_RGB2BGR);
    return image.data;
}


void load_undistort(char *filename, int width, int height, cv::Mat &MapX, cv::Mat &MapY)
{
    char label[1024];
    FILE *fp = fopen(filename, "r");
    if (fp == NULL)
	{
        printf("file %s not found!\n", filename);
        exit(1);
	}
    double fx, fy, cu, cv, k1, k2, k3, p1, p2;

    fscanf(fp, "%lf\t#%[^\n]", &fx, label);
    fscanf(fp, "%lf\t#%[^\n]", &fy, label);
    fscanf(fp, "%lf\t#%[^\n]", &cu, label);
    fscanf(fp, "%lf\t#%[^\n]", &cv, label);
    fscanf(fp, "%lf\t#%[^\n]", &k1, label);
    fscanf(fp, "%lf\t#%[^\n]", &k2, label);
    fscanf(fp, "%lf\t#%[^\n]", &k3, label);
    fscanf(fp, "%lf\t#%[^\n]", &p1, label);
    fscanf(fp, "%lf\t#%[^\n]", &p2, label);

    fclose(fp);

    cv::Mat cameraMatrix, distCoeffs, _cameraMatrix, R1;

    cameraMatrix = (cv::Mat_<double>(3, 3) << fx*width, 0, cu*width, 0, fy*height, cv*height, 0, 0, 1);
	_cameraMatrix = (cv::Mat_<double>(3, 3) << fx*width, 0, cu*width, 0, fy*height, cv*height, 0, 0, 1);
	distCoeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);
	R1 = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, R1, _cameraMatrix, cv::Size(width, height), CV_16SC2, MapX, MapY);
}


int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << 
            " rtsp://admin:1q2w3e4r@192.168.1.100:554/cam/realmonitor?channel=1&subtype=0"
            " parameters_file_undistort OPTIONAL" << std::endl;
        return 0;
    }

    char filename[1024];
    av_ffmpeg *afp = (av_ffmpeg *)malloc(sizeof(av_ffmpeg));
    int ret, key, undistorting = 0;;
    unsigned char *raw;
    cv::Mat MapX, MapY;

    char *infile = argv[1]; //"rtsp://admin:1q2w3e4r@192.168.1.111:554/cam/realmonitor?channel=1&subtype=0";
    init(afp, infile);

    if (argc == 3)
    {
        load_undistort(argv[2], afp->dst_width, afp->dst_height, MapX, MapY);
        undistorting = 1;
    }

    do
    {
        raw = get_image(afp);
        cv::Mat image(cv::Size(afp->dst_width, afp->dst_height), CV_8UC3, raw);
        if (undistorting)
            cv::remap(image, image, MapX, MapY, cv::INTER_LINEAR);
        cv::imshow("press ESC to exit", image);
        key = cv::waitKey(1);
        if (key == 27) // escape
            break;
        else if (key == 32) // backspace
        {
            sprintf(filename, "ff2cv_%d.png", afp->nb_frames);
            printf("saving %s\n", filename);
            imwrite(filename, image);
        }
    } while (!afp->end_of_stream);

    std::cout << afp->nb_frames << " frames decoded" << std::endl;

    av_frame_free(&afp->decframe);
    av_frame_free(&afp->frame);
    avcodec_free_context(&afp->avctx);
    avformat_close_input(&afp->inctx);

    free(afp->framebuf);
    free(afp);

    return 0;
}