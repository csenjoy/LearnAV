
#include <iostream>
#include <stdlib.h>
#include <assert.h>
#include <fstream>
#include <thread>
#include <chrono>

extern "C" {
#include "libavcodec/avcodec.h"
}

static int sYuvFrmCount = 0;
static int sH24FrmCount = 0;

static std::ofstream* sOut = nullptr;

struct H264CodecParam {
  int pixFormat_;
  int width, height;
  int fps;
  int bit_rate;
};//struct H264CodecParam


class H264CodecImpl {
 public:
  H264CodecImpl() {
    enc_ = avcodec_find_encoder(AVCodecID::AV_CODEC_ID_H264);
    if (enc_) {
      enc_ctx_ = avcodec_alloc_context3(enc_);
    }
    pkt_ = av_packet_alloc();
  }
  ~H264CodecImpl() {
    if (pkt_) av_packet_free(&pkt_);
#if defined(USE_AV_FRAME_GET_BUFFER)
    if (frm_) av_frame_free(&frm_);
#endif
    if (enc_ctx_) {
      avcodec_free_context(&enc_ctx_);
    }
    if (enc_) {
      enc_ = nullptr;
    }
  }


  int OpenCodec(const H264CodecParam& param) {
    int ret = 0;
    codec_param_ = param;

    if (enc_ == nullptr || enc_ctx_ == nullptr || pkt_ == nullptr) {
      return -1;
    }

    enc_ctx_->pix_fmt = Convert2AVPixelFormat(codec_param_.pixFormat_);
    enc_ctx_->width = codec_param_.width;
    enc_ctx_->height = codec_param_.height;
    enc_ctx_->framerate = AVRational{codec_param_.fps, 1};
    enc_ctx_->time_base = AVRational{1, codec_param_.fps};
    enc_ctx_->bit_rate = codec_param_.bit_rate;
    enc_ctx_->max_b_frames = 1;
    enc_ctx_->gop_size = 10;

#if defined(USE_AV_FRAME_GET_BUFFER)
    frm_ = av_frame_alloc();
    if (frm_ == nullptr) return -1;

    frm_->format = enc_ctx_->pix_fmt;
    frm_->width = enc_ctx_->width;
    frm_->height = enc_ctx_->height;
    ret = av_frame_get_buffer(frm_, 0);
    if (ret < 0) {
      return -2;
    }
#endif
    ret = avcodec_open2(enc_ctx_, enc_, nullptr);
    if (ret < 0) {
      return -3;
    }
    return 0;
  }
  /**
  * if return > 0 for has packet received
  */
  int Encode(unsigned char* buffer, size_t size, unsigned char* encbuffer, size_t encsize) {
    AVFrame* pAVFrm = nullptr;
    int ret = 0;
#if defined(USE_AV_FRAME_GET_BUFFER)
    if (buffer) {

      pAVFrm = frm_;
      ret = av_frame_make_writable(pAVFrm);
      if (ret < 0)
        return -1;

     
      unsigned char* pY = buffer;
      unsigned char* pU = pY + enc_ctx_->width * enc_ctx_->height;
      unsigned char* pV = pU + ((enc_ctx_->width * enc_ctx_->height) >> 2);

      for (int y = 0; y < enc_ctx_->height; y++) {
        memcpy(pAVFrm->data[0] + y * pAVFrm->linesize[0], pY + y * enc_ctx_->width, enc_ctx_->width);
      }

      /* Cb and Cr */
      for (int y = 0; y < enc_ctx_->height / 2; y++) {
        memcpy(pAVFrm->data[1] + y * pAVFrm->linesize[1], pU + y * enc_ctx_->width / 2, enc_ctx_->width / 2);
        memcpy(pAVFrm->data[2] + y * pAVFrm->linesize[2], pV + y * enc_ctx_->width / 2, enc_ctx_->width / 2);
      }
    }
#else
    AVFrame avFrm;
    memset(&avFrm, 0, sizeof(avFrm));
    ApplyPixelFormat(&avFrm, buffer, size, codec_param_);
    if (buffer) {
      pAVFrm = &avFrm;
    }
#endif
    ret = avcodec_send_frame(enc_ctx_, pAVFrm);
    while (ret >= 0) {
      ret = avcodec_receive_packet(enc_ctx_, pkt_);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        //none pkt receive 
        return 0;
      }
      else if (ret < 0) {
        //error during encoding
        return -1;
      }

      sH24FrmCount++;
      memcpy(encbuffer, pkt_->data, pkt_->size);
     
      //return avPkt.size;
      if (sOut) sOut->write((char *)pkt_->data, pkt_->size);
      av_packet_unref(pkt_);
    }
    //for error
    return -1;
  }

  int FlushCodec(unsigned char* encbuffer, size_t encsize) {

  }

  static AVPixelFormat Convert2AVPixelFormat(int pixFormat) {
    return AVPixelFormat(pixFormat);
  }

  static void ApplyPixelFormat(AVFrame* avFrm, unsigned char* buffer, size_t size, const H264CodecParam &param) {
    size_t Y_size = param.width * param.height;
    avFrm->data[0] = buffer;
    avFrm->linesize[0] = param.width;

    avFrm->data[1] = buffer + Y_size;
    avFrm->data[2] = avFrm->data[1] + (Y_size >> 2);
    avFrm->linesize[1] = avFrm->linesize[2] = param.width >> 1;
  }

 private:
  const AVCodec* enc_ = nullptr;
  AVCodecContext *enc_ctx_ = nullptr;
#if defined(USE_AV_FRAME_GET_BUFFER)
  AVFrame *frm_ = nullptr;
#endif
  AVPacket* pkt_ = nullptr;

  H264CodecParam codec_param_;
};//class H264Codec


/**
 * read yuv420p file and encode to h264 
*/
int main(int argc, char **argv) {
  //assume this width and height
  int srcFrmWidth = 1280, srcFrmHeight = 720;
  int srcPixFormat = AVPixelFormat::AV_PIX_FMT_YUV420P;

  const char* srcYuvFilename = "./YUV420P_1280x720.yuv";

  int srcFrmBufferSize = srcFrmWidth * srcFrmHeight + srcFrmWidth * srcFrmHeight / 2;
  unsigned char* srcFrmBuffer = (unsigned char *)malloc(srcFrmBufferSize);
  assert(srcFrmBuffer);


  std::ifstream inFile;
  inFile.open(srcYuvFilename, std::ios_base::binary | std::ios_base::in);
  assert(inFile.is_open());
  
  const char* outOneFrameYuv = "./YUV420P_1280x720_1.yuv";
  std::ofstream outFile;
  outFile.open(outOneFrameYuv, std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);
  assert(outFile.is_open());

  const char* outh264 = "./YUV420P_1280x720.h264";
  std::ofstream outH264File;
  outH264File.open(outh264, std::ios_base::binary | std::ios_base::trunc | std::ios_base::out);
  assert(outH264File.is_open());

  //1.reigster all codec
  //avcodec_register_all();

  H264CodecImpl h264Codec;
  H264CodecParam param;
  memset(&param, 0, sizeof(param));
  param.pixFormat_ = AV_PIX_FMT_YUV420P;
  param.width = 1280;
  param.height = 720;
  param.fps = 29;
  param.bit_rate = 117120;

  //2.open codec
  int ret = h264Codec.OpenCodec(param);
  assert(ret==0);

  size_t encbuffersize = srcFrmBufferSize;
  unsigned char *encbuffer = (unsigned char *)malloc(encbuffersize);
  assert(encbuffer);

  sOut = &outH264File;
  int readSize = 0;
  while (!inFile.eof()) {
    inFile.read((char*)srcFrmBuffer, srcFrmBufferSize);
    readSize = inFile.gcount();

    if (srcFrmBufferSize == readSize) {
      sYuvFrmCount++;
      ret = h264Codec.Encode(srcFrmBuffer, srcFrmBufferSize, encbuffer, encbuffersize);
      //_sleep()
      //std::this_thread::sleep_for(std::chrono::milliseconds(10));
/*     if (ret > 0) {
        auto pos = outH264File.tellp();
        outH264File.write((char*)encbuffer, ret);
        assert((outH264File.tellp() - pos) == ret);
      }*/
    }
    else if (readSize > 0) {
      assert(false);
    }
  }


  //flush codec
  ret = h264Codec.Encode(nullptr, 0, encbuffer, encbuffersize);
  if (ret > 0) {
/*    auto pos = outH264File.tellp();
    outH264File.write((char*)encbuffer, ret);
    assert((outH264File.tellp() - pos) == ret);*/
  }


  return 0;
}

