
#include <iostream>
#include <stdlib.h>
#include <assert.h>
#include <fstream>

#include "libavcodec/avcodec.h"

struct H264CodecParam {
  int pixFormat_;
  int width, height;
  int fps;
  int bit_rate;
};//struct H264CodecParam


class H264CodecImpl {
 public:
  H264CodecImpl() {
    enc_ = nullptr;
    enc_ctx_ = nullptr;
    enc_ = avcodec_find_encoder(AVCodecID::AV_CODEC_ID_H264);
    if (enc_) {
      enc_ctx_ = avcodec_alloc_context3(enc_);
    }
  }
  ~H264CodecImpl() {
    if (enc_ctx_) {
      avcodec_free_context(&enc_ctx_);
    }
    if (enc_) {
      enc_ = nullptr;
    }
  }


  int OpenCodec(const H264CodecParam& param) {
    codec_param_ = param;

    if (enc_ == nullptr || enc_ctx_ == nullptr) {
      return -1;
    }

    enc_ctx_->pix_fmt = Convert2AVPixelFormat(codec_param_.pixFormat_);
    enc_ctx_->width = codec_param_.width;
    enc_ctx_->height = codec_param_.height;
    enc_ctx_->framerate = AVRational{codec_param_.fps, 1};
    enc_ctx_->time_base = AVRational{1, codec_param_.fps};
    enc_ctx_->bit_rate = codec_param_.bit_rate;

    int ret = avcodec_open2(enc_ctx_, enc_, nullptr);
    if (ret < 0) {
      return -2;
    }


  }

  int Encode(unsigned char* buffer, size_t size, unsigned char* encbuffer, size_t* encsize) {
    AVFrame avFrm; AVPacket avPkt;

    memset(&avFrm, 0, sizeof(avFrm));
    av_frame_unref(&avFrm);

    ApplyPixelFormat(&avFrm, buffer, size, codec_param_);

    av_init_packet(&avPkt);
    avPkt.data = encbuffer;
    avPkt.size = (int)encsize;

    


  }

  static AVPixelFormat Convert2AVPixelFormat(int pixFormat) {
    return AVPixelFormat(pixFormat);
  }

  static int ApplyPixelFormat(AVFrame* avFrm, unsigned char* buffer, size_t size, const H264CodecParam &param) {
    avFrm->data[0] = buffer;
    avFrm->linesize[0] = param.height * param.width;

  }

 private:
  const AVCodec* enc_;
  AVCodecContext *enc_ctx_;

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
  inFile.open(srcYuvFilename, std::ios_base::binary);
  assert(inFile.is_open());
  
  const char* outOneFrameYuv = "./YUV420P_1280x720_1.yuv";
  std::ofstream outFile;
  outFile.open(outOneFrameYuv, std::ios_base::binary | std::ios_base::trunc);
  assert(outFile.is_open());

  const char* outh264 = "./YUV420P_1280x720.h264";
  std::ofstream outH264File;
  outH264File.open(outOneFrameYuv, std::ios_base::binary | std::ios_base::trunc);
  assert(outH264File.is_open());

  //1.reigster all codec
  avcodec_register_all();

  H264CodecImpl h264Codec;
  H264CodecParam param;

  //2.open codec
  int ret = h264Codec.OpenCodec(param);
  assert(ret==0);

  size_t encbuffersize = srcFrmBufferSize;
  unsigned char *encbuffer = (unsigned char *)malloc(encbuffersize);
  assert(encbuffer);

  inFile.read((char *)srcFrmBuffer, srcFrmBufferSize);
  if (srcFrmBufferSize == inFile.gcount()) {
	  //valid frame size
	  auto pos = outFile.tellp();
	  outFile.write((char *)srcFrmBuffer, srcFrmBufferSize);
	  assert((outFile.tellp() - pos) == srcFrmBufferSize);
    //3.encode
    h264Codec.Encode(srcFrmBuffer, srcFrmBufferSize, encbuffer, &encbuffersize);

	  pos = outH264File.tellp();
    outH264File.write((char *)encbuffer, encbuffersize);
	  assert((outH264File.tellp() - pos) == encbuffersize);
  }
  else {
	  assert(false);
  }




  return 0;
}

