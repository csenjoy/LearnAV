#ifndef BASE_FORMAT_H_
#define BASE_FORMAT_H_

#include <stdint.h>

namespace demo {

//Describe the raw video frame 

struct VideoFrameFormat {
  uint32_t format_id_;

  unsigned width_, height_;
  unsigned fps_;
  
};//class Video Frame Format




struct VideoFormat {

};//struct video format

struct VideoFrame {
 unsigned char *buffer_;
 unsigned long length_;
 unsigned long long timestamp_;
};//class VideoFrame

}

#endif