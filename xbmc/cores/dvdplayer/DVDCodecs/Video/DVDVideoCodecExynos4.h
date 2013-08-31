#pragma once

/*
 *      Copyright (C) 2005-2012 Team XBMC
 *      http://www.xbmc.org
 *
 *  This Program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  This Program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with XBMC; see the file COPYING.  If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "DVDVideoCodec.h"
#include "DVDResource.h"
#include "utils/BitstreamConverter.h"
#include "xbmc/linux/LinuxV4l2.h"
#include <string>
#include <queue>
#include <list>
#include "guilib/GraphicContext.h"

#define STREAM_BUFFER_SIZE        512000 //compressed frame size. 1080p mpeg4 10Mb/s can be >256k in size, so this is to make sure frame fits into buffer
#define FIMC_TO_VIDEO_BUFFERS_CNT 3 //2 begins to be slow. maybe on video only, but not on convert.
#define MFC_OUTPUT_BUFFERS_CNT    2 //1 doesn't work at all
#define CAPTURE_EXTRA_BUFFER_CNT  2 //these are extra buffers, better keep their count as big as going to be simultaneous dequeued buffers number

#define INT_ROUND(x, y) ((x % y) > 0 ? (int(x/y)+1)*y : (int(x/y))*y )

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MFCBuffer 
{
  int index;
  V4L2Buffer m_v4l2Buffer;
} MFCBuffer;

#ifdef __cplusplus
}
#endif

class CDVDVideoCodecExynos4 : public CDVDVideoCodec
{
public:
  CDVDVideoCodecExynos4();
  virtual ~CDVDVideoCodecExynos4();
  virtual bool Open(CDVDStreamInfo &hints, CDVDCodecOptions &options);
  virtual void Dispose();
  virtual int Decode(BYTE* pData, int iSize, double dts, double pts);
  virtual void Reset();
  bool GetPictureCommon(DVDVideoPicture* pDvdVideoPicture);
  virtual bool GetPicture(DVDVideoPicture* pDvdVideoPicture);
  virtual bool ClearPicture(DVDVideoPicture* pDvdVideoPicture);
  virtual void SetDropState(bool bDrop);
  virtual const char* GetName() { return m_name.c_str(); }; // m_name is never changed after open

protected:
  std::string m_name;
  unsigned int m_iDecodedWidth;
  unsigned int m_iDecodedHeight;
  unsigned int m_iConvertedWidth;
  unsigned int m_iConvertedHeight;
  int m_iDecoderHandle;
  int m_iConverterHandle;

  int m_MFCOutputBuffersCount;
  int m_MFCCaptureBuffersCount;
  int m_FIMCOutputBuffersCount;
  int m_FIMCCaptureBuffersCount;

  int m_iMFCCapturePlane1Size;
  int m_iMFCCapturePlane2Size;
  int m_iFIMCCapturePlane1Size;
  int m_iFIMCCapturePlane2Size;
  int m_iFIMCCapturePlane3Size;

  V4L2Buffer *m_v4l2MFCOutputBuffers;
  V4L2Buffer *m_v4l2MFCCaptureBuffers;
  V4L2Buffer *m_v4l2FIMCOutputBuffers;
  V4L2Buffer *m_v4l2FIMCCaptureBuffers;
  
  int m_iFIMCdequeuedBufferNumber;
  
  bool m_bVideoConvert;

  CBitstreamConverter m_converter;

  std::queue<double> m_pts;
  std::queue<double> m_dts;
  std::queue<int> m_index;

  bool m_bDropPictures;

  DVDVideoPicture   m_videoBuffer;
  bool m_bFIMCStartConverter;

  bool OpenDevices();
};

inline int align(int v, int a) {
  return ((v + a - 1) / a) * a;
}

#define err(msg, ...) \
  fprintf(stderr, "Error (%s:%s:%d): " msg "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#define dbg(msg, ...) \
// fprintf(stdout, "(%s:%s:%d): " msg "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)
#define msg(msg, ...) \
//  fprintf(stdout, "(%s:%s:%d): " msg "\n", __FILE__, __func__, __LINE__, ##__VA_ARGS__)

#define memzero(x) memset(&(x), 0, sizeof (x))
