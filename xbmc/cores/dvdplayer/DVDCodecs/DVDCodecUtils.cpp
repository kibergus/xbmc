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

#include "DVDCodecUtils.h"
#include "DVDClock.h"
#include "cores/VideoRenderers/RenderManager.h"
#include "utils/log.h"
#include "utils/fastmemcpy.h"
#include "DllSwScale.h"

#ifdef CLASSNAME
#undef CLASSNAME
#endif
#define CLASSNAME "CDVDCodecUtils"

// allocate a new picture (PIX_FMT_YUV420P)
DVDVideoPicture* CDVDCodecUtils::AllocatePicture(int iWidth, int iHeight)
{
  DVDVideoPicture* pPicture = new DVDVideoPicture;
  if (pPicture)
  {
    pPicture->iWidth = iWidth;
    pPicture->iHeight = iHeight;

    int w = iWidth / 2;
    int h = iHeight / 2;
    int size = w * h;
    int totalsize = (iWidth * iHeight) + size * 2;
    BYTE* data = new BYTE[totalsize];
    if (data)
    {
      pPicture->data[0] = data;
      pPicture->data[1] = pPicture->data[0] + (iWidth * iHeight);
      pPicture->data[2] = pPicture->data[1] + size;
      pPicture->data[3] = NULL;
      pPicture->iLineSize[0] = iWidth;
      pPicture->iLineSize[1] = w;
      pPicture->iLineSize[2] = w;
      pPicture->iLineSize[3] = 0;
    }
    else
    {
      CLog::Log(LOGFATAL, "CDVDCodecUtils::AllocatePicture, unable to allocate new video picture, out of memory.");
      delete pPicture;
      pPicture = NULL;
    }
  }
  return pPicture;
}

void CDVDCodecUtils::FreePicture(DVDVideoPicture* pPicture)
{
  delete[] pPicture->data[0];
  delete pPicture;
}

bool CDVDCodecUtils::CopyPicture(DVDVideoPicture* pDst, DVDVideoPicture* pSrc)
{
  BYTE *s, *d;
  int w = pSrc->iWidth;
  int h = pSrc->iHeight;

  s = pSrc->data[0];
  d = pDst->data[0];

  for (int y = 0; y < h; y++)
  {
    fast_memcpy(d, s, w);
    s += pSrc->iLineSize[0];
    d += pDst->iLineSize[0];
  }

  w >>= 1;
  h >>= 1;

  s = pSrc->data[1];
  d = pDst->data[1];
  for (int y = 0; y < h; y++)
  {
    fast_memcpy(d, s, w);
    s += pSrc->iLineSize[1];
    d += pDst->iLineSize[1];
  }

  s = pSrc->data[2];
  d = pDst->data[2];
  for (int y = 0; y < h; y++)
  {
    fast_memcpy(d, s, w);
    s += pSrc->iLineSize[2];
    d += pDst->iLineSize[2];
  }
  return true;
}

bool CDVDCodecUtils::CopyPicture(YV12Image* pImage, DVDVideoPicture *pSrc)
{
  BYTE *s = pSrc->data[0];
  BYTE *d = pImage->plane[0];
  int w = pImage->width * pImage->bpp;
  int h = pImage->height;
  if ((w == pSrc->iLineSize[0]) && ((unsigned int) pSrc->iLineSize[0] == pImage->stride[0]))
  {
    fast_memcpy(d, s, w*h);
  }
  else
  {
    for (int y = 0; y < h; y++)
    {
      fast_memcpy(d, s, w);
      s += pSrc->iLineSize[0];
      d += pImage->stride[0];
    }
  }
  s = pSrc->data[1];
  d = pImage->plane[1];
  w =(pImage->width  >> pImage->cshift_x) * pImage->bpp;
  h =(pImage->height >> pImage->cshift_y);
  if ((w==pSrc->iLineSize[1]) && ((unsigned int) pSrc->iLineSize[1]==pImage->stride[1]))
  {
    fast_memcpy(d, s, w*h);
  }
  else
  {
    for (int y = 0; y < h; y++)
    {
      fast_memcpy(d, s, w);
      s += pSrc->iLineSize[1];
      d += pImage->stride[1];
    }
  }
  s = pSrc->data[2];
  d = pImage->plane[2];
  if ((w==pSrc->iLineSize[2]) && ((unsigned int) pSrc->iLineSize[2]==pImage->stride[2]))
  {
    fast_memcpy(d, s, w*h);
  }
  else
  {
    for (int y = 0; y < h; y++)
    {
      fast_memcpy(d, s, w);
      s += pSrc->iLineSize[2];
      d += pImage->stride[2];
    }
  }
  return true;
}

DVDVideoPicture* CDVDCodecUtils::ConvertToNV12Picture(DVDVideoPicture *pSrc)
{
  // Clone a YV12 picture to new NV12 picture.
  DVDVideoPicture* pPicture = new DVDVideoPicture;
  if (pPicture)
  {
    *pPicture = *pSrc;

    int w = pPicture->iWidth / 2;
    int h = pPicture->iHeight / 2;
    int size = w * h;
    int totalsize = (pPicture->iWidth * pPicture->iHeight) + size * 2;
    BYTE* data = new BYTE[totalsize];
    if (data)
    {
      pPicture->data[0] = data;
      pPicture->data[1] = pPicture->data[0] + (pPicture->iWidth * pPicture->iHeight);
      pPicture->data[2] = NULL;
      pPicture->data[3] = NULL;
      pPicture->iLineSize[0] = pPicture->iWidth;
      pPicture->iLineSize[1] = pPicture->iWidth;
      pPicture->iLineSize[2] = 0;
      pPicture->iLineSize[3] = 0;
      pPicture->format = RENDER_FMT_NV12;
      
      // copy luma
      uint8_t *s = pSrc->data[0];
      uint8_t *d = pPicture->data[0];
      for (int y = 0; y < (int)pSrc->iHeight; y++)
      {
        fast_memcpy(d, s, pSrc->iWidth);
        s += pSrc->iLineSize[0];
        d += pPicture->iLineSize[0];
      }

      //copy chroma
      uint8_t *s_u, *s_v, *d_uv;
      for (int y = 0; y < (int)pSrc->iHeight/2; y++) {
        s_u = pSrc->data[1] + (y * pSrc->iLineSize[1]);
        s_v = pSrc->data[2] + (y * pSrc->iLineSize[2]);
        d_uv = pPicture->data[1] + (y * pPicture->iLineSize[1]);
        for (int x = 0; x < (int)pSrc->iWidth/2; x++) {
          *d_uv++ = *s_u++;
          *d_uv++ = *s_v++;
        }
      }
      
    }
    else
    {
      CLog::Log(LOGFATAL, "CDVDCodecUtils::AllocateNV12Picture, unable to allocate new video picture, out of memory.");
      delete pPicture;
      pPicture = NULL;
    }
  }
  return pPicture;
}

DVDVideoPicture* CDVDCodecUtils::ConvertToYUV422PackedPicture(DVDVideoPicture *pSrc, ERenderFormat format)
{
  // Clone a YV12 picture to new YUY2 or UYVY picture.
  DVDVideoPicture* pPicture = new DVDVideoPicture;
  if (pPicture)
  {
    *pPicture = *pSrc;

    int totalsize = pPicture->iWidth * pPicture->iHeight * 2;
    BYTE* data = new BYTE[totalsize];

    if (data)
    {
      pPicture->data[0] = data;
      pPicture->data[1] = NULL;
      pPicture->data[2] = NULL;
      pPicture->data[3] = NULL;
      pPicture->iLineSize[0] = pPicture->iWidth * 2;
      pPicture->iLineSize[1] = 0;
      pPicture->iLineSize[2] = 0;
      pPicture->iLineSize[3] = 0;
      pPicture->format = format;

      //if this is going to be used for anything else than testing the renderer
      //the library should not be loaded on every function call
      DllSwScale  dllSwScale;
      if (!dllSwScale.Load())
      {
        CLog::Log(LOGERROR,"CDVDCodecUtils::ConvertToYUY2Picture - failed to load rescale libraries!");
      }
      else
      {
        // Perform the scaling.
        uint8_t* src[] =       { pSrc->data[0],          pSrc->data[1],      pSrc->data[2],      NULL };
        int      srcStride[] = { pSrc->iLineSize[0],     pSrc->iLineSize[1], pSrc->iLineSize[2], 0    };
        uint8_t* dst[] =       { pPicture->data[0],      NULL,               NULL,               NULL };
        int      dstStride[] = { pPicture->iLineSize[0], 0,                  0,                  0    };

        int dstformat;
        if (format == RENDER_FMT_UYVY422)
          dstformat = PIX_FMT_UYVY422;
        else
          dstformat = PIX_FMT_YUYV422;

        struct SwsContext *ctx = dllSwScale.sws_getContext(pSrc->iWidth, pSrc->iHeight, PIX_FMT_YUV420P,
                                                           pPicture->iWidth, pPicture->iHeight, dstformat,
                                                           SWS_FAST_BILINEAR | SwScaleCPUFlags(), NULL, NULL, NULL);
        dllSwScale.sws_scale(ctx, src, srcStride, 0, pSrc->iHeight, dst, dstStride);
        dllSwScale.sws_freeContext(ctx);
      }
    }
    else
    {
      CLog::Log(LOGFATAL, "CDVDCodecUtils::ConvertToYUY2Picture, unable to allocate new video picture, out of memory.");
      delete pPicture;
      pPicture = NULL;
    }
  }
  return pPicture;
}

bool CDVDCodecUtils::CopyNV12Picture(YV12Image* pImage, DVDVideoPicture *pSrc)
{
  BYTE *s = pSrc->data[0];
  BYTE *d = pImage->plane[0];
  int w = pSrc->iWidth;
  int h = pSrc->iHeight;
  // Copy Y
  if ((w == pSrc->iLineSize[0]) && ((unsigned int) pSrc->iLineSize[0] == pImage->stride[0]))
  {
    fast_memcpy(d, s, w*h);
  }
  else
  {
    for (int y = 0; y < h; y++)
    {
      fast_memcpy(d, s, w);
      s += pSrc->iLineSize[0];
      d += pImage->stride[0];
    }
  }
  
  s = pSrc->data[1];
  d = pImage->plane[1];
  w = pSrc->iWidth;
  h = pSrc->iHeight >> 1;
  // Copy packed UV (width is same as for Y as it's both U and V components)
  if ((w==pSrc->iLineSize[1]) && ((unsigned int) pSrc->iLineSize[1]==pImage->stride[1]))
  {
    fast_memcpy(d, s, w*h);
  }
  else
  {
    for (int y = 0; y < h; y++)
    {
      fast_memcpy(d, s, w);
      s += pSrc->iLineSize[1];
      d += pImage->stride[1];
    }
  }

  return true;
}

bool CDVDCodecUtils::CopyYUV422PackedPicture(YV12Image* pImage, DVDVideoPicture *pSrc)
{
  BYTE *s = pSrc->data[0];
  BYTE *d = pImage->plane[0];
  int w = pSrc->iWidth;
  int h = pSrc->iHeight;

  // Copy YUYV
  if ((w * 2 == pSrc->iLineSize[0]) && ((unsigned int) pSrc->iLineSize[0] == pImage->stride[0]))
  {
    fast_memcpy(d, s, w*h*2);
  }
  else
  {
    for (int y = 0; y < h; y++)
    {
      fast_memcpy(d, s, w*2);
      s += pSrc->iLineSize[0];
      d += pImage->stride[0];
    }
  }
  
  return true;
}

bool CDVDCodecUtils::CopyNV12MTPicture(YV12Image* pImage, DVDVideoPicture *pSrc)
{
#define NV12MT_HALIGN 64
#define NV12MT_VALIGN 32
#define INT_ROUND(x, y) ((x % y) > 0 ? (int(x/y)+1) : (int(x/y)) )

  unsigned int time = XbmcThreads::SystemClockMillis();
  
  BYTE *s = pSrc->data[0];
  BYTE *d = pImage->plane[0];
  int w = INT_ROUND(pSrc->iWidth, 128) * 128;
  int h = INT_ROUND(pSrc->iHeight, 32) * 32;

  int b;
  BYTE *s_offset, *d_offset;
  int dbx, dby;
  int num_bytes, num_lines;
  int nb_xb = INT_ROUND(w, NV12MT_HALIGN);
  int nb_yb = INT_ROUND(h, NV12MT_VALIGN);
  int nb_xmb = nb_xb / 2;
  int nb_ymb = nb_yb / 2;

  printf("NV12MT Y: nb_xb:%d nb_yb:%d \n", nb_xb, nb_yb);
  
  // Copy Y
  int ymb=0;
  for (; ymb<nb_ymb; ymb++)
  {
    bool zed = true;

    for (int xmb=0; xmb<nb_xmb; xmb++)
    {
      for (int yb=0; yb<2; yb++)
      {
        dby = (ymb*2) + yb;
        num_lines = ((dby+1)*NV12MT_VALIGN) < pSrc->iHeight ? NV12MT_VALIGN : pSrc->iHeight - (dby*NV12MT_VALIGN);

        for (int xb=0; xb<2; xb++)
        {
          b = (ymb*2*nb_xb) + (xmb*4) + (zed ? (yb*2) : ((1-yb)*2)) + xb;
          s_offset = s + (b * NV12MT_HALIGN * NV12MT_VALIGN);
          dbx = (xmb*2) + xb;
          d_offset = d + (dby*NV12MT_VALIGN * pImage->stride[0]) + (dbx*NV12MT_HALIGN);
          num_bytes = ((dbx+1)*NV12MT_HALIGN) < pSrc->iWidth ? NV12MT_HALIGN : pSrc->iWidth - (dbx*NV12MT_HALIGN);
          // printf("NV12MT: b:%d dbx:%d dby:%d s_offset:%d d_offset:%d\n", b, dbx, dby, s_offset, d_offset);

          for (int y=0; y<num_lines; y++)
          {
            fast_memcpy(d_offset, s_offset, num_bytes);
            s_offset += NV12MT_HALIGN;
            d_offset += pImage->stride[0];
          }
        }
      }

      zed = !zed;
    }
  }
  if (ymb*2 < nb_yb)
  {
      dby = nb_yb-1;
      num_lines = ((dby+1)*NV12MT_VALIGN) < pSrc->iHeight ? NV12MT_VALIGN : pSrc->iHeight - (dby*NV12MT_VALIGN);

      for (int xb=0; xb<nb_xb; xb++)
      {
          b = ((nb_yb-1)*nb_xb) + xb;
          s_offset = s + (b * NV12MT_HALIGN * NV12MT_VALIGN);
          dbx = xb;
          d_offset = d + (dby*NV12MT_VALIGN * pImage->stride[0]) + (dbx*NV12MT_HALIGN);
          num_bytes = ((dbx+1)*NV12MT_HALIGN) < pSrc->iWidth ? NV12MT_HALIGN : pSrc->iWidth - (dbx*NV12MT_HALIGN);
          // printf("NV12MT: b:%d dbx:%d dby:%d s_offset:%d d_offset:%d\n", b, dbx, dby, s_offset, d_offset);

          for (int y=0; y<num_lines; y++)
          {
              fast_memcpy(d_offset, s_offset, num_bytes);
              s_offset += NV12MT_HALIGN;
              d_offset += pImage->stride[0];
          }
      }
  }

  unsigned int ytime = XbmcThreads::SystemClockMillis() - time;
  
  s = pSrc->data[1];
  BYTE* du = pImage->plane[1];
  BYTE* dv = pImage->plane[2];
  w = INT_ROUND(pSrc->iWidth, 128) * 128;
  h = INT_ROUND(pSrc->iHeight / 2, 32) * 32;

  nb_xb = INT_ROUND(w, NV12MT_HALIGN);
  nb_yb = INT_ROUND(h, NV12MT_VALIGN);
  nb_xmb = nb_xb / 2;
  nb_ymb = nb_yb / 2;

  printf("NV12MT UV: nb_xb:%d nb_yb:%d \n", nb_xb, nb_yb);
  
  BYTE *du_offset, *dv_offset;
  int doff;

  // Copy packed UV (width is same as for Y as it's both U and V components)
  ymb=0;
  for (; ymb<nb_ymb; ymb++)
  {
    bool zed = true;

    for (int xmb=0; xmb<nb_xmb; xmb++)
    {
      for (int yb=0; yb<2; yb++)
      {
        dby = (ymb*2) + yb;
        num_lines = ((dby+1)*NV12MT_VALIGN) < (pSrc->iHeight>>1) ? NV12MT_VALIGN : (pSrc->iHeight>>1) - (dby*NV12MT_VALIGN);

        for (int xb=0; xb<2; xb++)
        {
          b = (ymb*2*nb_xb) + (xmb*4) + (zed ? (yb*2) : ((1-yb)*2)) + xb;
          s_offset = s + (b * NV12MT_HALIGN * NV12MT_VALIGN);
          dbx = (xmb*2) + xb;
          doff = (dby*NV12MT_VALIGN * pImage->stride[1]) + (dbx*NV12MT_HALIGN>>1);
          du_offset = du + doff;
          dv_offset = dv + doff;
          num_bytes = ((dbx+1)*NV12MT_HALIGN) < pSrc->iWidth ? NV12MT_HALIGN : (pSrc->iWidth) - (dbx*NV12MT_HALIGN);
          //printf("NV12MT: b:%d dbx:%d dby:%d s_offset:%d d_offset:%d num_lines:%d num_bytes:%d\n", b, dbx, dby, s_offset, d_offset, num_lines, num_bytes);

          for (int y=0; y<num_lines; y++)
          {
            for (int x=0; x<num_bytes; x+=2)
            {
              *(du_offset + (x>>1)) = *(s_offset + x);
              *(dv_offset + (x>>1)) = *(s_offset + x+1);
            }
            s_offset += NV12MT_HALIGN;
            du_offset += pImage->stride[1];
            dv_offset += pImage->stride[1];
          }
        }
      }

      zed = !zed;
    }
  }
  if (ymb*2 < nb_yb)
  {
      dby = nb_yb-1;
      num_lines = ((dby+1)*NV12MT_VALIGN) < (pSrc->iHeight>>1) ? NV12MT_VALIGN : (pSrc->iHeight>>1) - (dby*NV12MT_VALIGN);

      for (int xb=0; xb<nb_xb; xb++)
      {
          b = ((nb_yb-1)*nb_xb) + xb;
          s_offset = s + (b * NV12MT_HALIGN * NV12MT_VALIGN);
          dbx = xb;
          doff = (dby*NV12MT_VALIGN * pImage->stride[1]) + (dbx*NV12MT_HALIGN>>1);
          du_offset = du + doff;
          dv_offset = dv + doff;
          num_bytes = ((dbx+1)*NV12MT_HALIGN) < pSrc->iWidth ? NV12MT_HALIGN : (pSrc->iWidth) - (dbx*NV12MT_HALIGN);
          // printf("NV12MT: b:%d dbx:%d dby:%d s_offset:%d d_offset:%d\n", b, dbx, dby, s_offset, d_offset);

          for (int y=0; y<num_lines; y++)
          {
              for (int x=0; x<num_bytes; x+=2)
              {
                *(du_offset + (x>>1)) = *(s_offset + x);
                *(dv_offset + (x>>1)) = *(s_offset + x+1);
              }
              s_offset += NV12MT_HALIGN;
              du_offset += pImage->stride[1];
              dv_offset += pImage->stride[1];
          }
      }
  }

  printf("%s::%s NV12MT tm:%d/%d\n", CLASSNAME, __func__, ytime, XbmcThreads::SystemClockMillis() - time);
  
  return true;
}

bool CDVDCodecUtils::CopyDXVA2Picture(YV12Image* pImage, DVDVideoPicture *pSrc)
{
#ifdef HAS_DX
  // TODO: Optimize this later using shaders/swscale/etc.
  switch (pSrc->extended_format)
  {
    case MAKEFOURCC('N','V','1','2'):
      {
        IDirect3DSurface9* surface = (IDirect3DSurface9*)pSrc->data[3];

        D3DLOCKED_RECT rectangle;
        if (FAILED(surface->LockRect(&rectangle, NULL, 0)))
          return false;

        // Copy Y
        uint8_t* bits = (uint8_t*)(rectangle.pBits);
        uint8_t* d = pImage->plane[0];
        for (unsigned y = 0; y < pSrc->iHeight; y++)
        {
          memcpy(d, bits, pSrc->iWidth);
          bits += rectangle.Pitch;
          d += pImage->stride[0];
        }

        D3DSURFACE_DESC desc;
        if (FAILED(surface->GetDesc(&desc)))
          return false;
        
        // Copy packed UV
        uint8_t *s_uv = ((uint8_t*)(rectangle.pBits)) + desc.Height * rectangle.Pitch;
        uint8_t *d_uv = pImage->plane[1];
        for (unsigned y = 0; y < pSrc->iHeight >> 1; y++)
        {
          memcpy(d_uv, s_uv, pSrc->iWidth);
          s_uv += rectangle.Pitch;
          d_uv += pImage->stride[1];
        }

        if (FAILED(surface->UnlockRect()))
          return false;
      }
      return true;

    // Future...
    /*case MAKEFOURCC('Y','V','1','2'):
      return true;*/

    /*case MAKEFOURCC('Y','V','V','Y'):
      return true;*/

    default:
      CLog::Log(LOGWARNING, "CDVDCodecUtils::CopyDXVA2Picture colorspace not supported");
      return false;
  }
#endif
  return false;
}

bool CDVDCodecUtils::IsVP3CompatibleWidth(int width)
{
  // known hardware limitation of purevideo 3 (VP3). (the Nvidia 9400 is a purevideo 3 chip)
  // from nvidia's linux vdpau README: All current third generation PureVideo hardware
  // (G98, MCP77, MCP78, MCP79, MCP7A) cannot decode H.264 for the following horizontal resolutions:
  // 769-784, 849-864, 929-944, 1009–1024, 1793–1808, 1873–1888, 1953–1968 and 2033-2048 pixel.
  // This relates to the following macroblock sizes.
  int unsupported[] = {49, 54, 59, 64, 113, 118, 123, 128};
  for (unsigned int i = 0; i < sizeof(unsupported) / sizeof(int); i++)
  {
    if (unsupported[i] == (width + 15) / 16)
      return false;
  }
  return true;
}

double CDVDCodecUtils::NormalizeFrameduration(double frameduration)
{
  //if the duration is within 20 microseconds of a common duration, use that
  const double durations[] = {DVD_TIME_BASE * 1.001 / 24.0, DVD_TIME_BASE / 24.0, DVD_TIME_BASE / 25.0,
                              DVD_TIME_BASE * 1.001 / 30.0, DVD_TIME_BASE / 30.0, DVD_TIME_BASE / 50.0,
                              DVD_TIME_BASE * 1.001 / 60.0, DVD_TIME_BASE / 60.0};

  double lowestdiff = DVD_TIME_BASE;
  int    selected   = -1;
  for (size_t i = 0; i < sizeof(durations) / sizeof(durations[0]); i++)
  {
    double diff = fabs(frameduration - durations[i]);
    if (diff < DVD_MSEC_TO_TIME(0.02) && diff < lowestdiff)
    {
      selected = i;
      lowestdiff = diff;
    }
  }

  if (selected != -1)
    return durations[selected];
  else
    return frameduration;
}

struct EFormatMap {
  PixelFormat   pix_fmt;
  ERenderFormat format;
};

static const EFormatMap g_format_map[] = {
   { PIX_FMT_YUV420P,     RENDER_FMT_YUV420P    }
,  { PIX_FMT_YUVJ420P,    RENDER_FMT_YUV420P    }
,  { PIX_FMT_YUV420P10,   RENDER_FMT_YUV420P10  }
,  { PIX_FMT_YUV420P16,   RENDER_FMT_YUV420P16  }
,  { PIX_FMT_UYVY422,     RENDER_FMT_UYVY422    }
,  { PIX_FMT_YUYV422,     RENDER_FMT_YUYV422    }
,  { PIX_FMT_VAAPI_VLD,   RENDER_FMT_VAAPI      }
,  { PIX_FMT_DXVA2_VLD,   RENDER_FMT_DXVA       }
,  { PIX_FMT_NONE     ,   RENDER_FMT_NONE       }
};

ERenderFormat CDVDCodecUtils::EFormatFromPixfmt(int fmt)
{
  for(const EFormatMap *p = g_format_map; p->pix_fmt != PIX_FMT_NONE; ++p)
  {
    if(p->pix_fmt == fmt)
      return p->format;
  }
  return RENDER_FMT_NONE;
}

int CDVDCodecUtils::PixfmtFromEFormat(ERenderFormat fmt)
{
  for(const EFormatMap *p = g_format_map; p->pix_fmt != PIX_FMT_NONE; ++p)
  {
    if(p->format == fmt)
      return p->pix_fmt;
  }
  return PIX_FMT_NONE;
}
