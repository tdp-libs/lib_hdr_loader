// See the original_src folder for the original code.
//
// Read HDR code is partially derrived from here:
// https://www.flipcode.com/archives/HDR_Image_Reader.shtml
//
// Write HDR code is partially derrived from here:
// https://github.com/Allda/HDRlib/blob/master/HDRlib/rgbe.cpp
// And uses the following license: 
//
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "lib_hdr_loader/HDRLoader.h"

#include <cmath>
#include <cstring>
#include <iostream>

namespace lib_hdr_loader
{

namespace
{
#define MINELEN	     8      // Min scanline length for encoding.
#define MAXELEN	     0x7fff // Max scanline length for encoding.
#define MINRUNLENGTH 4      // Min RLE run length.

//##################################################################################################
bool oldDecrunch(std::istream& hdrStream, uint8_t* scanline, int len)
{
  int rshift = 0;
  while(len > 0)
  {
    scanline[0] = hdrStream.get();
    scanline[1] = hdrStream.get();
    scanline[2] = hdrStream.get();
    scanline[3] = hdrStream.get();

    if(hdrStream.eof())
      return false;

    if(scanline[0] == 1 && scanline[1] == 1 && scanline[2] == 1)
    {
      for(int i = scanline[3] << rshift; i>0; i--)
      {
        std::memcpy(&scanline[0], &scanline[-4], 4);
        scanline+=4;
        len--;
      }
      rshift += 8;
    }
    else
    {
      scanline+=4;
      len--;
      rshift = 0;
    }
  }

  return true;
}

//##################################################################################################
bool decrunch(std::istream& hdrStream, uint8_t* scanline, int len)
{
  if (len < MINELEN || len > MAXELEN)
    return oldDecrunch(hdrStream, scanline, len);

  if(hdrStream.get() != 2)
  {
    hdrStream.seekg(-1, std::ios::cur);
    return oldDecrunch(hdrStream, scanline, len);
  }

  scanline[1] = hdrStream.get();
  scanline[2] = hdrStream.get();
  int i = hdrStream.get();
  if (scanline[1] != 2 || scanline[2] & 128)
  {
    scanline[0] = 2;
    scanline[3] = i;
    return oldDecrunch(hdrStream, scanline + 4, len - 1);
  }

  // read each component
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < len; )
    {
      uint8_t code = hdrStream.get();
      if(code > 128) // run
      {
        code &= 127;
        uint8_t val = hdrStream.get();
        while(code--)
          scanline[(j++)*4+i] = val;
      }
      else
      {
        while(code--)	// non-run
          scanline[(j++)*4+i] = hdrStream.get();
      }
    }
  }

  return !hdrStream.eof();
}

//##################################################################################################
bool rle(std::ostream& hdrStream, const uint8_t* scanline, int len)
{
  auto data = [&](int i)
  {
    return char(scanline[i*4]);
  };

  for(int cur=0; cur<len;)
  {
    int begRun = cur;

    // Look for a run.
    int runCount = 0;
    int oldRunCount = 0;
    while((runCount<MINRUNLENGTH) && (begRun<len))
    {
      begRun += runCount;
      oldRunCount = runCount;
      runCount = 1;
      while(((begRun+runCount) < len) && (runCount<127)  && (data(begRun) == data(begRun + runCount)))
        runCount++;
    }

    // If data before next big run is a short run then write it as such
    if((oldRunCount>1)&&(oldRunCount == begRun - cur))
    {
      //write short run
      hdrStream << char(128 + oldRunCount);
      hdrStream << data(cur);
      cur = begRun;
    }

    //Write out bytes until we reach the start of the next run
    while(cur < begRun)
    {
      int nonrunCount = begRun - cur;
      if (nonrunCount > 128)
        nonrunCount = 128;

      hdrStream << char(nonrunCount);

      {
         const uint8_t* s = scanline+(cur*4);
         const uint8_t* sMax = s+(nonrunCount*4);
         for(; s<sMax; s+=4)
           hdrStream << char(*s);
      }

      cur += nonrunCount;
    }

    // Write out next run if one was found
    if (runCount >= MINRUNLENGTH)
    {
      hdrStream << char(128 + runCount);
      hdrStream << data(begRun);
      cur += runCount;
    }

    if(hdrStream.fail())
      return false;
  }

  return true;
}

//##################################################################################################
bool crunch(std::ostream& hdrStream, const uint8_t* scanline, int len)
{
  hdrStream << char(2) << char(2) << char((len>>8)&0xFF) << char(len&0xFF);
  for(size_t s=0; s<4; s++)
    if(!rle(hdrStream, scanline+s, len))
      return false;
  return true;
}

}

//##################################################################################################
bool loadHDRToRGBE(std::istream& hdrStream, const std::function<uint8_t*(size_t w, size_t h, const HDRHeader&)>& getImageBuffer, std::string& errorMessage)
{
  auto fail = [&](const auto& msg)
  {
    errorMessage = msg;
    return false;
  };

  HDRHeader header;

  // Parse the program type header. Looking for one of the following:
  //   "#?RGBE\n"
  //   "#?RADIANCE\n"
  {
    char str[16];
    for(size_t i=0; ; i++)
    {
      if(i==16)
        return fail("Program type section too long.");

      int c = hdrStream.get();
      if(c == std::istream::traits_type::eof())
        return fail("EOF reached while reading program type.");

      str[i] = c;

      if(c == '\n')
      {
        if(i==6 && std::memcmp(str, "#?RGBE", 6)==0)
        {
          header.programType = "RGBE";
          break;
        }

        if(i==10 && std::memcmp(str, "#?RADIANCE", 10)==0)
        {
          header.programType = "RADIANCE";
          break;
        }

        return fail("Incorrect header.");
      }
    }
  }

  // Read the comment
  for(char oldc=0; ; )
  {
    int c = hdrStream.get();
    if(c == std::istream::traits_type::eof())
      return fail("EOF reached while reading comment.");

    if(c == '\n' && oldc == '\n')
      break;

    oldc = c;
  }

  char reso[200];
  for(size_t i=0; ; i++)
  {
    if(i==200)
      return fail("Resolution section too long.");

    int c = hdrStream.get();
    if(c == std::istream::traits_type::eof())
      return fail("EOF reached while reading resolution.");

    reso[i] = c;

    if(c == '\n')
      break;
  }

  size_t w=0;
  size_t h=0;

  //This is not complete X and Y can be swapped to rotate the image and both can be either + or - to flip.
  if(!std::sscanf(reso, "-Y %zu +X %zu", &h, &w))
    return fail("Failed to parse resolution.");

  if(w<1 || w>16384 || h<1 || h>16384)
    return fail("Invalid resolution.");

  uint8_t* buffer = getImageBuffer(w, h, header);
  if(!buffer)
    return fail("Failed to get image buffer.");

  size_t stride = w*4;
  uint8_t* scanline = buffer;
  uint8_t* scanlineMax = scanline + stride*h;
  for(; scanline<scanlineMax; scanline+=stride)
    if(!decrunch(hdrStream, scanline, w))
      return fail("Decompression failed.");

  return true;
}

//##################################################################################################
bool saveRGBEToHDR(std::ostream& hdrStream, const uint8_t* buffer, size_t w, size_t h, const HDRHeader& header, std::string& errorMessage)
{
  auto fail = [&](const auto& msg)
  {
    errorMessage = msg;
    return false;
  };

  // Write the header out.
  hdrStream << "#?RADIANCE";
  hdrStream << '\n';
  hdrStream << "# " << header.comment << '\n';
  hdrStream << "FORMAT=" << header.format << '\n';
  hdrStream << '\n';
  hdrStream << "-Y " << h << " +X " << w;
  hdrStream << '\n';

  if(hdrStream.fail())
    return fail("Failed to write header.");

  // The scanline length is not compatible with RLE so write without RLE.
  if(w<MINELEN || w>MAXELEN)
  {
    hdrStream.write(reinterpret_cast<const char*>(buffer), size_t(w*h*4));
    if(!hdrStream.bad())
      return fail("Failed to write without RLE.");
    return true;
  }

  // Write out the RLE RGBE data
  size_t stride = w*4;
  const uint8_t* scanline = buffer;
  const uint8_t* scanlineMax = scanline + stride*h;
  for(; scanline<scanlineMax; scanline+=stride)
    if(!crunch(hdrStream, scanline, w))
      return fail("Compression failed.");

  return true;
}

//##################################################################################################
void rgbeToRGBA(const uint8_t* rgbe, float* rgba, size_t w, size_t h)
{
  size_t s = w*h*4;

  const uint8_t* i=rgbe;
  const uint8_t* iMax=i+s;

  float* o=rgba;
  for(; i<iMax; i+=4, o+=4)
  {
    float d = std::pow(2.0f, float(int(i[3]) - 128));
    for(size_t c=0; c<3; c++)
      o[c] = (float(i[c]) /  255.0f) * d; // Some implementations use 256 here but 255 makes more sense to me.
    o[3] = 1.0f;
  }
}

//##################################################################################################
void rgbaToRGBE(const float* rgba, uint8_t* rgbe, size_t w, size_t h)
{
  size_t s = w*h*4;

  const float* i=rgba;
  const float* iMax=i+s;

  uint8_t* o=rgbe;
  for(; i<iMax; i+=4, o+=4)
  {
    if(float v = std::max(i[0], std::max(i[1], i[2])); v < 1e-32f)
    {
      o[0] = 0;
      o[1] = 0;
      o[2] = 0;
      o[3] = 0;
    }
    else
    {
      int e;
      v = static_cast<float>(std::frexp(v,&e) * 256.0f/v);
      o[0] = uint8_t(i[0] * v);
      o[1] = uint8_t(i[1] * v);
      o[2] = uint8_t(i[2] * v);
      o[3] = uint8_t(e + 128);
    }
  }
}

}
