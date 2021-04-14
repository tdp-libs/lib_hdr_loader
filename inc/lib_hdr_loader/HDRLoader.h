#ifndef lib_hdr_loader_HDRLoader_h
#define lib_hdr_loader_HDRLoader_h

#include <sstream>
#include <functional>

namespace lib_hdr_loader
{

//##################################################################################################
//!
/*!
References:
 * http://paulbourke.net/dataformats/pic/
 * https://github.com/imbcmdth/hdr/blob/master/lib/hdr-load.js
 * https://github.com/Allda/HDRlib/blob/master/HDRlib/rgbe.h
*/
struct HDRHeader
{
  std::string programType{"RADIANCE"};   //!< Header at start of file RGBE or RADIANCE.
  std::string comment;                   //!< Comment to go in header.
  std::string format{"32-bit_rle_rgbe"}; //!< FORMAT    = Data format "32-bit_rle_rgbe" or "32-bit_rle_xyze".
  float gamma{1.0f};                     //!< GAMMA     = Gamma correction value.
  float exposure{1.0f};                  //!< EXPOSURE  = watts/steradian/m^2.
  float colorCorr[3]{1.0f, 1.0f, 1.0f};  //!< COLORCORR = Color correction scaling for each component.
  float pixelAspect{1.0f};               //!< PIXASPECT = Ratio of the height to width of the pixel.

  //! The CIE (x,y) chromaticity coordinates of the three (RGB) primaries and the white point.
  float primaries[8]{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
};

//##################################################################################################
bool loadHDRToRGBE(std::istream& hdrStream, const std::function<uint8_t*(size_t w, size_t h, const HDRHeader&)>& getImageBuffer, std::string& errorMessage);

//##################################################################################################
bool saveRGBEToHDR(std::ostream& hdrStream, const uint8_t* buffer, size_t w, size_t h, const HDRHeader& header, std::string& errorMessage);

//##################################################################################################
void rgbeToRGBA(const uint8_t* rgbe, float* rgba, size_t w, size_t h);

//##################################################################################################
void rgbaToRGBE(const float* rgba, uint8_t* rgbe, size_t w, size_t h);

}

#endif
