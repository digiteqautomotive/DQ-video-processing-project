#include <stdlib.h>
#include "YUVtoRGBConverter.h"


YUVtoRGBConverter::YUVtoRGBConverter(int width, int height):
	    //_rotate(0),
	    _flip(false), _width(labs(width)), _height(labs(height))
{}


void YUVtoRGBConverter::SetDimensions(int width, int height)
{
  _width = labs(width);
  _height = labs(height);
}


void YUV420toRGBConverter::Convert(const void* pBufferIn, void* pRgb) const 
{    
	//set the Y, U and V pointers to point to the correct positions within the byte array
  int nInPixelCount = _width *_height;
        //const yuvType *pY = (const yuvType *)pBufferIn;
  const yuvType *pU = (const yuvType *)pBufferIn + nInPixelCount;
  const yuvType *pV = pU + nInPixelCount/4;
        // let invert take precedence, not catering for both
  if(_flip) 
      FlipConvert(pBufferIn, pU, pV, (unsigned char*)pRgb);
  else				
  {
    //if(_rotate)
    // 	RotateConvert(pBufferIn, pU, pV, pRgb);
    //else
    	NonRotateConvert(pBufferIn, pU, pV, (unsigned char*)pRgb);
  }
}  
