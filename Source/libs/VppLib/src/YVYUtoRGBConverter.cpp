/** @file

MODULE		: YVYUtoRGB24Converter

FILE NAME	: YVYUtoRGB24Converter.cpp

DESCRIPTION	: Implementation of YUV420 (8 bpp) to RGB 24 bit 
		  colour conversion derived from the YUV420toRGBConverter base class.
					  
LICENSE: Software License Agreement (BSD License)

		Copyright (c) 2025, Digiteq Automotive All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

===========================================================================*/
#include "YVYUtoRGBConverter.h"

#include "typedfs.h"


/*===========================================================================
			Private Methods
===========================================================================*/


void YVYUtoRGB24Converter::Convert(const void* pInputBuffer, void* pRgb) const
{
unsigned char * Out = (unsigned char *)pRgb;
const unsigned char * In = (const unsigned char *)pInputBuffer;

int32_t X;

  if(_flip)
  {
    unsigned y = _height;
    Out += _width*(y-1)*3;
    while(y-- > 0)
    {
      unsigned x = (_width+1) / 2;
      while(x-- > 0)
      {
        int32_t Y = 256 * In[0];
        const signed char V = In[1] - 128;
        const signed char U = In[3] - 128; 
    
        X = (Y + 520*U) >> 8;		// B: Y + 2.032*U
        if(X>255) Out[0]=255;
        else if(X<0) Out[0]=0;
        else Out[0] = X;
        X = (Y -101*U - 149*V) >> 8;	// G: Y - 0.395*U - 0.581*V
        if(X>255) Out[1]=255;
        else if(X<0) Out[1]=0;
        else Out[1] = X;
        X = (Y + 292*V) >> 8;		// R: Y + 1.40*V;
        if(X>255) Out[2]=255;
        else if(X<0) Out[2]=0;
        else Out[2] = X;

        Y = 256 * In[2];

        X = (Y + 520*U) >> 8;		// B: Y + 2.032*U
        if(X>255) Out[3]=255;
        else if(X<0) Out[3]=0;
        else Out[3] = X;
        X = (Y -101*U - 149*V) >> 8;	// G: Y - 0.395*U - 0.581*V
        if(X>255) Out[4]=255;
        else if(X<0) Out[4]=0;
        else Out[4] = X;
        X = (Y + 292*V) >> 8;		// R: Y + 1.40*V;
        if(X>255) Out[5]=255;
        else if(X<0) Out[5]=0;
        else Out[5] = X;

        Out+=6;
        In+=4;
      }
      Out -= _width*2*3;
    }
  }
  else
  {
    int numberOf2Pixels = _height * ((_width+1)/2);
    while(numberOf2Pixels-->0)
    {
      int32_t Y = 256 * In[0];
      const signed char V = In[1] - 128; 
      const signed char U = In[3] - 128;
    
      X = (Y + 520*U) >> 8;		// B: Y + 2.032*U
      if(X>255) Out[0]=255;
      else if(X<0) Out[0]=0;
      else Out[0] = X;
      X = (Y -101*U - 149*V) >> 8;	// G: Y - 0.395*U - 0.581*V
      if(X>255) Out[1]=255;
      else if(X<0) Out[1]=0;
      else Out[1] = X;
      X = (Y + 292*V) >> 8;		// R: Y + 1.40*V;
      if(X>255) Out[2]=255;
      else if(X<0) Out[2]=0;
      else Out[2] = X;

      Y = 256 * In[2];

      X = (Y + 520*U) >> 8;		// B: Y + 2.032*U
      if(X>255) Out[3]=255;
      else if(X<0) Out[3]=0;
      else Out[3] = X;
      X = (Y -101*U - 149*V) >> 8;	// G: Y - 0.395*U - 0.581*V
      if(X>255) Out[4]=255;
      else if(X<0) Out[4]=0;
      else Out[4] = X;
      X = (Y + 292*V) >> 8;		// R: Y + 1.40*V;
      if(X>255) Out[5]=255;
      else if(X<0) Out[5]=0;
      else Out[5] = X;

      Out += 6;
      In += 4;
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////


void YVYUtoARGB32Converter::Convert(const void* pInputBuffer, void* pRgb) const
{
unsigned char * Out = (unsigned char *)pRgb;
const unsigned char * In = (const unsigned char *)pInputBuffer;

int32_t X;

  if(_flip)
  {
    unsigned y = (_height);
    Out += (_width)*(y-1)*4;
    while(y-- > 0)
    {
      unsigned x = (_width+1)/2;
      while(x-- > 0)
      {
        int32_t Y = 256 * In[0];
        const signed char U = In[1] - 128; 
        const signed char V = In[3] - 128;
    
        X = (Y + 454*U) >> 8;	// B Y + 1.772	Y + 2.034*U;
        if(X>255) Out[0]=255;
        else if(X<0) Out[0]=0;
        else Out[0] = X;
        X = (Y -88*U - 183*V) >> 8; // G Y - 0.344*U - 0.714 *V;   Y - 0.397*U - 0.58*V;
        if(X>255) Out[1]=255;
        else if(X<0) Out[1]=0;
        else Out[1] = X;
        X = (Y + 359*V) >> 8;	// R Y + 1.402*V;   Y + 1.137*V;
        if(X>255) Out[2]=255;
        else if(X<0) Out[2]=0;
        else Out[2] = X;
	Out[3] = 0;		// A

	Y = 256 * In[2];

        X = (Y + 454*U) >> 8;	// B Y + 1.772	Y + 2.034*U;
        if(X>255) Out[4]=255;
        else if(X<0) Out[4]=0;
        else Out[4] = X;
        X = (Y -88*U - 183*V) >> 8; // G Y - 0.344*U - 0.714 *V;   Y - 0.397*U - 0.58*V;
        if(X>255) Out[5]=255;
        else if(X<0) Out[5]=0;
        else Out[5] = X;
        X = (Y + 359*V) >> 8;	// R Y + 1.402*V;   Y + 1.137*V;
        if(X>255) Out[6]=255;
        else if(X<0) Out[6]=0;
        else Out[6] = X;
        Out[7] = 0;		// A

        Out+=8;
        In+=4;
      }
      Out -= (_width)*2*4;
    }
  }
  else
  {
    int numberOf2Pixels = ((_width+1)/2) * _height;
    while(numberOf2Pixels-->0)
    {
      int32_t Y = 256 * In[0];
      const signed char U = In[1] - 128; 
      const signed char V = In[3] - 128;
    
      X = (Y + 454*U) >> 8;	// B Y + 1.772	Y + 2.034*U;
      if(X>255) Out[0]=255;
      else if(X<0) Out[0]=0;
      else Out[0] = X;
      X = (Y -88*U - 183*V) >> 8; // G Y - 0.344*U - 0.714 *V;   Y - 0.397*U - 0.58*V;
      if(X>255) Out[1]=255;
      else if(X<0) Out[1]=0;
      else Out[1] = X;
      X = (Y + 359*V) >> 8;	// R Y + 1.402*V;   Y + 1.137*V;
      if(X>255) Out[2]=255;
      else if(X<0) Out[2]=0;
      else Out[2] = X;
      Out[3] = 0;		// A

      Y = 256 * In[2];

      X = (Y + 454*U) >> 8;	// B Y + 1.772	Y + 2.034*U;
      if(X>255) Out[4]=255;
      else if(X<0) Out[4]=0;
      else Out[4] = X;
      X = (Y -88*U - 183*V) >> 8; // G Y - 0.344*U - 0.714 *V;   Y - 0.397*U - 0.58*V;
      if(X>255) Out[5]=255;
      else if(X<0) Out[5]=0;
      else Out[5] = X;
      X = (Y + 359*V) >> 8;	// R Y + 1.402*V;   Y + 1.137*V;
      if(X>255) Out[6]=255;
      else if(X<0) Out[6]=0;
      else Out[6] = X;
      Out[7] = 0;		// A

      Out+=8;
      In+=4;
    }
  }
}
