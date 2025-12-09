#include "YUV444toRGBConverter.h"
#include "ConvertTesters.h"


#ifndef BYTE
typedef unsigned char BYTE;
#endif

/*
===========================================================================
	Macros.
===========================================================================
*/
#define RYUVRGB24C_RANGECHECK_0TO255(x) ( (((x) <= 255)&&((x) >= 0))?((x)):( ((x) > 255)?(255):(0) ) )


/*
===========================================================================
	Private Methods.
===========================================================================
*/

//Note: https://topic.alibabacloud.com/a/about-the-common-ayuv-formats-in-rgb-yuy2-yuyv-yvyu-uyvy-directshow-rgbyuv_8_8_10271129.html
// The ¨AYUV format has an alpha channel, and the YUV component is extracted for each pixel, and the image Data format is as follows:
// A0 Y0 U0 V0 A1 Y1 U1 V1 ... 


void OrigRealYUV444toRGB24Converter::Convert(const void* pInputBuffer, void* pRgb)
{
    BYTE *optr	=	(unsigned char *)pRgb;
    BYTE *inBuffer	= (BYTE *)pInputBuffer;

    int numberOfPixels = _width * _height;

    for (int i = 0; i < numberOfPixels; i++ )
    {
        BYTE y = inBuffer[ (i * 4 ) + 1 ];	// JFO: Original byte ordering was wrong, fixed.
        BYTE u = inBuffer[ (i * 4 ) + 2 ];
        BYTE v = inBuffer[ (i * 4 ) + 3 ];
        
        
        //drop the 4th byte (= alpha-channel which is always 0 in AYUV) to go from 32bit to 24 bit per pixel

        int r = y + 1.402 * (v - 128);
        int g = y - 0.344 * (u - 128) - 0.714 * (v - 128);
        int b = y + 1.772 * (u - 128);

        optr[i * 3] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(b);
        optr[(i * 3) + 1] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(g);
        optr[(i *3) + 2] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(r);
    }
}//end Convert



