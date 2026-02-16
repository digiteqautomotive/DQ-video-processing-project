#include "YUVtoRGBConverter.h"
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
https://www.computerlanguage.com/results.php?definition=YUV%2FRGB+conversion+formulas From YUV to RGB
   R = Y + 1.140V
   G = Y - 0.395U - 0.581V
   B = Y + 2.032U
https://www.pcmag.com/encyclopedia/term/yuvrgb-conversion-formulas
From YUV to RGB
   R = Y + 1.140V
   G = Y - 0.395U - 0.581V
   B = Y + 2.032U

This formula is somehow different, but it uses U' and V':
https://colorfaq.com/article/how-to-correctly-convert-yuv-to-rgb-3ZwZTlC1n
   B = 1.164(Y - 16)  + 2.018(U - 128)
   G = 1.164(Y - 16) - 0.813(V - 128) - 0.391(U - 128)
   R = 1.164(Y - 16) + 1.596(V - 128)
 ===========
   R = Y + 1.403V'
   G = Y - 0.344U' - 0.714V'
   B = Y + 1.770U'
*/

#define RYUVRGB24C_U0		 2.032		// B = Y + 2.032U
#define RYUVRGB24C_U1		-0.395
#define RYUVRGB24C_V1		-0.581		// G = Y - 0.395U - 0.581V
#define RYUVRGB24C_V0		 1.140		// R = Y + 1.140V

/*
===========================================================================
	Private Methods.
===========================================================================
*/

//Note: https://topic.alibabacloud.com/a/about-the-common-ayuv-formats-in-rgb-yuy2-yuyv-yvyu-uyvy-directshow-rgbyuv_8_8_10271129.html
// The ¨AYUV format has an alpha channel, and the YUV component is extracted for each pixel, and the image Data format is as follows:
// A0 Y0 U0 V0 A1 Y1 U1 V1 ... 


void OrigRealYUV444toRGB24Converter::Convert(const void* pInputBuffer, void* pRgb) const
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

        int r = y + RYUVRGB24C_V0 * (v - 128);
        int g = y + RYUVRGB24C_U1 * (u - 128) + RYUVRGB24C_V1 * (v - 128);
        int b = y + RYUVRGB24C_U0 * (u - 128);

        optr[i * 3] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(b);
        optr[(i * 3) + 1] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(g);
        optr[(i *3) + 2] = (BYTE)RYUVRGB24C_RANGECHECK_0TO255(r);
    }
}//end Convert



///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

#define _chrOff			128


void OrigRealYUV420toRGB24Converter::FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const
{
  unsigned char *optr	=	(unsigned char *)pRgb;
  const unsigned char *py	= (unsigned char *)pY;
  const unsigned char *pu	= (unsigned char *)pU;
  const unsigned char *pv	= (unsigned char *)pV;
  int		lumX	= _width;
  int		lumY	= _height;
  int		uvX	= _width >> 1;

  int x,y;
  int lumposy;
  int rgbposy;
  int uvposy;
  int r,b,g;

  int tworows  = lumX << 1;
  int rgb1row  = (_width * 3);
  int rgb2rows = (_width * 3) << 1;
  int lastrow = _width * (_height - 1) * 3;

  for(y=0,lumposy=0,uvposy=0,rgbposy=lastrow; y<lumY; y+=2,lumposy+=tworows,uvposy+=uvX,rgbposy-=rgb2rows)
  {
	int lumpos0 = lumposy;					// Reset to start of rows.
	int lumpos1 = lumposy + lumX;
	int uvpos	 = uvposy;

	int rgbpos0 = rgbposy;
	int rgbpos1 = rgbposy - rgb1row;

	for(x = 0; x < lumX; x += 2)
	{
		const double dlum00 = (double)((int)(py[lumpos0++]));

		const double du	 = (double)(((int)(pu[uvpos])) );
		const double dv	 = (double)(((int)(pv[uvpos++])) );

		const unsigned char lum00 = dlum00;
		const signed char u = (double)(du + _chrOff);
		const signed char v = (double)(dv + _chrOff);

			// Lum00, u and v.
  		// Fast calculation intermediate variables. 
		const double cc = (RYUVRGB24C_U0 * u) + 0.5;
		const double cb = (RYUVRGB24C_U1 * u) + (RYUVRGB24C_V1 * v) + 0.5;
		const double ca = (RYUVRGB24C_V0 * v) + 0.5;

		b = (int)(lum00 + cc);
		g = (int)(lum00 + cb);
		r = (int)(lum00 + ca);
  
			// R, G & B have range 0..255.
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum01.
		const double dlum01 = (double)((int)(py[lumpos0++]));
		const unsigned char lum01 = dlum01;
		b = (int)(lum01 + cc);
		g = (int)(lum01 + cb);
		r = (int)(lum01 + ca);
  
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum10.
		const double dlum10 = (double)((int)(py[lumpos1++]));
		const unsigned char lum10 = dlum10;

		b = (int)(lum10 + cc);
		g = (int)(lum10 + cb);
		r = (int)(lum10 + ca);
  
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum11.
		const double dlum11 = (double)((int)(py[lumpos1++]));
		const unsigned char lum11 = dlum11;

		b = (int)(lum11 + cc);
		g = (int)(lum11 + cb);
		r = (int)(lum11 + ca);
  
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
		optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

	}//end for x...
		
  }//end for y...

}//end OrigRealYUV420toRGB24Converter::FlipConvert


void OrigRealYUV420toRGB24Converter::NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const
{
  unsigned char* 	optr	=	(unsigned char *)pRgb;
  const unsigned char*				py		= (unsigned char *)pY;
  const unsigned char*				pu		= (unsigned char *)pU;
  const unsigned char*				pv		= (unsigned char *)pV;
  int		lumX	= _width;
  int		lumY	= _height;
  int		uvX		= _width >> 1;
  int x,y;
  int lumposy;
  int rgbposy;
  int uvposy;
  int r,b,g;

  int tworows  = lumX << 1;
  int rgb1row  = (_width * 3);
  int rgb2rows = (_width * 3) << 1;

  for(y=0,lumposy=0,uvposy=0,rgbposy=0;	y<lumY;	y+= 2,lumposy+=tworows,uvposy+=uvX,rgbposy+=rgb2rows)
  {
    int lumpos0 = lumposy;					// Reset to start of rows.
    int lumpos1 = lumposy + lumX;
    int uvpos	 = uvposy;

    int rgbpos0 = rgbposy;
    int rgbpos1 = rgbposy + rgb1row;

    for(x = 0; x < lumX; x += 2)
    {
	const double dlum00 = (double)((int)(py[lumpos0++]));

	const double du = (double)(((int)(pu[uvpos])));
	const double dv = (double)(((int)(pv[uvpos++])));

	const unsigned char lum00 = dlum00;
	const signed char u = (double)(du + _chrOff);
	const signed char v = (double)(dv + _chrOff);

			// Lum00, u and v.
  		// Fast calculation intermediate variables. 
	const double cc = (RYUVRGB24C_U0 * u) + 0.5;
	const double cb = (RYUVRGB24C_U1 * u) + (RYUVRGB24C_V1 * v) + 0.5;
	const double ca = (RYUVRGB24C_V0 * v) + 0.5;

	b = (int)(lum00 + cc);
	g = (int)(lum00 + cb);
	r = (int)(lum00 + ca);
  
			// R, G & B have range 0..255.
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum01.
	const double dlum01 = (double)((int)(py[lumpos0++]));
        const unsigned char lum01 = dlum01;
	b = (int)(lum01 + cc);
	g = (int)(lum01 + cb);
	r = (int)(lum01 + ca);
  
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos0++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum10.
	const double dlum10 = (double)((int)(py[lumpos1++]));
	const unsigned char lum10 = dlum10;

	b = (int)(lum10 + cc);
	g = (int)(lum10 + cb);
	r = (int)(lum10 + ca);
  
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

			// Lum11.
	const double dlum11 = (double)((int)(py[lumpos1++]));
        const unsigned char lum11 = dlum11;

	b = (int)(lum11 + cc);
	g = (int)(lum11 + cb);
	r = (int)(lum11 + ca);
  
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(r));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(g));
	optr[rgbpos1++] = (unsigned char)(RYUVRGB24C_RANGECHECK_0TO255(b));

    }//end for x...
		
  }//end for y...

}//end NonRotateConvert.


////////////////////////////////////////////////////////////////////////////////////////////////


//TODO: These constants are suspicious, needs to be checked:
// https://stackoverflow.com/questions/17892346/how-to-convert-rgb-yuv-rgb-both-ways
//void YUVfromRGB(double& Y, double& U, double& V, const double R, const double G, const double B)
//{
//  Y =  0.257 * R + 0.504 * G + 0.098 * B +  16;
//  U = -0.148 * R - 0.291 * G + 0.439 * B + 128;
//  V =  0.439 * R - 0.368 * G - 0.071 * B + 128;
//}


/*===========================================================================
	Constants.
===========================================================================*/
#define RRGB32YUVC_00		 0.299
#define RRGB32YUVC_01		 0.587
#define RRGB32YUVC_02		 0.114
#define RRGB32YUVC_10		-0.147
#define RRGB32YUVC_11		-0.289
#define RRGB32YUVC_12		 0.436
#define RRGB32YUVC_20		 0.615
#define RRGB32YUVC_21		-0.515
#define RRGB32YUVC_22		-0.100


/*===========================================================================
	Interface Methods.
===========================================================================*/
/** Double precision reference implementation.
The full real matix equation is used. The YUV output is represented with
8 bits per pel and the UV components are adjusted from their -128..127 range
to 0..255.
@param pRgb	: Packed RGB 8888 format.
@param pY		: Lum plane.
@param pU		: Chr U plane.
@param pV		: Chr V plane.
@return			: none.*/
void OrigRealRGB32toYUV420Converter::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  unsigned char*	py = (unsigned char *)pY;
  unsigned char*	pu = (unsigned char *)pU;
  unsigned char*	pv = (unsigned char *)pV;
  unsigned char* src = (unsigned char *)pRgb;

  /// Y have range 0..255, U & V have range -128..127.
  double	u,v;
  double	r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int chrOff	= yb*xBlks + xb;
      int lumOff	= (yb*_width + xb) << 1;
      //unsigned char*	t = src + lumOff*4;
      unsigned char*	t = src + ((((_height - (yb * 2) - 1) * _width) + (xb * 2)) * 4);

      /// Top left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      t++;	///< Alpha channel.
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u = 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v = 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Top right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      t++;	///< Alpha channel.
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      lumOff += _width;
      //t = t + _width*4 - 8;
      t = t - _width*4 - 8;
      /// Bottom left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      t++;	///< Alpha channel.
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Bottom right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t);
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Average the 4 chr values.
      pu[chrOff] = (unsigned char)((int)(0.5 + u/4));
      pv[chrOff] = (unsigned char)((int)(0.5 + v/4));
    }//end for xb & yb...
}

void OrigRealRGB32toYUV420Converter::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  unsigned char*	py = (unsigned char *)pY;
  unsigned char*	pu = (unsigned char *)pU;
  unsigned char*	pv = (unsigned char *)pV;

  /// Y have range 0..255, U & V have range -128..127.
  double	u,v;
  double	r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int chrOff	= yb*xBlks + xb;
      int lumOff	= (yb*_width + xb) << 1;
      const unsigned char *t = pRgb + lumOff*4;

      /// Top left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      t++;	///< Alpha channel.
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u = 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v = 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Top right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      t++;	///< Alpha channel.
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      lumOff += _width;
      t = t + _width*4 - 8;
      /// Bottom left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      t++;	///< Alpha channel.
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Bottom right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t);
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Average the 4 chr values.
      pu[chrOff] = (unsigned char)((int)(0.5 + u/4));
      pv[chrOff] = (unsigned char)((int)(0.5 + v/4));
    }//end for xb & yb...
}


///////////////////////////////////////////////////////////////////////////////////////////////////


void OrigRealRGB24toYUV420Converter::FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  unsigned char*	py = (unsigned char *)pY;
  unsigned char*	pu = (unsigned char *)pU;
  unsigned char*	pv = (unsigned char *)pV;
  unsigned char* src = (unsigned char *)pRgb;

  /// Y have range 0..255, U & V have range -128..127.
  double	u,v;
  double	r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int chrOff	= yb*xBlks + xb;
      int lumOff	= (yb*_width + xb) << 1;
      //unsigned char*	t = src + lumOff*3;
      unsigned char*	t = src + ((((_height - (yb * 2) - 1) * _width) + (xb * 2)) * 3);

      /// Top left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);     
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u = 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v = 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Top right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      lumOff += _width;
      //t = t + _width*4 - 8;
      t = t - _width*3 - 6;
      /// Bottom left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Bottom right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t);
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Average the 4 chr values.
      pu[chrOff] = (unsigned char)((int)(0.5 + u/4));
      pv[chrOff] = (unsigned char)((int)(0.5 + v/4));
    }//end for xb & yb...
}


void OrigRealRGB24toYUV420Converter::NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const
{
  unsigned char *py = (unsigned char *)pY;
  unsigned char *pu = (unsigned char *)pU;
  unsigned char *pv = (unsigned char *)pV;  

  /// Y have range 0..255, U & V have range -128..127.
  double	u,v;
  double	r,g,b;

  /// Step in 2x2 pel blocks. (4 pels per block).
  int xBlks = _width >> 1;
  int yBlks = _height >> 1;
  for(int yb = 0; yb < yBlks; yb++)
    for(int xb = 0; xb < xBlks; xb++)
    {
      int chrOff	= yb*xBlks + xb;
      int lumOff	= (yb*_width + xb) << 1;
      const unsigned char *t = pRgb + lumOff*3;

      /// Top left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u = 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v = 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Top right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      lumOff += _width;
      t = t + _width*3 - 6;
      /// Bottom left pel.  255->0.999.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t++);
      py[lumOff] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Bottom right pel.
      b = (double)(*t++);
      g = (double)(*t++);
      r = (double)(*t);
      py[lumOff+1] = (unsigned char)((int)(0.5 + RRGB32YUVC_00*r + RRGB32YUVC_01*g + RRGB32YUVC_02*b));

      u += 128.0 + RRGB32YUVC_10*r + RRGB32YUVC_11*g + RRGB32YUVC_12*b;
      v += 128.0 + RRGB32YUVC_20*r + RRGB32YUVC_21*g + RRGB32YUVC_22*b;

      /// Average the 4 chr values.
      pu[chrOff] = (unsigned char)((int)(0.5 + u/4));
      pv[chrOff] = (unsigned char)((int)(0.5 + v/4));
    }//end for xb & yb...
}




