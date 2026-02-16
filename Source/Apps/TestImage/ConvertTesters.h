#pragma once


class OrigRealYUV444toRGB24Converter: public CVideoConverter
{
public:
  // Construction and destruction.
  OrigRealYUV444toRGB24Converter(void) { }
  OrigRealYUV444toRGB24Converter(int width, int height): CVideoConverter(width,height) { }
  virtual ~OrigRealYUV444toRGB24Converter(void) {}

  // Interface.
  virtual void Convert(const void* pInputBuffer, void* pRgb) const;
}; //end OrigRealYUV444toRGB24Converter.


class OrigRealYUV420toRGB24Converter: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  OrigRealYUV420toRGB24Converter(void) { }
  OrigRealYUV420toRGB24Converter(int width, int height): YUV420toRGBConverter(width,height) {}

  // Interface.

protected:
  virtual void FlipConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
  virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, unsigned char *pRgb) const;
}; //end OrigRealYUV420toRGB24Converter.



#include "RGBtoYUV420Converter.h"


class OrigRealRGB24toYUV420Converter: public RGBtoYUV420Converter
{
public:
		// Construction and destruction.
	OrigRealRGB24toYUV420Converter(void) {}
	OrigRealRGB24toYUV420Converter(int width, int height): RGBtoYUV420Converter(width,height) {}

	bool isInputVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const {return true;}

		// Interface.
protected:
	void FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
	void NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
}; //end RealRGB24toYUV420Converter.


class OrigRealRGB32toYUV420Converter: public RGBtoYUV420Converter
{
public:
		// Construction and destruction.
	OrigRealRGB32toYUV420Converter(void) { }
	OrigRealRGB32toYUV420Converter(int width, int height): RGBtoYUV420Converter(width,height) {}	
        bool isInputVideoFormatSupported(const GUID &VideoFormatIn, const GUID &VideoFormatOut) const {return true;};

		// Interface.
protected:
	void FlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
	void NonFlipConvert(const unsigned char* pRgb, unsigned char* pY, unsigned char* pU, unsigned char* pV) const;
};//end RealRGB32toYUV420Converter.
