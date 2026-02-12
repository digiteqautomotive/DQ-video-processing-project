#pragma once


class OrigRealYUV444toRGB24Converter: public YUVtoRGBConverter
{
public:
  // Construction and destruction.
  OrigRealYUV444toRGB24Converter(void) { }
  OrigRealYUV444toRGB24Converter(int width, int height): YUVtoRGBConverter(width,height) { }
  virtual ~OrigRealYUV444toRGB24Converter(void) {}

  // Interface.
  virtual void Convert(const void* pInputBuffer, void* pRgb) const;
}; //end OrigRealYUV444toRGB24Converter.


class OrigRealYUV420toRGB24Converter: public YUV420toRGBConverter
{
public:
  // Construction and destruction.
  OrigRealYUV420toRGB24Converter(void) { }
  OrigRealYUV420toRGB24Converter(int width, int height): YUV420toRGBConverter(width,height) { }
  virtual ~OrigRealYUV420toRGB24Converter(void) {}

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
	OrigRealRGB24toYUV420Converter(void) { }
	OrigRealRGB24toYUV420Converter(int width, int height): RGBtoYUV420Converter(width,height) { }
	virtual ~OrigRealRGB24toYUV420Converter(void) {}

		// Interface.
protected:
	void FlipConvert(const unsigned char* pRgb, void* pY, void* pU, void* pV) const;
	void NonFlipConvert(const unsigned char* pRgb, void* pY, void* pU, void* pV) const;
}; //end RealRGB24toYUV420Converter.


class OrigRealRGB32toYUV420Converter: public RGBtoYUV420Converter
{
public:
		// Construction and destruction.
	OrigRealRGB32toYUV420Converter(void) { }
	OrigRealRGB32toYUV420Converter(int width, int height): RGBtoYUV420Converter(width,height) { }
	virtual ~OrigRealRGB32toYUV420Converter(void) {}

		// Interface.
protected:
	void FlipConvert(const unsigned char* pRgb, void* pY, void* pU, void* pV) const;
	void NonFlipConvert(const unsigned char* pRgb, void* pY, void* pU, void* pV) const;
};//end RealRGB32toYUV420Converter.
