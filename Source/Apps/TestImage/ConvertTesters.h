#pragma once


class YUV444toRGBConverter;


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
  virtual void FlipConvert(const void* pY, const void* pU, const void* pV, void* pRgb) const;
  virtual void NonRotateConvert(const void* pY, const void* pU, const void* pV, void* pRgb) const;
}; //end OrigRealYUV420toRGB24Converter.



#include "RGBtoYUV420Converter.h"


class OrigRealRGB32toYUV420Converter: public RGBtoYUV420Converter
{
public:
		// Construction and destruction.
	OrigRealRGB32toYUV420Converter(void) { }
	OrigRealRGB32toYUV420Converter(int width, int height): RGBtoYUV420Converter(width,height) { }
	virtual ~OrigRealRGB32toYUV420Converter(void) {}

		// Interface.
	void Convert(const void* pRgb, void* pYUV);
        void Convert(const void* pRgb, void *pY, void *pU, void *pV);

private:
	void FlipConvert(const void* pRgb, void* pY, void* pU, void* pV);
	void NonFlipConvert(const void* pRgb, void* pY, void* pU, void* pV);
};//end RealRGB32toYUV420Converter.
