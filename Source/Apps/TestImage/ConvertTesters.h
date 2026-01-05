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