#pragma once


class YUV444toRGBConverter;


class OrigRealYUV444toRGB24Converter: public YUV444toRGBConverter
{
public:
  // Construction and destruction.
  OrigRealYUV444toRGB24Converter(void) { }
  OrigRealYUV444toRGB24Converter(int width, int height): YUV444toRGBConverter(width,height) { }
  virtual ~OrigRealYUV444toRGB24Converter(void) {}

  // Interface.
  virtual void Convert(const void* pInputBuffer, void* pRgb);



};//end RealYUV444toRGB24Converter.