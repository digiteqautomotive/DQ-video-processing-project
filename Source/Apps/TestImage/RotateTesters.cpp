#include "PicRotateRGBBase.h"
#include "DirectShow/CommonDefs.h"
#include "RotateTesters.h"


int OrigRotateRGB24Impl::BytesPerPixel(void) const
{
  return BITS_PER_PIXEL_RGB24/8;
}


////////////////////////////////////////////////////////


int OrigRotateRGB32Impl::BytesPerPixel(void) const
{
	return BITS_PER_PIXEL_RGB32/8;
}
