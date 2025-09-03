#include <stdio.h>
#include <stdlib.h>

#include "PicRotateRGBBase.h"
#include "DirectShow/CommonDefs.h"
#include "RotateTesters.h"


OrigRotateRGB24Impl::OrigRotateRGB24Impl()
{
	m_eMode = ROTATE_NONE;
}

OrigRotateRGB24Impl::~OrigRotateRGB24Impl()
{;}

int OrigRotateRGB24Impl::BytesPerPixel()
{
	return BITS_PER_PIXEL_RGB24/8;
}


////////////////////////////////////////////////////////


OrigRotateRGB32Impl::OrigRotateRGB32Impl()
{;}

OrigRotateRGB32Impl::~OrigRotateRGB32Impl()
{;}

int OrigRotateRGB32Impl::BytesPerPixel()
{
	return BITS_PER_PIXEL_RGB32/8;
}
