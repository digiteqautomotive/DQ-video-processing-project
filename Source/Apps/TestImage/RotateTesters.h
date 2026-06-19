#pragma once


class OrigRotateRGB32Impl: public PicRotateRGBBase
{
public:
	OrigRotateRGB32Impl()	{}

	virtual int BytesPerPixel(void) const;
};


class OrigRotateRGB24Impl: public PicRotateRGBBase
{
public:
	OrigRotateRGB24Impl()	{}

	virtual int BytesPerPixel(void) const;
};
