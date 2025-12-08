#pragma once


class OrigRotateRGB32Impl: public PicRotateRGBBase
{
public:
	OrigRotateRGB32Impl();
	~OrigRotateRGB32Impl();

	virtual int BytesPerPixel();
};


class OrigRotateRGB24Impl: public PicRotateRGBBase
{
public:
	OrigRotateRGB24Impl();
	~OrigRotateRGB24Impl();

	virtual int BytesPerPixel();
};
