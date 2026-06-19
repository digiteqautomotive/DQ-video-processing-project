#pragma once


class OrigRotateBPP16Impl: public PicRotateRGBBase
{
public:
	OrigRotateBPP16Impl()	{}

	virtual int BytesPerPixel(void) const;
};


class OrigRotateRGB24Impl: public PicRotateRGBBase
{
public:
	OrigRotateRGB24Impl()	{}

	virtual int BytesPerPixel(void) const;
};


class OrigRotateRGB32Impl: public PicRotateRGBBase
{
public:
	OrigRotateRGB32Impl()	{}

	virtual int BytesPerPixel(void) const;
};


class OrigRotateRGB64Impl: public PicRotateRGBBase
{
public:
	OrigRotateRGB64Impl()	{}

	virtual int BytesPerPixel(void) const;
};
