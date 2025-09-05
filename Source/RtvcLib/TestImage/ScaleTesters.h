#pragma once


class OrigScalerRGB24Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerRGB24Impl(void) { }
	OrigScalerRGB24Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerRGB24Impl(void) {}
	virtual int GetVideoFormat(void) const {return 24;}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

};//end PicScalerRGB24Impl.


class OrigScalerARGB32Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerARGB32Impl(void) { }
	OrigScalerARGB32Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerARGB32Impl(void) {}
	virtual int GetVideoFormat(void) const {return 32;}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

}; //end PicScalerRGB32Impl.


class OrigScalerRGB32Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerRGB32Impl(void) { }
	OrigScalerRGB32Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerRGB32Impl(void) {}
	virtual int GetVideoFormat(void) const {return 32;}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

};//end PicScalerRGB32Impl.


class OrigScalerUYVYImpl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerUYVYImpl(void) { }
	OrigScalerUYVYImpl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerUYVYImpl(void) {}
	virtual int GetVideoFormat(void) const {return 16;}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

}; //end OrigScalerUYVYImpl.


class OrigScalerYUYVImpl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerYUYVImpl(void) { }
	OrigScalerYUYVImpl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerYUYVImpl(void) {}
	virtual int GetVideoFormat(void) const {return 16;}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

}; //end OrigScalerUYVYImpl.
