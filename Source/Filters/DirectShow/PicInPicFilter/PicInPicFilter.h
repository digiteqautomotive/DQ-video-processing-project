/** @file

MODULE				: VideoMixingFilter

FILE NAME			: PicInPicFilter.h

DESCRIPTION			: Picture in picture Filter for RGB24 and RGB32 Media
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the CSIR nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

===========================================================================
*/
#pragma once

#include "DirectShow/VideoMixingBase.h"
#include <DirectShow/CSettingsInterface.h>
#include "../FilterParameterStringConstants.h"


// {17E57D6D-C6F2-4084-9165-A0A6CBA099FB}
static const GUID CLSID_VPP_PicInPicFilter = 
{ 0x17e57d6d, 0xc6f2, 0x4084, { 0x91, 0x65, 0xa0, 0xa6, 0xcb, 0xa0, 0x99, 0xfb } };

// {44B81B95-38B1-49CC-8851-73CC3471DEA5}
static const GUID CLSID_PicInPicProperties = 
{ 0x44b81b95, 0x38b1, 0x49cc, { 0x88, 0x51, 0x73, 0xcc, 0x34, 0x71, 0xde, 0xa5 } };


#ifdef _BUILD_FOR_SHORT
typedef short yuvType;
#else
typedef signed char yuvType ;
#endif

// Forward declarations
class PicInPicBase;
class PicScalerBase;

/**
 * @brief PicInPicFilter that accepts two RGB24 inputs.
 * The filter should only be confgured after the inputs have been connected. */
class PicInPicFilter: public VideoMixingBase, public ISpecifyPropertyPages,
                      public IPersistPropertyBag, public IPropertyBag, public IPersistStream
{
public:
	DECLARE_IUNKNOWN;

	/// Constructor
	PicInPicFilter();
	/// Destructor
	~PicInPicFilter();

	/// Static object-creation method (for the class factory)
	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 
	
	///  From CBasePin
	virtual HRESULT CheckOutputType(const CMediaType* pMediaType);

	/// Overridden from VideoMixingBase 
	virtual HRESULT ReceiveFirstSample(IMediaSample *pSample);
	/// Overridden from VideoMixingBase 
	virtual HRESULT ReceiveSecondSample(IMediaSample *pSample);
	/// Overridden from VideoMixingBase 
	virtual HRESULT CreateVideoMixer(const CMediaType *pMediaType, int nIndex);
	/// Overridden from VideoMixingBase 
        virtual HRESULT SetOutputDimensions(const BITMAPINFOHEADER* pBmih1, const BITMAPINFOHEADER* pBmih2, int& nOutputWidth, int& nOutputHeight, int& nOutputSize);

	HRESULT GenerateOutputSample(IMediaSample *pSample, int nIndex);

	virtual void initParameters();
        virtual HRESULT StartStreaming(void);

	/// For configuration dialog
	STDMETHODIMP GetPages(CAUUID *pPages);

	/// For configuration dialog
	STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

	// TEMP Overriding to prevent RGB32
	virtual void InitialiseInputTypes()
	{
		AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24, &FORMAT_VideoInfo);
	}
	virtual void InitialiseOutputTypes()
	{
		AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24, &FORMAT_VideoInfo);
	}

	STDMETHODIMP SetParameter( const char* type, const char* value);

		// IPersistPropertyBag
	virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);
	virtual HRESULT STDMETHODCALLTYPE InitNew(void);        
	virtual HRESULT STDMETHODCALLTYPE Load(__RPC__in_opt IPropertyBag *pPropBag, __RPC__in_opt IErrorLog *pErrorLog);
	virtual HRESULT STDMETHODCALLTYPE Save(__RPC__in_opt IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties);

		// IPropertyBag
	STDMETHODIMP Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog);
	STDMETHODIMP Write(LPCOLESTR pszPropName, VARIANT *pVar);

		// IPersistStream
	virtual HRESULT STDMETHODCALLTYPE GetSizeMax(ULARGE_INTEGER *pcbSize);
	virtual HRESULT STDMETHODCALLTYPE IsDirty(void);
	virtual HRESULT STDMETHODCALLTYPE Load(IStream *pStm);
	virtual HRESULT STDMETHODCALLTYPE Save(IStream *pStm, BOOL fClearDirty);

private:
        void VFlipImg1(const BYTE* pSrc, BYTE* pDest);
	bool parameterChangeAffectsOutput( const char* szParam );
	void reconfigure();
	/// Pointers to our last received Media Sample
	BYTE* m_pSampleBuffers[2];
	/// Sizes
	int m_nSampleSizes[2];

	enum SUB_PIC_POSITION
	{
		SUB_PIC_POSITION_1		= 0,
		SUB_PIC_POSITION_2		= 1,
		SUB_PIC_POSITION_3		= 2,
		SUB_PIC_POSITION_4		= 3,
		SUB_PIC_POSITION_CUSTOM = 4
	};

	/// Position where subvideo is inserted
	//int m_nPosition;
	/// target width: if zero the width of the primary picture is used
  int m_nTargetWidth;
  /// target height: if zero the height of the primary picture is used
  int m_nTargetHeight;
	/// target width of sub pic: if zero the width of the secondary picture is used
  int m_nSubPictureWidth;
  /// target height of sub pic: if zero the width of the secondary picture is used
  int m_nSubPictureHeight;
	/// custom X offset from bottom left corner used when position = SUB_PIC_POSITION_CUSTOM
  int m_nCustomOffsetX;
  /// custom Y offset from bottom left corner used when position = SUB_PIC_POSITION_CUSTOM
  int m_nCustomOffsetY;
	/// picture in pic class
  PicInPicBase* m_pPicInPic;
	/// target picture scaler used if primary picture output dimensions != input picture dimensions
  PicScalerBase* m_pTargetPicScaler;
  /// target picture scaler used if secondary picture output dimensions != input picture dimensions
  PicScalerBase* m_pSubPicScaler;
	/// Stores bytes per needed to store pixel according to media type
  unsigned m_nBitsPerPixel;
  /// buffer to scale secondary image into if required
  BYTE* m_pBufferForScaledSecondaryImage;
  bool m_Vflip;
  bool m_DirtyProps;
};
