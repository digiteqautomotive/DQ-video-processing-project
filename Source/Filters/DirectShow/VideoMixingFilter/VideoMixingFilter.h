/** @file

MODULE				: VideoMixingFilter

FILE NAME			: VideoMixingFilter.h

DESCRIPTION			: Video Mixing Filter for AYUV, RGB24 and RGB32 Media
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2014, CSIR
Copyright (c) 2025, Jaroslav Fojtik, Digiteq Automotive All rights reserved.

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
===========================================================================*/
#pragma once

#include <DirectShow/VideoMixingBase.h>
#include <DirectShow/CSettingsInterface.h>
#include "../FilterParameterStringConstants.h"


// {2ECB8B03-633F-4D79-89DF-B390B213916D}
DEFINE_GUID(CLSID_VPP_VideoMixingFilter, 
0x2ecb8b03, 0x633f, 0x4d79, 0x89, 0xdf, 0xb3, 0x90, 0xb2, 0x13, 0x91, 0x6d);

// {2799F9E3-1DEF-4C71-AEB8-279A7EDD2060}
DEFINE_GUID(CLSID_VideoMixingProperties, 
0x2799f9e3, 0x1def, 0x4c71, 0xae, 0xb8, 0x27, 0x9a, 0x7e, 0xdd, 0x20, 0x60);


// Forward declarations
class PicConcatBase;

/**\ingroup DirectShowFilters
 * Video Mixing Filter for RGB24 and RGB32 Media */
class VideoMixingFilter: public VideoMixingBase, public ISpecifyPropertyPages
{
public:
	DECLARE_IUNKNOWN;

	/// Constructor
	VideoMixingFilter();
	/// Destructor
	~VideoMixingFilter();

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

	HRESULT GenerateOutputSample(IMediaSample *pSample, int nIndex, const BYTE *pSampleBuffer0, const BYTE *pSampleBuffer1);

	void initParameters();

        virtual void OnDisconnect(int nIndex);

        void FeedRegion(void* pImg) const;

        HRESULT StartStreaming(void);

		// For configuration dialog
	STDMETHODIMP GetPages(CAUUID *pPages);

		// For configuration dialog
	STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

		// IPersistPropertyBag
        virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);
        virtual HRESULT STDMETHODCALLTYPE InitNew(void) {return VideoMixingBase::InitNew();}

protected:		
	BYTE *m_pSampleBuffers[2];	///< Pointers to our last received Media Sample.
	int m_nSampleSizes[2];		///< Sizes

	int m_nOrientation;		///< 0 = horizontal, 1 = vertical

	PicConcatBase *m_pPicConcat;	///< Picture concatenation class

		/// Stores bytes per needed to store pixel according to media type
	unsigned m_nBitsPerPixel;
	bool m_Vflip;			///< Negative height sometimes enforces internal neccesity of flip.
	unsigned m_FeedHeight, m_FeedWidth;
	unsigned m_Feed_xPos, m_Feed_yPos;
};
