/** @file

MODULE:		VideoCompareFilter

FILE NAME:	VideoCompareFilter.h

DESCRIPTION:	Video Compare Filter for AYUV, RGB24 and RGB32 Media
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Jaroslav Fojtik, Digiteq Automotive All rights reserved.

===========================================================================*/
#pragma once

#include <DirectShow/VideoMixingBase.h>
#include <DirectShow/CSettingsInterface.h>
#include "../FilterParameterStringConstants.h"
#include "DirectShow/FrameQueue.h"
#include "PicCompareBase.h"


// {598FB8F7-C4C5-47B8-8461-95962919A7D1}
DEFINE_GUID(CLSID_VPP_VideoCompareFilter,
0x598fb8f7, 0xc4c5, 0x47b8, 0x84, 0x61, 0x95, 0x96, 0x29, 0x19, 0xa7, 0xd1);


// {07807C54-CA6E-47EA-BB29-2EF1AC33AF2B}
DEFINE_GUID(CLSID_VideoCompareProperties, 
0x7807c54, 0xca6e, 0x47ea, 0xbb, 0x29, 0x2e, 0xf1, 0xac, 0x33, 0xaf, 0x2b);


// Forward declarations
class PicConcatBase;

/**\ingroup DirectShowFilters
 * Video Compare Filter for RGB24 and RGB32 Media */
class VideoCompareFilter: public VideoMixingBase, public ISpecifyPropertyPages
{
public:
	DECLARE_IUNKNOWN;

	/// Constructor
	VideoCompareFilter();
	/// Destructor
	~VideoCompareFilter();

	/// Static object-creation method (for the class factory)
	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 
	
	///  From CBasePin
	virtual HRESULT CheckOutputType(const CMediaType* pMediaType);

		/// Overridden from VideoMixingBase 
	virtual HRESULT ReceiveFirstSample(IMediaSample *pSample);
	virtual HRESULT ReceiveSecondSample(IMediaSample *pSample);
	virtual HRESULT CreateVideoMixer(const CMediaType *pMediaType, int nIndex);
        virtual HRESULT SetOutputDimensions(const BITMAPINFOHEADER* pBmih1, const BITMAPINFOHEADER* pBmih2, int& nOutputWidth, int& nOutputHeight, int& nOutputSize);

	HRESULT GenerateOutputSample(IMediaSample *pSample, int nIndex, const BYTE *pSampleBuffer0, const BYTE *pSampleBuffer1);

	void initParameters();

        virtual void OnDisconnect(int nIndex);

        void FeedRegion(void* pImg) const;

        HRESULT StartStreaming(void);
        HRESULT StopStreaming(void);

		// For configuration dialog
	STDMETHODIMP GetPages(CAUUID *pPages);

		// For configuration dialog
	STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

		// IPersistPropertyBag
        virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);
        virtual HRESULT STDMETHODCALLTYPE InitNew(void) {return VideoMixingBase::InitNew();}

protected:
        FrameQueue m_FQ[2];
	BYTE *m_pSampleBuffers[2];	///< Pointers to our last received Media Sample.
	int m_nSampleSizes[2];		///< Sizes
        REFERENCE_TIME TimeStart1, TimeEnd1, TimeStart2, TimeEnd2;

	int m_nSynchronisation;		///< 0-channel 1; 1-channel 2; 2-mix

	PicCompareBase *m_pPicCompare;	///< Picture concatenation class

		/// Stores bytes per needed to store pixel according to media type
	unsigned m_nBitsPerPixel;
	bool m_Vflip;			///< Negative height sometimes enforces internal neccesity of flip.
	unsigned m_FeedHeight, m_FeedWidth;
	unsigned m_Feed_xPos, m_Feed_yPos;
};
