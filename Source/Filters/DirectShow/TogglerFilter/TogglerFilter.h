/** @file

MODULE				: VideoMixingFilter

FILE NAME			: TogglerFilter.h

DESCRIPTION			: Picture in picture Filter for RGB24 and RGB32 Media
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive; designed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#pragma once

#include "DirectShow/VideoMixingBase.h"
#include "DirectShow/CSettingsInterface.h"
#include "../FilterParameterStringConstants.h"

#include <list>


// {FEB2CFBF-726F-4358-B166-93D2C9A841A9}
static const GUID CLSID_VPP_TogglerFilter = 
{ 0xfeb2cfbf, 0x726f, 0x4358, { 0xb1, 0x66, 0x93, 0xd2, 0xc9, 0xa8, 0x41, 0xa9 } };

// {3D1EF8A8-22D6-4474-8343-4C4C6B0977CC}
static const GUID CLSID_TogglerProperties = 
{ 0x3d1ef8a8, 0x22d6, 0x4474, { 0x83, 0x43, 0x4c, 0x4c, 0x6b, 0x9, 0x77, 0xcc } };


#define FILTER_PARAM_CHOOSE_INPUT	"Choose Input"
#define FILTER_PARAM_DURATION		"Duration"
#define FILTER_PARAM_ACT_WIDTH		"Actual width"
#define FILTER_PARAM_ACT_HEIGHT		"Actual height"


// Forward declarations
class PicInPicBase;
class PicScalerBase;


/** @brief TogglerFilter that accepts two RGB24 inputs.
 * The filter should only be confgured after the inputs have been connected. */
class TogglerFilter: public VideoMixingBase, public ISpecifyPropertyPages
{
public:	
	DECLARE_IUNKNOWN;

	/// Constructor
	TogglerFilter();
	/// Destructor
	~TogglerFilter();

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

	//HRESULT GenerateOutputSample(IMediaSample *pSample, int nIndex);

	void initParameters();
        virtual HRESULT StartStreaming(void);
        virtual HRESULT StopStreaming(void);

	/// For configuration dialog
	STDMETHODIMP GetPages(CAUUID *pPages);

	/// For configuration dialog
	STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

	STDMETHODIMP SetParameter( const char* type, const char* value);

		// IPersistPropertyBag
	virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);	

private:
        void VFlipImg1(const BYTE* pSrc, BYTE* pDest);
        void VFlipImg2(const BYTE* pSrc, BYTE* pDest);
	bool parameterChangeAffectsOutput( const char* szParam);
	void reconfigure();

	/// Pointers to our last received Media Sample
	//BYTE* m_pSampleBuffers[2];
	/// Sizes
	int m_nSampleSizes[2];

	/// Position where subvideo is inserted
	//int m_nPosition;
  int m_nTargetWidth;			///< target width: if zero the width of the primary picture is used
  int m_nTargetHeight;			///< target height: if zero the height of the primary picture is used
  PicScalerBase* m_PicScaler1;		///< target picture scaler used if primary picture output dimensions != input picture dimensions
  PicScalerBase* m_PicScaler2;		///< target picture scaler used if secondary picture output dimensions != input picture dimensions
	/// Stores bytes per needed to store pixel according to media type
  unsigned m_nBitsPerPixel;
  /// buffer to scale secondary image into if required
  BYTE* m_pBufferForScaledSecondaryImage;
  unsigned m_szBufferForScaledSecondaryImage;
  bool m_Vflip;
  bool m_bThrottling;
  bool m_DuplicateFrame;
  bool m_bToggle;
  REFERENCE_TIME m_duration;
  unsigned m_MaxInterval;
};
