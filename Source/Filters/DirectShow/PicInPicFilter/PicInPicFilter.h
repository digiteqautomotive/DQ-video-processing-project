/** @file

MODULE				: VideoMixingFilter

FILE NAME			: PicInPicFilter.h

DESCRIPTION			: Picture in picture Filter for RGB24 and RGB32 Media
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2012, CSIR
              2025-2026 Digiteq Automotive; designed by Jaroslav Fojtik		All rights reserved.

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

#include "DirectShow/VideoMixingBase.h"
#include "DirectShow/CSettingsInterface.h"
#include "../FilterParameterStringConstants.h"

#include <list>


// {17E57D6D-C6F2-4084-9165-A0A6CBA099FB}
static const GUID CLSID_VPP_PicInPicFilter = 
{ 0x17e57d6d, 0xc6f2, 0x4084, { 0x91, 0x65, 0xa0, 0xa6, 0xcb, 0xa0, 0x99, 0xfb } };

// {44B81B95-38B1-49CC-8851-73CC3471DEA5}
static const GUID CLSID_PicInPicProperties = 
{ 0x44b81b95, 0x38b1, 0x49cc, { 0x88, 0x51, 0x73, 0xcc, 0x34, 0x71, 0xde, 0xa5 } };


class FrameQueue
{
public:
  FrameQueue(void) {}
  ~FrameQueue();

  mutable CCritSec m_csFrame2Queue;
  std::list<IMediaSample*> q;

  void AddFrame(IMediaSample*pFrame);
  void Erase(void);
  IMediaSample *PeekFrame(void) const;
  IMediaSample *PeekFrame(REFERENCE_TIME &tStart, REFERENCE_TIME &tStop);
  void PopFrame(void);
};


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
class PicInPicFilter: public VideoMixingBase, public ISpecifyPropertyPages
{
public:
	typedef enum
	{
	  THREAD_NOT_INITIALISED =0,
          THREAD_INITIALISING,
          THREAD_RUNNING,
          THREAD_STOPPING,
          THREAD_STOPPED,
          THREAD_ABORTED
        } E_THR_STATUS;

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
	bool parameterChangeAffectsOutput( const char* szParam );
	void reconfigure();

/*
        volatile char ThreadStatus;
#ifdef _WIN32
	//HANDLE        StopEvent;	///< stop signal for log thread
	HANDLE        MsgWakeUp;	///< new log entry signal for log thread
	HANDLE        WriteThread;	///< log thread
	static UINT WINAPI WorkerThread(void*);
#else
	volatile int StopEvent;
	pthread_cond_t MsgWakeUp;
	pthread_mutex_t mtx_MsgWakeUp;
	pthread_t WriteThread;
	static void *WorkerThread(void*);
#endif
        HRESULT InitializeOutputSample(IMediaSample **ppOutSample, int nInputIndex, int nOutputIndex);
        LONGLONG m_MediaStart,m_MediaEnd;
*/

	/// Pointers to our last received Media Sample
	//BYTE* m_pSampleBuffers[2];
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
  REFERENCE_TIME TimeStart2, TimeEnd2;
  unsigned m_szBufferForScaledSecondaryImage;
  bool m_Vflip;
  bool m_bThrottling;
  bool m_DuplicateFrame;
  FrameQueue FQ;
};
