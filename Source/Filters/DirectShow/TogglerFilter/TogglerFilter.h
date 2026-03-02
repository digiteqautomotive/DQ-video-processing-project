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


// Forward declarations
class PicInPicBase;
class PicScalerBase;


/**
 * @brief TogglerFilter that accepts two RGB24 inputs.
 * The filter should only be confgured after the inputs have been connected. */
class TogglerFilter: public VideoMixingBase, public ISpecifyPropertyPages
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
