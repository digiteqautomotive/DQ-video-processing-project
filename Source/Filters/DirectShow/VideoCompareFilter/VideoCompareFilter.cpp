/** @file

MODULE				: VideoCompareFilter

FILE NAME			: VideoCompareFilter.cpp

DESCRIPTION			: Video Compare Filter for AYUV, RGB24 and RGB32 Media
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Jaroslav Fojtik, Digiteq Automotive All rights reserved.

===========================================================================*/
#include "stdafx.h"
#include "VideoCompareFilter.h"
#include <DirectShow/CommonDefs.h>
#include <PicCompRGB24Impl.h>
#include <PicCompRGB32Impl.h>
#include <PicCompYUYV_Impl.h>
#include <Dvdmedia.h>			// VIDEOINFOHEADER2

#include "DirectShow/VariantTools.h"

#if defined(USE_SSE)
 #include "PicCompRGB32_SSE2.h"
 extern "C" unsigned GetFeaturesCPU(void);
 const unsigned FeaturesCPU = GetFeaturesCPU();
#endif


DEFINE_GUID(MEDIASUBTYPE_HDYC, 0x43594448, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);


#ifndef MAX
 #define MAX(a,b) ((a>b)?(a):(b))
#endif


OLECHAR OLE_PARAM_ORIENTATION[] = olestr(FILTER_PARAM_ORIENTATION);
OLECHAR OLE_PARAM_SYNCHRONISATION[] = olestr(FILTER_PARAM_SYNCHRONISATION);


VideoCompareFilter::VideoCompareFilter(): VideoMixingBase(NAME("DQ VPP Video Compare"), 0, CLSID_VPP_VideoCompareFilter),
	m_pPicCompare(NULL),
        m_Vflip(false),
        m_FeedHeight(0), m_FeedWidth(0),
	m_Feed_xPos(0), m_Feed_yPos(0),
        m_nSynchronisation(0)
{
  m_pSampleBuffers[0] = NULL;
  m_pSampleBuffers[1] = NULL;

  m_nSampleSizes[0] = 0;
  m_nSampleSizes[1] = 0;

  TimeStart1 = TimeEnd1 = TimeStart2 = TimeEnd2 = 0;

	// Virtual method of subclass can't be called from base class contructor, hence must be called in child contructor
  Initialise();
	// Init parameters
  initParameters();
}


VideoCompareFilter::~VideoCompareFilter()
{
  if(m_pPicCompare)
  {
    delete m_pPicCompare;
    m_pPicCompare = NULL;
  }

  for(int i=0; i<2; i++)
  {
    if (m_pSampleBuffers[i])
    {
      delete[] m_pSampleBuffers[i];
      m_pSampleBuffers[i] = NULL;
    }
  }
}


CUnknown * WINAPI VideoCompareFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  VideoCompareFilter* pFilter = new VideoCompareFilter();
  if(pHr) *pHr = (pFilter==NULL) ? E_OUTOFMEMORY : S_OK;
  return pFilter;
}


void VideoCompareFilter::InitialiseInputTypes(void)
{
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB32, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_ARGB32, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_AYUV, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUYV, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUY2, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YVYU, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_UYVY, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_HDYC, &FORMAT_VideoInfo);
}


void VideoCompareFilter::InitialiseOutputTypes(void)
{
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB32, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_ARGB32, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_AYUV, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUYV, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUY2, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YVYU, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_UYVY, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_HDYC, &FORMAT_VideoInfo);
}


void VideoCompareFilter::initParameters(void)
{
  addParameter(FILTER_PARAM_SYNCHRONISATION, OLE_PARAM_SYNCHRONISATION, &m_nSynchronisation);
}


/// This method renders output frame.
/// @TODO: It looks like it is called twice time than necessary, must be limited framerate.
HRESULT VideoCompareFilter::GenerateOutputSample(IMediaSample *pSample, int nIndex, const BYTE *pSampleBuffer0, const BYTE *pSampleBuffer1)
{
	// Prepare output sample
  const AM_SAMPLE2_PROPERTIES * const pProps = m_vInputPins[nIndex]->SampleProps();
  DbgLog((LOG_TRACE,0,TEXT("Video Mixer: Sample Received from input %d: start time: %I64d end time: %I64d"), nIndex, pProps->tStart, pProps->tStop));
  if (pProps->dwStreamId != AM_STREAM_MEDIA) {
  	return m_vInputPins[nIndex]->Receive(pSample);
	}

	// Provide frame rate limitation.
  if(m_nSynchronisation >= 2)
  {
    if(m_iRepeatTime_us > 0)
    {
      const unsigned CurrentTick = GetTickCount_us();
      if(m_LastFrame_us+m_iRepeatTime_us - CurrentTick < (unsigned)0x8000000) return S_OK;
    }
  }
	
  IMediaSample * pOutSample;
	// Set up the output sample
  HRESULT hr = InitializeOutputSample(pSample, &pOutSample, nIndex, 0);

  if(FAILED(hr)) return hr;

  BYTE *pBufferOut = NULL;
  hr = pOutSample->GetPointer(&pBufferOut);
  if (FAILED(hr))
  {
    return hr;
  }

	// Concat the pictures
  if(pSampleBuffer0 && pSampleBuffer1)
  {
    if(m_FeedHeight>0 && m_FeedWidth>0) FeedRegion(pBufferOut);
    m_pPicCompare->Compare(pSampleBuffer0, pSampleBuffer1, pBufferOut, m_Vflip);    
  }
  else if(pSampleBuffer0)
  {
    const int nWidth = labs(m_pPicCompare->GetOutWidth());
    const int nHeight = labs(m_pPicCompare->GetOutHeight());
    if(m_Vflip)
    {
      const int RowLen = (nWidth*m_nBitsPerPixel)/8;
      BYTE *pDst = pBufferOut;
      const BYTE *pSrc = pSampleBuffer0 + (nWidth*nHeight*m_nBitsPerPixel) / 8;
      int y = nHeight;
      while(y-->0)
      {
        pSrc -= RowLen;
        memcpy(pDst, pSrc, RowLen);
	pDst += RowLen;
      }
    }
    else
        memcpy(pBufferOut, pSampleBuffer0, (nWidth*nHeight * m_nBitsPerPixel)/8);
  }
  else if(pSampleBuffer1)
  {
    const int nWidth = labs(m_pPicCompare->GetOutHeight());
    const int nHeight = labs(m_pPicCompare->GetOutHeight());
    if(m_Vflip)
    {
      const int RowLen = (nWidth*m_nBitsPerPixel)/8;
      BYTE *pDst = pBufferOut;
      const BYTE *pSrc = pSampleBuffer1 + (nWidth * nHeight * m_nBitsPerPixel) / 8;
      int y = nHeight;
      while(y-->0)
      {
        pSrc -= RowLen;
        memcpy(pDst, pSrc, RowLen);
	pDst += RowLen;
      }
    }
    else
        memcpy(pBufferOut, pSampleBuffer1, (nWidth*nHeight * m_nBitsPerPixel)/8);		
  }

  pOutSample->SetActualDataLength(m_nOutputSize);
  pOutSample->SetSyncPoint(TRUE);

	// Stop the clock and log it (if PERF is defined)
  MSR_STOP(m_idTransform);

  if (FAILED(hr)) {
		DbgLog((LOG_TRACE,1,TEXT("Error from transform")));
  } 
  else 
  {
		// the Transform() function can return S_FALSE to indicate that the
		// sample should not be delivered; we only deliver the sample if it's
		// really S_OK (same as NOERROR, of course.)
    if (hr == NOERROR) 
    {
      hr = m_vOutputPins[0]->GetInputPin()->Receive(pOutSample);
      m_LastFrame_us += m_iRepeatTime_us;
    } 
    else 
    {
			// S_FALSE returned from Transform is a PRIVATE agreement
			// We should return NOERROR from Receive() in this cause because returning S_FALSE
			// from Receive() means that this is the end of the stream and no more data should
			// be sent.
      if (S_FALSE == hr) 
      {
			//  Release the sample before calling notify to avoid
			//  deadlocks if the sample holds a lock on the system
			//  such as DirectDraw buffers do
        pOutSample->Release();
        return NOERROR;
      }
    }
  }
	// release the output buffer. If the connected pin still needs it,
	// it will have addrefed it itself.
  pOutSample->Release();
  return hr;
}


HRESULT VideoCompareFilter::ReceiveFirstSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  if(!m_vOutputPins[0]->IsConnected()) return S_FALSE;	// Do nothing when no output is connected.
  if(!m_vInputPins[1]->IsConnected())			// Do not modify anything  when input1 is not connected.
  {
    HRESULT hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);
    if(FAILED(hr)) DbgLog((EVENTLOG_WARNING_TYPE, 0, TEXT("VideoCompareFilter::ReceiveFirstSample(): m_vOutputPins[0]->GetInputPin()->Receive(pSample) Code: %X"), hr));
    return hr;
  }

	// Copy the secondary sample into our buffer
  BYTE *pBuffer = NULL;
  HRESULT hr = pSample->GetPointer(&pBuffer);
  if(FAILED(hr))
  {
    DbgLog((EVENTLOG_ERROR_TYPE, 0, TEXT("VideoCompareFilter::ReceiveFirstSample(): pSample->GetPointer(&pBuffer) Code: %X"), hr));
    return hr;
  }
  if(pBuffer==NULL || m_pSampleBuffers[0]==NULL) return E_POINTER;


  switch(m_nSynchronisation)
  {
    case 0:{ 
           pSample->GetTime(&TimeStart1,&TimeEnd1);
           if(TimeStart1 > TimeEnd2)
           {
             IMediaSample *pSample2 = m_FQ[1].PeekFrame(TimeStart1,TimeEnd1);
             if(pSample2)
             {
               pSample2->GetTime(&TimeStart2,&TimeEnd2);
               BYTE *pBuffer2 = NULL;
               if(SUCCEEDED(pSample2->GetPointer(&pBuffer2)))
               {
                 memcpy(m_pSampleBuffers[1], pBuffer2, m_nSampleSizes[1]);
               }
               pSample2->Release();
             } 
           }
           return GenerateOutputSample(pSample, 0, pBuffer,m_pSampleBuffers[1]);
          }
    case 1: m_FQ[1].Erase();
            m_FQ[0].AddFrame(pSample);
	    return S_OK;
  }

  m_FQ[0].Erase();
  m_FQ[1].Erase();
  CAutoLock lck(&m_csReceive);
  memcpy(m_pSampleBuffers[0], pBuffer, m_nSampleSizes[0]);
  return GenerateOutputSample(pSample, 0, m_pSampleBuffers[0], m_pSampleBuffers[1]);
}


HRESULT VideoCompareFilter::ReceiveSecondSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  if(!m_vOutputPins[0]->IsConnected()) return S_FALSE;	// Do nothing when no output is connected.
  if(!m_vInputPins[0]->IsConnected())			// Do not modify anything  when input0 is not connected.
  {
    HRESULT hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);
    if(FAILED(hr)) DbgLog((EVENTLOG_WARNING_TYPE, 0, TEXT("VideoCompareFilter::ReceiveSecondSample(): m_vOutputPins[0]->GetInputPin()->Receive(pSample) Code: %X"), hr));
    return hr;
  }

	// Copy the secondary sample into our buffer  
  BYTE *pBuffer = NULL;
  HRESULT hr = pSample->GetPointer(&pBuffer);
  if(FAILED(hr))
  {
    DbgLog((EVENTLOG_ERROR_TYPE, 0, TEXT("VideoCompareFilter::ReceiveSecondSample(): pSample->GetPointer(&pBuffer) Code: %X"), hr));
    return hr;
  }
  if(pBuffer==NULL || m_pSampleBuffers[1]==NULL) return E_POINTER;

  switch(m_nSynchronisation)
  {
    case 0: m_FQ[0].Erase();
            m_FQ[1].AddFrame(pSample);
	    return S_OK;
    case 1:{ 
           pSample->GetTime(&TimeStart2,&TimeEnd2);
           if(TimeStart2 > TimeEnd1)
           {
             IMediaSample *pSample1 = m_FQ[0].PeekFrame(TimeStart2,TimeEnd2);
             if(pSample1)
             {
               pSample1->GetTime(&TimeStart1,&TimeEnd1);
               BYTE *pBuffer1 = NULL;
               if(SUCCEEDED(pSample1->GetPointer(&pBuffer1)))
               {
                 memcpy(m_pSampleBuffers[0], pBuffer1, m_nSampleSizes[0]);           
               }
               pSample1->Release();		// PSample1 cannot be kept, because some filters have only one buffer :(.
             } 
           }
           return GenerateOutputSample(pSample, 0, m_pSampleBuffers[0],pBuffer);
          }
  }

  m_FQ[0].Erase();
  m_FQ[1].Erase();
  CAutoLock lck(&m_csReceive);
  memcpy(m_pSampleBuffers[1], pBuffer, m_nSampleSizes[1]);
  return GenerateOutputSample(pSample, 0, m_pSampleBuffers[0], m_pSampleBuffers[1]);
}


HRESULT VideoCompareFilter::StartStreaming(void)
{
  if(m_pSampleBuffers[0]!=NULL) memset(m_pSampleBuffers[0], 0, m_nSampleSizes[0]);
  if(m_pSampleBuffers[1]!=NULL) memset(m_pSampleBuffers[1], 0, m_nSampleSizes[1]);
  m_FQ[0].Erase();
  m_FQ[1].Erase();
  TimeStart1 = TimeEnd1 = TimeStart2 = TimeEnd2 = 0;
  return VideoMixingBase::StartStreaming();
}


HRESULT VideoCompareFilter::StopStreaming(void)
{
  m_FQ[0].Erase();
  m_FQ[1].Erase();
  return VideoMixingBase::StopStreaming();
}


HRESULT VideoCompareFilter::CreateVideoMixer(const CMediaType *pMediaType, int nIndex)
{
  const BITMAPINFOHEADER *bmih = NULL;
	// Create temporary sample buffers
  if(pMediaType->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER* const pVih = (const VIDEOINFOHEADER*) pMediaType->pbFormat;
    if(pVih)
    {
      if(pVih->AvgTimePerFrame>0) m_iRepeatTime_us=pVih->AvgTimePerFrame/10;
      bmih = &pVih->bmiHeader;
    }
  }
  else if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2* const pVih2 = (const VIDEOINFOHEADER2*) pMediaType->pbFormat;
    if(pVih2)
    {
      if(pVih2->AvgTimePerFrame>0) m_iRepeatTime_us=pVih2->AvgTimePerFrame/10;
      bmih = &pVih2->bmiHeader;
    }
  }
  if(bmih==NULL) return E_FAIL;

  if(nIndex>=2) return E_INVALIDARG;
  m_nSampleSizes[nIndex] = DIBSIZE(*bmih);
  if(m_pSampleBuffers[nIndex])
  {
		// Recreate in case dimensions have changed
    delete[] m_pSampleBuffers[nIndex];
  }
  m_pSampleBuffers[nIndex] = new BYTE[m_nSampleSizes[nIndex]];


	// Create appropriate picture concatenator
  if(pMediaType->subtype==MEDIASUBTYPE_YUY2 || pMediaType->subtype==MEDIASUBTYPE_YUYV || pMediaType->subtype==MEDIASUBTYPE_YVYU ||
     pMediaType->subtype==MEDIASUBTYPE_UYVY || pMediaType->subtype==MEDIASUBTYPE_HDYC)
  {
    if(m_pPicCompare!=NULL)
    {
      if(m_pPicCompare->GetVideoFormat()!=16) 
      {
        delete m_pPicCompare; m_pPicCompare=NULL;
      }
    }
    if(m_pPicCompare==NULL)
    {
#ifdef USE_SSE
      if(FeaturesCPU & 4)	// SSE2
          m_pPicCompare = new PicCompareYUYV_SSE2();
      else
#endif
      m_pPicCompare = new PicCompareYUYV_Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_YUYV;
    }
    return S_OK;
  }

  if(pMediaType->subtype == MEDIASUBTYPE_RGB24)
  {
    if(m_pPicCompare!=NULL)
    {
      if(m_pPicCompare->GetVideoFormat()!=24) 
      {
        delete m_pPicCompare; m_pPicCompare=NULL;
      }
    }
    if(m_pPicCompare==NULL)
    {
#ifdef USE_SSE
      if(FeaturesCPU & 4)	// SSE2
          m_pPicCompare = new PicCompareRGB24_SSE2();
      else
#endif
      m_pPicCompare = new PicCompareRGB24Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
    }
    return S_OK;
  }

  if(pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_ARGB32 || pMediaType->subtype==MEDIASUBTYPE_AYUV)
  {
    if(m_pPicCompare!=NULL)
    {
      if(m_pPicCompare->GetVideoFormat()!=32) 
      {
        delete m_pPicCompare; m_pPicCompare=NULL;
      }
    }
    if(m_pPicCompare==NULL)
    {
#ifdef USE_SSE
      if(FeaturesCPU & 4)	// SSE2
          m_pPicCompare = new PicCompareRGB32_SSE2();
      else
#endif
          m_pPicCompare = new PicCompareRGB32Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
    }
    return S_OK;
  }

 return VFW_E_TYPE_NOT_ACCEPTED;
}


/// Pin disconnection must reshedule output geometry.
void VideoCompareFilter::OnDisconnect(int nIndex, RTVC_DIRECTION eDirection)
{
  VideoMixingBase::OnDisconnect(nIndex,eDirection);
  if(eDirection == RTVC_IN)
  {
		// Get first input dimensions
    AM_MEDIA_TYPE mediaType1;
    BITMAPINFOHEADER* pBmih1 = NULL;
    if(m_vInputPins[0]->IsConnected() && nIndex!=0)
    {
      if(SUCCEEDED(m_vInputPins[0]->ConnectionMediaType(&mediaType1)))
      {
        if(mediaType1.formattype==FORMAT_VideoInfo)
        {
	  VIDEOINFOHEADER* pVih = (VIDEOINFOHEADER*)mediaType1.pbFormat;
	  pBmih1 = &pVih->bmiHeader;
        }
        if(mediaType1.formattype==FORMAT_VideoInfo2)
        {
	  VIDEOINFOHEADER2* pVih2 = (VIDEOINFOHEADER2*)mediaType1.pbFormat;
	  pBmih1 = &pVih2->bmiHeader;
        }
      }
    }

		// Get second input dimensions
    BITMAPINFOHEADER* pBmih2 = NULL;
    AM_MEDIA_TYPE mediaType2;
    if(m_vInputPins[1]->IsConnected() && nIndex!=1)
    {
      if(SUCCEEDED(m_vInputPins[1]->ConnectionMediaType(&mediaType2)))
      {
        if(mediaType2.formattype==FORMAT_VideoInfo)
        {
	  VIDEOINFOHEADER* const pVih = (VIDEOINFOHEADER*)mediaType2.pbFormat;
	  pBmih2 = &pVih->bmiHeader;
        }
        if(mediaType2.formattype==FORMAT_VideoInfo2)
        {
	  VIDEOINFOHEADER2* const pVih2 = (VIDEOINFOHEADER2*)mediaType2.pbFormat;
	  pBmih2 = &pVih2->bmiHeader;
        }
      }
    }
		// Leave the output dimensions up to the sub class
    int m_nOutputWidth, m_nOutputHeight, m_nOutputSize;
    SetOutputDimensions(pBmih1, pBmih2, m_nOutputWidth, m_nOutputHeight, m_nOutputSize);

		// Free format blocks
    if(pBmih1)
        FreeMediaType(mediaType1);
    if(pBmih2)
        FreeMediaType(mediaType2);

    if(nIndex<=1)
    {
      if(m_pSampleBuffers[nIndex] != NULL)
      {
        free(m_pSampleBuffers[nIndex]);
        m_pSampleBuffers[nIndex] = NULL;
      }
      m_nSampleSizes[nIndex] = 0;
    }
  }
}


HRESULT VideoCompareFilter::SetOutputDimensions(const BITMAPINFOHEADER* pBmih1, const BITMAPINFOHEADER* pBmih2, int& nOutputWidth, int& nOutputHeight, int& nOutputSize)
{
  m_FeedHeight = m_FeedWidth = m_Feed_xPos = m_Feed_yPos = 0;
  if(pBmih1 && pBmih2)
  {
		// Verify that the dimensions match and set output width and height
    if(labs(pBmih1->biWidth) != labs(pBmih2->biWidth))
        return VFW_E_TYPE_NOT_ACCEPTED;
    if(labs(pBmih1->biHeight) != labs(pBmih2->biHeight))
        return VFW_E_TYPE_NOT_ACCEPTED;
    nOutputSize =  m_nSampleSizes[0];
    nOutputWidth = pBmih1->biWidth;
    nOutputHeight = pBmih1->biHeight;
  }
  else
  {
    if(pBmih1)
    {
      nOutputSize =  m_nSampleSizes[0];
      nOutputWidth = pBmih1->biWidth;
      nOutputHeight = pBmih1->biHeight;      
    }
    else if(pBmih2)
    {
      nOutputSize =  m_nSampleSizes[1];
      nOutputWidth = pBmih2->biWidth;
      nOutputHeight = pBmih2->biHeight;
    }
    else
    {
      nOutputWidth = 0;
      nOutputHeight = 0;
      nOutputSize =  0;
    }
  }
  if(m_nBitsPerPixel == BITS_PER_PIXEL_YUYV)
  {
    if(nOutputWidth & 1) return VFW_E_TYPE_NOT_ACCEPTED;
  }
  if(m_pPicCompare)
    m_pPicCompare->SetDimensions(nOutputWidth, nOutputHeight);

return S_OK;
}


/// Final agreement on output geometry.
HRESULT VideoCompareFilter::CheckOutputType(const CMediaType* pMediaType)
{
	//Make sure the input and output types are related
  if(pMediaType->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if(m_pPicCompare==NULL) return E_POINTER;

  	// Video format
  m_Vflip = false;
  const BITMAPINFOHEADER * pBiOut = NULL;
  if(pMediaType->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER * const pVihOut = (const VIDEOINFOHEADER*)pMediaType->pbFormat;
    if(pVihOut!=NULL) pBiOut = &(pVihOut->bmiHeader);
  }
  else
  if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2 * const pVihOut2 = (const VIDEOINFOHEADER2*)pMediaType->pbFormat;
    if(pVihOut2!=NULL) pBiOut = &(pVihOut2->bmiHeader);
  }

  if(pBiOut)
  {
    if(pBiOut->biHeight != m_pPicCompare->GetOutHeight())
    {
      if(labs(pBiOut->biHeight) == labs(m_pPicCompare->GetOutHeight()))
        m_Vflip = true;
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(pBiOut->biWidth != m_pPicCompare->GetOutWidth())
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
  }
  else
      return VFW_E_TYPE_NOT_ACCEPTED;

  if(m_nBitsPerPixel == BITS_PER_PIXEL_RGB24)
  {
    if (*(pMediaType->Subtype()) == MEDIASUBTYPE_RGB24)
    {
      return S_OK;
    }
  }
  else if(m_nBitsPerPixel == BITS_PER_PIXEL_RGB32)
  {
    if(*(pMediaType->Subtype())==MEDIASUBTYPE_AYUV)
    {
      if(ConnectionFormat[0]==MEDIASUBTYPE_AYUV || ConnectionFormat[1]==MEDIASUBTYPE_AYUV)
        return S_OK;
    }
    else if(*(pMediaType->Subtype())==MEDIASUBTYPE_RGB32 || *(pMediaType->Subtype())==MEDIASUBTYPE_ARGB32)
    {
      if(ConnectionFormat[0]==MEDIASUBTYPE_ARGB32 || ConnectionFormat[0]==MEDIASUBTYPE_RGB32 ||
         ConnectionFormat[1]==MEDIASUBTYPE_ARGB32 || ConnectionFormat[1]==MEDIASUBTYPE_RGB32)
        return S_OK;
    }
  }
  else if(m_nBitsPerPixel == BITS_PER_PIXEL_YUYV)
  {
    if(*(pMediaType->Subtype())==MEDIASUBTYPE_YUYV || *(pMediaType->Subtype())==MEDIASUBTYPE_YUY2)
    {
      if(ConnectionFormat[0]==MEDIASUBTYPE_YUYV || ConnectionFormat[0]==MEDIASUBTYPE_YUY2 ||
         ConnectionFormat[1]==MEDIASUBTYPE_YUYV || ConnectionFormat[1]==MEDIASUBTYPE_YUY2)
        return S_OK;
    }
    if(*(pMediaType->Subtype())==MEDIASUBTYPE_UYVY || *(pMediaType->Subtype())==MEDIASUBTYPE_HDYC)
    {
      if(ConnectionFormat[0]==MEDIASUBTYPE_UYVY || ConnectionFormat[0]==MEDIASUBTYPE_HDYC ||
         ConnectionFormat[1]==MEDIASUBTYPE_UYVY || ConnectionFormat[1]==MEDIASUBTYPE_HDYC)
        return S_OK;
    }
    if(*(pMediaType->Subtype())==MEDIASUBTYPE_YVYU)
    {
      if(ConnectionFormat[0]==MEDIASUBTYPE_YVYU || ConnectionFormat[0]==MEDIASUBTYPE_YVYU)
        return S_OK;
    }
  }
  return S_FALSE;
}


void VideoCompareFilter::FeedRegion(void* pImg) const
{
  unsigned char *pDst = (unsigned char*)pImg;
  const unsigned PxSz = m_nBitsPerPixel/8;

  pDst += PxSz*(m_Feed_yPos*m_nOutputWidth + m_Feed_xPos);
  unsigned y = m_FeedHeight;
  while(y-- > 0)
  {
    memset(pDst, 0x0, PxSz*m_FeedWidth);	// Feed whole row at a time.
    pDst += PxSz*m_nOutputWidth;		// Next row.
  } //end for y...
}


/// For configuration dialog
HRESULT VideoCompareFilter::GetPages(CAUUID *pPages)
{
  if (pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if (pPages->pElems == NULL) 
  {
	return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_VideoCompareProperties;
  return S_OK;
}


HRESULT VideoCompareFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == IID_ISpecifyPropertyPages)  return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  if(riid == (IID_ISettingsInterface))	 return GetInterface((ISettingsInterface*) this, ppv);
  if(riid == IID_IPersistPropertyBag)	 return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPropertyBag)		 return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_IPersistStream)	 return GetInterface((IPersistStream*)this, ppv);
  //if(riid == IID_IPersist)		 return GetInterface((IPersist*)this, ppv);
			// Call the parent class.
  return VideoMixingBase::NonDelegatingQueryInterface(riid, ppv);
}


//----------------- IPersistPropertyBag------------------


HRESULT VideoCompareFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_VideoCompareFilter;
  return S_OK;
}
