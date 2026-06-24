/** @file

MODULE:			VideoMixingFilter

FILE NAME:		VideoMixingFilter.cpp

DESCRIPTION:		Video Mixing Filter for AYUV, RGB24 and RGB32 Media
					  
LICENSE:		Software License Agreement (BSD License)

Copyright (c) 2008-2014, CSIR
Copyright (c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik, All rights reserved.

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
#include "stdafx.h"
#include "VideoMixingFilter.h"
#include <DirectShow/CommonDefs.h>
#include "PicConcat2B_Impl.h"
#include "PicConcat3B_Impl.h"
#include "PicConcat4B_Impl.h"
#include "PicConcat6B_Impl.h"
#include "PicConcat8B_Impl.h"
#include <Dvdmedia.h>			// VIDEOINFOHEADER2

#include "DirectShow/VariantTools.h"


#ifndef MAX
 #define MAX(a,b) ((a>b)?(a):(b))
#endif

DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_R16G16B16, 0x36315242, 0x0000,0x0010, 0x80,0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_B16G16R16, 0x36314252, 0x0000,0x0010, 0x80,0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_A16R16G16B16, 0x36314241, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_A16B16G16R16, 0x36315241, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

static const GUID * const VideoMixSupportedFormats[] = {
	&MEDIASUBTYPE_RGB24,
	&MEDIASUBTYPE_RGB32,
	&MEDIASUBTYPE_ARGB32,
	&MEDIASUBTYPE_RGB555,
	&MEDIASUBTYPE_RGB565,
	&MEDIASUBTYPE_AYUV,
	&MEDIASUBTYPE_YV24,		// YUV tripplets
	&MEDIASUBTYPE_R16G16B16,
        &MEDIASUBTYPE_B16G16R16,
	&MEDIASUBTYPE_A16R16G16B16,
	&MEDIASUBTYPE_A16B16G16R16};


OLECHAR OLE_PARAM_ORIENTATION[] = olestr(FILTER_PARAM_ORIENTATION);
OLECHAR OLE_PARAM_SYNCHRONISATION[] = olestr(FILTER_PARAM_SYNCHRONISATION);


VideoMixingFilter::VideoMixingFilter(): VideoMixingBase(NAME("DQ VPP Video Mixer"), 0, CLSID_VPP_VideoMixingFilter),
	m_pPicConcat(NULL),
        m_Vflip(false),
        m_FeedHeight(0), m_FeedWidth(0),
	m_Feed_xPos(0), m_Feed_yPos(0),
        m_nSynchronisation(0),
        m_nOrientation(0)
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


VideoMixingFilter::~VideoMixingFilter()
{
  if(m_pPicConcat)
  {
    delete m_pPicConcat;
    m_pPicConcat = NULL;
  }

  for(int i=0; i<2; i++)
  {
    if(m_pSampleBuffers[i])
    {
      delete[] m_pSampleBuffers[i];
      m_pSampleBuffers[i] = NULL;
    }
  }
}


CUnknown * WINAPI VideoMixingFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  VideoMixingFilter* pFilter = new VideoMixingFilter();
  if(pHr) *pHr = (pFilter==NULL) ? E_OUTOFMEMORY : S_OK;
  return pFilter;
}


void VideoMixingFilter::initParameters(void)
{
  addParameter(FILTER_PARAM_ORIENTATION, OLE_PARAM_ORIENTATION, &m_nOrientation);
  addParameter(FILTER_PARAM_SYNCHRONISATION, OLE_PARAM_SYNCHRONISATION, &m_nSynchronisation);
}


void VideoMixingFilter::InitialiseInputTypes(void)
{
  for(int i=0; i<sizeof(VideoMixSupportedFormats)/sizeof(GUID*); i++)
  {
    AddInputType(&MEDIATYPE_Video, VideoMixSupportedFormats[i], &FORMAT_VideoInfo);
    AddInputType(&MEDIATYPE_Video, VideoMixSupportedFormats[i], &FORMAT_VideoInfo2);
  }
// AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUYV, &FORMAT_VideoInfo);
// AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUY2, &FORMAT_VideoInfo);
// AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YVYU, &FORMAT_VideoInfo);
// AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_HDYC, &FORMAT_VideoInfo);
}


void VideoMixingFilter::InitialiseOutputTypes(void)
{
for(int i=0; i<sizeof(VideoMixSupportedFormats)/sizeof(GUID*); i++)
  {
    AddOutputType(&MEDIATYPE_Video, VideoMixSupportedFormats[i], &FORMAT_VideoInfo);
    AddOutputType(&MEDIATYPE_Video, VideoMixSupportedFormats[i], &FORMAT_VideoInfo2);
  }
//  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUYV, &FORMAT_VideoInfo);
//  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUY2, &FORMAT_VideoInfo);
//  AddOutputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YVYU, &FORMAT_VideoInfo);
}


/// This method renders output frame.
/// @TODO: It looks like it is called twice time than necessary, must be limited framerate.
HRESULT VideoMixingFilter::GenerateOutputSample(IMediaSample *pSample, int nIndex, const BYTE *pSampleBuffer0, const BYTE *pSampleBuffer1)
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

	////Set orientation
  if (m_nOrientation == 0)
  {
		// Horizontal 3 (RIGHT)
    m_pPicConcat->SetOrientation(PicConcatBase::RIGHT);
  }
  else
  {
		// Vertical = 1 (BOTTOM)
    m_pPicConcat->SetOrientation(PicConcatBase::BOTTOM);
  }

	// Concat the pictures
  if(pSampleBuffer0 && pSampleBuffer1)
  {
    if(m_FeedHeight>0 && m_FeedWidth>0) FeedRegion(pBufferOut);
    m_pPicConcat->Concat(pSampleBuffer0, pSampleBuffer1, pBufferOut, m_Vflip);    
  }
  else if(pSampleBuffer0)
  {
    const int nWidth = labs(m_pPicConcat->Get1stWidth());
    const int nHeight = labs(m_pPicConcat->Get1stHeight());
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
    const int nWidth = labs(m_pPicConcat->Get2ndWidth());
    const int nHeight = labs(m_pPicConcat->Get2ndHeight());
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


HRESULT VideoMixingFilter::ReceiveFirstSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  if(!m_vOutputPins[0]->IsConnected()) return S_FALSE;	// Do nothing when no output is connected.
  if(!m_vInputPins[1]->IsConnected())			// Do not modify anything  when input1 is not connected.
  {
    HRESULT hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);
    if(FAILED(hr)) DbgLog((EVENTLOG_WARNING_TYPE, 0, TEXT("VideoMixingFilter::ReceiveFirstSample(): m_vOutputPins[0]->GetInputPin()->Receive(pSample) Code: %X"), hr));
    return hr;
  }

	// Copy the secondary sample into our buffer
  BYTE *pBuffer = NULL;
  HRESULT hr = pSample->GetPointer(&pBuffer);
  if(FAILED(hr))
  {
    DbgLog((EVENTLOG_ERROR_TYPE, 0, TEXT("VideoMixingFilter::ReceiveFirstSample(): pSample->GetPointer(&pBuffer) Code: %X"), hr));
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


HRESULT VideoMixingFilter::ReceiveSecondSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  if(!m_vOutputPins[0]->IsConnected()) return S_FALSE;	// Do nothing when no output is connected.
  if(!m_vInputPins[0]->IsConnected())			// Do not modify anything  when input0 is not connected.
  {
    HRESULT hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);
    if(FAILED(hr)) DbgLog((EVENTLOG_WARNING_TYPE, 0, TEXT("VideoMixingFilter::ReceiveSecondSample(): m_vOutputPins[0]->GetInputPin()->Receive(pSample) Code: %X"), hr));
    return hr;
  }

	// Copy the secondary sample into our buffer  
  BYTE *pBuffer = NULL;
  HRESULT hr = pSample->GetPointer(&pBuffer);
  if(FAILED(hr))
  {
    DbgLog((EVENTLOG_ERROR_TYPE, 0, TEXT("VideoMixingFilter::ReceiveSecondSample(): pSample->GetPointer(&pBuffer) Code: %X"), hr));
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


HRESULT VideoMixingFilter::StartStreaming(void)
{
  if(m_pSampleBuffers[0]!=NULL) memset(m_pSampleBuffers[0], 0, m_nSampleSizes[0]);
  if(m_pSampleBuffers[1]!=NULL) memset(m_pSampleBuffers[1], 0, m_nSampleSizes[1]);
  m_FQ[0].Erase();
  m_FQ[1].Erase();
  TimeStart1 = TimeEnd1 = TimeStart2 = TimeEnd2 = 0;
  return VideoMixingBase::StartStreaming();
}


HRESULT VideoMixingFilter::StopStreaming(void)
{
  m_FQ[0].Erase();
  m_FQ[1].Erase();
  return VideoMixingBase::StopStreaming();
}


HRESULT VideoMixingFilter::CreateVideoMixer(const CMediaType *pMediaType, int nIndex)
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
  if(pMediaType->subtype==MEDIASUBTYPE_RGB555 || pMediaType->subtype==MEDIASUBTYPE_RGB565)
  {
    if(m_pPicConcat!=NULL)
    {
      if(m_pPicConcat->GetVideoFormat()!=16) 
      {
        delete m_pPicConcat; m_pPicConcat=NULL;
      }
    }
    if(m_pPicConcat==NULL)
    {
      m_pPicConcat = new PicConcat2B_Impl();
      m_nBitsPerPixel = 16;
    }
    return S_OK;
  }

  if(pMediaType->subtype==MEDIASUBTYPE_RGB24 || pMediaType->subtype==MEDIASUBTYPE_YV24)
  {
    if(m_pPicConcat!=NULL)
    {
      if(m_pPicConcat->GetVideoFormat()!=24) 
      {
        delete m_pPicConcat; m_pPicConcat=NULL;
      }
    }
    if(m_pPicConcat==NULL)
    {
      m_pPicConcat = new PicConcat3B_Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
    }
    return S_OK;
  }

  if(pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_ARGB32 || pMediaType->subtype==MEDIASUBTYPE_AYUV)
  {
    if(m_pPicConcat!=NULL)
    {
      if(m_pPicConcat->GetVideoFormat()!=32) 
      {
        delete m_pPicConcat; m_pPicConcat=NULL;
      }
    }
    if(m_pPicConcat==NULL)
    {
      m_pPicConcat = new PicConcat4B_Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
    }
    return S_OK;
  }

  if(pMediaType->subtype==MEDIASUBTYPE_R16G16B16 || pMediaType->subtype==MEDIASUBTYPE_B16G16R16)
  {
    if(m_pPicConcat!=NULL)
    {
      if(m_pPicConcat->GetVideoFormat()!=48) 
      {
        delete m_pPicConcat; m_pPicConcat=NULL;
      }
    }
    if(m_pPicConcat==NULL)
    {
      m_pPicConcat = new PicConcat6B_Impl();
      m_nBitsPerPixel = 48;
    }
    return S_OK;
  }

  if(pMediaType->subtype==MEDIASUBTYPE_A16R16G16B16 || pMediaType->subtype==MEDIASUBTYPE_A16B16G16R16)
  {
    if(m_pPicConcat!=NULL)
    {
      if(m_pPicConcat->GetVideoFormat()!=64) 
      {
        delete m_pPicConcat; m_pPicConcat=NULL;
      }
    }
    if(m_pPicConcat==NULL)
    {
      m_pPicConcat = new PicConcat8B_Impl();
      m_nBitsPerPixel = 64;
    }
    return S_OK;
  }

 return VFW_E_TYPE_NOT_ACCEPTED;
}


/// Pin disconnection must reshedule output geometry.
void VideoMixingFilter::OnDisconnect(int nIndex, RTVC_DIRECTION eDirection)
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


HRESULT VideoMixingFilter::SetOutputDimensions(const BITMAPINFOHEADER* pBmih1, const BITMAPINFOHEADER* pBmih2, int& nOutputWidth, int& nOutputHeight, int& nOutputSize)
{
  m_FeedHeight = m_FeedWidth = m_Feed_xPos = m_Feed_yPos = 0;
  if(pBmih1 && pBmih2)
  {
		// Verify that the dimensions match and set output width and height
    switch(m_nOrientation)
    {
	case 0: nOutputWidth = labs(pBmih1->biWidth) + labs(pBmih2->biWidth);
		nOutputHeight = MAX(labs(pBmih1->biHeight),labs(pBmih2->biHeight));
		if(labs(pBmih1->biHeight) != labs(pBmih2->biHeight)) 
		{
	          if(labs(pBmih1->biHeight) < labs(pBmih2->biHeight)) 
		  {
		    m_FeedWidth = labs(pBmih1->biWidth);
		    m_FeedHeight = nOutputHeight - labs(pBmih1->biHeight);
		  }
		  else
		  {
		    m_FeedWidth = labs(pBmih2->biWidth);
		    m_FeedHeight = nOutputHeight - labs(pBmih2->biHeight);
		    m_Feed_xPos = labs(pBmih1->biWidth);
		  }
		}
		break;
	case 1:
		nOutputWidth = MAX(labs(pBmih1->biWidth), labs(pBmih2->biWidth));
		nOutputHeight = labs(pBmih1->biHeight) + labs(pBmih2->biHeight);
	        if(labs(pBmih1->biWidth) != labs(pBmih2->biWidth))
		{                  
		  if(labs(pBmih1->biWidth) < labs(pBmih2->biWidth))
		  {
                    m_Feed_xPos = labs(pBmih1->biWidth);
		    m_FeedHeight = labs(pBmih1->biHeight);
		    m_Feed_yPos = nOutputHeight - m_FeedHeight;
		  }
		  else
		  {
                    m_Feed_xPos = labs(pBmih2->biWidth);
		    m_FeedHeight = labs(pBmih2->biHeight);
		  }
		  m_FeedWidth = nOutputWidth - m_Feed_xPos;
		}
		break;		
    }
    nOutputSize =  (nOutputWidth*nOutputHeight*m_nBitsPerPixel)/8;;

    if(m_nBitsPerPixel == BITS_PER_PIXEL_YUYV)
    {
      if((pBmih1->biWidth & 1) || (pBmih2->biWidth & 1)) return VFW_E_TYPE_NOT_ACCEPTED;
    }

		// Setup the picture concatenator
    if(m_pPicConcat)
    {
	m_pPicConcat->Set1stDimensions(pBmih1->biWidth, pBmih1->biHeight);
	m_pPicConcat->Set2ndDimensions(pBmih2->biWidth, pBmih2->biHeight);
    }
  }
  else
  {
    if(pBmih1)
    {
	nOutputWidth = pBmih1->biWidth;
	nOutputHeight = pBmih1->biHeight;
        nOutputSize =  m_nSampleSizes[0];
        if(m_pPicConcat)
        {
	  m_pPicConcat->Set1stDimensions(pBmih1->biWidth, pBmih1->biHeight);
	  m_pPicConcat->Set2ndDimensions(0, 0);
        }
    }
    else if(pBmih2)
    {
      nOutputWidth = pBmih2->biWidth;
      nOutputHeight = pBmih2->biHeight;
      nOutputSize =  m_nSampleSizes[1];
      if(m_pPicConcat)
      {
	m_pPicConcat->Set1stDimensions(0, 0);
	m_pPicConcat->Set2ndDimensions(pBmih2->biWidth, pBmih2->biHeight);
      }
    }
    else
    {
      nOutputWidth = 0;
      nOutputHeight = 0;
      nOutputSize =  0;
      if(m_pPicConcat)
      {
        m_pPicConcat->Set1stDimensions(0, 0);
        m_pPicConcat->Set2ndDimensions(0, 0);
      }      
    }
  }
  if(m_nBitsPerPixel == BITS_PER_PIXEL_YUYV)
  {
    if(nOutputWidth & 1) return VFW_E_TYPE_NOT_ACCEPTED;
  }
  if(m_pPicConcat)
    m_pPicConcat->SetOutDimensions(nOutputWidth, nOutputHeight);

return S_OK;
}


/// Final agreement on output geometry.
HRESULT VideoMixingFilter::CheckOutputType(const CMediaType* pMediaType)
{
	//Make sure the input and output types are related
  if(pMediaType->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if(m_pPicConcat==NULL) return E_POINTER;

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
    if(pBiOut->biHeight != m_pPicConcat->GetOutHeight())
    {
      if(labs(pBiOut->biHeight) == labs(m_pPicConcat->GetOutHeight()))
        m_Vflip = true;
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(pBiOut->biWidth != m_pPicConcat->GetOutWidth())
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
  }
  else
      return VFW_E_TYPE_NOT_ACCEPTED;

  switch(m_nBitsPerPixel)
  {
    case 16:
          if(*(pMediaType->Subtype()) == MEDIASUBTYPE_RGB565)
          {
            if((ConnectionFormat[0]==MEDIASUBTYPE_RGB565 && m_vInputPins[0]->IsConnected()) ||
               (ConnectionFormat[1]==MEDIASUBTYPE_RGB565 && m_vInputPins[1]->IsConnected()))
            return S_OK;
          }
          else if(*(pMediaType->Subtype()) == MEDIASUBTYPE_RGB555)
          {
	    if((ConnectionFormat[0]==MEDIASUBTYPE_RGB555 && m_vInputPins[0]->IsConnected()) ||
               (ConnectionFormat[1]==MEDIASUBTYPE_RGB555 && m_vInputPins[1]->IsConnected()))
            return S_OK;
          }
          break;
    case BITS_PER_PIXEL_RGB24:
          if(*(pMediaType->Subtype()) == MEDIASUBTYPE_RGB24)
          {
            if((ConnectionFormat[0]==MEDIASUBTYPE_RGB24 && m_vInputPins[0]->IsConnected()) ||
               (ConnectionFormat[1]==MEDIASUBTYPE_RGB24 && m_vInputPins[1]->IsConnected()))
            return S_OK;
          }
          else if(*(pMediaType->Subtype()) == MEDIASUBTYPE_YV24)
          {
	    if((ConnectionFormat[0]==MEDIASUBTYPE_YV24 && m_vInputPins[0]->IsConnected()) ||
               (ConnectionFormat[1]==MEDIASUBTYPE_YV24 && m_vInputPins[1]->IsConnected()))
            return S_OK;
          }
          break;
    case BITS_PER_PIXEL_RGB32:
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
	  break;
    case 48:
         if(*(pMediaType->Subtype())==MEDIASUBTYPE_R16G16B16)
         {
           if((ConnectionFormat[0]==MEDIASUBTYPE_R16G16B16 && m_vInputPins[0]->IsConnected()) ||
              (ConnectionFormat[1]==MEDIASUBTYPE_R16G16B16 && m_vInputPins[1]->IsConnected()))
               return S_OK;
         }
         else if(*(pMediaType->Subtype())==MEDIASUBTYPE_B16G16R16)
         {
           if((ConnectionFormat[0]==MEDIASUBTYPE_B16G16R16 && m_vInputPins[0]->IsConnected()) ||
              (ConnectionFormat[1]==MEDIASUBTYPE_B16G16R16 && m_vInputPins[1]->IsConnected()))
               return S_OK;
         }
         break;
    case 64:
         if(*(pMediaType->Subtype())==MEDIASUBTYPE_A16R16G16B16)
         {
           if((ConnectionFormat[0]==MEDIASUBTYPE_A16R16G16B16 && m_vInputPins[0]->IsConnected()) ||
              (ConnectionFormat[1]==MEDIASUBTYPE_A16R16G16B16 && m_vInputPins[1]->IsConnected()))
               return S_OK;
         }
         else if(*(pMediaType->Subtype())==MEDIASUBTYPE_A16B16G16R16)
         {
           if((ConnectionFormat[0]==MEDIASUBTYPE_A16B16G16R16 && m_vInputPins[0]->IsConnected()) ||
              (ConnectionFormat[1]==MEDIASUBTYPE_A16B16G16R16 && m_vInputPins[1]->IsConnected()))
               return S_OK;
         }
         break;
  }
  return S_FALSE;
}


void VideoMixingFilter::FeedRegion(void* pImg) const
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
HRESULT VideoMixingFilter::GetPages(CAUUID *pPages)
{
  if (pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if (pPages->pElems == NULL) 
  {
	return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_VideoMixingProperties;
  return S_OK;
}


HRESULT VideoMixingFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
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


HRESULT VideoMixingFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_VideoMixingFilter;
  return S_OK;
}
