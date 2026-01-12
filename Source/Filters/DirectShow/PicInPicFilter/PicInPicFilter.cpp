/** @file

MODULE				: PicInPicFilter

FILE NAME			: PicInPicFilter.cpp

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2017, CSIR
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
#include "stdafx.h"
#include "PicInPicFilter.h"
#include <DirectShow/CommonDefs.h>
#include <PicInPicRGB24Impl.h>
#include <PicInPicRGB32Impl.h>
#include <PicInPicYV24Impl.h>
#include <PicScalerRGB24Impl.h>
#include <PicScalerRGB32Impl.h>
#include <Dvdmedia.h>			// VIDEOINFOHEADER2

#ifdef USE_MMX
#include <PicScalerARGB32MMX.h>
#endif
#ifdef USE_SSE
 #include <PicScalerARGB32SSE.h>
#endif


#if defined(USE_MMX) || defined(USE_SSE)
extern "C" unsigned GetFeaturesCPU(void);
const unsigned FeaturesCPU = GetFeaturesCPU();
#endif

#include "../VariantTools.cpp"

#include <process.h>


DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);


PicScalerBase *GetPicScallerRGB32(void)
{
#ifdef USE_SSE
  if(FeaturesCPU & 2)
    return new PicScalerARGB32SSE();
#endif
#ifdef USE_MMX
  if(FeaturesCPU & 1)
      return new PicScalerARGB32MMX();
#endif
  return new PicScalerRGB32Impl();
}


PicInPicFilter::PicInPicFilter(): VideoMixingBase(NAME("DQ VPP Picture in Picture Filter"), 0, CLSID_VPP_PicInPicFilter),
  m_nTargetWidth(0),
  m_nTargetHeight(0),
  m_nSubPictureWidth(0),
  m_nSubPictureHeight(0),
  m_nCustomOffsetX(0),
  m_nCustomOffsetY(0),
  m_pPicInPic(NULL),
  m_pTargetPicScaler(NULL),
  m_pSubPicScaler(NULL),
  m_nBitsPerPixel(0),
  m_pBufferForScaledSecondaryImage(NULL),
  m_Vflip(false),
  m_DirtyProps(true)
  //ThreadStatus(THREAD_NOT_INITIALISED),
  //m_MediaStart(-1), m_MediaEnd(-1)
{
  //m_pSampleBuffers[0] = NULL;
  //m_pSampleBuffers[1] = NULL;

  m_nSampleSizes[0] = 0;
  m_nSampleSizes[1] = 0;

	// Init parameters
  initParameters();

  //unsigned int ThreadID;
  //MsgWakeUp = CreateEvent(NULL, TRUE, FALSE, NULL);
  //WriteThread = (HANDLE)_beginthreadex(NULL, 0, WorkerThread, this, 0, &ThreadID);
}


PicInPicFilter::~PicInPicFilter()
{
/*
  #ifdef _WIN32
  ThreadStatus = THREAD_STOPPING;
  //if(StopEvent) SetEvent(StopEvent);
  if(MsgWakeUp) SetEvent(MsgWakeUp);
  if(WriteThread)
  {
    if(WAIT_TIMEOUT==WaitForSingleObject(WriteThread,1000)) // Wait for thread to be finished WIN.
    {     
      TerminateThread(WriteThread,-1);		// Kill thread, that gets locked.
    }
  }
  if(WriteThread) CloseHandle(WriteThread);
  if(MsgWakeUp) CloseHandle(MsgWakeUp);
  //if(StopEvent) CloseHandle(StopEvent);
 #else
  StopEvent = 1;
  pthread_cond_signal(&MsgWakeUp);
  pthread_join(WriteThread, NULL);	// Wait for thread to be finished LNX.
 #endif
*/
  if(m_pPicInPic)
  {
    delete m_pPicInPic;
    m_pPicInPic = NULL;
  }
  if (m_pTargetPicScaler)
  {
    delete m_pTargetPicScaler;
    m_pTargetPicScaler = NULL;
  }
  if(m_pSubPicScaler)
  {
    delete m_pSubPicScaler;
    m_pSubPicScaler = NULL;
  }
  if(m_pBufferForScaledSecondaryImage)
  {
    free(m_pBufferForScaledSecondaryImage);
    m_pBufferForScaledSecondaryImage = NULL;
  }

/*
  for(int i=0; i<2; i++)
  {
    if (m_pSampleBuffers[i])
    {
      delete[] m_pSampleBuffers[i];
      m_pSampleBuffers[i] = NULL;
    }
  }
*/
}


CUnknown * WINAPI PicInPicFilter::CreateInstance( LPUNKNOWN pUnk, HRESULT *pHr )
{
  PicInPicFilter* pFilter = new PicInPicFilter();
  return pFilter;
}


void PicInPicFilter::initParameters()
{
  addParameter(FILTER_PARAM_TARGET_WIDTH, &m_nTargetWidth, 0);		// width output picture is scaled to
  addParameter(FILTER_PARAM_TARGET_HEIGHT, &m_nTargetHeight, 0);	// height output picture is scaled to

  addParameter(FILTER_PARAM_SUB_PIC_WIDTH, &m_nSubPictureWidth, 0);	// width inner picture is scaled to
  addParameter(FILTER_PARAM_SUB_PIC_HEIGHT, &m_nSubPictureHeight, 0);	// height inner picture is scaled to

  // currently unused: supposed to be one of the standard positions to insert 
  // the picture i.e. top left or top right corner.
  // addParameter(SUB_PICTURE_POSITION, &m_nPosition, (int)SUB_PIC_POSITION_1);
  addParameter(FILTER_PARAM_OFFSET_X, &m_nCustomOffsetX, 0);		// x offset from bottom left corner
  addParameter(FILTER_PARAM_OFFSET_Y, &m_nCustomOffsetY, 0);		// y offset from bottom left corner
}


/*
HRESULT PicInPicFilter::GenerateOutputSample(IMediaSample *pSample, int nIndex)
{
	// Prepare output sample
  AM_SAMPLE2_PROPERTIES * const pProps = m_vInputPins[nIndex]->SampleProps();
  DbgLog((LOG_TRACE,0,TEXT("Video Mixer: Sample Received from input 0: start time: %I64d end time: %I64d"), pProps->tStart, pProps->tStop));
  if(pProps->dwStreamId != AM_STREAM_MEDIA) 
  {
    return m_vInputPins[nIndex]->Receive(pSample);
  }

	// Provide frame rate limitation.
  if(m_iRepeatTime > 0)
  {
    const unsigned CurrentTick = GetTickCount_us();
    if(m_LastFrame_us+m_iRepeatTime - CurrentTick < (unsigned)0x8000000) return S_OK;
  }

  IMediaSample * pOutSample;
	// Set up the output sample
  HRESULT hr = CMultiIOBaseFilter::InitializeOutputSample(pSample, &pOutSample, nIndex, 0);
  if(FAILED(hr)) return hr;
  hr = pOutSample->SetActualDataLength(m_nOutputSize);
  if(FAILED(hr)) return hr;

  BYTE *pBufferOut = NULL;
  hr = pOutSample->GetPointer(&pBufferOut);
  if (FAILED(hr))
  {
    return hr;
  }

	// Prepare canvas
  if(m_pSampleBuffers[0])
  {
    if(m_pTargetPicScaler)	// the target scaler is only created if the input dimensions of the primary picture don't match the target dimensions
    {
      m_pTargetPicScaler->Scale((void*)pBufferOut, (void*)m_pSampleBuffers[0], m_Vflip);
    }
    else
    {
      if(m_Vflip)	// Code to flip image vertically
        VFlipImg1(m_pSampleBuffers[0],pBufferOut);
      else
        memcpy(pBufferOut, m_pSampleBuffers[0], (labs(m_pPicInPic->GetWidth()*m_pPicInPic->GetHeight())*(size_t)m_nBitsPerPixel)/8);
    }
  }
  else
  {		// TODO: is zeros the right filler
    memset(pBufferOut, 0, labs(m_nOutputWidth * m_nOutputHeight* m_nBitsPerPixel)/8);
  }

	// do pic in pic transform
  if(m_pSampleBuffers[0] && m_pSampleBuffers[1])
  {    
    if(m_pSubPicScaler)
    {
      m_pSubPicScaler->Scale((void*)m_pBufferForScaledSecondaryImage, (void*)m_pSampleBuffers[1]);
      m_pPicInPic->Insert((void*)m_pBufferForScaledSecondaryImage, pBufferOut, m_Vflip);
    }
    else	// copy direct from buffer
    {
      m_pPicInPic->Insert((void*)m_pSampleBuffers[1], pBufferOut,m_Vflip);
    }
  }
  else if(m_pSampleBuffers[1])
  {
    if(m_pSubPicScaler)
    {
      m_pSubPicScaler->Scale((void*)m_pBufferForScaledSecondaryImage, (void*)m_pSampleBuffers[1]);
      m_pPicInPic->Insert((void*)m_pBufferForScaledSecondaryImage, pBufferOut,m_Vflip);
    }
    else
    {		// copy direct from buffer
      m_pPicInPic->Insert((void*)m_pSampleBuffers[1], pBufferOut,m_Vflip);
    }
  }

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
      m_LastFrame_us += m_iRepeatTime;
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
*/


void PicInPicFilter::VFlipImg1(const BYTE* pSrc, BYTE* pDest)
{
const int nRowLength = (m_pPicInPic->GetWidth() * m_nBitsPerPixel)/8;
 ASSERT(pSrc!=NULL);
 ASSERT(pDest!=NULL);
 pDest += ((labs(m_pPicInPic->GetHeight()) - 1) * nRowLength);
 int i = m_pPicInPic->GetHeight();
 while(i-- > 0)
 {
   memcpy(pDest, pSrc, nRowLength);
   pSrc+=nRowLength; pDest-=nRowLength;
 }
}


HRESULT PicInPicFilter::ReceiveFirstSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  if(!m_vOutputPins[0]->IsConnected()) return S_FALSE;	// Do nothing when no output is connected.

  BYTE *pBufferIn1 = NULL;
  HRESULT hr = pSample->GetPointer(&pBufferIn1);
  if(FAILED(hr)) return hr;
  if(pBufferIn1==NULL) return E_POINTER;
  long SampleSize1 = pSample->GetSize();

  if(SampleSize1 < m_nSampleSizes[0]) return E_INVALIDARG;		// Check buffer size for frame 0.

  // TODO: UsingDifferentAllocators InputPin()->PeekAllocator() != OutputPin()->PeekAllocator();

	// When only input1 is connected, do not cache anything.
  if(!m_vInputPins[1]->IsConnected() || m_pBufferForScaledSecondaryImage==NULL)
  {
    if(m_pTargetPicScaler==NULL && !m_Vflip)    
      hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);		// Populate intact sample to the output.
    else
    {
      IMediaSample *pOutSample = NULL;
      {
        CAutoLock lck(&m_csReceive);
        HRESULT hr = CMultiIOBaseFilter::InitializeOutputSample(pSample, &pOutSample, 0, 0);	// Set up the output sample
        if(FAILED(hr)) return hr;
        hr = pOutSample->SetActualDataLength(m_nOutputSize);
        if(FAILED(hr)) return hr;
        BYTE *pBufferOut = NULL;
        hr = pOutSample->GetPointer(&pBufferOut);
        if(FAILED(hr)) return hr;
        if(m_pTargetPicScaler)
            m_pTargetPicScaler->Scale(pBufferOut, pBufferIn1, m_Vflip);
        else
            VFlipImg1(pBufferIn1,pBufferOut);
      }
      pOutSample->SetSyncPoint(TRUE);
      hr = m_vOutputPins[0]->GetInputPin()->Receive(pOutSample);
      pOutSample->Release();
    }
    m_LastFrame_us += m_iRepeatTime;
    return hr;   
  }

  if(m_pTargetPicScaler==NULL && !m_Vflip)
  {
    m_pPicInPic->Insert(m_pBufferForScaledSecondaryImage, pBufferIn1, m_Vflip);
    //pOutSample->SetSyncPoint(TRUE);
    hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);
    m_LastFrame_us += m_iRepeatTime;
  }
  else
  {    
    IMediaSample *pOutSample = NULL;
    {
      CAutoLock lck(&m_csReceive);		// There is questionable wchich part of a code should be locked. I guess InitializeOutputSample should be sufficient.
      hr = CMultiIOBaseFilter::InitializeOutputSample(pSample, &pOutSample, 0, 0);
      if(FAILED(hr) || pOutSample==NULL) return hr;
      hr = pOutSample->SetActualDataLength(m_nOutputSize);
      if(FAILED(hr)) return hr;
      BYTE *pBufferOut = NULL;
      hr = pOutSample->GetPointer(&pBufferOut);
      if (FAILED(hr)) return hr;  
      if(m_pTargetPicScaler)	// the target scaler is only created if the input dimensions of the primary picture don't match the target dimensions
      {
        m_pTargetPicScaler->Scale((void*)pBufferOut, pBufferIn1, m_Vflip);
      }
      else
      {
        if(m_Vflip)	// Code to flip image vertically
          VFlipImg1(pBufferIn1,pBufferOut);
        else
          memcpy(pBufferOut, pBufferIn1, m_nSampleSizes[0]);
      }

      m_pPicInPic->Insert(m_pBufferForScaledSecondaryImage, pBufferOut, m_Vflip);
    }
    pOutSample->SetSyncPoint(TRUE);
    hr = m_vOutputPins[0]->GetInputPin()->Receive(pOutSample);		// Call downstream pin callback outside lock.
    m_LastFrame_us += m_iRepeatTime;
    pOutSample->Release();
  }
  return hr;


	// Copy the primary sample into our buffer
//  if(m_pSampleBuffers[0]==NULL) return E_POINTER;
//  memcpy(m_pSampleBuffers[0], pBufferIn1, m_nSampleSizes[0]);		// Copy the buffer

/*
 if(pSample->GetMediaTime(&m_MediaStart,&m_MediaEnd) != NOERROR) 
  {
    m_MediaStart = m_MediaEnd = -1;
  }
#ifdef _WIN32
    SetEvent(MsgWakeUp);
#else
    pthread_cond_signal(&MsgWakeUp);
#endif
  return S_OK;
*/
 //return GenerateOutputSample(pSample, 0);
}


/// The second video only asynchronously copies frame to a buffer.
HRESULT PicInPicFilter::ReceiveSecondSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  if(!m_vOutputPins[0]->IsConnected()) return S_FALSE;	// Do nothing when no output is connected.
  if(m_pBufferForScaledSecondaryImage==NULL) return E_UNEXPECTED;	// This should not occur, buffer should be allocated when 2nd video is attached.

	// Copy the secondary sample into our buffer  
  BYTE *pBuffer = NULL;
  HRESULT hr = pSample->GetPointer(&pBuffer);
  if(FAILED(hr)) return hr;
  if(pBuffer==NULL) return E_POINTER;
  long SampleSize2 = pSample->GetSize();
  if(SampleSize2 < m_nSampleSizes[1]) return E_INVALIDARG;		// Check buffer size for frame 1.

  //if(!m_vInputPins[0]->IsConnected())		// No video 1 is present.
  //{
  //}

  // CAutoLock lck(&m_csReceive); no need
  if(m_pSubPicScaler)
  {
    m_pSubPicScaler->Scale(m_pBufferForScaledSecondaryImage, pBuffer);
  }
  else		// copy direct from buffer
  {
    memcpy(m_pBufferForScaledSecondaryImage, pBuffer, m_nSampleSizes[1]);	// Copy the buffer
  }
 
  return S_OK;
}


/*
HRESULT PicInPicFilter::InitializeOutputSample(IMediaSample **ppOutSample, int nInputIndex, int nOutputIndex)
{
	AM_SAMPLE2_PROPERTIES * const pProperties = m_vInputPins[nInputIndex]->SampleProps();

	IMediaSample *pOutSample;
	//Let's get rid of the const

	AM_SAMPLE2_PROPERTIES* pProps = (AM_SAMPLE2_PROPERTIES*)pProperties;
	// default - times are the same
	DWORD dwFlags = 0;
	// This will prevent the image renderer from switching us to DirectDraw
	// when we can't do it without skipping frames because we're not on a
	// keyframe.  If it really has to switch us, it still will, but then we
	// will have to wait for the next keyframe
	if (!(pProps->dwSampleFlags & AM_SAMPLE_SPLICEPOINT)) {
		dwFlags |= AM_GBF_NOTASYNCPOINT;
	}

	ASSERT(m_vOutputPins[nOutputIndex]->GetAllocator() != NULL);
	HRESULT hr = m_vOutputPins[nOutputIndex]->GetAllocator()->GetBuffer(
		&pOutSample
		, pProps->dwSampleFlags & AM_SAMPLE_TIMEVALID ?
		&pProps->tStart : NULL
		, pProps->dwSampleFlags & AM_SAMPLE_STOPVALID ?
		&pProps->tStop : NULL
		, dwFlags
		);
	*ppOutSample = pOutSample;
	if (FAILED(hr)) {
		return hr;
	}

	ASSERT(pOutSample);
	IMediaSample2 *pOutSample2;
	if (SUCCEEDED(pOutSample->QueryInterface(IID_IMediaSample2,
		(void **)&pOutSample2))) {
			//  Modify it
			AM_SAMPLE2_PROPERTIES OutProps;
			EXECUTE_ASSERT(SUCCEEDED(pOutSample2->GetProperties(
				FIELD_OFFSET(AM_SAMPLE2_PROPERTIES, tStart), (PBYTE)&OutProps)
				));
			OutProps.dwTypeSpecificFlags = pProps->dwTypeSpecificFlags;
			OutProps.dwSampleFlags =
				(OutProps.dwSampleFlags & AM_SAMPLE_TYPECHANGED) |
				(pProps->dwSampleFlags & ~AM_SAMPLE_TYPECHANGED);
			OutProps.tStart = pProps->tStart;
			OutProps.tStop  = pProps->tStop;
			OutProps.cbData = FIELD_OFFSET(AM_SAMPLE2_PROPERTIES, dwStreamId);
			hr = pOutSample2->SetProperties(
				FIELD_OFFSET(AM_SAMPLE2_PROPERTIES, dwStreamId),
				(PBYTE)&OutProps
				);
			pOutSample2->Release();
	} else {
		if (pProps->dwSampleFlags & AM_SAMPLE_TIMEVALID) {
			pOutSample->SetTime(&pProps->tStart,
				&pProps->tStop);
		}
		if (pProps->dwSampleFlags & AM_SAMPLE_SPLICEPOINT) {
			pOutSample->SetSyncPoint(TRUE);
		}
		if (pProps->dwSampleFlags & AM_SAMPLE_DATADISCONTINUITY) {
			pOutSample->SetDiscontinuity(TRUE);
		}
		// Copy the media times
		
		if(m_MediaStart>=0) {
			pOutSample->SetMediaTime(&m_MediaStart,&m_MediaEnd);
		}
	}
	return S_OK;
}
*/


/** 
#ifdef _WIN32
unsigned WINAPI
#else
void *
#endif
        PicInPicFilter::WorkerThread(void* arg)
{
DWORD waitobj;
  PicInPicFilter *Filter = (PicInPicFilter*)arg;
  //HANDLE Events[] = {Filter->StopEvent, Filter->MsgWakeUp};
  HRESULT hr;

  hr = CoInitializeEx(0, COINIT_MULTITHREADED|COINIT_DISABLE_OLE1DDE);

  Filter->ThreadStatus = THREAD_RUNNING;

  //while((waitobj=WaitForMultipleObjects(sizeof(Events)/sizeof(HANDLE), Events, FALSE, INFINITE)) != WAIT_OBJECT_0)

  while(1)
  {
    DWORD waitobj  = WaitForSingleObject(Filter->MsgWakeUp,INFINITE);
    if(Filter->ThreadStatus != THREAD_RUNNING) break;
    //if(waitobj != WAIT_OBJECT_0)
    //{
    //    ResetEvent(Filter->MsgWakeUp);
    //    continue;
    //}
    ResetEvent(Filter->MsgWakeUp);

    IMediaSample *pOutSample = NULL;
	// Set up the output sample
    hr = Filter->InitializeOutputSample(&pOutSample, 0, 0);
    if(FAILED(hr)) 
    {
      if(pOutSample) pOutSample->Release();
      continue;
    }
    hr = pOutSample->SetActualDataLength(Filter->m_nOutputSize);
    if(FAILED(hr)) {pOutSample->Release();continue;}

    BYTE *pBufferOut = NULL;
    hr = pOutSample->GetPointer(&pBufferOut);
    if(FAILED(hr))
    {
      pOutSample->Release();
      continue;
    }

	// Prepare canvas
  if(Filter->m_pSampleBuffers[0])
  {
    if(Filter->m_pTargetPicScaler)	// the target scaler is only created if the input dimensions of the primary picture don't match the target dimensions
    {
      Filter->m_pTargetPicScaler->Scale((void*)pBufferOut, (void*)Filter->m_pSampleBuffers[0], Filter->m_Vflip);
    }
    else
    {
      if(Filter->m_Vflip)	// Code to flip image vertically
        Filter->VFlipImg1(Filter->m_pSampleBuffers[0],pBufferOut);
      else
        memcpy(pBufferOut, Filter->m_pSampleBuffers[0], (labs(Filter->m_pPicInPic->GetWidth()*Filter->m_pPicInPic->GetHeight())*(size_t)Filter->m_nBitsPerPixel)/8);
    }
  }
  else
  {		// TODO: is zeros the right filler
    memset(pBufferOut, 0, labs(Filter->m_nOutputWidth * Filter->m_nOutputHeight* Filter->m_nBitsPerPixel)/8);
  }

	// do pic in pic transform
  if(Filter->m_pSampleBuffers[0] && Filter->m_pSampleBuffers[1])
  {    
    if(Filter->m_pSubPicScaler)
    {
      Filter->m_pSubPicScaler->Scale((void*)Filter->m_pBufferForScaledSecondaryImage, (void*)Filter->m_pSampleBuffers[1]);
      Filter->m_pPicInPic->Insert((void*)Filter->m_pBufferForScaledSecondaryImage, pBufferOut, Filter->m_Vflip);
    }
    else	// copy direct from buffer
    {
      Filter->m_pPicInPic->Insert((void*)Filter->m_pSampleBuffers[1], pBufferOut,Filter->m_Vflip);
    }
  }
    else if(Filter->m_pSampleBuffers[1])
    {
      if(Filter->m_pSubPicScaler)
      {
        Filter->m_pSubPicScaler->Scale((void*)Filter->m_pBufferForScaledSecondaryImage, (void*)Filter->m_pSampleBuffers[1]);
        Filter->m_pPicInPic->Insert((void*)Filter->m_pBufferForScaledSecondaryImage, pBufferOut,Filter->m_Vflip);
      }
      else
      {		// copy direct from buffer
        Filter->m_pPicInPic->Insert((void*)Filter->m_pSampleBuffers[1], pBufferOut, Filter->m_Vflip);
      }
    }

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
      if(hr == NOERROR) 
      {
        hr = Filter->m_vOutputPins[0]->GetInputPin()->Receive(pOutSample);
        Filter->m_LastFrame_us += Filter->m_iRepeatTime;
      } 
      else 
      {
	  // S_FALSE returned from Transform is a PRIVATE agreement
	  // We should return NOERROR from Receive() in this cause because returning S_FALSE
	  // from Receive() means that this is the end of the stream and no more data should
	  // be sent.
        if(S_FALSE == hr) 
        {
	  //  Release the sample before calling notify to avoid
	  //  deadlocks if the sample holds a lock on the system
	  //  such as DirectDraw buffers do
          pOutSample->Release();
          continue;
        }
      }
    }
	// release the output buffer. If the connected pin still needs it,
	// it will have addrefed it itself.
    pOutSample->Release();
  }

  CoUninitialize();
  Filter->ThreadStatus = THREAD_STOPPED;
return 0;
}
*/


HRESULT PicInPicFilter::StartStreaming(void)
{
  //if(m_pSampleBuffers[0]!=NULL) memset(m_pSampleBuffers[0], 0, m_nSampleSizes[0]);
  //if(m_pSampleBuffers[1]!=NULL) memset(m_pSampleBuffers[1], 0, m_nSampleSizes[1]);
  //m_MediaStart = m_MediaEnd = -1;
  if(m_pBufferForScaledSecondaryImage!=NULL && m_szBufferForScaledSecondaryImage>0)
      memset(m_pBufferForScaledSecondaryImage, 0, m_szBufferForScaledSecondaryImage);
  return VideoMixingBase::StartStreaming();
}


HRESULT PicInPicFilter::CreateVideoMixer(const CMediaType *pMediaType, int nIndex)
{
  const BITMAPINFOHEADER *bmih = NULL;
	// Create temporary sample buffers
  if(pMediaType->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER* const pVih = (const VIDEOINFOHEADER*) pMediaType->pbFormat;
    if(pVih)
    {
      if(pVih->AvgTimePerFrame>0) m_iRepeatTime=pVih->AvgTimePerFrame/10;
      bmih = &pVih->bmiHeader;
    }
  }
  else if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2* const pVih2 = (const VIDEOINFOHEADER2*) pMediaType->pbFormat;
    if(pVih2)
    {
      if(pVih2->AvgTimePerFrame>0) m_iRepeatTime=pVih2->AvgTimePerFrame/10;
      bmih = &pVih2->bmiHeader;
    }
  }
  if(bmih==NULL) return E_FAIL;

  if(nIndex>=2) return E_INVALIDARG;
  m_nSampleSizes[nIndex] = DIBSIZE(*bmih);
/*
  if(m_pSampleBuffers[nIndex])
  {		// Recreate in case dimensions have changed
    delete[] m_pSampleBuffers[nIndex];
  }
  m_pSampleBuffers[nIndex] = new BYTE[m_nSampleSizes[nIndex]];
*/

  if(m_pPicInPic!=NULL)
  {
    if(!m_pPicInPic->isVideoFormatSupported(pMediaType->subtype))
    {
      delete m_pPicInPic;
      m_pPicInPic = NULL;      
    }
  }
  if(m_pTargetPicScaler)
  {
    if(!m_pTargetPicScaler->isVideoFormatSupported(pMediaType->subtype))
    {
      delete m_pTargetPicScaler;
      m_pTargetPicScaler = NULL;
    }
  }

	// Create appropriate picture concatenator
  if(pMediaType->subtype==MEDIASUBTYPE_YV24)
  {    
    if(m_pPicInPic==NULL)
    {
      m_pPicInPic = new PicInPicYV24Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
    }
    return S_OK;
  }
  if(pMediaType->subtype==MEDIASUBTYPE_RGB24)
  {    
    if(m_pPicInPic==NULL)
    {
      m_pPicInPic = new PicInPicRGB24Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
    }
    return S_OK;
  }
  else if(pMediaType->subtype==MEDIASUBTYPE_ARGB32 || pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_AYUV)
  {
    if(m_pPicInPic==NULL)
    {
      m_pPicInPic = new PicInPicRGB32Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
    }
    return S_OK;
  }

  return VFW_E_TYPE_NOT_ACCEPTED;
}


HRESULT PicInPicFilter::SetOutputDimensions(const BITMAPINFOHEADER* pBmih1, const BITMAPINFOHEADER* pBmih2, int& nOutputWidth, int& nOutputHeight, int& nOutputSize)
{
	// target picture scaler
  if(m_nTargetWidth>0 && m_nTargetHeight>0)
  {
	// if target width and height have been set use those dimensions
    nOutputWidth = m_nTargetWidth;
    nOutputHeight = m_nTargetHeight;
    nOutputSize = (m_nOutputWidth * m_nOutputHeight * m_nBitsPerPixel) / 8;
  }
  else
  {
    if(pBmih1==NULL)
    {
      if(pBmih2==NULL) return E_POINTER;
      nOutputSize = m_nSampleSizes[1];
      nOutputWidth = pBmih2->biWidth;
      nOutputHeight = pBmih2->biHeight;
    }
    else
    {	// Use size of first input
      nOutputSize = m_nSampleSizes[0];
      nOutputWidth = pBmih1->biWidth;
      nOutputHeight = pBmih1->biHeight;
    }
  }  

  reconfigure();
  return S_OK;
}


HRESULT PicInPicFilter::CheckOutputType(const CMediaType *pMediaType)
{
	//Make sure the input and output types are related
  if(pMediaType->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  	// Video format
  m_Vflip = false;
  if(pMediaType->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER * const pVihOut = (const VIDEOINFOHEADER*)pMediaType->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(pBiOut->biHeight != m_nOutputHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutputHeight))
        m_Vflip = true;
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(pBiOut->biWidth != m_nOutputWidth)
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
  }
  else if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2 * const pVihOut = (const VIDEOINFOHEADER2*)pMediaType->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(pBiOut->biHeight != m_nTargetHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutputHeight))
        m_Vflip = true;
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(pBiOut->biWidth != m_nOutputWidth)
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
  }
  else
      return VFW_E_TYPE_NOT_ACCEPTED;


  if(m_nBitsPerPixel == BITS_PER_PIXEL_RGB24)
  {
    if(*(pMediaType->Subtype()) == MEDIASUBTYPE_YV24)
    {
      if(ConnectionFormat[0]==MEDIASUBTYPE_YV24 || ConnectionFormat[1]==MEDIASUBTYPE_YV24)
          return S_OK;
    }
    else if(pMediaType->subtype == MEDIASUBTYPE_RGB24)
    {
      if(ConnectionFormat[0]==MEDIASUBTYPE_RGB24 || ConnectionFormat[1]==MEDIASUBTYPE_RGB24)
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
  return S_FALSE;
}


/// Override to control state dependent behaviour
STDMETHODIMP PicInPicFilter::SetParameter( const char* type, const char* value )
{
HRESULT hr;
	// For now, one cannot set any parameters once the output has been connected -> this will affect the buffersize
  if(m_vOutputPins[0] && m_vOutputPins[0]->IsConnected())
  {
    if(parameterChangeAffectsOutput(type))
    {
      char Buffer[80];
      int pLength;
      hr = CSettingsInterface::GetParameter(type,sizeof(Buffer),Buffer,&pLength);
      if(FAILED(hr)) return hr;
      if(strcmp(Buffer,value)) return E_FAIL;
      return S_FALSE;			// Nothing to change, parameter is identical.
    }
  }

  hr = CSettingsInterface::SetParameter(type,value);
  if(SUCCEEDED(hr)) reconfigure();
  return hr;
}


bool PicInPicFilter::parameterChangeAffectsOutput( const char* szParam )
{
  if((strcmp(szParam, FILTER_PARAM_TARGET_WIDTH) == 0) ||
		 ( strcmp(szParam, FILTER_PARAM_TARGET_HEIGHT) == 0))
  {
    return true;
  }
  return false;
}


void PicInPicFilter::reconfigure(void)
{
  const BITMAPINFOHEADER* pBmih1 = &m_VideoInHeader[0].bmiHeader;
  const BITMAPINFOHEADER* pBmih2 = &m_VideoInHeader[1].bmiHeader;

  if(m_pBufferForScaledSecondaryImage != NULL)
  {
    free(m_pBufferForScaledSecondaryImage);
    m_pBufferForScaledSecondaryImage = NULL;
  }

	// target picture scaler
  if(m_nTargetWidth > 0 && m_nTargetHeight > 0)
  {
    if(m_pTargetPicScaler)
    {
      if(m_nBitsPerPixel!=m_pTargetPicScaler->GetVideoFormat())
      {
	delete m_pTargetPicScaler;
	m_pTargetPicScaler = NULL;
      }
    }
    if(!m_pTargetPicScaler)
    {
      switch(m_nBitsPerPixel)
      {
        case BITS_PER_PIXEL_RGB24: m_pTargetPicScaler = new PicScalerRGB24Impl();
				   break;
        case BITS_PER_PIXEL_RGB32: m_pTargetPicScaler = GetPicScallerRGB32();
				   break;
        default: return;
      }
    }
    m_pTargetPicScaler->SetInDimensions(pBmih1->biWidth, pBmih1->biHeight);
    m_pTargetPicScaler->SetOutDimensions(m_nTargetWidth, m_nTargetHeight);
  }
  else
  {
    if (m_pTargetPicScaler)
    {
      delete m_pTargetPicScaler;
      m_pTargetPicScaler = NULL;
    }
  }

	// Check sub picture dimensions.
  int iSecondaryWidth = pBmih2->biWidth;
  int iSecondaryHeight = pBmih2->biHeight;
	// check if pin of subpicture has been connected
	// we can only configure the pip once both pins have been connected
  if(iSecondaryWidth>0 || iSecondaryHeight>0)
  {
    if(m_nSubPictureWidth>getOutputWidth() || m_nSubPictureHeight>getOutputHeight() ||
       (m_nSubPictureWidth==0 && iSecondaryWidth>getOutputWidth()) ||
       (m_nSubPictureHeight==0 && iSecondaryHeight>getOutputHeight()))
    {
      m_nSubPictureWidth = iSecondaryWidth;
      m_nSubPictureHeight = iSecondaryHeight;

	// If the inner picture is bigger than half the target picture
      if(m_nSubPictureWidth>getOutputWidth())
	  m_nSubPictureWidth = getOutputWidth()/2;
      if(m_nSubPictureHeight>getOutputHeight())
	  m_nSubPictureHeight = getOutputHeight()/2;
    }

    m_pPicInPic->SetDimensions(getOutputWidth(), getOutputHeight());

    if((m_nSubPictureWidth!=0 && iSecondaryWidth!=m_nSubPictureWidth) || 
       (m_nSubPictureHeight!=0 && iSecondaryHeight!=m_nSubPictureHeight))
    {
      if(m_pSubPicScaler)
      {
        if(m_nBitsPerPixel != m_pSubPicScaler->GetVideoFormat())
        {
	  delete m_pSubPicScaler;
	  m_pSubPicScaler = NULL;
        }
      }
      if(!m_pSubPicScaler)
      {
        switch(m_nBitsPerPixel)
        {
          case BITS_PER_PIXEL_RGB24: m_pSubPicScaler = new PicScalerRGB24Impl();
				     break;
          case BITS_PER_PIXEL_RGB32: m_pSubPicScaler = GetPicScallerRGB32();
				     break;
          default:		     return;
        }
      }
      m_pSubPicScaler->SetInDimensions(iSecondaryWidth, iSecondaryHeight);
      m_pSubPicScaler->SetOutDimensions(m_nSubPictureWidth, m_nSubPictureHeight);

	// recreate 2nd picture buffer
      m_szBufferForScaledSecondaryImage = labs(m_nSubPictureWidth * m_nSubPictureHeight * m_nBitsPerPixel) / 8;
      if(m_szBufferForScaledSecondaryImage>0)
          m_pBufferForScaledSecondaryImage = (BYTE*)malloc(m_szBufferForScaledSecondaryImage);

	// Check validity of sub picture position
      if(m_nCustomOffsetX + m_nSubPictureWidth > getOutputWidth())
      {
        m_nCustomOffsetX = getOutputWidth() - m_nSubPictureWidth;
      }
      if(m_nCustomOffsetY + m_nSubPictureHeight > getOutputHeight())
      {
        m_nCustomOffsetY = getOutputHeight() - m_nSubPictureHeight;
      }
      m_pPicInPic->SetSubDimensions(m_nSubPictureWidth, m_nSubPictureHeight);
    }
    else
    {
      if (m_pSubPicScaler)
      {
        delete m_pSubPicScaler;
        m_pSubPicScaler = NULL;
      }
	// recreate 2nd picture buffer
      m_szBufferForScaledSecondaryImage = labs(iSecondaryWidth*iSecondaryHeight*m_nBitsPerPixel) / 8;
      if(m_szBufferForScaledSecondaryImage>0)
          m_pBufferForScaledSecondaryImage = (BYTE*)malloc(m_szBufferForScaledSecondaryImage);

      if(m_nCustomOffsetX + iSecondaryWidth > getOutputWidth())
      {
        m_nCustomOffsetX = getOutputWidth() - iSecondaryWidth;
      }
      if(m_nCustomOffsetY + iSecondaryHeight > getOutputHeight())
      {
        m_nCustomOffsetY = getOutputHeight() - iSecondaryHeight;
      }
      m_pPicInPic->SetSubDimensions(iSecondaryWidth, iSecondaryHeight);
    }

    if(m_nCustomOffsetX<0) m_nCustomOffsetX=0;
    if(m_nCustomOffsetY<0) m_nCustomOffsetY=0;

	// Setup the picture in picture
    m_pPicInPic->SetPos(m_nCustomOffsetX, m_nCustomOffsetY);
  }
  else		// No source or target image resize.
  {
    m_pPicInPic->SetDimensions(pBmih1->biWidth, pBmih1->biHeight);
	// recreate 2nd picture buffer
    m_szBufferForScaledSecondaryImage = labs(iSecondaryWidth*iSecondaryHeight*m_nBitsPerPixel) / 8;
    if(m_szBufferForScaledSecondaryImage>0)
      m_pBufferForScaledSecondaryImage = (BYTE*)malloc(m_szBufferForScaledSecondaryImage);
  }
}


/// For configuration dialog
HRESULT PicInPicFilter::GetPages(CAUUID *pPages)
{
  if(pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if(pPages->pElems == NULL) 
  {
	return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_PicInPicProperties;
  return S_OK;
}


/// For configuration dialog
HRESULT PicInPicFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == IID_ISpecifyPropertyPages) return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  if(riid == (IID_ISettingsInterface))  return GetInterface((ISettingsInterface*) this, ppv);
  if(riid == IID_IPersistPropertyBag)   return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPropertyBag)          return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_IPersistStream)        return GetInterface((IPersistStream*) this, ppv);

  			// Call the parent class.
  return VideoMixingBase::NonDelegatingQueryInterface(riid, ppv);
}


//----------------- IPersistPropertyBag------------------


HRESULT PicInPicFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_PicInPicFilter;
  return S_OK;
}


HRESULT PicInPicFilter::InitNew(void)
{
  m_nTargetWidth = m_nTargetHeight = 0;
  return S_OK;
}


OLECHAR OLE_PARAM_TARGET_WIDTH[] =  olestr(FILTER_PARAM_TARGET_WIDTH);  // L"TEST";  //  sstr(FILTER_PARAM_TARGET_WIDTH);
OLECHAR OLE_PARAM_TARGET_HEIGHT[] = olestr(FILTER_PARAM_TARGET_HEIGHT);
OLECHAR OLE_PARAM_SUB_PIC_WIDTH[] =  olestr(FILTER_PARAM_SUB_PIC_WIDTH);
OLECHAR OLE_PARAM_SUB_PIC_HEIGHT[] = olestr(FILTER_PARAM_SUB_PIC_HEIGHT);
OLECHAR OLE_PARAM_OFFSET_X[] =  olestr(FILTER_PARAM_OFFSET_X);
OLECHAR OLE_PARAM_OFFSET_Y[] = olestr(FILTER_PARAM_OFFSET_Y);


HRESULT PicInPicFilter::Load(IPropertyBag *pPropBag, IErrorLog *pErrorLog)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  memset(&var,0,sizeof(var));

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_TARGET_WIDTH));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TARGET_WIDTH,sizeof(OLE_PARAM_TARGET_WIDTH));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      IntFromVariant(m_nTargetWidth,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_TARGET_HEIGHT));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TARGET_HEIGHT,sizeof(OLE_PARAM_TARGET_HEIGHT));
    VariantClear(&var);
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      IntFromVariant(m_nTargetHeight,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_SUB_PIC_WIDTH));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_SUB_PIC_WIDTH,sizeof(OLE_PARAM_SUB_PIC_WIDTH));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      IntFromVariant(m_nSubPictureWidth,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_SUB_PIC_HEIGHT));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_SUB_PIC_HEIGHT,sizeof(OLE_PARAM_SUB_PIC_HEIGHT));
    VariantClear(&var);
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      IntFromVariant(m_nSubPictureHeight,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_OFFSET_X));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_OFFSET_X,sizeof(OLE_PARAM_OFFSET_X));
    VariantClear(&var);
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      IntFromVariant(m_nCustomOffsetX,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_OFFSET_Y));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_OFFSET_Y,sizeof(OLE_PARAM_OFFSET_Y));
    VariantClear(&var);
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      IntFromVariant(m_nCustomOffsetY,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  VariantClear(&var);
return S_OK;
}


HRESULT PicInPicFilter::Save(IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_TARGET_WIDTH));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TARGET_WIDTH,sizeof(OLE_PARAM_TARGET_WIDTH));
    var.vt = VT_INT;
    var.intVal = m_nTargetWidth;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }
  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_TARGET_HEIGHT));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TARGET_HEIGHT,sizeof(OLE_PARAM_TARGET_HEIGHT));
    var.vt = VT_INT;
    var.intVal = m_nTargetHeight;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }
  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_SUB_PIC_WIDTH));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_SUB_PIC_WIDTH,sizeof(OLE_PARAM_SUB_PIC_WIDTH));
    var.vt = VT_INT;
    var.intVal = m_nSubPictureWidth;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }
  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_SUB_PIC_HEIGHT));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_SUB_PIC_HEIGHT,sizeof(OLE_PARAM_SUB_PIC_HEIGHT));
    var.vt = VT_INT;
    var.intVal = m_nSubPictureHeight;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }
  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_OFFSET_X));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_OFFSET_X,sizeof(OLE_PARAM_OFFSET_X));
    var.vt = VT_INT;
    var.intVal = m_nCustomOffsetX;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }
  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_OFFSET_Y));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_OFFSET_Y,sizeof(OLE_PARAM_OFFSET_Y));
    var.vt = VT_INT;
    var.intVal = m_nCustomOffsetY;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

return S_OK;
}


//----------------- IPropertyBag------------------

HRESULT PicInPicFilter::Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  VariantClear(pVar);

  if(oleicmp(pszPropName, OLE_PARAM_TARGET_WIDTH) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nTargetWidth;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_TARGET_HEIGHT) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nTargetHeight;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_SUB_PIC_WIDTH) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nSubPictureWidth;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_SUB_PIC_HEIGHT) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nSubPictureHeight;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_OFFSET_X) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nCustomOffsetX;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_OFFSET_Y) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nCustomOffsetY;
    return S_OK;
  }
  return E_INVALIDARG;
}


HRESULT PicInPicFilter::Write(LPCOLESTR pszPropName, VARIANT *pVar)
{
CheckPointer(pszPropName, E_INVALIDARG);
CheckPointer(pVar, E_INVALIDARG);
  if(oleicmp(pszPropName, OLE_PARAM_TARGET_WIDTH) == 0)
  {   
    if(IntFromVariant(m_nTargetWidth,pVar)) return S_OK;
  }
  else if(oleicmp(pszPropName, OLE_PARAM_TARGET_HEIGHT) == 0)
  {
    if(IntFromVariant(m_nTargetHeight,pVar)) return S_OK;    
  }
  if(oleicmp(pszPropName, OLE_PARAM_SUB_PIC_WIDTH) == 0)
  {   
    if(IntFromVariant(m_nSubPictureWidth,pVar)) return S_OK;
  }
  else if(oleicmp(pszPropName, OLE_PARAM_SUB_PIC_HEIGHT) == 0)
  {
    if(IntFromVariant(m_nSubPictureHeight,pVar)) return S_OK;    
  }
  if(oleicmp(pszPropName, OLE_PARAM_OFFSET_X) == 0)
  {   
    if(IntFromVariant(m_nCustomOffsetX,pVar)) return S_OK;
  }
  else if(oleicmp(pszPropName, OLE_PARAM_OFFSET_Y) == 0)
  {
    if(IntFromVariant(m_nCustomOffsetY,pVar)) return S_OK;    
  }
  return E_INVALIDARG;
}


//----------------- IPersistStream------------------


HRESULT PicInPicFilter::GetSizeMax(ULARGE_INTEGER *pcbSize)
{
CheckPointer(pcbSize, E_INVALIDARG);
  pcbSize->QuadPart = sizeof(m_nTargetWidth) + sizeof(m_nTargetHeight) + sizeof(m_nSubPictureWidth) + sizeof(m_nSubPictureHeight) +
		      sizeof(m_nCustomOffsetX) + sizeof(m_nCustomOffsetY);
  return S_OK;
}


HRESULT PicInPicFilter::IsDirty(void)
{
  return m_DirtyProps ? S_OK : S_FALSE;
}


HRESULT PicInPicFilter::Load(IStream *pStm)
{
ULONG pcbRead;
int nOutWidth, nOutHeight, nSubPicWidth, nSubPicHeight, nCustomOffsetX, nCustomOffsetY;
HRESULT hr;
  ASSERT(sizeof(nOutWidth)==sizeof(m_nTargetWidth));
  ASSERT(sizeof(nOutHeight)==sizeof(m_nTargetHeight));
  CheckPointer(pStm, E_INVALIDARG);

  if(FAILED(hr=pStm->Read(&nOutWidth, sizeof(nOutWidth), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nOutHeight, sizeof(nOutHeight), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nSubPicWidth, sizeof(nSubPicWidth), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nSubPicHeight, sizeof(nSubPicHeight), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nCustomOffsetX, sizeof(nCustomOffsetX), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nCustomOffsetY, sizeof(nCustomOffsetY), &pcbRead))) return hr;
  if(m_nTargetWidth!=nOutWidth || m_nTargetHeight!=nOutHeight || m_nSubPictureWidth!=nSubPicWidth || m_nSubPictureHeight!=nSubPicHeight ||
     m_nCustomOffsetX!=nCustomOffsetX || m_nCustomOffsetY!=nCustomOffsetY)
  {
    m_nTargetWidth = nOutWidth;
    m_nTargetHeight = nOutHeight;
    m_nSubPictureWidth = nSubPicWidth;
    m_nSubPictureHeight = nSubPicHeight;
    m_nCustomOffsetX = nCustomOffsetX;
    m_nCustomOffsetY = nCustomOffsetY;
    m_DirtyProps = true;
  }
return S_OK;
}


HRESULT PicInPicFilter::Save(IStream *pStm, BOOL fClearDirty)
{
HRESULT hr;
CheckPointer(pStm, E_INVALIDARG);
  if(FAILED(hr=pStm->Write(&m_nTargetWidth, sizeof(m_nTargetWidth), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nTargetHeight, sizeof(m_nTargetHeight), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nSubPictureWidth, sizeof(m_nSubPictureWidth), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nSubPictureHeight, sizeof(m_nSubPictureHeight), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nCustomOffsetX, sizeof(m_nCustomOffsetX), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nCustomOffsetY, sizeof(m_nCustomOffsetY), NULL))) return hr;
  if(fClearDirty) m_DirtyProps=false;
return S_OK;
}

