/** @file

MODULE				: Tee

FILE NAME			: Tee.cpp

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2012, CSIR
	      2025-2026, Digiteq automotive, programmed by Jaroslav Fojtik	All rights reserved.

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
	// STL
#include <cassert>
	// VPP includes
#include "Tee.h"
#include "DirectShow/VariantTools.h"

#include <Dvdmedia.h>			// VIDEOINFOHEADER2


const OLECHAR OLE_PARAM_COPY[] =  olestr(PARAM_COPY);


RtvcTee::RtvcTee(): CMultiIOBaseFilter(NAME("DQ VPP Tee Filter"), 0, CLSID_VPP_Tee),
	m_bCopy(true)
{
	//Hack virtual method of subclass can't be called from base class contructor, hence must be called in child contructor
  Initialise();
  initParameters();
}


void RtvcTee::initParameters(void)
{
  addParameter(PARAM_COPY, OLE_PARAM_COPY, &m_bCopy,  m_bCopy);
}


CUnknown * WINAPI RtvcTee::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  RtvcTee *pFilter = new RtvcTee();
  if(pHr) *pHr = (pFilter==NULL) ? E_OUTOFMEMORY : S_OK;
  return pFilter;
}


HRESULT RtvcTee::Receive(IMediaSample *pSample, int nIndex)
{
  if(nIndex<0 || nIndex>=(int)m_vInputPins.size()) return E_INVALIDARG;
  if(pSample==NULL) return E_INVALIDARG;

  HRESULT hr;
  if(nIndex == 0)
  {
		//We have a live stream - display it
    CAutoLock lck(&m_csReceive);
    AM_SAMPLE2_PROPERTIES * const pProps = m_vInputPins[0]->SampleProps();
    if (pProps->dwStreamId != AM_STREAM_MEDIA) 
    {
	return m_vInputPins[0]->Receive(pSample);		// JFO: Why to call Receive from input pin????
    }

    const int nSize = pSample->GetActualDataLength();
    BYTE *pBufferIn  = NULL;
    hr = pSample->GetPointer(&pBufferIn);
    if(FAILED(hr)) return hr;
    if(pBufferIn==NULL) return E_POINTER;


    if(m_bCopy)
    {			// Copy the input sample to all outputs
      for(size_t i=0;  i<m_vOutputPins.size(); i++)
      {
        if(m_vOutputPins[i]->IsConnected())
        {
	  IMediaSample *pOutSample = NULL;
				// Set up the output sample
	  hr = InitializeOutputSample(pSample, &pOutSample, nIndex, i);
	  if(FAILED(hr))	return hr;
          if(pOutSample==NULL) continue;

				// Start timing the transform (if PERF is defined)
	  MSR_START(m_idTransform);

	  hr = pOutSample->SetActualDataLength(nSize);
	  if(FAILED(hr)) return hr;

				// Get pointers to the underlying buffers.	
	  BYTE *pBufferOut = NULL;

	  hr = pOutSample->GetPointer(&pBufferOut);
	  if(FAILED(hr)) return hr;				
				// Process the data.				
	  memcpy(pBufferOut, pBufferIn, nSize);

	  pOutSample->SetSyncPoint(TRUE);

				// Stop the clock and log it (if PERF is defined)
	  MSR_STOP(m_idTransform);
	  if(FAILED(hr))
          {
	    DbgLog((LOG_TRACE,1,TEXT("Error from transform")));
	  } 
	  else 
	  {		// the Transform() function can return S_FALSE to indicate that the
			// sample should not be delivered; we only deliver the sample if it's
			// really S_OK (same as NOERROR, of course.)
	    if(hr == NOERROR) 
	    {
	      hr = m_vOutputPins[i]->Deliver(pOutSample);
	      //m_bSampleSkipped = FALSE;	// last thing no longer dropped
	    } 
	    else 
	    {		// S_FALSE returned from Transform is a PRIVATE agreement
			// We should return NOERROR from Receive() in this cause because returning S_FALSE
			// from Receive() means that this is the end of the stream and no more data should
			// be sent.
	      if(S_FALSE == hr) 
	      {
			//  Release the sample before calling notify to avoid
			//  deadlocks if the sample holds a lock on the system
			//  such as DirectDraw buffers do
	        pOutSample->Release();
	        /*m_bSampleSkipped = TRUE;
	        if (!m_bQualityChanged) {
	        NotifyEvent(EC_QUALITY_CHANGE,0,0);
	        m_bQualityChanged = TRUE;
	        }*/
	        return NOERROR;
	      }
	    }
          }
			// release the output buffer. If the connected pin still needs it,
			// it will have addrefed it itself.
          pOutSample->Release();
        }
      }
    }
    else
    {
      for(size_t i=0; i<m_vOutputPins.size(); i++)
      {
        if(m_vOutputPins[i]->IsConnected())
        {
	    if(hr == NOERROR) 
	    {
	      hr = m_vOutputPins[i]->Deliver(pSample);
	      //m_bSampleSkipped = FALSE;	// last thing no longer dropped
	    } 
	    else 
	    {		// S_FALSE returned from Transform is a PRIVATE agreement
			// We should return NOERROR from Receive() in this cause because returning S_FALSE
			// from Receive() means that this is the end of the stream and no more data should
			// be sent.
	      if(S_FALSE == hr) 
	      {
	        return NOERROR;
	      }
	    }
        }
      }
    }
    return hr;
  }
  else
  {		// There are only two possible indexes for this multiplexer
    return E_INVALIDARG;
  }
}


void RtvcTee::InitialiseInputTypes()
{
  AddInputType(NULL, NULL, NULL);
}


void RtvcTee::InitialiseOutputTypes()
{
  AddOutputType(NULL, NULL, NULL);
}


HRESULT RtvcTee::DecideBufferSize( IMemAllocator* pAlloc, ALLOCATOR_PROPERTIES* pRequestProperties, int m_nIndex )
{
	// Get media type of the output pin
  AM_MEDIA_TYPE mt;
  HRESULT hr = m_vInputPins[0]->ConnectionMediaType(&mt);
  if(FAILED(hr)) return hr;

	// This simple tee will only cater for the following types
  assert(mt.formattype==FORMAT_VideoInfo || mt.formattype==FORMAT_VideoInfo2 || mt.formattype==FORMAT_WaveFormatEx);

  if(mt.formattype == FORMAT_VideoInfo)
  {
    if(mt.pbFormat!=NULL)
    {
	BITMAPINFOHEADER *pbmi = HEADER(mt.pbFormat);
	pRequestProperties->cbBuffer = DIBSIZE(*pbmi);
    }
  }
  else if(mt.formattype == FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 * const pVih2 = (VIDEOINFOHEADER2*)mt.pbFormat;
    if(pVih2) pRequestProperties->cbBuffer = DIBSIZE(pVih2->bmiHeader);
  }
  else if(mt.formattype == FORMAT_WaveFormatEx)
  {
    WAVEFORMATEX *pWavHeader = (WAVEFORMATEX*)mt.pbFormat;
    pRequestProperties->cbBuffer = pWavHeader->nSamplesPerSec * pWavHeader->nChannels * (pWavHeader->wBitsPerSample >> 3);
  }

  if(pRequestProperties->cbAlign == 0)
  {
    pRequestProperties->cbAlign = 1;
  }
  if (pRequestProperties->cBuffers == 0)
  {
    pRequestProperties->cBuffers = 1;
  }
	// Release the format block.
  FreeMediaType(mt);

	// Set allocator properties.
  ALLOCATOR_PROPERTIES Actual;
  hr = pAlloc->SetProperties(pRequestProperties, &Actual);
  if(FAILED(hr)) 
  {
    return hr;
  }
	// Even when it succeeds, check the actual result.
  if(pRequestProperties->cbBuffer > Actual.cbBuffer) 
  {
    return E_FAIL;
  }
return S_OK;
}


HRESULT RtvcTee::GetMediaType( int iPosition, CMediaType* pMediaType, int nOutputPinIndex )
{
  if(iPosition < 0)
  {
    return E_INVALIDARG;
  }
  if(iPosition == 0)
  {
    return m_vInputPins[0]->ConnectionMediaType(pMediaType);
  }
  return VFW_S_NO_MORE_ITEMS;
}


HRESULT RtvcTee::CheckOutputType( const CMediaType* pMediaType )
{
  AM_MEDIA_TYPE mediaType;
  HRESULT hr = m_vInputPins[0]->ConnectionMediaType(&mediaType);
  if((*pMediaType->Type() == mediaType.majortype) &&
     (*pMediaType->Subtype() == mediaType.subtype) &&
     (*pMediaType->FormatType() == mediaType.formattype))
  {
    return S_OK;
  }
  else
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }
}


HRESULT RtvcTee::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_Tee;
  return S_OK;
}


HRESULT RtvcTee::GetPages(CAUUID *pPages)
{
  if(pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if(pPages->pElems == NULL) 
  {
    pPages->cElems = 0;
    return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_VPP_TeeProperties;
  return S_OK;
}


HRESULT RtvcTee::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == IID_IPersistPropertyBag)   return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPersistStream)	return GetInterface(static_cast<IPersistStream*>(this), ppv);
  if(riid == IID_IPropertyBag)		return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_ISpecifyPropertyPages) return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  
      // Call the parent class.
  return CMultiIOBaseFilter::NonDelegatingQueryInterface(riid, ppv);
}

