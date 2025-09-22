/** @file

MODULE				: VideoMixingFilter

FILE NAME			: VideoMixingFilter.cpp

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2014, CSIR
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
#include "stdafx.h"
#include "VideoMixingFilter.h"
#include <DirectShow/CommonDefs.h>
#include <PicConcatRGB24Impl.h>
#include <PicConcatRGB32Impl.h>
#include "../VersionInfo.h"
#include <Dvdmedia.h>			// VIDEOINFOHEADER2


#ifndef MAX
 #define MAX(a,b) ((a>b)?(a):(b))
#endif


VideoMixingFilter::VideoMixingFilter(): VideoMixingBase(NAME("CSIR VPP Video Mixer"), 0, CLSID_VPP_VideoMixingFilter),
	m_pPicConcat(NULL),
        m_Vflip(false)
{
	m_pSampleBuffers[0] = NULL;
	m_pSampleBuffers[1] = NULL;

	m_nSampleSizes[0] = 0;
	m_nSampleSizes[1] = 0;

	// Init parameters
	initParameters();
}


VideoMixingFilter::~VideoMixingFilter()
{
	if (m_pPicConcat)
	{
		delete m_pPicConcat;
		m_pPicConcat = NULL;
	}

	for (int i = 0; i < 2; i++)
	{
		if (m_pSampleBuffers[i])
		{
			delete[] m_pSampleBuffers[i];
			m_pSampleBuffers[i] = NULL;
		}
	}
}


CUnknown * WINAPI VideoMixingFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
	VideoMixingFilter* pFilter = new VideoMixingFilter();
	return pFilter;
}


void VideoMixingFilter::initParameters(void)
{
	addParameter(FILTER_PARAM_ORIENTATION, &m_nOrientation, 0);
}


/// This method renders output frame.
/// @TODO: It looks like it is called twice time than necessary, must be limited framerate.
HRESULT VideoMixingFilter::GenerateOutputSample(IMediaSample *pSample, int nIndex)
{
	// Prepare output sample
	const AM_SAMPLE2_PROPERTIES * const pProps = m_vInputPins[nIndex]->SampleProps();
	DbgLog((LOG_TRACE,0,TEXT("Video Mixer: Sample Received from input 0: start time: %I64d end time: %I64d"), pProps->tStart, pProps->tStop));
	if (pProps->dwStreamId != AM_STREAM_MEDIA) {
		return m_vInputPins[nIndex]->Receive(pSample);
	}

	IMediaSample * pOutSample;
	// Set up the output sample
	HRESULT hr = InitializeOutputSample(pSample, &pOutSample, nIndex, 0);

	if (FAILED(hr)) {
		return hr;
	}

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
	if (m_pSampleBuffers[0] && m_pSampleBuffers[1])
		m_pPicConcat->Concat(m_pSampleBuffers[0], m_pSampleBuffers[1], pBufferOut, m_Vflip);
	else if (m_pSampleBuffers[0])
	{
		const int nWidth = labs(m_pPicConcat->Get1stWidth());
		const int nHeight = labs(m_pPicConcat->Get1stHeight());
		if(m_Vflip)
		{
		  const int RowLen = (nWidth*m_nBitsPerPixel)/8;
		  BYTE *pDst = pBufferOut;
	          const BYTE *pSrc = m_pSampleBuffers[0] + (nWidth*nHeight*m_nBitsPerPixel) / 8;
		  int y = nHeight;
		  while(y-->0)
		  {
		    pSrc -= RowLen;
                    memcpy(pDst, pSrc, RowLen);
		    pDst += RowLen;
		  }
		}
		else
		  memcpy(pBufferOut, m_pSampleBuffers[0], (nWidth*nHeight * m_nBitsPerPixel)/8);
	}
	else
	{
		const int nWidth = labs(m_pPicConcat->Get2ndWidth());
		const int nHeight = labs(m_pPicConcat->Get2ndHeight());
		if(m_Vflip)
		{
		  const int RowLen = (nWidth*m_nBitsPerPixel)/8;
		  BYTE *pDst = pBufferOut;
	          const BYTE *pSrc = m_pSampleBuffers[1] + (nWidth * nHeight * m_nBitsPerPixel) / 8;
		  int y = nHeight;
		  while(y-->0)
		  {
		    pSrc -= RowLen;
                    memcpy(pDst, pSrc, RowLen);
		    pDst += RowLen;
		  }
		}
		else
		  memcpy(pBufferOut, m_pSampleBuffers[1], (nWidth*nHeight * m_nBitsPerPixel)/8);		
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


HRESULT VideoMixingFilter::ReceiveFirstSample( IMediaSample *pSample )
{
	// Copy the secondary sample into our buffer
	if (pSample)
	{
		BYTE *pBuffer = NULL;
		HRESULT hr = pSample->GetPointer(&pBuffer);
		if (FAILED(hr))
		{
			return hr;
		}

		// Copy the buffer
		memcpy(m_pSampleBuffers[0], pBuffer, m_nSampleSizes[0]);
		return GenerateOutputSample(pSample, 0);
	}
	else
	{
		return E_POINTER;
	}
}


HRESULT VideoMixingFilter::ReceiveSecondSample( IMediaSample *pSample )
{
	// Copy the secondary sample into our buffer
	if (pSample)
	{
		BYTE *pBuffer = NULL;
		HRESULT hr = pSample->GetPointer(&pBuffer);
		if (FAILED(hr))
		{
			return hr;
		}

		// Copy the buffer
		memcpy(m_pSampleBuffers[1], pBuffer, m_nSampleSizes[1]);
		return GenerateOutputSample(pSample, 1);
	}
	else
	{
		return E_POINTER;
	}
}


HRESULT VideoMixingFilter::CreateVideoMixer(const CMediaType *pMediaType, int nIndex)
{
  const BITMAPINFOHEADER *bmih = NULL;
	// Create temporary sample buffers
  if(pMediaType->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER* const pVih = (const VIDEOINFOHEADER*) pMediaType->pbFormat;
    if(pVih) bmih = &pVih->bmiHeader;
  }
  else if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2* const pVih2 = (const VIDEOINFOHEADER2*) pMediaType->pbFormat;
    if(pVih2) bmih = &pVih2->bmiHeader;
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
  if(pMediaType->subtype == MEDIASUBTYPE_RGB24)
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
      m_pPicConcat = new PicConcatRGB24Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
    }
    return S_OK;
  }

  if(pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_ARGB32)
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
      m_pPicConcat = new PicConcatRGB32Impl();
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
    }
    return S_OK;
  }

 return VFW_E_TYPE_NOT_ACCEPTED;
}


/// Pin disconnection must reshedule output geometry.
void VideoMixingFilter::OnDisconnect(int nIndex)
{
  VideoMixingBase::OnDisconnect(nIndex);

		// Get first input dimensions
  AM_MEDIA_TYPE mediaType1;
  BITMAPINFOHEADER* pBmih1 = NULL;
  if(m_vInputPins[0]->IsConnected() && nIndex!=0)
  {
    if(SUCCEEDED(m_vInputPins[0]->ConnectionMediaType(&mediaType1)))
    {
      if(mediaType1.formattype==FORMAT_VideoInfo)
      {
	VIDEOINFOHEADER* pVih = (VIDEOINFOHEADER*) mediaType1.pbFormat;
	pBmih1 = &pVih->bmiHeader;
      }
      if(mediaType1.formattype==FORMAT_VideoInfo2)
      {
	VIDEOINFOHEADER2* pVih2 = (VIDEOINFOHEADER2*) mediaType1.pbFormat;
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
	VIDEOINFOHEADER* const pVih = (VIDEOINFOHEADER*) mediaType2.pbFormat;
	pBmih2 = &pVih->bmiHeader;
      }
      if(mediaType2.formattype==FORMAT_VideoInfo2)
      {
	VIDEOINFOHEADER2* const pVih2 = (VIDEOINFOHEADER2*) mediaType2.pbFormat;
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
}


HRESULT VideoMixingFilter::SetOutputDimensions(BITMAPINFOHEADER* pBmih1, BITMAPINFOHEADER* pBmih2, int& nOutputWidth, int& nOutputHeight, int& nOutputSize)
{
  if(pBmih1 && pBmih2)
  {
		// Verify that the dimensions match and set output width and height
    switch (m_nOrientation)
    {
	case 0: 
		{
		//if (pBmih1->biHeight != pBmih2->biHeight) return E_FAIL; 			// Height must be the same
		nOutputWidth = labs(pBmih1->biWidth) + labs(pBmih2->biWidth);
		nOutputHeight = MAX(labs(pBmih1->biHeight),labs(pBmih2->biHeight));
		break;
		}
	case 1: 
		{
		//if (pBmih1->biWidth!= pBmih2->biWidth) return E_FAIL; 			// Width must be the same
		nOutputWidth = MAX(labs(pBmih1->biWidth), labs(pBmih2->biWidth));
		nOutputHeight = labs(pBmih1->biHeight) + labs(pBmih2->biHeight);
		break;
		}
    }
    nOutputSize =  m_nSampleSizes[0] + m_nSampleSizes[1];

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
	m_pPicConcat->Set1stDimensions(pBmih1->biWidth, pBmih1->biHeight);
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
      m_pPicConcat->Set2ndDimensions(pBmih2->biWidth, pBmih2->biHeight);
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
  if(m_pPicConcat)
    m_pPicConcat->SetOutDimensions(nOutputWidth, nOutputHeight);

return S_OK;
}


/// Final agreement on output geometry.
HRESULT VideoMixingFilter::CheckOutputType( const CMediaType* pMediaType )
{
	//Make sure the input and output types are related
  if(pMediaType->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if(m_pPicConcat==NULL) return E_POINTER;

  	// Video format
  m_Vflip = false;
  if(pMediaType->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER * const pVihOut = (const VIDEOINFOHEADER*)pMediaType->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
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
  else if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2 * const pVihOut = (const VIDEOINFOHEADER2*)pMediaType->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
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


  if(m_nBitsPerPixel == BITS_PER_PIXEL_RGB24)
  {
    if (*(pMediaType->Subtype()) == MEDIASUBTYPE_RGB24)
    {
      return S_OK;
    }
  }
  else if(m_nBitsPerPixel == BITS_PER_PIXEL_RGB32)
  {
    if(*(pMediaType->Subtype())==MEDIASUBTYPE_RGB32 || *(pMediaType->Subtype())==MEDIASUBTYPE_ARGB32)
    {
      return S_OK;
    }
  }
  return S_FALSE;
}


void VideoMixingFilter::doGetVersion(std::string& sVersion)
{
  sVersion = VersionInfo::toString();
}

