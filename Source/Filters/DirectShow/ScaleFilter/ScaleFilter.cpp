/** @file

MODULE				: ScaleFilter

FILE NAME			: ScaleFilter.cpp

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2015, CSIR
Copyright (c) 2025, Jaroslav Fojtik
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

// CSIR includes
#include "ScaleFilter.h"
#include "../VersionInfo.h"
#include <DirectShow/CommonDefs.h>
#include <DirectShow/CustomMediaTypes.h>
#include "DirectShow/DirectShowMediaFormats.h"
#include <PicScalerRGB24Impl.h>
#include <PicScalerRGB32Impl.h>
#include <PicScalerARGB32Impl.h>
#include <PicScalerYUV420PImpl.h>
#include <PicScalerYUYVImpl.h>
#include <PicScalerUYVYImpl.h>

#ifdef USE_MMX
#include <PicScalerARGB32MMX.h>
#endif
#ifdef USE_SSE
 #include <PicScalerARGB32SSE.h>
#endif


DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00,  0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


#if defined(USE_MMX) || defined(USE_SSE)
extern "C" unsigned GetFeaturesCPU(void);
const unsigned FeaturesCPU = GetFeaturesCPU();
#endif

PicScalerBase *GetPicScallerRGB32(void)
{
  //if(FeaturesCPU == 0x80) FeaturesCPU=GetFeaturesCPU();
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


ScaleFilter::ScaleFilter()
  : CCustomBaseFilter(NAME("CSIR VPP Scale Filter"), 0, CLSID_VPP_ScaleFilter),
  m_pScaler(NULL),
  m_nBitsPerPixel(BITS_PER_PIXEL_RGB24)
{
  //Call the initialize input method to load all acceptable input types for this filter
  InitialiseInputTypes();
  // Init parameters
  initParameters();
}

ScaleFilter::~ScaleFilter()
{
  if (m_pScaler)
  {
    delete m_pScaler;
    m_pScaler = NULL;
  }
}

CUnknown * WINAPI ScaleFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  ScaleFilter *pFilter = new ScaleFilter();
  if (pFilter == NULL)
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pFilter;
}


void ScaleFilter::InitialiseInputTypes()
{
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB32, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_ARGB32, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUY2, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUYV, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YVYU, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUV420P_S, &FORMAT_VideoInfo); // I420 = IYUV = YUV420p (sometimes YUV420p can refer to YV12) https://gist.github.com/Jim-Bar/3cbba684a71d1a9d468a6711a6eddbeb
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_I420, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YV12, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_UYVY, &FORMAT_VideoInfo);
}

HRESULT ScaleFilter::SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt)
{
  HRESULT hr = CCustomBaseFilter::SetMediaType(direction, pmt);
  if (direction == PINDIR_INPUT)
  {
    //Set defaults
    if (m_nOutWidth == 0)
    {
      m_nOutWidth = m_nInWidth;
    }
    if (m_nOutHeight == 0)
    {
      m_nOutHeight = m_nInHeight;
    }
    //Determine whether we are connected to a RGB24 or 32 source
    if (pmt->majortype == MEDIATYPE_Video)
    {
      //The scaler might already exist if the filter has been connected previously
      if(m_pScaler)
      {
        delete m_pScaler;
        m_pScaler = NULL;
      }
      if(pmt->subtype==MEDIASUBTYPE_RGB24)
      {
	m_pScaler = new PicScalerRGB24Impl();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
      }
      else if(pmt->subtype==MEDIASUBTYPE_RGB32)
      {
        m_pScaler = GetPicScallerRGB32();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
      }
      else if(pmt->subtype==MEDIASUBTYPE_ARGB32)
      {
        m_pScaler = GetPicScallerRGB32();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
      }
      else if(pmt->subtype==MEDIASUBTYPE_YUV420P_S || pmt->subtype==MEDIASUBTYPE_I420 || pmt->subtype==MEDIASUBTYPE_YV12)
      {
        m_pScaler = new PicScalerYUV420PImpl();
        m_nBitsPerPixel = BITS_PER_PIXEL_YUV420P;
      } else
      if(pmt->subtype==MEDIASUBTYPE_YUYV || pmt->subtype==MEDIASUBTYPE_YUY2 || pmt->subtype==MEDIASUBTYPE_YVYU)
      {
        m_pScaler = new PicScalerYUYVImpl();		// YVYU and YUYV are different, but the can use a same scaller.
        m_nBitsPerPixel = BITS_PER_PIXEL_YUYV;
      }
      if(pmt->subtype==MEDIASUBTYPE_UYVY)
      {
        m_pScaler = new PicScalerUYVYImpl();
        m_nBitsPerPixel = BITS_PER_PIXEL_YUYV;		// Same as UYVY
      }
    }
  }
  return hr;
}

HRESULT ScaleFilter::GetMediaType(int iPosition, CMediaType *pMediaType)
{
  if(iPosition < 0) return E_INVALIDARG;
  if(pMediaType == NULL) return E_POINTER;
 
  if (iPosition == 0)
  {
    // Get the input pin's media type and return this as the output media type - we want to retain
    // all the information about the image
    HRESULT hr = m_pInput->ConnectionMediaType(pMediaType);
    if (FAILED(hr))
    {
      return hr;
    }

    // Get the bitmap info header and adapt the cropped 
    //make sure that it's a video info header
    if(pMediaType->formattype!=FORMAT_VideoInfo && pMediaType->formattype!=FORMAT_VideoInfo2)
        return VFW_E_TYPE_NOT_ACCEPTED;
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
    //Now we need to calculate the size of the output image
    BITMAPINFOHEADER* pBi = &(pVih->bmiHeader);

	// Set height
    pBi->biHeight = (pBi->biHeight<0) ? -m_nOutHeight : m_nOutHeight;	// Propagate negative height to output.
    if(pBi->biHeight == 0) return E_INVALIDARG;

	// Set width
    pBi->biWidth = m_nOutWidth;
    if(pBi->biWidth <= 0) return E_INVALIDARG;

	// Set size
    if(pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_ARGB32)
        pBi->biSizeImage = labs(pBi->biWidth*pBi->biHeight) * (BITS_PER_PIXEL_RGB32 / 8);
    else if(pMediaType->subtype==MEDIASUBTYPE_YUV420P_S || pMediaType->subtype==MEDIASUBTYPE_I420 || pMediaType->subtype==MEDIASUBTYPE_YV12)
        pBi->biSizeImage = (labs(pBi->biWidth*pBi->biHeight) * BITS_PER_PIXEL_YUV420P) / 8;
    else if(pMediaType->subtype==MEDIASUBTYPE_YUY2 || pMediaType->subtype==MEDIASUBTYPE_YUYV || pMediaType->subtype==MEDIASUBTYPE_YVYU ||
            pMediaType->subtype==MEDIASUBTYPE_UYVY)
       pBi->biSizeImage = (labs(pBi->biWidth*pBi->biHeight) * BITS_PER_PIXEL_YUYV) / 8;
    else
        pBi->biSizeImage = labs(pBi->biWidth*pBi->biHeight) * (BITS_PER_PIXEL_RGB24 / 8);
    pMediaType->lSampleSize = pBi->biSizeImage;

    // Adjust recs
    pVih->rcSource.top = 0;
    pVih->rcSource.left = 0;
    pVih->rcSource.right = pBi->biWidth;
    pVih->rcSource.bottom = pBi->biHeight;

    pVih->rcTarget.top = 0;
    pVih->rcTarget.left = 0;
    pVih->rcTarget.right = pBi->biWidth;
    pVih->rcTarget.bottom = pBi->biHeight;

    return S_OK;
  }
  return VFW_S_NO_MORE_ITEMS;
}

HRESULT ScaleFilter::DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp)
{
  // Leaving this var so that we can cater for RGB32 at a later stage
  pProp->cbBuffer = (m_nOutPixels * m_nBitsPerPixel)/8;
  if (m_nBitsPerPixel == BITS_PER_PIXEL_YUV420P)
  {
    //Adjust the buffer requirements for our custom format
    pProp->cbBuffer *= sizeof(yuvType);
  }

  if (pProp->cbAlign == 0)
  {
    pProp->cbAlign = 1;
  }
  if (pProp->cBuffers == 0)
  {
    pProp->cBuffers = 1;
  }

  // Set allocator properties.
  ALLOCATOR_PROPERTIES Actual;
  HRESULT hr = pAlloc->SetProperties(pProp, &Actual);
  if (FAILED(hr))
  {
    return hr;
  }
  // Even when it succeeds, check the actual result.
  if (pProp->cbBuffer > Actual.cbBuffer)
  {
    return E_FAIL;
  }
  return S_OK;
}

HRESULT ScaleFilter::CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut)
{
	//Make sure the input and output types are related
  if (mtOut->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }
  	// Video format
  if (mtOut->formattype!=FORMAT_VideoInfo && mtOut->formattype!=FORMAT_VideoInfo2)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

	//Subtypes
  if (mtIn->subtype == MEDIASUBTYPE_RGB24)
  {
    if (mtOut->subtype != MEDIASUBTYPE_RGB24)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if(mtIn->subtype == MEDIASUBTYPE_RGB32 || mtIn->subtype == MEDIASUBTYPE_ARGB32)
  {
    if(mtOut->subtype != MEDIASUBTYPE_RGB32 && mtOut->subtype != MEDIASUBTYPE_ARGB32)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if(mtIn->subtype==MEDIASUBTYPE_YUY2 || mtIn->subtype==MEDIASUBTYPE_YUYV)
  {
    if(mtOut->subtype!=MEDIASUBTYPE_YUY2 && mtOut->subtype!=MEDIASUBTYPE_YUYV)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if (mtIn->subtype == MEDIASUBTYPE_YVYU)
  {
    if (mtOut->subtype != MEDIASUBTYPE_YVYU)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
  if (mtIn->subtype == MEDIASUBTYPE_UYVY)
  {
    if (mtOut->subtype != MEDIASUBTYPE_UYVY)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if(mtIn->subtype==MEDIASUBTYPE_YUV420P_S || mtIn->subtype==MEDIASUBTYPE_I420 || mtIn->subtype==MEDIASUBTYPE_YV12)
  {
    if(mtOut->subtype!=MEDIASUBTYPE_YUV420P_S && mtOut->subtype!=MEDIASUBTYPE_I420 && mtOut->subtype!=MEDIASUBTYPE_YV12)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
	// No known subtype match.
  return VFW_E_TYPE_NOT_ACCEPTED;
}


STDMETHODIMP ScaleFilter::SetParameter(const char* type, const char* value)
{
  // For now, one cannot set any parameters once the output has been connected -> this will affect the buffersize
  if (m_pOutput)
  {
    if (m_pOutput->IsConnected())
    {
      return E_FAIL;
    }
  }

  // TODO: get rid of enum or get rid of string!!!!
  if (SUCCEEDED(CSettingsInterface::SetParameter(type, value)))
  {
    RecalculateFilterParameters();
    return S_OK;
  }
  else
  {
    return E_FAIL;
  }
}


void ScaleFilter::initParameters()
{
  addParameter(FILTER_PARAM_TARGET_WIDTH, &m_nOutWidth, 0);
  addParameter(FILTER_PARAM_TARGET_HEIGHT, &m_nOutHeight, 0);
//  addParameter(FILTER_PARAM_MODE, &m_eMode, MODE_ASPECT_RATIO_CORRECT_SCALING1);
}


HRESULT ScaleFilter::ApplyTransform(BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{
  //make sure we were able to initialize our converter
  ASSERT(m_pScaler);
  //Call scaling conversion code
  m_pScaler->SetInDimensions(m_nInWidth, m_nInHeight);
  m_pScaler->SetOutDimensions(m_nOutWidth, m_nOutHeight);
  int res = m_pScaler->Scale((void*)pBufferOut, (void*)pBufferIn);
  ASSERT(res == 1);
  lOutActualDataLength = labs(m_nOutWidth * m_nOutHeight * m_nBitsPerPixel) / 8;
  return S_OK;
}


HRESULT ScaleFilter::RecalculateFilterParameters()
{
  // Update the number of out pixels
  m_nOutPixels = labs(m_nOutWidth * m_nOutHeight);
  return S_OK;
}


void ScaleFilter::doGetVersion(std::string& sVersion)
{
  sVersion = VersionInfo::toString();
}
