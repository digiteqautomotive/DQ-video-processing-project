/** @file

MODULE				: ScaleFilter

FILE NAME			: ScaleFilter.cpp

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2015, CSIR
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
#include "ScaleFilter.h"
#include <DirectShowExt/DirectShowMediaFormats.h>
#include <DirectShowExt/FilterParameterStringConstants.h>
#include <DirectShowExt/ParameterConstants.h>
#include <ImageUtils/PicCropperRGB24Impl.h>
#include <ImageUtils/PicScalerRGB24Impl.h>
#include <ImageUtils/PicScalerYUV420PImpl.h>
#include <artist/Media/MediaUtil.h>

ScaleFilter::ScaleFilter()
  : CCustomBaseFilter(NAME("CSIR VPP Scale Filter"), 0, CLSID_VPP_ScaleFilter),
  m_pScaler(NULL),
  m_nBytesPerPixel(BYTES_PER_PIXEL_RGB24),
  m_eMode(MODE_ASPECT_RATIO_CORRECT_SCALING1),
  m_uiTopCrop(0),
  m_uiBottomCrop(0),
  m_uiRightCrop(0),
  m_uiLeftCrop(0),
  m_uiWidthAfterCropping(0),
  m_uiHeightAfterCropping(0),
  m_pCropper(nullptr),
  m_pCropBuffer(nullptr)
{
  //Call the initialize input method to load all acceptable input types for this filter
  InitialiseInputTypes();
  // Init parameters
  initParameters();
}

ScaleFilter::~ScaleFilter()
{
  if (m_pCropBuffer)
  {
    delete m_pCropBuffer;
    m_pCropBuffer = NULL;
  }
  if (m_pCropper)
  {
    delete m_pCropper;
    m_pCropper = NULL;
  }
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
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUV420P_S, &FORMAT_VideoInfo);
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
      if (m_pScaler)
      {
        delete m_pScaler;
        m_pScaler = NULL;
      }
      if (pmt->subtype == MEDIASUBTYPE_RGB24)
      {
        m_pScaler = new PicScalerRGB24Impl();
        m_nBytesPerPixel = BYTES_PER_PIXEL_RGB24;
      }
      else if (pmt->subtype == MEDIASUBTYPE_YUV420P_S)
      {
        m_pScaler = new PicScalerYUV420PImpl();
        m_nBytesPerPixel = BYTES_PER_PIXEL_YUV420P;
      }
    }
  }
  return hr;
}

HRESULT ScaleFilter::GetMediaType(int iPosition, CMediaType *pMediaType)
{
  if (iPosition < 0)
  {
    return E_INVALIDARG;
  }
  else if (iPosition == 0)
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
    ASSERT(pMediaType->formattype == FORMAT_VideoInfo);
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
    //Now we need to calculate the size of the output image
    BITMAPINFOHEADER* pBi = &(pVih->bmiHeader);

    // Set height
    pBi->biHeight = m_nOutHeight;
    ASSERT(pBi->biHeight > 0);

    // Set width
    pBi->biWidth = m_nOutWidth;
    ASSERT(pBi->biWidth > 0);
    // Set size
    pBi->biSizeImage = pBi->biWidth * pBi->biHeight * BYTES_PER_PIXEL_RGB24;
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
  pProp->cbBuffer = static_cast<long>(m_nOutPixels * m_nBytesPerPixel);
  if (m_nBytesPerPixel == BYTES_PER_PIXEL_YUV420P)
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
  //Subtypes
  if (mtIn->subtype == MEDIASUBTYPE_RGB24)
  {
    if (mtOut->subtype != MEDIASUBTYPE_RGB24)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }
  if (mtIn->subtype == MEDIASUBTYPE_YUV420P_S)
  {
    if (mtOut->subtype != MEDIASUBTYPE_YUV420P_S)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }
  // Video format
  if (mtOut->formattype != FORMAT_VideoInfo)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }
  return S_OK;
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
  addParameter(FILTER_PARAM_MODE, &m_eMode, MODE_ASPECT_RATIO_CORRECT_SCALING1);
}

HRESULT ScaleFilter::ApplyTransform(BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{
  //make sure we were able to initialize our converter
  ASSERT(m_pScaler);

  switch (m_eMode)
  {
    case MODE_ASPECT_RATIO_CORRECT_SCALING1:
    {
      if (m_pCropper)
      {
        int res = m_pCropper->Crop((void*)pBufferIn, (void*)m_pCropBuffer);
        ASSERT(res == 1);
        // assume its the same aspect ratio, just scale directly
        m_pScaler->SetInDimensions(m_uiWidthAfterCropping, m_uiHeightAfterCropping);
        m_pScaler->SetOutDimensions(m_nOutWidth, m_nOutHeight);
        res = m_pScaler->Scale((void*)pBufferOut, (void*)m_pCropBuffer);
        ASSERT(res == 1);
        lOutActualDataLength = static_cast<long>(m_nOutWidth * m_nOutHeight * m_nBytesPerPixel);
        return S_OK;
      }
      else
      {
        // assume its the same aspect ratio, just scale directly
        m_pScaler->SetInDimensions(m_nInWidth, m_nInHeight);
        m_pScaler->SetOutDimensions(m_nOutWidth, m_nOutHeight);
        int res = m_pScaler->Scale((void*)pBufferOut, (void*)pBufferIn);
        ASSERT(res == 1);
        lOutActualDataLength = static_cast<long>(m_nOutWidth * m_nOutHeight * m_nBytesPerPixel);
        return S_OK;
      }
      break;
    }
    case MODE_STANDARD:
    default:
    {
      //Call scaling conversion code
      m_pScaler->SetInDimensions(m_nInWidth, m_nInHeight);
      m_pScaler->SetOutDimensions(m_nOutWidth, m_nOutHeight);
      int res = m_pScaler->Scale((void*)pBufferOut, (void*)pBufferIn);
      ASSERT(res == 1);
      lOutActualDataLength = static_cast<long>(m_nOutWidth * m_nOutHeight * m_nBytesPerPixel);
      return S_OK;
      break;
    }
  }
}

HRESULT ScaleFilter::RecalculateFilterParameters()
{
  // Update the number of out pixels
  m_nOutPixels = m_nOutWidth * m_nOutHeight;

  if (m_pCropBuffer)
  {
    delete m_pCropBuffer;
    m_pCropBuffer = nullptr;
  }
  if (m_pCropper)
  {
    delete m_pCropper;
    m_pCropper = nullptr;
  }
  
  m_uiTopCrop = 0;
  m_uiBottomCrop = 0;
  m_uiLeftCrop = 0;
  m_uiRightCrop = 0;
  m_uiWidthAfterCropping = 0;
  m_uiHeightAfterCropping = 0;

  switch (m_eMode)
  {
    case MODE_ASPECT_RATIO_CORRECT_SCALING1:
    {
      uint32_t uiTopBottomCrop = 0;
      uint32_t uiLeftRightCrop = 0;
      double dInAspectRatio = m_nInWidth / static_cast<double>(m_nInHeight);
      double dOutAspectRatio = m_nOutWidth / static_cast<double>(m_nOutHeight);
      double EPSILON = 0.001;
      // check if the aspect ratios differ and cropping is necessary
      if ( std::fabs(dInAspectRatio - dOutAspectRatio) > EPSILON)
      {
        if (artist::determineCropParameters(m_nInWidth, m_nInHeight, m_nOutWidth, m_nOutHeight, uiTopBottomCrop, uiLeftRightCrop))
        {
          m_uiWidthAfterCropping = m_nInWidth - uiLeftRightCrop;
          m_uiHeightAfterCropping = m_nInHeight - uiTopBottomCrop;
          // for now just divide (almost) equally
          if (uiTopBottomCrop > 0)
          {
            m_uiTopCrop = uiTopBottomCrop / 2;
            m_uiBottomCrop = uiTopBottomCrop - m_uiTopCrop;
          }
          if (uiLeftRightCrop > 0)
          {
            m_uiLeftCrop = uiLeftRightCrop / 2;
            m_uiRightCrop = uiLeftRightCrop - m_uiLeftCrop;
          }
          ASSERT(m_uiTopCrop > 0 || m_uiBottomCrop > 0 || m_uiLeftCrop > 0 || m_uiRightCrop > 0);
         
          m_pCropper = new PicCropperRGB24Impl();
          m_nBytesPerPixel = BYTES_PER_PIXEL_RGB24;
          // Create temp buffer for crop
          m_pCropBuffer = new BYTE[static_cast<int>(m_uiWidthAfterCropping * m_uiHeightAfterCropping * m_nBytesPerPixel)];
          m_pCropper->SetInDimensions(m_nInWidth, m_nInHeight);
          m_pCropper->SetOutDimensions(m_uiWidthAfterCropping, m_uiHeightAfterCropping);
          m_pCropper->SetCrop(m_uiLeftCrop, m_uiRightCrop, m_uiTopCrop, m_uiBottomCrop);
        }
        else
        {
          // this might occur as this method is called for every parameter change.
          // in that case width might be valid for determineCropParameters, but height might not be.
          // just ignore if this fails. In the worst case, no cropper is created and standard mode is applied
        }
      }
      else
      {
        // same aspect ratio->don't need cropper
      }
      break;
    }
  }
  return S_OK;
}
