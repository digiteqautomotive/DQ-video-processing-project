/** @file

MODULE				: YUV444ToRGBFilter

FILE NAME			: YUV444ToRGBFilter.cpp

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
#include "YUV444ToRGBFilter.h"
#include <DirectShow/CommonDefs.h>
#include "DirectShow/CustomMediaTypes.h"
#include "DirectShow/DirectShowMediaFormats.h"
#include "AYUV444toRGB24Converter.h"
#include "RealYUV420toRGB24Converter.h"

#include <Dvdmedia.h>			// VIDEOINFOHEADER2


DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);


YUV444toRGBFilter::YUV444toRGBFilter():
  CCustomBaseFilter(NAME("DQ VPP AYUV444 to RGB Converter"), 0, CLSID_VPP_YUV444toRGBColorConverter),
  m_pConverter(new AYUV444toRGB24Converter(m_nInWidth, m_nInHeight)),
  m_nSizeUV(0),
  m_bInvert(true)
{
  //Call the initialise input method to load all acceptable input types for this filter
  InitialiseInputTypes();
  initParameters();

  // init converter
  //m_pConverter->SetFlip(m_bInvert);
  //m_pConverter->SetChrominanceOffset(m_nChrominanceOffset);
}

YUV444toRGBFilter::~YUV444toRGBFilter()
{
  if(m_pConverter)
  {
    delete m_pConverter;
    m_pConverter = NULL;
  }
}

CUnknown * WINAPI YUV444toRGBFilter::CreateInstance( LPUNKNOWN pUnk, HRESULT *pHr )
{
  YUV444toRGBFilter *pFilter = new YUV444toRGBFilter();
  if (pFilter== NULL) 
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pFilter;
}


void YUV444toRGBFilter::InitialiseInputTypes()
{
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_AYUV, &FORMAT_VideoInfo);

  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUV420P_S, &FORMAT_VideoInfo); // I420 = IYUV = YUV420p (sometimes YUV420p can refer to YV12) https://gist.github.com/Jim-Bar/3cbba684a71d1a9d468a6711a6eddbeb
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_I420, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YV12, &FORMAT_VideoInfo);
}


HRESULT YUV444toRGBFilter::SetMediaType( PIN_DIRECTION direction, const CMediaType *pmt )
{
  ASSERT(m_pConverter);

  HRESULT hr = CCustomBaseFilter::SetMediaType(direction, pmt);
  if (direction == PINDIR_INPUT)
  {
    m_nSizeUV = m_nInPixels >> 2;
    m_pConverter->SetDimensions(m_nInWidth, m_nInHeight );
  }
  else
  {
/*	NEVER DO THIS NASTY JOB!
      VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pmt->pbFormat;
      //always flip the image
      pVih->bmiHeader.biHeight *= -1;
*/
  }
  return hr;
}


HRESULT YUV444toRGBFilter::GetMediaType(int iPosition, CMediaType *pMediaType)
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
    if(FAILED(hr)) return hr;

    BITMAPINFOHEADER *pbmi = NULL;
    if(pMediaType->formattype==FORMAT_VideoInfo)
    {
      VIDEOINFOHEADER * const pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
      if(pVih) {pbmi=&(pVih->bmiHeader);}
    }
    else if(pMediaType->formattype==FORMAT_VideoInfo2)
    {
      VIDEOINFOHEADER2 * const pVih2 = (VIDEOINFOHEADER2*)pMediaType->pbFormat;
      if(pVih2) {pbmi=&(pVih2->bmiHeader);}
    }
    if(pbmi == NULL) return VFW_E_TYPE_NOT_ACCEPTED;


    // The hackery below should not be ncessary: we simply convert to RGB24
#if 0
    if (pbmi->biBitCount == 32)
    {
      // Set the bit count to 24 since the RBGtoYUV filter also accepts 32 bit but this filter only
      // outputs 24 bit RGB.
      pbmi->biBitCount = 24;
      pbmi->biSizeImage = m_nInPixels * BYTES_PER_PIXEL_RGB24;
    }
    else if (pbmi->biBitCount == 24)
    {
      // We still need to set the size of the out image: the original image might have been scaled
      // So we need to recalculate the size in RGB 24
      pbmi->biSizeImage = m_nInPixels * BYTES_PER_PIXEL_RGB24;
    }
    // It's a 12bit YUV source
    else if (pbmi->biBitCount == 12)
    {
      pbmi->biBitCount = 24;
      // We still need to set the size of the out image: the original image might have been scaled
      // So we need to recalculate the size in RGB 24
      pbmi->biSizeImage = m_nInPixels * BITS_PER_PIXEL_RGB24 * 8;
    }
#else

    pbmi->biBitCount = 24;
    pbmi->biSizeImage = m_nInPixels * BITS_PER_PIXEL_RGB24 / 8;

#endif

/*
    // negative height of YUV DIBS should be ignored according to 
    // http://msdn.microsoft.com/en-us/library/windows/desktop/dd407212(v=vs.85).aspx
    // we will set our output height negative according to m_bInvert
    if (m_bInvert)
    {
      if (pvi->bmiHeader.biHeight < 0) // make sure height is not already negative
        pvi->bmiHeader.biHeight = -1 * pvi->bmiHeader.biHeight;
    }
    else
    {
      // we will flip the image: make sure height is positive
      if (pvi->bmiHeader.biHeight > 0) // make sure height is not already negative
        pvi->bmiHeader.biHeight = -1 * pvi->bmiHeader.biHeight;
    }
*/

#if 0
    if (pvi->bmiHeader.biHeight < 0)
    {
      if (m_bInvert)
        pvi->bmiHeader.biHeight = -1 * pvi->bmiHeader.biHeight;
    }
    else
    {
      if (m_bInvert)
        pvi->bmiHeader.biHeight = -1 * pvi->bmiHeader.biHeight;
    }
#endif

    pMediaType->SetSampleSize( pbmi->biSizeImage );
    // adapt compression in case necessary
    pbmi->biCompression = BI_RGB;
    //Reset the media subtype
    // Change the output format from MEDIASUBTYPE_YUV444P to RGB24
    pMediaType->SetSubtype(&MEDIASUBTYPE_RGB24);
    return S_OK;
  }
  return VFW_S_NO_MORE_ITEMS;
}

HRESULT YUV444toRGBFilter::DecideBufferSize( IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp )
{
  AM_MEDIA_TYPE mt;
  HRESULT hr = m_pOutput->ConnectionMediaType(&mt);
  if (FAILED(hr))
  {
    return hr;
  }
  //Make sure that the format type is our custom format
  ASSERT(mt.formattype == FORMAT_VideoInfo);
  BITMAPINFOHEADER *pbmi = HEADER(mt.pbFormat);
  pProp->cbBuffer = DIBSIZE(*pbmi);

  if (pProp->cbAlign == 0)
  {
    pProp->cbAlign = 1;
  }
  if (pProp->cBuffers == 0)
  {
    pProp->cBuffers = 1;
  }
  // Release the format block.
  FreeMediaType(mt);

  // Set allocator properties.
  ALLOCATOR_PROPERTIES Actual;
  hr = pAlloc->SetProperties(pProp, &Actual);
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


HRESULT YUV444toRGBFilter::ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{
  ASSERT(m_pConverter);
  ASSERT(pBufferIn);
  ASSERT(pBufferOut);

  m_pConverter->Convert(pBufferIn, (void*)pBufferOut);
  DbgLog((LOG_TRACE, 0, TEXT("Converted from YUV444 to RGB Directly")));
  //RGB24 stores 3 Bytes per pixel
  lOutActualDataLength = m_nInPixels * BITS_PER_PIXEL_RGB24 / 8;
  return S_OK;
}


HRESULT YUV444toRGBFilter::CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut)
{
  //Does this also occur when format changes occur such as when changing the renderer size???
  long lSize = mtOut->GetSampleSize();

  // Check the major type.
  if (mtOut->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  BITMAPINFOHEADER *pBiOut = NULL;
  BITMAPINFOHEADER *pBiIn = NULL;

  if(mtOut->formattype == FORMAT_VideoInfo)
  {
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)mtOut->pbFormat;
    pBiOut = &(pVih->bmiHeader);
  }
  if(mtOut->formattype == FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 *pVih2 = (VIDEOINFOHEADER2*)mtOut->pbFormat;
    pBiOut = &(pVih2->bmiHeader);
  }

  if(mtIn->formattype == FORMAT_VideoInfo)
  {
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)mtIn->pbFormat;
    pBiIn = &(pVih->bmiHeader);
  }
  if(mtIn->formattype == FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 *pVih2 = (VIDEOINFOHEADER2*)mtIn->pbFormat;
    pBiIn = &(pVih2->bmiHeader);
  }

  if(pBiIn==NULL || pBiOut==NULL) return VFW_E_TYPE_NOT_ACCEPTED;

  if(labs(pBiIn->biWidth)!=labs(pBiOut->biWidth) ||
     labs(pBiIn->biHeight)!=labs(pBiOut->biHeight)) return VFW_E_TYPE_NOT_ACCEPTED;

  if(m_pConverter) {delete m_pConverter; m_pConverter=NULL;}
  
  if(mtOut->subtype==MEDIASUBTYPE_RGB32 || mtOut->subtype==MEDIASUBTYPE_ARGB32)
  {
    if(mtIn->subtype == MEDIASUBTYPE_AYUV)
        m_pConverter = new AYUV444toARGB32Converter(pBiIn->biWidth,pBiIn->biHeight);
    if(mtIn->subtype==MEDIASUBTYPE_YUV420P_S || mtIn->subtype==MEDIASUBTYPE_I420 || mtIn->subtype==MEDIASUBTYPE_YV12)
        m_pConverter = new YUV420toRGB32Converter(pBiIn->biWidth,pBiIn->biHeight);
  }

  if(mtOut->subtype == MEDIASUBTYPE_RGB24)
  {
    if(mtIn->subtype == MEDIASUBTYPE_AYUV)
        m_pConverter = new AYUV444toRGB24Converter(pBiIn->biWidth,pBiIn->biHeight);
    if(mtIn->subtype==MEDIASUBTYPE_YUV420P_S || mtIn->subtype==MEDIASUBTYPE_I420 || mtIn->subtype==MEDIASUBTYPE_YV12)
        m_pConverter = new YUV420toRGB24Converter(pBiIn->biWidth,pBiIn->biHeight);
  }

  if(m_pConverter==NULL) return VFW_E_TYPE_NOT_ACCEPTED;

  if(mtOut->subtype==MEDIASUBTYPE_RGB32 || mtOut->subtype==MEDIASUBTYPE_ARGB32 || mtOut->subtype == MEDIASUBTYPE_RGB24)
  {
    m_pConverter->SetFlip(m_bInvert ^ (pBiOut->biHeight<0));
  }

  return S_OK;	    // Everything is good.
}


STDMETHODIMP YUV444toRGBFilter::SetParameter(const char* type, const char* value)
{
  if (SUCCEEDED(CSettingsInterface::SetParameter(type, value)))
  {
    //m_pConverter->SetChrominanceOffset(m_nChrominanceOffset);
    m_pConverter->SetFlip(m_bInvert);
    return S_OK;
  }
  else
  {
    return E_FAIL;
  }
}
