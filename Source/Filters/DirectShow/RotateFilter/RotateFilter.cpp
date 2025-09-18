/** @file

MODULE				: RotateFilter

FILE NAME			: RotateFilter.cpp

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2017, CSIR
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

#include "RotateFilter.h"
#include <DirectShow/CommonDefs.h>
#include <PicRotateRGB24Impl.h>
#include <PicRotateRGB32Impl.h>
#include "../VersionInfo.h"
#include <Dvdmedia.h>			// VIDEOINFOHEADER2


RotateFilter::RotateFilter(): CCustomBaseFilter(NAME("CSIR VPP Rotate Filter"), 0, CLSID_VPP_RotateFilter),
	m_pRotate(NULL),
	m_nBitsPerPixel(BITS_PER_PIXEL_RGB24),
	m_nOutWidth(0),
	m_nOutHeight(0),
	m_nStride(0),
	m_nPadding(0)
{
	//Call the initialise input method to load all acceptable input types for this filter
	InitialiseInputTypes();
	// Init parameters
	initParameters();
}

RotateFilter::~RotateFilter()
{
	if (m_pRotate)
	{
		delete m_pRotate;
		m_pRotate = NULL;
	}
}

CUnknown * WINAPI RotateFilter::CreateInstance( LPUNKNOWN pUnk, HRESULT *pHr )
{
	RotateFilter *pFilter = new RotateFilter();
	if (pFilter== NULL) 
	{
		*pHr = E_OUTOFMEMORY;
	}
	return pFilter;
}


void RotateFilter::InitialiseInputTypes()
{
	AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24, &FORMAT_VideoInfo);
	AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB32, &FORMAT_VideoInfo);
        AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_ARGB32, &FORMAT_VideoInfo);
}

HRESULT RotateFilter::SetMediaType( PIN_DIRECTION direction, const CMediaType *pmt )
{
	HRESULT hr = CCustomBaseFilter::SetMediaType(direction, pmt);
	if (direction == PINDIR_INPUT)
	{
		//Determine whether we are connected to a RGB24 or 32 source
		if (pmt->majortype == MEDIATYPE_Video)
		{
			//The converter might already exist if the filter has been connected previously
			if (m_pRotate)
			{
				delete m_pRotate;
				m_pRotate = NULL;
			}
			if (pmt->subtype == MEDIASUBTYPE_RGB24)
			{
				m_pRotate = new PicRotateRGB24Impl();
				m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
			}
			else if (pmt->subtype == MEDIASUBTYPE_RGB32 || pmt->subtype == MEDIASUBTYPE_ARGB32)
			{
				m_pRotate = new PicRotateRGB32Impl();
				m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
			}
		}
	}
	return hr;
}


HRESULT RotateFilter::GetMediaType( int iPosition, CMediaType *pMediaType )
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
	        BITMAPINFOHEADER* pBi = NULL;
		RECT *prcSource = NULL;
		RECT *prcTarget = NULL;
		if(pMediaType->formattype==FORMAT_VideoInfo)
                {
                  VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
		  if(pVih) {pBi=&(pVih->bmiHeader); prcSource=&pVih->rcSource; prcTarget=&pVih->rcTarget;}
                }
                else if(pMediaType->formattype==FORMAT_VideoInfo2)
                {
	          VIDEOINFOHEADER2 *pVih2 = (VIDEOINFOHEADER2*)pMediaType->pbFormat;
		  if(pVih2) {pBi=&(pVih2->bmiHeader); prcSource=&pVih2->rcSource; prcTarget=&pVih2->rcTarget;}
		}
                if(pBi==NULL) return VFW_E_TYPE_NOT_ACCEPTED;

				  //Now we need to calculate the size of the output image		
		switch (m_nRotation)
		{
		case ROTATE_NONE:
		case ROTATE_180_DEGREES_CLOCKWISE:
		case ROTATE_FLIP_HORIZONTAL:
		case ROTATE_FLIP_VERTICAL:
			{
				m_nOutWidth = m_nInWidth;
				m_nOutHeight = m_nInHeight;
				break;
			}
		case ROTATE_90_DEGREES_CLOCKWISE:
		case ROTATE_270_DEGREES_CLOCKWISE:
		case ROTATE_90_DEGS_CCK_VFLIP:
		case ROTATE_FLIP_DIAGONALLY:
			{
				m_nOutWidth = m_nInHeight;
				m_nOutHeight = m_nInWidth;
				break;
			}
		}
		pBi->biHeight = m_nOutHeight;
		if(pBi->biHeight <= 0) return E_INVALIDARG;

		pBi->biWidth = m_nOutWidth;
		if(pBi->biWidth <= 0) return E_INVALIDARG;
		pBi->biSizeImage = (pBi->biWidth * pBi->biHeight * m_nBitsPerPixel) / 8;

		prcSource->top = 0;
		prcSource->left = 0;
		prcSource->right = pBi->biWidth;
		prcSource->bottom = pBi->biHeight;

		prcTarget->top = 0;
		prcTarget->left = 0;
		prcTarget->right = pBi->biWidth;
		prcTarget->bottom = pBi->biHeight;

		pMediaType->lSampleSize = pBi->biSizeImage;

		RecalculateFilterParameters();
		return S_OK;
	}
	return VFW_S_NO_MORE_ITEMS;
}

HRESULT RotateFilter::DecideBufferSize( IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp )
{
	// Adding padding to take stride into account
	pProp->cbBuffer = ((m_nOutWidth + m_nPadding) * m_nOutHeight * m_nBitsPerPixel) / 8;

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


HRESULT RotateFilter::CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut)
{
	//Make sure the input and output types are related
  if (mtOut->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }
  	// Video format
  m_Vflip = false;
  if(mtOut->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER * const pVihOut = (const VIDEOINFOHEADER*)mtOut->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(pBiOut->biHeight != m_nOutHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutHeight))
        m_Vflip = true;
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(pBiOut->biWidth != m_nOutWidth)
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
  }
  else if(mtOut->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2 * const pVihOut = (const VIDEOINFOHEADER2*)mtOut->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(pBiOut->biHeight != m_nOutHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutHeight))
        m_Vflip = true;
      else
      {
        DbgLog((LOG_TRACE, 0, TEXT("Rotator - Unaccepted height at last negoitation stage")));
        return VFW_E_TYPE_NOT_ACCEPTED;
      }
    }
    if(pBiOut->biWidth != m_nOutWidth)
    {
      DbgLog((LOG_TRACE, 0, TEXT("Rotator - Unaccepted width at last negoitation stage")));
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }
  else
      return VFW_E_TYPE_NOT_ACCEPTED;

	// Setup rotator parameters.
  if(m_pRotate)
  {
    m_pRotate->SetInDimensions(m_nInWidth, m_nInHeight);
    if(m_Vflip)		// Substitute operation when next DShow object enforces VFlip.
    {
      switch(m_nRotation)
      {
        case ROTATE_NONE: m_pRotate->SetRotateMode(ROTATE_FLIP_VERTICAL); break;
        case ROTATE_FLIP_VERTICAL: m_pRotate->SetRotateMode(ROTATE_NONE); break;

        case ROTATE_180_DEGREES_CLOCKWISE: m_pRotate->SetRotateMode(ROTATE_FLIP_HORIZONTAL); break;
        case ROTATE_FLIP_HORIZONTAL: m_pRotate->SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE); break;

        case ROTATE_90_DEGREES_CLOCKWISE: m_pRotate->SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP); break;
        case ROTATE_90_DEGS_CCK_VFLIP: m_pRotate->SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE); break;

        case ROTATE_270_DEGREES_CLOCKWISE: m_pRotate->SetRotateMode(ROTATE_FLIP_DIAGONALLY); break;
        case ROTATE_FLIP_DIAGONALLY: m_pRotate->SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE); break;
     
        default: m_pRotate->SetRotateMode((ROTATE_MODE)m_nRotation);  // TODO: dopsat ROTATE_FLIP_DIAGONALLY!!!!
     }
    }
    else
        m_pRotate->SetRotateMode((ROTATE_MODE)m_nRotation);
  }

	// Check videotype compatibility
  if(mtIn->subtype == MEDIASUBTYPE_RGB24)
  {
    if(mtOut->subtype == MEDIASUBTYPE_RGB24) return S_OK;
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if(mtIn->subtype==MEDIASUBTYPE_RGB32 || mtIn->subtype==MEDIASUBTYPE_ARGB32)
  {
    if(mtOut->subtype==MEDIASUBTYPE_RGB32 || mtOut->subtype==MEDIASUBTYPE_ARGB32) return S_OK;
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  return VFW_E_TYPE_NOT_ACCEPTED;
}


STDMETHODIMP RotateFilter::SetParameter( const char* type, const char* value )
{
	// For now, one cannot set any parameters once the output has been connected -> this will affect the buffer size
	if (m_pOutput)
	{
		if (m_pOutput->IsConnected())
		{
			return E_FAIL;
		}
	}

	if (SUCCEEDED(CSettingsInterface::SetParameter(type, value)))
	{
		return S_OK;
	}
	else
	{
		return E_FAIL;
	}
}


HRESULT RotateFilter::ApplyTransform(BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{
  if(m_pRotate==NULL)
  {
    DbgLog((LOG_TRACE, 0, TEXT("Rotator is not initialised - unable to rotate")));
    return E_FAIL;
  }
  if(pBufferIn==NULL || pBufferOut==NULL) return E_POINTER;

  lOutActualDataLength = (labs(m_nOutWidth*m_nOutHeight) * m_nBitsPerPixel)/8;
  if(lOutActualDataLength > lOutBufferSize) return E_INVALIDARG;	// Output buffer is too small
  if(lOutActualDataLength > lActualDataLength) return E_INVALIDARG;	// Input buffer is too small

  if(m_pRotate->Rotate((void*)pBufferIn, (void*)pBufferOut))  return S_OK;
  return E_FAIL;
}


void RotateFilter::RecalculateFilterParameters()
{
// 	m_nStride =  (m_nOutWidth * (m_nBitCount / 8) + 3) & ~3;
// 	m_nPadding = m_nStride - (m_nBytesPerPixel * m_nOutWidth);

	if (m_nBitsPerPixel == BITS_PER_PIXEL_RGB24)
	{
		m_nStride =  (m_nOutWidth * (m_nBitCount / 8) + 3) & ~3;
		m_nPadding = m_nStride - (m_nBitsPerPixel * m_nOutWidth)/8;
	}
	else if(m_nBitsPerPixel == BITS_PER_PIXEL_RGB32)
	{
		m_nStride =  (m_nOutWidth * (m_nBitCount >> 3));
		m_nPadding = 32 - (m_nStride%32);
		//m_nStride += m_nStride % 32;
	}
}


void RotateFilter::doGetVersion(std::string& sVersion)
{
  sVersion = VersionInfo::toString();
}
