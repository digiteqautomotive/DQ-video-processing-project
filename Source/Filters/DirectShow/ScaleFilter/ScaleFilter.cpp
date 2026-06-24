/** @file

MODULE:			ScaleFilter

FILE NAME:		ScaleFilter.cpp

DESCRIPTION:

LICENSE:		Software License Agreement (BSD License)

Copyright (c) 2008-2015, CSIR
Copyright (c) 2025-2026, Digiteq automotive, programmed by Jaroslav Fojtik	All rights reserved.

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

// CSIR includes
#include "ScaleFilter.h"
#include "DirectShow/CommonDefs.h"
#include "DirectShow/CustomMediaTypes.h"
#include "DirectShow/DirectShowMediaFormats.h"

#include <PicScalerRGB24Impl.h>
#include <PicScalerRGB32Impl.h>
#include <PicScalerARGB32Impl.h>
#include <PicScalerRGB565Impl.h>
#include <PicScalerYUV420PImpl.h>
#include <PicScalerYUYVImpl.h>
#include <PicScalerUYVYImpl.h>
#include <PicScalerYV24Impl.h>

#ifdef USE_MMX
#include <PicScalerARGB32MMX.h>
#endif
#ifdef USE_SSE
 #include <PicScalerARGB32SSE.h>
#endif

#include <wmcodecdsp.h>			// MEDIASUBTYPE_I420; MEDIASUBTYPE_AVC1
#include <Dvdmedia.h>			// VIDEOINFOHEADER2


#if defined(USE_MMX) || defined(USE_SSE)
extern "C" unsigned GetFeaturesCPU(void);
const unsigned FeaturesCPU = GetFeaturesCPU();
#endif

#include "DirectShow/VariantTools.h"


DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_HDYC, 0x43594448, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

const OLECHAR OLE_PARAM_TARGET_WIDTH[] =  olestr(FILTER_PARAM_TARGET_WIDTH);  // L"TEST";  //  sstr(FILTER_PARAM_TARGET_WIDTH);
const OLECHAR OLE_PARAM_TARGET_HEIGHT[] = olestr(FILTER_PARAM_TARGET_HEIGHT);


class ScaleFilterInputPin: public CTransformInputPin
{
public: ScaleFilterInputPin(__in_opt LPCTSTR pObjectName, __inout ScaleFilter *pTransformFilter,  __inout HRESULT * phr, __in_opt LPCWSTR pName):        
            CTransformInputPin(pObjectName,pTransformFilter,phr,pName)
	{}

        STDMETHODIMP ReceiveConnection(IPin * pConnector, const AM_MEDIA_TYPE *pmt);
};


HRESULT ScaleFilterInputPin::ReceiveConnection(IPin * pConnector, const AM_MEDIA_TYPE *pmt)
{
  if(m_Connected && pConnector==m_Connected && m_pTransformFilter!=NULL && pmt!=NULL)
  {
    if(m_mt.subtype == pmt->subtype)
    if(SUCCEEDED(((ScaleFilter*)m_pTransformFilter)->ReceiveConnection(pmt)))
    {
      CMediaType * pcmt = (CMediaType*)pmt;
      return(SetMediaType(pcmt));
                  //return CompleteConnect(pConnector);
    }
  }
  return CTransformInputPin::ReceiveConnection(pConnector,pmt);
}


/////////////////////////////////////////////////////////////


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


/////////////////////////////////////////////////////////////


ScaleFilter::ScaleFilter(): CCustomBaseFilter(NAME("DQ VPP Scale Filter"), 0, CLSID_VPP_ScaleFilter),
  m_pScaler(NULL),
  m_nBitsPerPixel(0)
  //m_nReqOutWidth(0), m_nReqOutHeight(0)  
{
  //Call the initialize input method to load all acceptable input types for this filter
  InitialiseInputTypes();
  // Init parameters
  initParameters();
}


void ScaleFilter::initParameters(void)
{
  addParameter(FILTER_PARAM_TARGET_WIDTH, OLE_PARAM_TARGET_WIDTH, &m_nReqOutWidth, 0);
  addParameter(FILTER_PARAM_TARGET_HEIGHT, OLE_PARAM_TARGET_HEIGHT, &m_nReqOutHeight, 0);
  addParameter(FILTER_PARAM_ACT_WIDTH, &m_nOutWidth, m_nOutWidth, true);
  addParameter(FILTER_PARAM_ACT_HEIGHT, &m_nOutHeight, m_nOutHeight, true);
  // addParameter(FILTER_PARAM_MODE, &m_eMode, MODE_ASPECT_RATIO_CORRECT_SCALING1); - J.F. removed
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
  if(pHr) *pHr = (pFilter==NULL) ? E_OUTOFMEMORY : S_OK;
  return pFilter;
}


static const GUID * const ScalleSupportedFormats[] = {
	&MEDIASUBTYPE_RGB24,&MEDIASUBTYPE_RGB32,&MEDIASUBTYPE_ARGB32,
	&MEDIASUBTYPE_YUY2, &MEDIASUBTYPE_YUYV, &MEDIASUBTYPE_YVYU, &MEDIASUBTYPE_UYVY, &MEDIASUBTYPE_HDYC,
	&MEDIASUBTYPE_YUV420P_S, &MEDIASUBTYPE_I420, &MEDIASUBTYPE_YV12, &MEDIASUBTYPE_IYUV,  // I420 = IYUV = YUV420p (sometimes YUV420p can refer to YV12) https://gist.github.com/Jim-Bar/3cbba684a71d1a9d468a6711a6eddbeb
	&MEDIASUBTYPE_AYUV, &MEDIASUBTYPE_YV24,
	&MEDIASUBTYPE_RGB565, &MEDIASUBTYPE_RGB555};

void ScaleFilter::InitialiseInputTypes(void)
{
  for(int i=0; i<sizeof(ScalleSupportedFormats)/sizeof(GUID*); i++)
  {
    AddInputType(&MEDIATYPE_Video, ScalleSupportedFormats[i], &FORMAT_VideoInfo);
    AddInputType(&MEDIATYPE_Video, ScalleSupportedFormats[i], &FORMAT_VideoInfo2);
  }
}


HRESULT ScaleFilter::SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt)
{
  HRESULT hr = CCustomBaseFilter::SetMediaType(direction, pmt);
  if (direction == PINDIR_INPUT)
  {
    //Set defaults
    if (pmt->majortype == MEDIATYPE_Video)
    {
      //The scaler might already exist if the filter has been connected previously
      if(m_pScaler)
      {
        if(!m_pScaler->isVideoFormatSupported(pmt->subtype))
        {
          delete m_pScaler;
          m_pScaler = NULL;
        }
      }
	// Determine video source type
      if(pmt->subtype==MEDIASUBTYPE_RGB24)
      {
        if(m_pScaler==NULL) m_pScaler = new PicScalerRGB24Impl();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
      }
      if(pmt->subtype==MEDIASUBTYPE_YV24)
      {
	if(m_pScaler==NULL) m_pScaler = new PicScalerYV24Impl();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
      }
      else if(pmt->subtype==MEDIASUBTYPE_RGB32)
      {
        if(m_pScaler==NULL) m_pScaler = GetPicScallerRGB32();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
      }
      else if(pmt->subtype==MEDIASUBTYPE_ARGB32 || pmt->subtype==MEDIASUBTYPE_AYUV)
      {
        if(m_pScaler==NULL) m_pScaler = GetPicScallerRGB32();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
      }
      else if(pmt->subtype==MEDIASUBTYPE_YUV420P_S || pmt->subtype==MEDIASUBTYPE_I420 || pmt->subtype==MEDIASUBTYPE_YV12 || pmt->subtype==MEDIASUBTYPE_IYUV)
      {
        if(m_pScaler==NULL) m_pScaler = new PicScalerYUV420PImpl();
        m_nBitsPerPixel = BITS_PER_PIXEL_YUV420P;
      } 
      else if(pmt->subtype==MEDIASUBTYPE_YUYV || pmt->subtype==MEDIASUBTYPE_YUY2 || pmt->subtype==MEDIASUBTYPE_YVYU)
      {
        if(m_pScaler==NULL) m_pScaler = new PicScalerYUYVImpl();		// YVYU and YUYV are different, but the can use a same scaller.
        m_nBitsPerPixel = BITS_PER_PIXEL_YUYV;
      }
      else if(pmt->subtype==MEDIASUBTYPE_UYVY || pmt->subtype==MEDIASUBTYPE_HDYC)
      {
        if(m_pScaler==NULL) m_pScaler = new PicScalerUYVYImpl();
        m_nBitsPerPixel = BITS_PER_PIXEL_YUYV;					// Same bpp as YUYV
      }
      else if(pmt->subtype==MEDIASUBTYPE_RGB565)
      {
        if(m_pScaler==NULL) m_pScaler = new PicScalerRGB565Impl();
        m_nBitsPerPixel = 16;
      }
      else if(pmt->subtype==MEDIASUBTYPE_RGB555)
      {
        if(m_pScaler==NULL) m_pScaler = new PicScalerRGB555Impl();
        m_nBitsPerPixel = 16;
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
    if (FAILED(hr)) return hr;

    // Get the bitmap info header and adapt the cropped 
    //make sure that it's a video info header
    BITMAPINFOHEADER *pBi = NULL;
    RECT *rcSource;
    RECT *rcTarget;
    if(pMediaType->formattype==FORMAT_VideoInfo)
    {
      VIDEOINFOHEADER * const pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
      if(pVih) {pBi=&(pVih->bmiHeader); rcSource=&(pVih->rcSource); rcTarget=&(pVih->rcTarget);}
    }
    else if(pMediaType->formattype==FORMAT_VideoInfo2)
    {    
      if(pMediaType->pbFormat != NULL)
      {
        if(pMediaType->cbFormat >= sizeof(VIDEOINFOHEADER2))
        {
          VIDEOINFOHEADER2 tempVINF2;
	  memcpy(&tempVINF2,pMediaType->pbFormat,sizeof(VIDEOINFOHEADER2));
        
	  pMediaType->ResetFormatBuffer();
          pMediaType->AllocFormatBuffer(sizeof(VIDEOINFOHEADER));

          VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
          if(pVih)
          {
            pMediaType->formattype = FORMAT_VideoInfo;

            pBi = &(pVih->bmiHeader); rcSource=&(pVih->rcSource); rcTarget=&(pVih->rcTarget);
            memset(pVih,0,sizeof(VIDEOINFOHEADER));
            pVih->rcSource = tempVINF2.rcSource;
            pVih->rcTarget = tempVINF2.rcTarget;
            pVih->dwBitRate = tempVINF2.dwBitRate;
            pVih->AvgTimePerFrame = tempVINF2.AvgTimePerFrame;
            pVih->dwBitErrorRate = tempVINF2.dwBitErrorRate;
            memcpy(pBi, &tempVINF2.bmiHeader, sizeof(BITMAPINFOHEADER));
          }	
        }
      }
    }
    if(pBi == NULL) return VFW_E_TYPE_NOT_ACCEPTED;

    //Now we need to calculate the size of the output image
    if(m_nReqOutHeight!=0 && m_nReqOutWidth!=0)
    {
	// Set height
      pBi->biHeight = (pBi->biHeight<0) ? -m_nReqOutHeight : m_nReqOutHeight;	// Propagate negative height to output.
      if(pBi->biHeight == 0) return E_INVALIDARG;

	// Set width
      pBi->biWidth = m_nReqOutWidth;
      if(pBi->biWidth <= 0) return E_INVALIDARG;

	// Set size
      if(pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_ARGB32 || pMediaType->subtype==MEDIASUBTYPE_AYUV)
          pBi->biSizeImage = labs(pBi->biWidth*pBi->biHeight) * (BITS_PER_PIXEL_RGB32 / 8);
      else if(pMediaType->subtype==MEDIASUBTYPE_YUV420P_S || pMediaType->subtype==MEDIASUBTYPE_I420 || pMediaType->subtype==MEDIASUBTYPE_YV12 || pMediaType->subtype==MEDIASUBTYPE_IYUV)
      {
        if((pBi->biWidth & 1)==1)
            pBi->biWidth &= ~1;			// Odd width values cannot be used for I420 encoding.
        if((pBi->biHeight & 1)==1)
            pBi->biHeight &= ~1;		// Nor odd height can be accepted.
        pBi->biSizeImage = (labs(pBi->biWidth*pBi->biHeight) * BITS_PER_PIXEL_YUV420P) / 8;
      }
      else if(pMediaType->subtype==MEDIASUBTYPE_YUY2 || pMediaType->subtype==MEDIASUBTYPE_YUYV || pMediaType->subtype==MEDIASUBTYPE_YVYU ||
             pMediaType->subtype==MEDIASUBTYPE_UYVY || pMediaType->subtype==MEDIASUBTYPE_HDYC)
      {
        if((pBi->biWidth & 1)==1)
            pBi->biWidth &= ~1;			// Odd values cannot be used for YUYV encoding.
        pBi->biSizeImage = (labs(pBi->biWidth*pBi->biHeight) * BITS_PER_PIXEL_YUYV) / 8;
      }
      else if(pMediaType->subtype==MEDIASUBTYPE_RGB565 || pMediaType->subtype==MEDIASUBTYPE_RGB555)
      {
        pBi->biSizeImage = labs(pBi->biWidth*pBi->biHeight) * 2;
      }
      else		// MEDIASUBTYPE_RGB24 or MEDIASUBTYPE_YV24
          pBi->biSizeImage = labs(pBi->biWidth*pBi->biHeight) * (BITS_PER_PIXEL_RGB24 / 8);
     pMediaType->lSampleSize = pBi->biSizeImage;
    }
	// Adjust recs
    rcSource->top = 0;
    rcSource->left = 0;
    rcSource->right = pBi->biWidth;
    rcSource->bottom = pBi->biHeight;

    rcTarget->top = 0;
    rcTarget->left = 0;
    rcTarget->right = pBi->biWidth;
    rcTarget->bottom = pBi->biHeight;

    return S_OK;
  }
  return VFW_S_NO_MORE_ITEMS;
}


HRESULT ScaleFilter::DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp)
{
  // Leaving this var so that we can cater for RGB32 at a later stage
  pProp->cbBuffer = (m_nOutPixels * m_nBitsPerPixel) / 8;

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
  m_Vflip = false;
  if(mtOut->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER * const pVihOut = (const VIDEOINFOHEADER*)mtOut->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(m_nOutHeight!=0 && pBiOut->biHeight!=m_nOutHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutHeight))
        m_Vflip = true;
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(m_nOutWidth!=0 && pBiOut->biWidth!=m_nOutWidth)
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
  }
  else if(mtOut->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2 * const pVihOut = (const VIDEOINFOHEADER2*)mtOut->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(m_nOutHeight!=0 && pBiOut->biHeight != m_nOutHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutHeight))
        m_Vflip = true;
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(m_nOutWidth!=0 && pBiOut->biWidth!=m_nOutWidth)
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
  }
  else
      return VFW_E_TYPE_NOT_ACCEPTED;


	//Subtypes
  if(mtIn->subtype == MEDIASUBTYPE_RGB24)
  {
    if (mtOut->subtype != MEDIASUBTYPE_RGB24)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
  if(mtIn->subtype == MEDIASUBTYPE_YV24)
  {
    if (mtOut->subtype != MEDIASUBTYPE_YV24)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if(mtIn->subtype==MEDIASUBTYPE_RGB32 || mtIn->subtype==MEDIASUBTYPE_ARGB32)
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
  if(mtIn->subtype == MEDIASUBTYPE_YVYU)
  {
    if (mtOut->subtype != MEDIASUBTYPE_YVYU)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
  if(mtIn->subtype == MEDIASUBTYPE_AYUV)
  {
    if (mtOut->subtype != MEDIASUBTYPE_AYUV)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
  if(mtIn->subtype==MEDIASUBTYPE_UYVY || mtIn->subtype==MEDIASUBTYPE_HDYC)
  {
    if (mtOut->subtype!=MEDIASUBTYPE_UYVY && mtOut->subtype!=MEDIASUBTYPE_HDYC)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if(mtIn->subtype==MEDIASUBTYPE_YUV420P_S || mtIn->subtype==MEDIASUBTYPE_I420 || mtIn->subtype==MEDIASUBTYPE_YV12 || mtIn->subtype==MEDIASUBTYPE_IYUV)
  {
    if(mtOut->subtype!=MEDIASUBTYPE_YUV420P_S && mtOut->subtype!=MEDIASUBTYPE_I420 && mtOut->subtype!=MEDIASUBTYPE_YV12 && mtOut->subtype!=MEDIASUBTYPE_IYUV)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
  if (mtIn->subtype == MEDIASUBTYPE_RGB565)
  {
    if (mtOut->subtype != MEDIASUBTYPE_RGB565)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
  if(mtIn->subtype == MEDIASUBTYPE_RGB555)
  {
    if (mtOut->subtype != MEDIASUBTYPE_RGB555)
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
			  // TODO: get rid of enum or get rid of string!!!!
  return CSettingsInterface::SetParameter(type, value);
}


HRESULT ScaleFilter::ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{
	//make sure we were able to initialize our converter
  if(pBufferIn==NULL || pBufferOut==NULL) return E_POINTER;

  lOutActualDataLength = labs(m_nInWidth * m_nInHeight * m_nBitsPerPixel) / 8;		// Calculate input data length
  if(lOutActualDataLength > lInBufferSize) return E_INVALIDARG;			// Input buffer is too small

  if(m_nOutWidth==m_nInWidth && m_nOutHeight==m_nInHeight)
  {
    if(lOutActualDataLength > lOutBufferSize) return E_INVALIDARG;
    memcpy(pBufferOut,pBufferIn,lOutActualDataLength);
  }
  else
  {
    m_nOutPixels = labs(m_nOutWidth*m_nOutHeight);		// Update the number of out pixels
    lOutActualDataLength = (m_nOutPixels * m_nBitsPerPixel) / 8;	// Calculate output data length
    if(lOutActualDataLength > lOutBufferSize) return E_INVALIDARG;
    if(m_pScaler==NULL)
    {
      DbgLog((LOG_TRACE, 0, TEXT("Scaller is not initialised - unable to scale")));
      return E_FAIL;
    }
	//Call scaling conversion code
    m_pScaler->SetInDimensions(m_nInWidth, m_nInHeight);
    m_pScaler->SetOutDimensions(m_nOutWidth, m_nOutHeight);
    int res = m_pScaler->Scale(pBufferOut, pBufferIn, m_Vflip);
    ASSERT(res == 1);
  }
  return S_OK;
}


HRESULT ScaleFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == IID_IPersistPropertyBag)   return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPersistStream)	return GetInterface(static_cast<IPersistStream*>(this), ppv);
  if(riid == IID_IPropertyBag)		return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_ISpecifyPropertyPages) return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);

	// Call the parent class.
  return CCustomBaseFilter::NonDelegatingQueryInterface(riid, ppv);
}


HRESULT ScaleFilter::GetPages(CAUUID *pPages)
{
  if(pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if (pPages->pElems == NULL)
  {
    pPages->cElems = 0;
    return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_ScaleProperties;
  return S_OK;
}


CBasePin *ScaleFilter::GetPin(int n)
{
  if(m_pInput == NULL)
  {
    HRESULT hr = S_OK;
    m_pInput = new ScaleFilterInputPin(NAME("Scale input pin"),this,&hr,L"XForm In");  //ScaleFilterInputPin
    ASSERT(SUCCEEDED(hr));
    if (m_pInput == NULL) return NULL;
    m_pOutput = new CTransformOutputPin(NAME("Scale output pin"), this, &hr, L"XForm Out");   // Pin name
    ASSERT(SUCCEEDED(hr));    
    if (m_pOutput == NULL) return NULL;
  }
  return CCustomBaseFilter::GetPin(n);
}


/// This inherited function allows format change reconnection.
HRESULT ScaleFilter::ReceiveConnection(const AM_MEDIA_TYPE *pmt)
{
  if(pmt!=NULL && pmt->majortype==MEDIATYPE_Video)		// || pmt->majortype==MEDIATYPE_Video2)
  {
    VIDEOINFOHEADER * const pVih = (VIDEOINFOHEADER*)pmt->pbFormat;
    if(pVih)
    {
      const BITMAPINFOHEADER* const pBiIn = &(pVih->bmiHeader);
      if(pBiIn->biWidth>0 && pBiIn->biHeight!=0)
      {
        //m_nInWidth = pBiIn->biWidth;
        //m_nInHeight = pBiIn->biHeight;
        return S_OK;
      }
    }
  }  
  return E_FAIL;
}


//----------------- IPersistPropertyBag------------------


HRESULT ScaleFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_ScaleFilter;
  return S_OK;
}
