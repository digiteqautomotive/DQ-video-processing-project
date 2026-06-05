/** @file

MODULE:			TogglerFilter

FILE NAME:		TogglerFilter.cpp

DESCRIPTION: 
					  
LICENSE:		Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive; designed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#include "stdafx.h"
#include "TogglerFilter.h"
#include <DirectShow/CommonDefs.h>

#include "PicScalerRGB24Impl.h"
#include "PicScalerRGB32Impl.h"
#include "PicScalerYV24Impl.h"
#include "PicScalerRGB565Impl.h"

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

#include "DirectShow/VariantTools.h"

#include <process.h>


//////////////////////////////////////////////////////////


DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_HDYC, 0x43594448, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);
DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);
DEFINE_GUID(MEDIASUBTYPE_YUV420P_S, 0xc528b79f, 0xf025, 0x4eb7, 0x92, 0xb0, 0x46, 0xce, 0x81, 0x67, 0xe9, 0xf8);


const OLECHAR OLE_PARAM_TARGET_WIDTH[] =  olestr(FILTER_PARAM_TARGET_WIDTH);  // L"TEST";  //  sstr(FILTER_PARAM_TARGET_WIDTH);
const OLECHAR OLE_PARAM_TARGET_HEIGHT[] = olestr(FILTER_PARAM_TARGET_HEIGHT);
const OLECHAR OLE_PARAM_THROTTLE[] = olestr(FILTER_THROTTLE);
const OLECHAR OLE_PARAM_DUPLICATE_FRAMES[] = olestr(FILTER_DUPLICATE_FRAMES);
const OLECHAR OLE_PARAM_FLIP[] =  olestr(FILTER_PARAM_ORIENTATION);
const OLECHAR OLE_PARAM_CHOOSE_INPUT[] = olestr(FILTER_PARAM_CHOOSE_INPUT);
const OLECHAR OLE_PARAM_DURATION[] = olestr(FILTER_PARAM_DURATION);


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


TogglerFilter::TogglerFilter(): VideoMixingBase(NAME("DQ VPP Toggle Filter"), 0, CLSID_VPP_TogglerFilter),
  m_nTargetWidth(0),		// Initialised in initParameters(), no need to initialise twice.
  m_nTargetHeight(0),
  m_bThrottling(false),
  m_DuplicateFrame(true),
  m_PicScaler1(NULL),
  m_PicScaler2(NULL),
  m_nBitsPerPixel(0),
  m_Vflip(false),
  m_bToggle(false),
  m_duration(0),
  m_MaxInterval(50000000)
{
  m_nSampleSizes[0] = 0;
  m_nSampleSizes[1] = 0;

	// Virtual method of subclass can't be called from base class contructor, hence must be called in child contructor
  Initialise();
	// Init parameters
  initParameters();
}


void TogglerFilter::InitialiseInputTypes(void)
{
  AddInputType(&MEDIATYPE_Video, NULL, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, NULL, &FORMAT_VideoInfo2);
}


void TogglerFilter::InitialiseOutputTypes(void)
{
  AddOutputType(&MEDIATYPE_Video, NULL, &FORMAT_VideoInfo);
  AddOutputType(&MEDIATYPE_Video, NULL, &FORMAT_VideoInfo2);
}


void TogglerFilter::initParameters(void)
{
  addParameter(FILTER_PARAM_TARGET_WIDTH, OLE_PARAM_TARGET_WIDTH, &m_nTargetWidth);	// width output picture is scaled to
  addParameter(FILTER_PARAM_TARGET_HEIGHT, OLE_PARAM_TARGET_HEIGHT, &m_nTargetHeight);	// height output picture is scaled to

  addParameter(FILTER_PARAM_ACT_WIDTH, &m_nOutputWidth, true);
  addParameter(FILTER_PARAM_ACT_HEIGHT, &m_nOutputHeight, true);


  addParameter(FILTER_PARAM_CHOOSE_INPUT,OLE_PARAM_CHOOSE_INPUT, &m_bToggle, m_bToggle);
  addParameter(FILTER_PARAM_ORIENTATION, OLE_PARAM_FLIP, &m_Vflip, m_Vflip);

  addParameter(FILTER_THROTTLE, OLE_PARAM_THROTTLE, &m_bThrottling, m_bThrottling);
  addParameter(FILTER_DUPLICATE_FRAMES, OLE_PARAM_DUPLICATE_FRAMES, &m_DuplicateFrame, m_DuplicateFrame);

  m_vParams.push_back(new ParameterRefTime(FILTER_PARAM_DURATION, OLE_PARAM_DURATION, &m_MaxInterval, m_MaxInterval));
}


TogglerFilter::~TogglerFilter()
{
  if (m_PicScaler1)
  {
    delete m_PicScaler1;
    m_PicScaler1 = NULL;
  }
  if(m_PicScaler2)
  {
    delete m_PicScaler2;
    m_PicScaler2 = NULL;
  }
}


CUnknown * WINAPI TogglerFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  TogglerFilter* pFilter = new TogglerFilter();
  if(pHr) *pHr = (pFilter==NULL) ? E_OUTOFMEMORY : S_OK;
  return pFilter;
}


void TogglerFilter::VFlipImg1(const BYTE* pSrc, BYTE* pDest)
{
const int nRowLength = (labs(m_VideoInHeader[0].bmiHeader.biWidth) * m_nBitsPerPixel)/8;
 ASSERT(pSrc!=NULL);
 ASSERT(pDest!=NULL);
 pDest += ((labs(m_VideoInHeader[0].bmiHeader.biHeight) - 1) * nRowLength);
 int i = m_nOutputHeight;
 while(i-- > 0)
 {
   memcpy(pDest, pSrc, nRowLength);
   pSrc+=nRowLength; pDest-=nRowLength;
 }
}


void TogglerFilter::VFlipImg2(const BYTE* pSrc, BYTE* pDest)
{
const int nRowLength = (labs(m_VideoInHeader[1].bmiHeader.biWidth) * m_nBitsPerPixel)/8;
 ASSERT(pSrc!=NULL);
 ASSERT(pDest!=NULL);
 pDest += ((labs(m_VideoInHeader[1].bmiHeader.biHeight) - 1) * nRowLength);
 int i = m_nOutputHeight;
 while(i-- > 0)
 {
   memcpy(pDest, pSrc, nRowLength);
   pSrc+=nRowLength; pDest-=nRowLength;
 }
}


HRESULT TogglerFilter::ReceiveFirstSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()

  if(!m_vOutputPins[0]->IsConnected())
  {
    FQ1.Erase(); FQ2.Erase();
    return S_FALSE;	// Do nothing when no output is connected.
  }

  if(m_bThrottling)
  {
    pSample->GetTime(&TimeStart1,&TimeEnd1);
    if(TimeStart1 > TimeEnd2)
    {
      IMediaSample *pSample2 = FQ2.PeekFrame(TimeStart1,TimeEnd1);
      if(pSample2)
      {
        pSample2->GetTime(&TimeStart2,&TimeEnd2);        
        pSample2->Release();
      }
    }
  }
  else
  {
    FQ1.Erase(); FQ2.Erase();
  }

  HRESULT hr = S_OK;
  if(!m_bToggle || !m_vInputPins[1]->IsConnected())
  {
    if(m_PicScaler1==NULL && !m_Vflip && !m_DuplicateFrame)    
      hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);		// Populate intact sample to the output.
    else
    {
      FQ1.Erase();
      BYTE *pBufferIn1 = NULL;
      hr = pSample->GetPointer(&pBufferIn1);
      if(FAILED(hr))
      {
        DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveFirstSample - cannot get pointer to pSample1 data, error %Xh"), hr));
        return hr;
      }
      if(pBufferIn1==NULL)
      {
        DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveFirstSample - NULL pointer in 2nd sample input data.")));
        return E_POINTER;
      }

      const long SampleSize1 = pSample->GetSize();
      const long RealOutputSize = (m_nOutputSize==0) ? SampleSize1 : m_nOutputSize;
      IMediaSample *pOutSample = NULL;
      {
        CAutoLock lck(&m_csReceive);
        HRESULT hr = CMultiIOBaseFilter::InitializeOutputSample(pSample, &pOutSample, 0, 0);	// Set up the output sample
        if(FAILED(hr))
        {
           DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveFirstSample - CMultiIOBaseFilter::InitializeOutputSample failed hr=%Xh"),hr));
           return hr;
        }
        hr = pOutSample->SetActualDataLength(RealOutputSize);
        if(FAILED(hr))
        {
          DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveFirstSample - pOutSample->SetActualDataLength(%d) failed hr=%Xh"),RealOutputSize,hr));
          pOutSample->Release();
          return hr;
        }
        BYTE *pBufferOut = NULL;
        hr = pOutSample->GetPointer(&pBufferOut);
        if(FAILED(hr))
        {
          DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveFirstSample -  pOutSample->GetPointer returned empty frame")));
	  pOutSample->Release();
	  return hr;
        }
        if(m_PicScaler1)
            m_PicScaler1->Scale(pBufferOut, pBufferIn1, m_Vflip);
        else
        {
          if(m_Vflip && m_nOutputSize>0)
            VFlipImg1(pBufferIn1,pBufferOut);
          else
            memcpy(pBufferOut, pBufferIn1, RealOutputSize);
        }
      }
      pOutSample->SetSyncPoint(TRUE);
      hr = m_vOutputPins[0]->GetInputPin()->Receive(pOutSample);
      pOutSample->Release();
    }

    if(m_MaxInterval > 0)
    {
      REFERENCE_TIME TimeStart1, TimeEnd1;
      if(SUCCEEDED(pSample->GetTime(&TimeStart1,&TimeEnd1)))
      {
        m_duration += TimeEnd1 - TimeStart1;
        if(m_duration > m_MaxInterval)
        {
          m_bToggle=!m_bToggle; m_duration=0;
          FQ2.Erase();				// unblock queue 2
        }
      }
    }
  }
  else
  {
    if(m_bThrottling) FQ1.AddFrame(pSample);
  }
  return hr;
}


/// The second video only asynchronously copies frame to a buffer.
HRESULT TogglerFilter::ReceiveSecondSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  
  if(!m_vOutputPins[0]->IsConnected())
  {
    FQ1.Erase(); FQ2.Erase();
    return S_FALSE;	// Do nothing when no output is connected.
  }

  if(m_bThrottling)
  {
    pSample->GetTime(&TimeStart2,&TimeEnd2);
    if(TimeStart2 > TimeEnd1)
    {
      IMediaSample *pSample1 = FQ1.PeekFrame(TimeStart2,TimeEnd2);
      if(pSample1)
      {
        pSample1->GetTime(&TimeStart1,&TimeEnd1);
        pSample1->Release();
      }
    }
  }
  else
  {
    FQ1.Erase(); FQ2.Erase();
  }

	// Copy the secondary sample into our buffer  
  HRESULT hr = S_OK;
  if(m_bToggle || !m_vInputPins[0]->IsConnected())
  {
    if(m_PicScaler2==NULL && !m_Vflip && !m_DuplicateFrame)
    {
      hr = m_vOutputPins[0]->GetInputPin()->Receive(pSample);		// Populate intact sample to the output.
    }
    else
    {
      FQ2.Erase();
      BYTE *pBufferIn2 = NULL;
      hr = pSample->GetPointer(&pBufferIn2);
      if(FAILED(hr))
      {
        DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveSecondSample - cannot get pointer to pSample2 data, error %Xh"), hr));
        return hr;
      }
      if(pBufferIn2==NULL)
      {
        DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveSecondSample - NULL pointer in 2nd sample input data.")));
        return E_POINTER;
      }
      const long SampleSize1 = pSample->GetSize();
      const long RealOutputSize = (m_nOutputSize==0) ? SampleSize1 : m_nOutputSize;

      IMediaSample *pOutSample = NULL;
      {
        CAutoLock lck(&m_csReceive);
        HRESULT hr = CMultiIOBaseFilter::InitializeOutputSample(pSample, &pOutSample, 1, 0);	// Set up the output sample
        if(FAILED(hr))
        {
           DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveSecondSample - CMultiIOBaseFilter::InitializeOutputSample(1) failed hr=%Xh"),hr));
           return hr;
        }
        hr = pOutSample->SetActualDataLength(RealOutputSize);
        if(FAILED(hr))
        {
          DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveSecondSample - pOutSample->SetActualDataLength(%d) failed hr=%Xh"),RealOutputSize,hr));
          pOutSample->Release();
          return hr;
        }
        BYTE *pBufferOut = NULL;
        hr = pOutSample->GetPointer(&pBufferOut);
        if(FAILED(hr))
	{
          DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::ReceiveSecondSample - CMultiIOBaseFilter::InitializeOutputSample(1) returned empty frame")));
	  pOutSample->Release();
	  return hr;
        }
        if(m_PicScaler2)
            m_PicScaler2->Scale(pBufferOut, pBufferIn2, m_Vflip);
        else
        {
          if(m_Vflip && m_nOutputSize>0)
            VFlipImg2(pBufferIn2,pBufferOut);
          else
            memcpy(pBufferOut,pBufferIn2,RealOutputSize);
        }
      }
      pOutSample->SetSyncPoint(TRUE);
      hr = m_vOutputPins[0]->GetInputPin()->Receive(pOutSample);
      pOutSample->Release();
    }

    if(m_MaxInterval > 0)
    {
      if(SUCCEEDED(pSample->GetTime(&TimeStart2,&TimeEnd2)))
      {
        m_duration += TimeEnd2 - TimeStart2;
        if(m_duration > m_MaxInterval)
        {
          m_bToggle=!m_bToggle;m_duration=0;
          FQ1.Erase();				// unblock queue 1
        }
      }
    }
  }
  else
  {
    if(m_bThrottling) FQ2.AddFrame(pSample);
  }
  return hr;
}


HRESULT TogglerFilter::StartStreaming(void)
{
  m_duration = 0;
  FQ1.Erase(); FQ2.Erase();
  TimeStart2 = TimeEnd2 = TimeStart1 = TimeEnd1 = 0;
  return VideoMixingBase::StartStreaming();
}


HRESULT TogglerFilter::StopStreaming(void)
{
  FQ1.Erase(); FQ2.Erase();
  return VideoMixingBase::StopStreaming();
}


HRESULT TogglerFilter::CreateVideoMixer(const CMediaType *pMediaType, int nIndex)
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
  if(nIndex>=2) return E_INVALIDARG;
  if(bmih!=NULL)
  {
    m_nSampleSizes[nIndex] = DIBSIZE(*bmih);

    if(m_PicScaler1)
    {
      if(!m_PicScaler1->isVideoFormatSupported(pMediaType->subtype))
      {
        delete m_PicScaler1;
        m_PicScaler1 = NULL;
      }
    }
    if(m_PicScaler2)
    {
      if(!m_PicScaler2->isVideoFormatSupported(pMediaType->subtype))
      {
        delete m_PicScaler2;
        m_PicScaler2 = NULL;
      }
    }

	// Create appropriate picture concatenator
    if(pMediaType->subtype==MEDIASUBTYPE_YUV420P_S || pMediaType->subtype==MEDIASUBTYPE_I420 || pMediaType->subtype==MEDIASUBTYPE_YV12 || pMediaType->subtype==MEDIASUBTYPE_IYUV)
    {
      m_nBitsPerPixel = BITS_PER_PIXEL_YUV420P;
      return S_OK;
    }
    if(pMediaType->subtype==MEDIASUBTYPE_YV24)
    {    
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;    
      return S_OK;
    }
    if(pMediaType->subtype==MEDIASUBTYPE_RGB24)
    {    
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
      return S_OK;
    }
    if(pMediaType->subtype==MEDIASUBTYPE_RGB565 || pMediaType->subtype==MEDIASUBTYPE_RGB555 ||
       pMediaType->subtype==MEDIASUBTYPE_YUYV || pMediaType->subtype==MEDIASUBTYPE_YUY2 || pMediaType->subtype==MEDIASUBTYPE_YVYU ||
       pMediaType->subtype==MEDIASUBTYPE_UYVY || pMediaType->subtype==MEDIASUBTYPE_HDYC)
    {    
      m_nBitsPerPixel = 16;
      return S_OK;
    }
    if(pMediaType->subtype==MEDIASUBTYPE_ARGB32 || pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_AYUV)
    {
      m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;    
      return S_OK;
    }

    m_nBitsPerPixel = 0;		// Unknown image video format.
    return S_OK;
  }

  DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::CreateVideoMixer - No acceptable type.")));
  return VFW_E_TYPE_NOT_ACCEPTED;
}


HRESULT TogglerFilter::SetOutputDimensions(const BITMAPINFOHEADER* pBmih1, const BITMAPINFOHEADER* pBmih2, int& nOutputWidth, int& nOutputHeight, int& nOutputSize)
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
    if(pBmih1==NULL || pBmih1->biWidth==0 || pBmih1->biHeight==0)
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

  return S_OK;
}


HRESULT TogglerFilter::SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt, int nIndex)
{
HRESULT hr = VideoMixingBase::SetMediaType(direction,pmt,nIndex);
  if(SUCCEEDED(hr))
      reconfigure();
  return hr; 
}


HRESULT TogglerFilter::CheckOutputType(const CMediaType *pMediaType)
{
	//Make sure the input and output types are related
  if(pMediaType->majortype != MEDIATYPE_Video)
  {
    DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::CheckOutputType - Another type than MEDIATYPE_Video is not supported.")));
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
      {
        if(pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_ARGB32 || pMediaType->subtype==MEDIASUBTYPE_RGB24)
            m_Vflip = true;
      }
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(pBiOut->biWidth != m_nOutputWidth)
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
    return S_OK;
  }
  else if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2 * const pVihOut = (const VIDEOINFOHEADER2*)pMediaType->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(pBiOut->biHeight != m_nTargetHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutputHeight))
      {
        if(pMediaType->subtype==MEDIASUBTYPE_RGB32 || pMediaType->subtype==MEDIASUBTYPE_ARGB32 || pMediaType->subtype==MEDIASUBTYPE_RGB24)
            m_Vflip = true;
      }
      else
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted height at this stage.
    }
    if(pBiOut->biWidth != m_nOutputWidth)
        return VFW_E_TYPE_NOT_ACCEPTED;		// Unaccepted width at this stage.
    return S_OK;
  }
  else
  {
    DbgLog((LOG_ERROR, 0, TEXT("TogglerFilter::CheckOutputType - Another type than MEDIATYPE_Video is not supported.")));
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  return S_FALSE;
}


/// Override to control state dependent behaviour
STDMETHODIMP TogglerFilter::SetParameter(const char* type, const char* value)
{
HRESULT hr;
	// For now, one cannot set any parameters once the output has been connected -> this will affect the buffersize
  if(m_vOutputPins[0] && m_vOutputPins[0]->IsConnected())
  {
    if(parameterChangeAffectsOutput(type))
    {
      bool Ok;
      std::string Str(CSettingsInterface::GetParameter(type,&Ok));
      if(!Ok) return E_FAIL;
      if(Str != value) return E_FAIL;
      return S_FALSE;			// Nothing to change, parameter is identical.
    }
  }

  hr = CSettingsInterface::SetParameter(type,value);
  if(SUCCEEDED(hr)) reconfigure();			///!!!!!!!!!!!!Pozor na bezici video!!!!!!!!!!!!
  return hr;
}


bool TogglerFilter::parameterChangeAffectsOutput( const char* szParam )
{
  if((strcmp(szParam, FILTER_PARAM_TARGET_WIDTH) == 0) ||
     (strcmp(szParam, FILTER_PARAM_TARGET_HEIGHT) == 0))
  {
    return true;
  }
  return false;
}


void TogglerFilter::reconfigure(void)
{
  const BITMAPINFOHEADER* pBmih1 = &m_VideoInHeader[0].bmiHeader;
  const BITMAPINFOHEADER* pBmih2 = &m_VideoInHeader[1].bmiHeader;
								///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  GUID VideoFormat;
  if(m_vInputPins[0]->IsConnected()) VideoFormat=ConnectionFormat[0];
  else if(m_vInputPins[1]->IsConnected()) VideoFormat=ConnectionFormat[1];
  else memset(&VideoFormat,0,sizeof(GUID));

	// target picture scaler
  if(m_vInputPins[0]->IsConnected() && (labs(pBmih1->biWidth)!=m_nOutputWidth || labs(pBmih1->biHeight)!=m_nOutputHeight))
  {
    if(m_PicScaler1)
    {
      if(!m_PicScaler1->isVideoFormatSupported(VideoFormat))
      {
        PicScalerBase *PSB = m_PicScaler1;
        m_PicScaler1 = NULL;
	delete PSB;
      }
    }    
    if(!m_PicScaler1)
    {
      switch(m_nBitsPerPixel)
      {
        case 16:                   m_PicScaler1 = new PicScalerRGB565Impl();
				     break;
        case BITS_PER_PIXEL_RGB24:if(VideoFormat==MEDIASUBTYPE_YV24)
				     m_PicScaler1 = new PicScalerYV24Impl();
				   else
				     m_PicScaler1 = new PicScalerRGB24Impl();
				   break;
        case BITS_PER_PIXEL_RGB32:m_PicScaler1 = GetPicScallerRGB32();
				   break;
        default: return;
      }
    }
    m_PicScaler1->SetInDimensions(pBmih1->biWidth, pBmih1->biHeight);
    m_PicScaler1->SetOutDimensions(m_nOutputWidth, m_nOutputHeight);
  }
  else
  {
    if(m_PicScaler1)
    {
      PicScalerBase *PSB = m_PicScaler1;
      m_PicScaler1 = NULL;
      delete PSB;  
    }
  }

  if(m_vInputPins[1]->IsConnected() && (labs(pBmih2->biWidth)!=m_nOutputWidth && labs(pBmih2->biHeight)!=m_nOutputHeight))
  {
    if(m_PicScaler2)
    {
      if(!m_PicScaler2->isVideoFormatSupported(VideoFormat))
      {
        PicScalerBase *PSB = m_PicScaler2;
        m_PicScaler2 = NULL;
	delete PSB;
      }
    }
    if(!m_PicScaler2)
    {
      switch(m_nBitsPerPixel)
      {
        case 16:                   m_PicScaler2 = new PicScalerRGB565Impl();
				   break;
        case BITS_PER_PIXEL_RGB24: if(VideoFormat==MEDIASUBTYPE_YV24)
				   m_PicScaler2 = new PicScalerYV24Impl();
				   else
				   m_PicScaler2 = new PicScalerRGB24Impl();
				   break;
        case BITS_PER_PIXEL_RGB32: m_PicScaler2 = GetPicScallerRGB32();
				   break;
        default: return;
      }
    }
    m_PicScaler2->SetInDimensions(pBmih2->biWidth, pBmih2->biHeight);
    m_PicScaler2->SetOutDimensions(m_nOutputWidth, m_nOutputHeight);
  }
  else
  {
    if(m_PicScaler2)
    {
      PicScalerBase *PSB = m_PicScaler2;
      m_PicScaler2 = NULL;
      delete PSB;
    }
  }

}


/// For configuration dialog
HRESULT TogglerFilter::GetPages(CAUUID *pPages)
{
  if(pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if(pPages->pElems == NULL) 
  {
    pPages->cElems = 0;
    return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_TogglerProperties;
  return S_OK;
}


/// For configuration dialog
HRESULT TogglerFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
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


HRESULT TogglerFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_TogglerFilter;
  return S_OK;
}
