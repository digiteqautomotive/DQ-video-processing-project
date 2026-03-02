/** @file

MODULE				: TogglerFilter

FILE NAME			: TogglerFilter.cpp

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive; designed by Jaroslav Fojtik		All rights reserved.

===========================================================================*/
#include "stdafx.h"
#include "TogglerFilter.h"
#include <DirectShow/CommonDefs.h>

#include "PicInPicRGB24Impl.h"
#include "PicInPicRGB32Impl.h"
#include "PicInPicYV24Impl.h"
#include "PicInPicRGB565Impl.h"
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


///////////////////////////////////////////////////////////


FrameQueue::~FrameQueue()
{
  while(!q.empty())
  {
    IMediaSample *pFrame = q.front();
    q.pop_front();
    if(pFrame) pFrame->Release();
  }
}


void FrameQueue::Erase(void)
{
IMediaSample *pFrame;
  while(!q.empty())
  {
    {		// locked scope
      CAutoLock lck(&m_csFrame2Queue);
      pFrame = q.front();
      q.pop_front();
    }
    if(pFrame) pFrame->Release();
  }
}

void FrameQueue::AddFrame(IMediaSample*pFrame)
{
  if(pFrame)
  {
    pFrame->AddRef();
    CAutoLock lck(&m_csFrame2Queue);
    q.push_back(pFrame);
  }
}


IMediaSample *FrameQueue::PeekFrame(void) const
{
CAutoLock lck(&m_csFrame2Queue);
  if(q.empty()) return NULL;
  IMediaSample *IS = q.front();
  if(IS) IS->AddRef();
return IS;
}


IMediaSample *FrameQueue::PeekFrame(REFERENCE_TIME &tStart, REFERENCE_TIME &tStop)
{
//LONGLONG tStart2, tStop2;
REFERENCE_TIME TimeStart2, TimeEnd2;
IMediaSample *IS = NULL;
CAutoLock lck(&m_csFrame2Queue);

  while(!q.empty())
  {
    std::list<IMediaSample*>::iterator it = q.begin();
    IS = *it;
    if(IS!=NULL)
    {
      it++;
      if(it==q.end()) 
      {
        q.pop_front();
        return(IS);		// IS is the last element in queue.
      }
      //IS->GetMediaTime(&tStart2,&tStop2);
      IS->GetTime(&TimeStart2,&TimeEnd2);
      if(TimeStart2>=tStart || TimeEnd2>=tStart)
      {
        q.pop_front();
        return(IS);
      }
    }
    q.pop_front();
  }

  if(IS) IS->AddRef();
return IS;
}


void FrameQueue::PopFrame(void)
{
  IMediaSample *pFrame;
  {
    CAutoLock lck(&m_csFrame2Queue);
    if(q.empty()) return;
    pFrame = q.front();
    q.pop_front();
  }
  if(pFrame) pFrame->Release();		// It is better to unlock queue here.
}


//////////////////////////////////////////////////////////


DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);

const OLECHAR OLE_PARAM_TARGET_WIDTH[] =  olestr(FILTER_PARAM_TARGET_WIDTH);  // L"TEST";  //  sstr(FILTER_PARAM_TARGET_WIDTH);
const OLECHAR OLE_PARAM_TARGET_HEIGHT[] = olestr(FILTER_PARAM_TARGET_HEIGHT);
const OLECHAR OLE_PARAM_SUB_PIC_WIDTH[] = olestr(FILTER_PARAM_SUB_PIC_WIDTH);
const OLECHAR OLE_PARAM_SUB_PIC_HEIGHT[] = olestr(FILTER_PARAM_SUB_PIC_HEIGHT);
const OLECHAR OLE_PARAM_OFFSET_X[] = olestr(FILTER_PARAM_OFFSET_X);
const OLECHAR OLE_PARAM_OFFSET_Y[] = olestr(FILTER_PARAM_OFFSET_Y);
const OLECHAR OLE_PARAM_THROTTLE[] = olestr(FILTER_THROTTLE);
const OLECHAR OLE_PARAM_DUPLICATE_FRAMES[] = olestr(FILTER_DUPLICATE_FRAMES);


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


TogglerFilter::TogglerFilter(): VideoMixingBase(NAME("DQ VPP Picture in Picture Filter"), 0, CLSID_VPP_TogglerFilter),
  //m_nTargetWidth(0),		// Initialised in initParameters(), no need to initialise twice.
  //m_nTargetHeight(0),
  //m_nSubPictureWidth(0),
  //m_nSubPictureHeight(0),
  //m_nCustomOffsetX(0),
  //m_nCustomOffsetY(0),
  //m_bThrottling(false),
  //m_DuplicateFrame(false),
  m_pPicInPic(NULL),
  m_pTargetPicScaler(NULL),
  m_pSubPicScaler(NULL),
  m_nBitsPerPixel(0),
  m_pBufferForScaledSecondaryImage(NULL),
  m_Vflip(false)
{
  m_nSampleSizes[0] = 0;
  m_nSampleSizes[1] = 0;

	// Init parameters
  initParameters();
}


void TogglerFilter::initParameters(void)
{
  addParameter(FILTER_PARAM_TARGET_WIDTH, OLE_PARAM_TARGET_WIDTH, &m_nTargetWidth, 0);		// width output picture is scaled to
  addParameter(FILTER_PARAM_TARGET_HEIGHT, OLE_PARAM_TARGET_HEIGHT, &m_nTargetHeight, 0);	// height output picture is scaled to

  addParameter(FILTER_PARAM_SUB_PIC_WIDTH, OLE_PARAM_SUB_PIC_WIDTH, &m_nSubPictureWidth, 0);	// width inner picture is scaled to
  addParameter(FILTER_PARAM_SUB_PIC_HEIGHT, OLE_PARAM_SUB_PIC_HEIGHT, &m_nSubPictureHeight, 0);	// height inner picture is scaled to

  // currently unused: supposed to be one of the standard positions to insert 
  // the picture i.e. top left or top right corner.
  // addParameter(SUB_PICTURE_POSITION, &m_nPosition, (int)SUB_PIC_POSITION_1);

  addParameter(FILTER_PARAM_OFFSET_X, OLE_PARAM_OFFSET_X, &m_nCustomOffsetX, 0);		// x offset from bottom left corner
  addParameter(FILTER_PARAM_OFFSET_Y, OLE_PARAM_OFFSET_Y, &m_nCustomOffsetY, 0);		// y offset from bottom left corner

  addParameter(FILTER_THROTTLE, OLE_PARAM_THROTTLE, &m_bThrottling, false);
  addParameter(FILTER_DUPLICATE_FRAMES, OLE_PARAM_DUPLICATE_FRAMES, &m_DuplicateFrame, false);
}


TogglerFilter::~TogglerFilter()
{
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
}


CUnknown * WINAPI TogglerFilter::CreateInstance( LPUNKNOWN pUnk, HRESULT *pHr)
{
  TogglerFilter* pFilter = new TogglerFilter();
  if(pHr) *pHr = (pFilter==NULL) ? E_OUTOFMEMORY : S_OK;
  return pFilter;
}


void TogglerFilter::VFlipImg1(const BYTE* pSrc, BYTE* pDest)
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


HRESULT TogglerFilter::ReceiveFirstSample(IMediaSample *pSample)
{

  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()

  if(!m_vOutputPins[0]->IsConnected())
  {
    FQ.Erase();
    return S_FALSE;	// Do nothing when no output is connected.
  }

  if(m_bThrottling)
  {
    REFERENCE_TIME TimeStart1, TimeEnd1;
    pSample->GetTime(&TimeStart1,&TimeEnd1);
    if(TimeStart1 > TimeEnd2)
    {
      IMediaSample *pSample2 = FQ.PeekFrame(TimeStart1,TimeEnd1);
      if(pSample2)
      {
        pSample2->GetTime(&TimeStart2,&TimeEnd2);
        BYTE *pBuffer = NULL;
        if(SUCCEEDED(pSample2->GetPointer(&pBuffer)))
        {        
          if(pBuffer!=NULL && m_pBufferForScaledSecondaryImage!=NULL)
          {
            if(m_pSubPicScaler)
            {
              m_pSubPicScaler->Scale(m_pBufferForScaledSecondaryImage, pBuffer);
            }
            else		// copy direct from buffer
            {
              memcpy(m_pBufferForScaledSecondaryImage, pBuffer, m_nSampleSizes[1]);	// Copy the buffer
            }
          }
        }
        pSample2->Release();
      }
    }
  }

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

  if(m_pTargetPicScaler==NULL && !m_Vflip && !m_DuplicateFrame)
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
}


/// The second video only asynchronously copies frame to a buffer.
HRESULT TogglerFilter::ReceiveSecondSample(IMediaSample *pSample)
{
  ASSERT(pSample!=NULL);				// Already tested in VideoMixingBase::Receive()
  if(!m_vOutputPins[0]->IsConnected()) return S_FALSE;	// Do nothing when no output is connected.
 
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

  if(m_bThrottling)
  {
    FQ.AddFrame(pSample);
  }
  else
  {  
    if(m_pBufferForScaledSecondaryImage==NULL) return E_UNEXPECTED;	// This should not occur, buffer should be allocated when 2nd video is attached.
    if(m_pSubPicScaler)
    {
      m_pSubPicScaler->Scale(m_pBufferForScaledSecondaryImage, pBuffer);
    }
    else		// copy direct from buffer
    {
      memcpy(m_pBufferForScaledSecondaryImage, pBuffer, m_nSampleSizes[1]);	// Copy the buffer
    }    
  }
  return S_OK;
}


HRESULT TogglerFilter::StartStreaming(void)
{
  if(m_pBufferForScaledSecondaryImage!=NULL && m_szBufferForScaledSecondaryImage>0)
      memset(m_pBufferForScaledSecondaryImage, 0, m_szBufferForScaledSecondaryImage);
  FQ.Erase();
  TimeStart2 = TimeEnd2 = 0;
  return VideoMixingBase::StartStreaming();
}


HRESULT TogglerFilter::StopStreaming(void)
{
  FQ.Erase();
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
  if(pMediaType->subtype==MEDIASUBTYPE_RGB565)
  {    
    if(m_pPicInPic==NULL)
    {
      m_pPicInPic = new PicInPicRGB565Impl();
      m_nBitsPerPixel = 16;
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


HRESULT TogglerFilter::CheckOutputType(const CMediaType *pMediaType)
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

  switch(m_nBitsPerPixel)
  {
    case 16:
          if(*(pMediaType->Subtype()) == MEDIASUBTYPE_RGB565)
          {
            if((ConnectionFormat[0]==MEDIASUBTYPE_RGB565 && m_vInputPins[0]->IsConnected()) ||
               (ConnectionFormat[1]==MEDIASUBTYPE_RGB565 && m_vInputPins[1]->IsConnected()))
                return S_OK;
          }
          break;

    case BITS_PER_PIXEL_RGB24:
	  if(*(pMediaType->Subtype()) == MEDIASUBTYPE_YV24)
	  {
	    if((ConnectionFormat[0]==MEDIASUBTYPE_YV24 && m_vInputPins[0]->IsConnected()) ||
               (ConnectionFormat[1]==MEDIASUBTYPE_YV24 && m_vInputPins[1]->IsConnected()))
                return S_OK;
	  }
          else if(pMediaType->subtype == MEDIASUBTYPE_RGB24)
          {
            if((ConnectionFormat[0]==MEDIASUBTYPE_RGB24 && m_vInputPins[0]->IsConnected()) ||
               (ConnectionFormat[1]==MEDIASUBTYPE_RGB24 && m_vInputPins[1]->IsConnected()))
                return S_OK;
          }
          break;

    case BITS_PER_PIXEL_RGB32:
         if(*(pMediaType->Subtype())==MEDIASUBTYPE_AYUV)
         {
           if((ConnectionFormat[0]==MEDIASUBTYPE_AYUV && m_vInputPins[0]->IsConnected()) ||
              (ConnectionFormat[1]==MEDIASUBTYPE_AYUV && m_vInputPins[1]->IsConnected()))
               return S_OK;
         }
         else if(*(pMediaType->Subtype())==MEDIASUBTYPE_RGB32 || *(pMediaType->Subtype())==MEDIASUBTYPE_ARGB32)
         {
           if(((ConnectionFormat[0]==MEDIASUBTYPE_ARGB32 || ConnectionFormat[0]==MEDIASUBTYPE_RGB32) && m_vInputPins[0]->IsConnected()) ||
              ((ConnectionFormat[1]==MEDIASUBTYPE_ARGB32 || ConnectionFormat[1]==MEDIASUBTYPE_RGB32) && m_vInputPins[1]->IsConnected()))
               return S_OK;
         }
         break;
  }
  return S_FALSE;
}


/// Override to control state dependent behaviour
STDMETHODIMP TogglerFilter::SetParameter( const char* type, const char* value )
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
  if(m_pBufferForScaledSecondaryImage != NULL)			// tohle je uplnì blbe, nesmi se to spustit na bezicim videu od SetParameter()
  {
    void *Buf2Delete = m_pBufferForScaledSecondaryImage;
    m_pBufferForScaledSecondaryImage = NULL;
    free(Buf2Delete);    
  }

  GUID VideoFormat;
  if(m_vInputPins[0]->IsConnected()) VideoFormat=ConnectionFormat[0];
  else if(m_vInputPins[1]->IsConnected()) VideoFormat=ConnectionFormat[1];
  else memset(&VideoFormat,0,sizeof(GUID));

	// target picture scaler
  if(m_nTargetWidth > 0 && m_nTargetHeight > 0)
  {
    if(m_pTargetPicScaler)
    {
      if(!m_pTargetPicScaler->isVideoFormatSupported(VideoFormat))
      {
	delete m_pTargetPicScaler;
	m_pTargetPicScaler = NULL;
      }
    }
    if(!m_pTargetPicScaler)
    {
      switch(m_nBitsPerPixel)
      {
        case 16:                   m_pTargetPicScaler = new PicScalerRGB565Impl();
				   break;
        case BITS_PER_PIXEL_RGB24: if(VideoFormat==MEDIASUBTYPE_YV24)
				     m_pTargetPicScaler = new PicScalerYV24Impl();
				   else
				     m_pTargetPicScaler = new PicScalerRGB24Impl();
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
    if(m_pTargetPicScaler)
    {
      PicScalerBase *PSB = m_pTargetPicScaler;
      m_pTargetPicScaler = NULL;
      delete PSB;
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
        if(!m_pSubPicScaler->isVideoFormatSupported(VideoFormat))
        {
          PicScalerBase *PSB = m_pSubPicScaler;
          m_pSubPicScaler = NULL;
	  delete PSB;
        }
      }
      if(!m_pSubPicScaler)
      {
        switch(m_nBitsPerPixel)
        {
          case 16:		     m_pSubPicScaler = new PicScalerRGB565Impl();
				     break;	
          case BITS_PER_PIXEL_RGB24: if(VideoFormat==MEDIASUBTYPE_YV24)
                                       m_pSubPicScaler = new PicScalerYV24Impl();
                                     else
                                       m_pSubPicScaler = new PicScalerRGB24Impl();
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
