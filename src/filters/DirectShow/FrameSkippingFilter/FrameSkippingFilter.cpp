/** @file

MODULE                : FrameSkippingFilter

FILE NAME			        : FrameSkippingFilter.cpp

DESCRIPTION           : This filter skips a specified number of frames depending on the parameter denoted by "skipFrame"

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2014, CSIR
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
#include "FrameSkippingFilter.h"
#include <set>
#include <dvdmedia.h>
#include <artist/Media/MediaUtil.h>
#include <Util/StringUtil.h>

// timestamp unit is in 10^-7
const double TIMESTAMP_FACTOR = 10000000.0;

FrameSkippingFilter::FrameSkippingFilter(LPUNKNOWN pUnk, HRESULT *pHr)
  : CTransInPlaceFilter(NAME("CSIR VPP Frame Skipping Filter"), pUnk, CLSID_VPP_FrameSkippingFilter, pHr, false),
  m_uiSkipFrameNumber(0),
  m_uiTotalFrames(1),
  m_uiCurrentFrame(0),
  m_dSkipRatio(1.0),
  m_dTargetFrameRate(0.0),
  m_bIsTimeSet(false),
  m_dTimeFrame(0.0),
  m_dTimeCurrent(0.0),
  m_dTargetTimeFrame(0.0),
  m_tStart(0),
  m_tStop(0)
{
  // Init parameters
  initParameters();
}

FrameSkippingFilter::~FrameSkippingFilter()
{

}

CUnknown * WINAPI FrameSkippingFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  FrameSkippingFilter *pFilter = new FrameSkippingFilter(pUnk, pHr);
  if (pFilter == NULL)
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pFilter;
}

STDMETHODIMP FrameSkippingFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if (riid == (IID_ISettingsInterface))
  {
    return GetInterface((ISettingsInterface*) this, ppv);
  }
  if (riid == (IID_ISpecifyPropertyPages))
  {
    return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  }
  else
  {
    return CTransInPlaceFilter::NonDelegatingQueryInterface(riid, ppv);
  }
}

HRESULT FrameSkippingFilter::Transform(IMediaSample *pSample)
{
  /*  Check for other streams and pass them on */
  // don't skip control info
  AM_SAMPLE2_PROPERTIES * const pProps = m_pInput->SampleProps();
  if (pProps->dwStreamId != AM_STREAM_MEDIA) {
    return S_OK;
  }
  // select mode
  if (m_uiFrameSkippingMode == FSKIP_ACHIEVE_TARGET_RATE)
  {
    if (m_dTargetFrameRate != 0.0)
    {
      assert(m_dTargetFrameRate > 0.0);
      //set initial time frame
      m_dTargetTimeFrame = (1 / m_dTargetFrameRate); // timestamp unit is in 10^-7
      HRESULT hr = pSample->GetTime(&m_tStart, &m_tStop);
      if (SUCCEEDED(hr))
      {
        //m_bIsTimeSet runs only once with each rendering to initialize the 1st targetTimeFrame
        if (!m_bIsTimeSet)
        {
          m_dTimeCurrent = m_tStart / TIMESTAMP_FACTOR;
          m_dTimeFrame = m_dTimeCurrent + m_dTargetTimeFrame;
          m_bIsTimeSet = true;
          return S_OK;
        }
        else
        {
          m_dTimeCurrent = m_tStart / TIMESTAMP_FACTOR;
          if (m_dTimeCurrent > m_dTimeFrame)
          {
            int multiplier = static_cast<int>(ceil((m_dTimeCurrent - m_dTimeFrame) / m_dTargetTimeFrame));
            if (multiplier == 0) { multiplier = 1; }
            m_dTimeFrame += (m_dTargetTimeFrame *multiplier);
            return S_OK;
          }
          else
          {
            return S_FALSE;
          }
        }

      }
      else
      {
        return hr;
      }
    }
    else
    {
      return S_OK;
    }
  }

  else if (m_uiFrameSkippingMode == FSKIP_SKIP_X_FRAMES_EVERY_Y)
  {
    if (m_vFramesToBeSkipped.empty())
      return S_OK;

    int iSkip = m_vFramesToBeSkipped[m_uiCurrentFrame++];
    if (m_uiCurrentFrame >= m_vFramesToBeSkipped.size())
    {
      m_uiCurrentFrame = 0;
    }

    if (iSkip == 1)
    {
      return S_FALSE;
    }
    return S_OK;
  }
  else
  {
    assert(false);
    return S_OK;
  }
  
#if 0
  // adjust frame duration here?

  BYTE *pBufferIn, *pBufferOut;
  HRESULT hr = pSample->GetPointer(&pBufferIn);
  if (FAILED(hr))
  {
    return hr;
  }

  VIDEOINFOHEADER *pVih1 = (VIDEOINFOHEADER*)mtIn->pbFormat;

  m_dSkipRatio
#endif
}

DEFINE_GUID(MEDIASUBTYPE_I420, 0x30323449, 0x0000, 0x0010, 0x80, 0x00,
  0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71);

HRESULT FrameSkippingFilter::CheckInputType(const CMediaType* mtIn)
{
  // Check the major type.
  if (mtIn->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if (
    (mtIn->subtype != MEDIASUBTYPE_RGB24) && (mtIn->subtype != MEDIASUBTYPE_RGB32) && (mtIn->subtype != MEDIASUBTYPE_I420)
    )
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if (mtIn->formattype != FORMAT_VideoInfo)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }
  return S_OK;
}

HRESULT FrameSkippingFilter::Run(REFERENCE_TIME tStart)
{
  if (m_uiFrameSkippingMode == FSKIP_SKIP_X_FRAMES_EVERY_Y)
  {
    m_vFramesToBeSkipped.clear();

    int iSkip = 0, iTotal = 0;
    bool res = artist::calcFrameSkippingParameters(m_dSourceFrameRate, m_dTargetFrameRate, iSkip, iTotal);
    if (res)
    {
      m_uiSkipFrameNumber = iSkip;
      m_uiTotalFrames = iTotal;
      // calculate which frames should be dropped
      if (m_uiSkipFrameNumber < m_uiTotalFrames && m_uiSkipFrameNumber > 0)
      {
        double dRatio = m_uiTotalFrames / static_cast<double>(m_uiSkipFrameNumber);
        std::set<int> toBeSkipped;
        std::set<int> toBePlayed;
        // populate to be skipped: note that this index is 1-indexed
        for (size_t iCount = 1; iCount <= m_uiSkipFrameNumber; ++iCount)
        {
#if _MSC_VER > 1600
          int iToBeSkipped = static_cast<int>(round(iCount * dRatio));
#else
          int iToBeSkipped = static_cast<int>(floor(iCount * dRatio + 0.5));
#endif
          toBeSkipped.insert(iToBeSkipped);
        }

        // populate to be played
        for (size_t iCount = 1; iCount <= m_uiTotalFrames; ++iCount)
        {
          auto found = toBeSkipped.find(iCount);
          if (found == toBeSkipped.end())
          {
            toBePlayed.insert(iCount);
            m_vFramesToBeSkipped.push_back(0);
          }
          else
          {
            m_vFramesToBeSkipped.push_back(1);
          }
        }
      }
      else
      {
        // invalid input
        m_vFramesToBeSkipped.clear();
      }
    }
  }
  return CTransInPlaceFilter::Run(tStart);
}

HRESULT FrameSkippingFilter::Stop(void)
{
  m_uiCurrentFrame = 0;
  m_vFramesToBeSkipped.clear();
  m_bIsTimeSet = false;
  return CTransInPlaceFilter::Stop();
}

CBasePin* FrameSkippingFilter::GetPin(int n)
{
  HRESULT hr = S_OK;

  // Create an input pin if not already done

  if (m_pInput == NULL) {

    m_pInput = new CTransInPlaceInputPin(NAME("TransInPlace input pin")
      , this        // Owner filter
      , &hr         // Result code
      , L"Input"    // Pin name
      );

    // Constructor for CTransInPlaceInputPin can't fail
    ASSERT(SUCCEEDED(hr));
  }

  // Create an output pin if not already done

  if (m_pInput != NULL && m_pOutput == NULL) {

    m_pOutput = new FrameSkippingOutputPin(NAME("Frame skipping output pin")
      , this       // Owner filter
      , &hr        // Result code
      , L"Output"  // Pin name
      );

    // a failed return code should delete the object

    ASSERT(SUCCEEDED(hr));
    if (m_pOutput == NULL) {
      delete m_pInput;
      m_pInput = NULL;
    }
  }

  // Return the appropriate pin

  ASSERT(n >= 0 && n <= 1);
  if (n == 0) {
    return m_pInput;
  }
  else if (n == 1) {
    return m_pOutput;
  }
  else {
    return NULL;
  }
} // GetPin

STDMETHODIMP FrameSkippingFilter::SetParameter(const char* type, const char* value)
{
  HRESULT hr = CSettingsInterface::SetParameter(type, value);
  if (SUCCEEDED(hr))
  { 
    if (m_uiTotalFrames > 0)
      m_dSkipRatio = m_uiSkipFrameNumber / static_cast<double>(m_uiTotalFrames);
    else
      m_dSkipRatio = 0.0;
  }
  return hr;
}

FrameSkippingOutputPin::FrameSkippingOutputPin
(__in_opt LPCTSTR             pObjectName
, __inout CTransInPlaceFilter *pFilter
, __inout HRESULT             *phr
, __in_opt LPCWSTR             pName
)
: CTransInPlaceOutputPin(pObjectName, pFilter, phr, pName)
{

}

// EnumMediaTypes
// - pass through to our downstream filter
STDMETHODIMP FrameSkippingOutputPin::EnumMediaTypes(__deref_out IEnumMediaTypes **ppEnum)
{
  HRESULT hr = CTransInPlaceOutputPin::EnumMediaTypes(ppEnum);
 
  if (SUCCEEDED(hr))
  {
    // modify frame duration
    AM_MEDIA_TYPE *pmt = NULL;
    while (hr = (*ppEnum)->Next(1, &pmt, NULL), hr == S_OK)
    {
      adjustAverageTimePerFrameInVideoInfoHeader(pmt);
    }
  }
  return hr;
} // EnumMediaTypes

HRESULT
FrameSkippingOutputPin::SetMediaType(const CMediaType* pmtOut)
{
  const AM_MEDIA_TYPE* pMediaType = pmtOut;
  adjustAverageTimePerFrameInVideoInfoHeader(const_cast<AM_MEDIA_TYPE*>(pMediaType));
  return CTransInPlaceOutputPin::SetMediaType(pmtOut);
}

inline void FrameSkippingOutputPin::adjustAverageTimePerFrameInVideoInfoHeader(AM_MEDIA_TYPE * pmt)
{
  if (((FrameSkippingFilter*)m_pFilter)->m_dSkipRatio > 0.0)
  {
    if ((FORMAT_VideoInfo == pmt->formattype))
    {
      VIDEOINFOHEADER* pV = (VIDEOINFOHEADER*)pmt->pbFormat;
      REFERENCE_TIME duration = static_cast<REFERENCE_TIME>(pV->AvgTimePerFrame / ((FrameSkippingFilter*)m_pFilter)->m_dSkipRatio);
      pV->AvgTimePerFrame = duration;
    }
    else if ((FORMAT_VideoInfo2 == pmt->formattype))
    {
      VIDEOINFOHEADER2* pV = (VIDEOINFOHEADER2*)pmt->pbFormat;
      REFERENCE_TIME duration = static_cast<REFERENCE_TIME>(pV->AvgTimePerFrame / ((FrameSkippingFilter*)m_pFilter)->m_dSkipRatio);
      pV->AvgTimePerFrame = duration;
    }
  }
}

