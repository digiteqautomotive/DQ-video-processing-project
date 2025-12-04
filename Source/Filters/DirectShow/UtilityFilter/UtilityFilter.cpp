/** @file

MODULE			: UtilityFilter

FILE NAME		: UtilityFilter.cpp

DESCRIPTION		:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2015, CSIR
Copyright (c) 2025, Jaroslav Fojtik, Digiteq Automotive All rights reserved.

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
#include "UtilityFilter.h"
#include "DirectShow/DirectShowHelper.h"
#include "DirectShow/StatusInterface.h"
#include "GeneralUtils/Conversion.h"
#include <Gdiplus.h>
#include <GdiPlusInit.h>
#include <numeric>

#define _CONVERT_WITH_ATL
#include "GeneralUtils/StringUtil.h"

#include <Dvdmedia.h>			// VIDEOINFOHEADER2

#include "../VariantTools.cpp"


using namespace Gdiplus;
//using namespace artist;

UtilityFilter::UtilityFilter(LPUNKNOWN pUnk, HRESULT *pHr):
  CTransInPlaceFilter(NAME("DQ VPP Utility Filter"), pUnk, CLSID_VPP_UtilityFilter, pHr, false),
  m_dEstimatedFramerate(-1.0),
  m_dEstimatedBitrate(-1.0),
  m_bSeenFirstFrame(false),
  m_tPreviousTimestamp(0),
  m_tMaxDifferenceBetweenFrames(50000000), // 5 seconds max
  m_dTimerFrequency(0.0),
  m_iPerformanceFrequency(0),
  m_gdiplusStartupInput(new Gdiplus::GdiplusStartupInput()),
  m_pGdiToken(NULL),
  m_hrInterfaceAquired(S_OK),
  m_DirtyProps(true)
{
  GdiplusStartup(&m_pGdiToken, m_gdiplusStartupInput, NULL); //gdi+ init
  QueryPerformanceFrequency((LARGE_INTEGER*)&m_iPerformanceFrequency);
  // Init parameters
  initParameters();
}

UtilityFilter::~UtilityFilter()
{
  GdiplusShutdown(m_pGdiToken); //gdi+ end session
  delete m_gdiplusStartupInput;
}

CUnknown * WINAPI UtilityFilter::CreateInstance( LPUNKNOWN pUnk, HRESULT *pHr )
{
  UtilityFilter *pFilter = new UtilityFilter(pUnk, pHr);
  if (pFilter== NULL) 
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pFilter;
}

STDMETHODIMP UtilityFilter::NonDelegatingQueryInterface( REFIID riid, void **ppv )
{
  if(riid == IID_ISpecifyPropertyPages)
      return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  if(riid == (IID_ISettingsInterface))  
      return GetInterface((ISettingsInterface*)this, ppv);
  if(riid == (IID_IStatusInterface))
      return GetInterface((IStatusInterface*)this, ppv);
  if(riid == (IID_IFilterInfoSourceInterface))
      return GetInterface((IFilterInfoSourceInterface*)this, ppv);
  if(riid == IID_IPersistPropertyBag)
      return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPropertyBag)
      return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_IPersistStream)
      return GetInterface((IPersistStream*)this, ppv);

  return CTransInPlaceFilter::NonDelegatingQueryInterface(riid, ppv);
}


HRESULT UtilityFilter::drawTextOntoFrame(const std::string& sText, IMediaSample *pSample)
{
  // get image properties
  AM_MEDIA_TYPE mt;
  HRESULT hr = InputPin()->ConnectionMediaType(&mt);
  if (FAILED(hr))
  {
    return hr;
  }
  const BITMAPINFOHEADER *pbmi = NULL;
  if(mt.formattype==FORMAT_VideoInfo)
  {
    pbmi = HEADER(mt.pbFormat);
  }
  else if(mt.formattype==FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 * const pVih2 = (VIDEOINFOHEADER2*)mt.pbFormat;
    if(pVih2) pbmi=&(pVih2->bmiHeader);
  }
  else
      return VFW_E_TYPE_NOT_ACCEPTED;

  // Get image buffer
  BYTE *pBuffer(NULL);
  hr = pSample->GetPointer(&pBuffer);
  if(FAILED(hr))
  {
    // Release the format block.
    FreeMediaType(mt);
    return hr;
  }

  std::wstring wsText = StringUtil::stringToWideString(sText);

  BITMAPINFO bitmapInfo;
  bitmapInfo.bmiHeader = *pbmi;

  Bitmap bm(&bitmapInfo, (void*)pBuffer);
  Graphics* pGraphics = Graphics::FromImage(&bm);

  // Initialize font
  Gdiplus::Font myFont(L"Arial", 16);
  //RectF layoutRect(0.0f, 0.0f, 200.0f, 50.0f);
  RectF layoutRect((float)m_uiX, (float)m_uiY, 350.0f, 50.0f);
  StringFormat format;
  format.SetAlignment(StringAlignmentNear);
  SolidBrush blackBrush(Color(255, 0, 0, 0));
  SolidBrush greenBrush(Color(255, 0, 255, 0));

  // Draw string
  pGraphics->DrawString(
    wsText.c_str(),
    sText.length(),
    &myFont,
    layoutRect,
    &format,
    &greenBrush);

	// Do we need to delete it?
  delete pGraphics;

	// Release the format block.
  FreeMediaType(mt);

  return S_OK;
}

HRESULT UtilityFilter::Transform(IMediaSample *pSample)
{
  using namespace Gdiplus;

  std::string sText;

  switch (m_uiFramerateEstimationMode)
  {
    case UTIL_NONE:
    {
      return S_OK;
    }
    case UTIL_TIME_STAMP_FRAME:
    {
      // %X = time
      sText = StringUtil::GetTimeString("%X");
      break;
    }
    case UTIL_DATE_TIME_STAMP_FRAME:
    {
      // %d/%m/%Y date
      sText = StringUtil::GetTimeString("%d/%m/%Y %X");
      break;
    }
    // always measure both frame and bit rates
    case UTIL_EST_FRAMERATE_TIMESTAMP:
    case UTIL_EST_FRAMERATE_SYSTEM_TIME:
    case UTIL_DATE_TIME_STAMP_AND_EST_FPS_TS:
    case UTIL_DATE_TIME_STAMP_AND_EST_FPS_ST:
    case UTIL_MEASURE_BITRATE:
    {
      REFERENCE_TIME tStart = 0, tStop = 0;
      HRESULT hr = pSample->GetTime(&tStart, &tStop);
      if (SUCCEEDED(hr))
      {
        REFERENCE_TIME tStartTime = 0;
        switch (m_uiFramerateEstimationMode)
        {
          case UTIL_EST_FRAMERATE_TIMESTAMP:
          case UTIL_DATE_TIME_STAMP_AND_EST_FPS_TS:
          case UTIL_MEASURE_BITRATE:
          {
            tStartTime = tStart;
            break;
          }
          case UTIL_EST_FRAMERATE_SYSTEM_TIME:
          case UTIL_DATE_TIME_STAMP_AND_EST_FPS_ST:
          {
            QueryPerformanceCounter((LARGE_INTEGER *)&tStartTime);
            break;
          }
        }

        // frame rate calculations
        if (!m_bSeenFirstFrame)
        {
          initFramerateEstimation(tStartTime);
          m_bSeenFirstFrame = true;
          return S_OK;
        }
        else
        {
          REFERENCE_TIME tDiff = getDifference(tStartTime);
          // Make sure timestamps are increasing
          if (tDiff > 0 && tDiff < m_tMaxDifferenceBetweenFrames)
          {
            m_qDurations.push_back(SampleInfo(tStartTime, tDiff, pSample->GetActualDataLength()));
            m_tPreviousTimestamp = tStartTime;

            // limit the number of measurements
            if (m_qDurations.size() > m_uiHistorySize)
            {
              m_qDurations.pop_front();
            }
          }
          else
          {
            // reset vars and restart estimation
            initFramerateEstimation(tStartTime);
          }
          // frame rate
          m_dEstimatedFramerate = calculateFramerate();
          // bit rate calculations
          m_dEstimatedBitrate = calculateBitrate();
        }


        // now set text according to mode
        switch (m_uiFramerateEstimationMode)
        {
          case UTIL_EST_FRAMERATE_TIMESTAMP:
          case UTIL_EST_FRAMERATE_SYSTEM_TIME:
          case UTIL_DATE_TIME_STAMP_AND_EST_FPS_TS:
          case UTIL_DATE_TIME_STAMP_AND_EST_FPS_ST:
          {
            sText = getString(m_dEstimatedFramerate);
            break;
          }
          case UTIL_MEASURE_BITRATE:
          {
            sText = getString(m_dEstimatedBitrate);
            break;
          }
        }
        break;
      }
      else
      {
        return hr;
      }
    }
    case UTIL_DETECT_UPSTREAM_SOURCE:
    {
      if (FAILED(m_hrInterfaceAquired)) return m_hrInterfaceAquired;
      int iLength = BUFFER_SIZE;
#if 1
      if (!m_pFilterInfoSourceInterface)
      {
        m_hrInterfaceAquired = CDirectShowHelper::FindFirstInterface(this, PINDIR_INPUT, IID_IFilterInfoSourceInterface, (void **)&m_pFilterInfoSourceInterface);
        if (FAILED(m_hrInterfaceAquired))
        {
          DbgLog((LOG_TRACE, 0, TEXT("Failed to find upstream IID_IFilterInfoSourceInterface interface")));
          return m_hrInterfaceAquired;
        }
      }

      HRESULT hr = m_pFilterInfoSourceInterface->GetMeasurement(m_pBuffer, &iLength);
      if (FAILED(hr))
      {
        DbgLog((LOG_TRACE, 0, TEXT("Failed to find get measurement from IID_IFilterInfoSourceInterface interface")));
        return hr;
      }
#endif
      sText = std::string(m_pBuffer, iLength);
      break;
    }
    default:
    {
      return S_OK;
    }
  }

  // check flag if text should be written to frame
  if (m_bWriteOnFrame)
  {
    DbgLog((LOG_TRACE, 0, TEXT("Utility: %s"), sText.c_str()));
    return drawTextOntoFrame(sText, pSample);
  }
  else
    return S_OK;
}


inline REFERENCE_TIME UtilityFilter::getDifference(REFERENCE_TIME tSampleStartTime)
{
  switch (m_uiFramerateEstimationMode)
  {
    case UTIL_EST_FRAMERATE_TIMESTAMP:
    case UTIL_DATE_TIME_STAMP_AND_EST_FPS_TS:
    case UTIL_MEASURE_BITRATE:
    {
      return tSampleStartTime - m_tPreviousTimestamp;
    }
    case UTIL_EST_FRAMERATE_SYSTEM_TIME:
    case UTIL_DATE_TIME_STAMP_AND_EST_FPS_ST:
    {
      return ((tSampleStartTime - m_tPreviousTimestamp) * 10000000 / m_iPerformanceFrequency);
    }
  }
  return 0;
}


HRESULT UtilityFilter::CheckInputType(const CMediaType* mtIn)
{
	// Check the major type.
  if(mtIn->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if(mtIn->formattype!=FORMAT_VideoInfo && mtIn->formattype!=FORMAT_VideoInfo2)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }
  return S_OK;
}


HRESULT UtilityFilter::Run( REFERENCE_TIME tStart )
{
  return CTransInPlaceFilter::Run(tStart);
}


HRESULT UtilityFilter::Stop( void )
{
  m_qDurations.clear();
  m_dEstimatedFramerate = 0.0;
  m_bSeenFirstFrame = false;
  m_tPreviousTimestamp = 0;

  return CTransInPlaceFilter::Stop();
}


void UtilityFilter::initFramerateEstimation(REFERENCE_TIME tSampleStartTime)
{
  m_tPreviousTimestamp = tSampleStartTime;
  // clear queue in case mode was changed
  m_qDurations.clear();
}


double UtilityFilter::calculateFramerate()
{
  if (!m_qDurations.empty())
  {
    // calc avg framerate
    REFERENCE_TIME uiTotal = 0;
    for (size_t i = 0; i < m_qDurations.size(); ++i)
    {
      uiTotal += m_qDurations[i].Diff;
    }
    uiTotal /= m_qDurations.size();

    // Avoid div by zero in cases where the timestamps are invalid
    if (uiTotal > 0)
    {
      double dAverageFramerate = static_cast<double>(UNITS / static_cast<double>(uiTotal));
      return dAverageFramerate;
    }
  }
  return 0.0;
}


double UtilityFilter::calculateBitrate()
{
  if (!m_qDurations.empty())
  {
    // calc avg bitrate
    REFERENCE_TIME tFirst = m_qDurations.front().TimeStamp;
    REFERENCE_TIME tLast = m_qDurations.back().TimeStamp;
    REFERENCE_TIME tDiff = tLast - tFirst;
    double dSeconds = tDiff / (double)UNITS;
    if (dSeconds == 0.0) return -1.0;

    unsigned uiTotalBytes = 0;
    for (size_t i = 0; i < m_qDurations.size(); ++i)
    {
      uiTotalBytes += m_qDurations[i].Size;
    }
    
    double dBitrate = uiTotalBytes/1000 * 8 / dSeconds;

    return dBitrate;
  }
  return -1.0;
}


STDMETHODIMP UtilityFilter::GetMeasurement(char* value, int* buffersize)
{
  if (static_cast<int>(m_sLastValue.length()) > *buffersize)
    return E_FAIL;
  memcpy(value, m_sLastValue.c_str(), m_sLastValue.length());
  *buffersize = m_sLastValue.length();
  return S_OK;
}


const std::string UtilityFilter::getString( double dAverageFramerate )
{
  switch (m_uiFramerateEstimationMode)
  {
  case UTIL_EST_FRAMERATE_TIMESTAMP:
  case UTIL_EST_FRAMERATE_SYSTEM_TIME:
    {
      m_sLastValue = StringUtil::doubleToString(dAverageFramerate, m_uiPrecision) + " fps";
      return m_sLastValue;
    }
  case UTIL_DATE_TIME_STAMP_AND_EST_FPS_TS:
  case UTIL_DATE_TIME_STAMP_AND_EST_FPS_ST:
    {
      std::string sFramerate = StringUtil::doubleToString(dAverageFramerate, m_uiPrecision) + " fps";
      std::string m_sLastValue = StringUtil::GetTimeString("%d/%m/%Y %X") + " " + sFramerate;
      return m_sLastValue;
    }
  case UTIL_MEASURE_BITRATE:
    {
      m_sLastValue = StringUtil::doubleToString(dAverageFramerate, m_uiPrecision) + " kbps";
      return m_sLastValue;
  }
  default:
    return "";
  }
}


STDMETHODIMP UtilityFilter::SetParameter( const char* type, const char* value )
{
  if (SUCCEEDED(CSettingsInterface::SetParameter(type, value)))
  {
    // reset members
    m_bSeenFirstFrame = false;
    m_tPreviousTimestamp = 0;
    return S_OK;
  }
  else
  {
    return E_FAIL;
  }
}


STDMETHODIMP UtilityFilter::GetPages(CAUUID *pPages)
{
  if(pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if(pPages->pElems == NULL) 
  {
	return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_UtilityProperties;
  return S_OK;
}


void UtilityFilter::initParameters(void) 
{
  addParameter(UF_PARAM_X, &m_uiX, 0);
  addParameter(UF_PARAM_Y, &m_uiY, 0);
  addParameter(UF_ESTIMATED_FRAMERATE, &m_dEstimatedFramerate, -1.0);
  addParameter(UF_ESTIMATED_BITRATE, &m_dEstimatedBitrate, -1.0);
  addParameter(UF_MODE, &m_uiFramerateEstimationMode, 1);
  addParameter("writeonframe", &m_bWriteOnFrame, true);
  addParameter(UF_PRECISION, &m_uiPrecision, 1);
  addParameter(UF_HISTORY_SIZE, &m_uiHistorySize, 200);
}


//----------------- IPersistPropertyBag------------------


HRESULT UtilityFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_UtilityFilter;
  return S_OK;
}


HRESULT UtilityFilter::InitNew(void)
{
  m_uiX = m_uiY = 0;  
  return S_OK;
}


OLECHAR OLE_UF_PARAM_X[] = olestr(UF_PARAM_X);
OLECHAR OLE_UF_PARAM_Y[] = olestr(UF_PARAM_Y);
OLECHAR OLE_UF_MODE[] = olestr(UF_MODE);


HRESULT UtilityFilter::Load(IPropertyBag *pPropBag, IErrorLog *pErrorLog)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  memset(&var,0,sizeof(var));

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_UF_PARAM_X));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_UF_PARAM_X,sizeof(OLE_UF_PARAM_X));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      UnsignedFromVariant(m_uiX,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_UF_PARAM_Y));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_UF_PARAM_Y,sizeof(OLE_UF_PARAM_Y));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      UnsignedFromVariant(m_uiY,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_UF_MODE));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_UF_MODE,sizeof(OLE_UF_MODE));
    VariantClear(&var);
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      UnsignedFromVariant(m_uiFramerateEstimationMode,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  VariantClear(&var);
return S_OK;
}


HRESULT UtilityFilter::Save(IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_UF_PARAM_X));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_UF_PARAM_X,sizeof(OLE_UF_PARAM_X));
    var.vt = VT_UINT;
    var.uintVal = m_uiX;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }
  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_UF_PARAM_Y));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_UF_PARAM_Y,sizeof(OLE_UF_PARAM_Y));
    var.vt = VT_UINT;
    var.uintVal = m_uiY;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }
  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_UF_MODE));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_UF_MODE,sizeof(OLE_UF_MODE));
    var.vt = VT_INT;
    var.intVal = m_uiFramerateEstimationMode;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

return S_OK;
}


//----------------- IPropertyBag------------------


HRESULT UtilityFilter::Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  VariantClear(pVar);

  if(oleicmp(pszPropName, OLE_UF_PARAM_X) == 0)
  {   
    pVar->vt = VT_UINT;
    pVar->uintVal = m_uiX;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_UF_PARAM_Y) == 0)
  {   
    pVar->vt = VT_UINT;
    pVar->uintVal = m_uiY;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_UF_MODE) == 0)
  {   
    pVar->vt = VT_UINT;
    pVar->uintVal = m_uiFramerateEstimationMode;
    return S_OK;
  }

  return E_INVALIDARG;
}


HRESULT UtilityFilter::Write(LPCOLESTR pszPropName, VARIANT *pVar)
{
CheckPointer(pszPropName, E_INVALIDARG);
CheckPointer(pVar, E_INVALIDARG);
  if(oleicmp(pszPropName, OLE_UF_PARAM_X) == 0)
  {   
    if(UnsignedFromVariant(m_uiX,pVar)) return S_OK;
  }
  else if(oleicmp(pszPropName, OLE_UF_PARAM_Y) == 0)
  {
    if(UnsignedFromVariant(m_uiY,pVar)) return S_OK;    
  }
  else if(oleicmp(pszPropName, OLE_UF_MODE) == 0)
  {
    if(UnsignedFromVariant(m_uiFramerateEstimationMode,pVar)) return S_OK;    
  }
  return E_INVALIDARG;
}


//----------------- IPersistStream------------------


HRESULT UtilityFilter::GetSizeMax(ULARGE_INTEGER *pcbSize)
{
CheckPointer(pcbSize, E_INVALIDARG);
  pcbSize->QuadPart = sizeof(m_uiX) + sizeof(m_uiY) + sizeof(m_uiFramerateEstimationMode);
  return S_OK;
}


HRESULT UtilityFilter::IsDirty(void)
{
  return m_DirtyProps ? S_OK : S_FALSE;
}


HRESULT UtilityFilter::Load(IStream *pStm)
{
ULONG pcbRead;
unsigned uiX, uiY, uiFramerateEstimationMode;
HRESULT hr;
  ASSERT(sizeof(uiX)==sizeof(m_uiX));
  ASSERT(sizeof(uiY)==sizeof(m_uiY));
  ASSERT(sizeof(uiFramerateEstimationMode)==sizeof(m_uiFramerateEstimationMode));
  CheckPointer(pStm, E_INVALIDARG);

  if(FAILED(hr=pStm->Read(&uiX, sizeof(uiX), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&uiY, sizeof(uiY), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&uiFramerateEstimationMode, sizeof(uiFramerateEstimationMode), &pcbRead))) return hr;
  if(m_uiX!=uiX || m_uiY!=uiY || m_uiFramerateEstimationMode!=uiFramerateEstimationMode)
  {
    m_uiX = uiX;
    m_uiY = uiY;
    m_uiFramerateEstimationMode = uiFramerateEstimationMode;
    m_DirtyProps = true;
  }
return S_OK;
}


HRESULT UtilityFilter::Save(IStream *pStm, BOOL fClearDirty)
{
HRESULT hr;
CheckPointer(pStm, E_INVALIDARG);
  if(FAILED(hr=pStm->Write(&m_uiX, sizeof(m_uiX), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_uiY, sizeof(m_uiY), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_uiFramerateEstimationMode, sizeof(m_uiFramerateEstimationMode), NULL))) return hr;
  if(fClearDirty) m_DirtyProps=false;
return S_OK;
}

