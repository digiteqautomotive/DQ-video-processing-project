/** @file

MODULE				: YUVSource

FILE NAME			: YuvSource.cpp

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
#include <streams.h>

#include <fstream>
#include <sstream>

#include "YuvSource.h"
#include "YuvOutputPin.h"

#include <Util/Conversion.h>
#include <Util/StringUtil.h>

const AMOVIESETUP_MEDIATYPE sudOpPinTypes =
{
  &MEDIATYPE_Video,       // Major type
  &MEDIASUBTYPE_NULL      // Minor type
};

using namespace artist;

/**********************************************
*
*  YuvSourceFilter Class
*
**********************************************/

// UNITS = 10 ^ 7  
// UNITS / 30 = 30 fps;
// UNITS / 20 = 20 fps, etc
const REFERENCE_TIME FPS_60 = UNITS / 60;
const REFERENCE_TIME FPS_30 = UNITS / 30;
const REFERENCE_TIME FPS_20 = UNITS / 20;
const REFERENCE_TIME FPS_10 = UNITS / 10;
const REFERENCE_TIME FPS_5 = UNITS / 5;
const REFERENCE_TIME FPS_4 = UNITS / 4;
const REFERENCE_TIME FPS_3 = UNITS / 3;
const REFERENCE_TIME FPS_2 = UNITS / 2;
const REFERENCE_TIME FPS_1 = UNITS / 1;

const REFERENCE_TIME rtDefaultFrameLength = FPS_10;

CUnknown * WINAPI YuvSourceFilter::CreateInstance(IUnknown *pUnk, HRESULT *phr)
{
  YuvSourceFilter *pNewFilter = new YuvSourceFilter(pUnk, phr );
  if (phr)
  {
    if (pNewFilter == NULL) 
      *phr = E_OUTOFMEMORY;
    else
      *phr = S_OK;
  }
  return pNewFilter;
}

YuvSourceFilter::YuvSourceFilter(IUnknown *pUnk, HRESULT *phr)
  : CSource(NAME("CSIR RTVC YUV Source"), pUnk, CLSID_VPP_YUVSource),
  m_iWidth(352),
  m_iHeight(288),
  m_sDimensions("352x288"),
  m_iFramesPerSecond(30),
  m_rtFrameLength(FPS_30),
  m_iNoFrames(150),		  //TODO: move to property page
  m_dBytesPerPixel(1.5),
  m_pYuvBuffer(NULL),
  m_iFileSize(0),
  m_iRead(0)
{
  // Init CSettingsInterface
  initParameters();

  m_pPin = new YuvOutputPin(phr, this);

  if (phr)
  {
    if (m_pPin == NULL)
      *phr = E_OUTOFMEMORY;
    else
      *phr = S_OK;
  }  
}

YuvSourceFilter::~YuvSourceFilter()
{
  if ( m_in1.is_open() )
  {
    m_in1.close();
  }
  if (m_pYuvBuffer)
    delete[] m_pYuvBuffer;

  delete m_pPin;
}

STDMETHODIMP YuvSourceFilter::Load( LPCOLESTR lpwszFileName, const AM_MEDIA_TYPE *pmt )
{
  // Store the URL
  m_sFile = StringUtil::wideStringToString(lpwszFileName);
 
  m_in1.open(m_sFile.c_str(), std::ifstream::in | std::ifstream::binary);
  if ( m_in1.is_open() )
  {
    m_in1.seekg( 0, std::ios::end );
    m_iFileSize = m_in1.tellg();
    // we need the file size to call recalculate
    createYuvFrameBuffer();
    m_in1.seekg(0, std::ios::beg);
    return S_OK;
  }
  else
  {
    SetLastError(("Failed to open file: " + m_sFile).c_str(), true);
    return E_FAIL;
  }
}

void YuvSourceFilter::createYuvFrameBuffer()
{
  std::string sFormatString = m_sFile;
  size_t pos = sFormatString.find("444");
  if (pos != std::string::npos)
  {
    m_dBytesPerPixel = 4.0;
    m_nYuvFormat = YUV444I;
  }
  else
  {
    m_dBytesPerPixel = 1.5;
    m_nYuvFormat = YUV420P;
  }

  // try and guess what the dimensions are based on file name
  std::transform(sFormatString.begin(), sFormatString.end(), sFormatString.begin(), ::tolower);

  pos = sFormatString.find("qcif");
  if (pos != std::string::npos)
  {
    setDimensions("176x144");
    return;
  }

  // NB search for cif last since 'qcif' and '4cif' contain 'cif'
  pos = sFormatString.find("cif");
  if (pos != std::string::npos)
  {
    setDimensions("352x288");
    return;
  }

  // try searching for x's
  pos = sFormatString.find("x");
  while (pos != std::string::npos)
  {
    std::string sWidth, sHeight;
    // check if there is at least a number before and after the x, else continue
    size_t uiBeforePos = pos - 1;
    while (isdigit(sFormatString[uiBeforePos]) && uiBeforePos >= 0)
    {
      --uiBeforePos;
    }
    if (uiBeforePos != pos - 1)
    {
      // we found at least one digit
      sWidth = sFormatString.substr(uiBeforePos + 1, pos - uiBeforePos - 1);
    }
    else
    {
      pos = sFormatString.find("x", pos + 1);
      continue;
    }

    size_t uiAfterPos = pos + 1;
    while (isdigit(sFormatString[uiAfterPos]) && uiAfterPos < sFormatString.length())
    {
      ++uiAfterPos;
    }
    if (uiAfterPos != pos + 1)
    {
      // we found at least one digit
      sHeight = sFormatString.substr(pos + 1, uiAfterPos - pos - 1);
    }
    else
    {
      pos = sFormatString.find("x", pos + 1);
      continue;
    }

    // try and convert to int
    bool bDummy = false;
    int iWidth = convert<int>(sWidth, bDummy);
    int iHeight = convert<int>(sHeight, bDummy);
    updatePictureBuffer(iWidth, iHeight, m_dBytesPerPixel);
    // update for property page
    m_sDimensions = sWidth + "x" + sHeight;
    break;
  }

  // just create picture buffer using default values
  recalculate();
  m_pYuvBuffer = new unsigned char[m_iFrameSize];

}

STDMETHODIMP YuvSourceFilter::GetCurFile( LPOLESTR * ppszFileName, AM_MEDIA_TYPE *pmt )
{
  *ppszFileName = NULL;

  if (m_sFile.length()!=0) 
  {
    std::wstring wsFileName = StringUtil::stringToWideString(m_sFile);
    DWORD n = sizeof(WCHAR)*(1 + lstrlenW(wsFileName.c_str()));
    *ppszFileName = (LPOLESTR)CoTaskMemAlloc(n);
    if (*ppszFileName != NULL) {
      CopyMemory(*ppszFileName, wsFileName.c_str(), n);
    }
  }
  return NOERROR;
}

STDMETHODIMP YuvSourceFilter::NonDelegatingQueryInterface( REFIID riid, void **ppv )
{
  if(riid == (IID_ISettingsInterface))
  {
    return GetInterface((ISettingsInterface*) this, ppv);
  }
  else if (riid == IID_IStatusInterface)
  {
    return GetInterface((IStatusInterface*) this, ppv);
  }
  else if (riid == IID_IFileSourceFilter)
  {
    return GetInterface((IFileSourceFilter*) this, ppv);
  }
  else if (riid == IID_ISpecifyPropertyPages)
  {
    return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  }
  else
  {
    return CSource::NonDelegatingQueryInterface(riid, ppv);
  }
}

STDMETHODIMP YuvSourceFilter::Stop()
{
  m_pPin->m_iCurrentFrame = 0;
  m_in1.seekg( 0 , std::ios::beg );
  return CSource::Stop();
}

STDMETHODIMP YuvSourceFilter::SetParameter( const char* type, const char* value )
{
  if (strcmp(type, SOURCE_DIMENSIONS) == 0)
  {
    if ( setDimensions(value) )
      return CSettingsInterface::SetParameter(type, value);
    else
      return E_FAIL;
  }
  else if (strcmp(type, SOURCE_FPS) == 0)
  {
    HRESULT hr = CSettingsInterface::SetParameter(type, value);
    if (SUCCEEDED(hr))
    {
      m_rtFrameLength = UNITS / m_iFramesPerSecond;
    }
    return hr;
  }
  else
  {
    HRESULT hr = CSettingsInterface::SetParameter(type, value);
    if (SUCCEEDED(hr)) recalculate();
    return hr;
  }
}

void YuvSourceFilter::recalculate()
{
  m_iFrameSize = static_cast<int>(m_iWidth*m_iHeight*m_dBytesPerPixel);
  m_iNoFrames = m_iFileSize/m_iFrameSize;
  // m_pPin->m_rtFrameLength = UNITS/m_iFramesPerSecond;
}

bool YuvSourceFilter::readFrame()
{
  if (m_in1.is_open())
  {
    m_in1.read( (char*)m_pYuvBuffer, m_iFrameSize );
    return true;
  }
  return false;
}

bool YuvSourceFilter::setDimensions( const std::string& sDimensions )
{
  size_t pos = sDimensions.find( "x" );
  if (pos != std::string::npos ) 
  {
    int iWidth = 0, iHeight = 0;
    std::istringstream istr( sDimensions.substr( 0, pos ) );
    istr >> iWidth;
    std::istringstream istr2( sDimensions.substr( pos + 1) );
    istr2 >> iHeight;
    return updatePictureBuffer(iWidth, iHeight, m_dBytesPerPixel);
  }
  return false;
}

bool YuvSourceFilter::updatePictureBuffer(int iWidth, int iHeight, double dBytesPerPixel)
{
  if (iWidth > 0 && iHeight > 0)
  {
    m_iWidth = iWidth;
    m_iHeight = iHeight;
    recalculate();

    if (m_pYuvBuffer)
    {
      delete[] m_pYuvBuffer;
    }
    m_pYuvBuffer = new unsigned char[m_iFrameSize];
    return true;
  }
  else
  {
    return false;
  }
}
