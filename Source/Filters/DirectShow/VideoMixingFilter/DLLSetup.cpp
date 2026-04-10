/** @file

MODULE				: DLLSetup

FILE NAME			: DLLSetup.cpp

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
Copyright (c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik, All rights reserved.

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
#include "VideoMixingFilter.h"
#include "VideoMixingProperties.h"
//////////////////////////////////////////////////////////////////////////
//###############################  Standard Filter DLL Code ###############################
static const WCHAR g_wszName[] = L"DQ VPP Video Mixer";   /// A name for the filter 

// The next bunch of structures define information for the class factory.
AMOVIESETUP_FILTER FilterInfo =
{
  &CLSID_VPP_VideoMixingFilter, // CLSID
  g_wszName,                    // Name
  MERIT_DO_NOT_USE,             // Merit
  0,                            // Number of AMOVIESETUP_PIN structs
  NULL                          // Pin registration information.
};

CFactoryTemplate g_Templates[] =
{
  {
    g_wszName,                          // Name
    &CLSID_VPP_VideoMixingFilter,       // CLSID
    VideoMixingFilter::CreateInstance,  // Method to create an instance of MyComponent
    NULL,                               // Initialization function
    &FilterInfo                         // Set-up information (for filters)
  },
  // This entry is for the property page.
  {
    L"Video Mixing Properties",
    &CLSID_VideoMixingProperties,
    VideoMixingProperties::CreateInstance,
    NULL, NULL
  }
};

int g_cTemplates = sizeof(g_Templates) / sizeof(g_Templates[0]);

//////////////////////////////////////////////////////////////////////////
// Functions needed by the DLL, for registration.

STDAPI DllRegisterServer(void)
{
  return AMovieDllRegisterServer2(TRUE);
}

STDAPI DllUnregisterServer()
{
  return AMovieDllRegisterServer2(FALSE);
}

//DLL Entry point
extern "C" BOOL WINAPI DllEntryPoint(HINSTANCE, ULONG, LPVOID);
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
  return DllEntryPoint((HINSTANCE)(hModule), dwReason, lpReserved);
}


//////////////////////////////////////////////////////////////////


HRESULT  VideoMixingProperties::ReadSettings(void)
{
int nLength = 0;
char szBuffer[BUFFER_SIZE];
HRESULT hr = m_pSettingsInterface->GetParameter(FILTER_PARAM_ORIENTATION, sizeof(szBuffer), szBuffer, &nLength);
  if(FAILED(hr)) return hr;

  int nMixingMode = atoi(szBuffer);
  if(nMixingMode >= sizeof(RADIO_BUTTON_IDS)/sizeof(int)) return E_INVALIDARG;
  int nRadioID = RADIO_BUTTON_IDS[nMixingMode];
  long lResult = SendMessage(GetDlgItem(m_Dlg, nRadioID),	// handle to destination control
			     (UINT) BM_SETCHECK,		// message ID
			     (WPARAM) 1,			// = 0; not used, must be zero
			     (LPARAM) 0				// = (LPARAM) MAKELONG ((short) nUpper, (short) nLower)
			    );					// returns LRESULT in lResult

  hr = m_pSettingsInterface->GetParameter(FILTER_PARAM_SYNCHRONISATION, sizeof(szBuffer), szBuffer, &nLength);
  if(FAILED(hr)) return hr;
  nMixingMode = atoi(szBuffer);
  if(nMixingMode >= sizeof(RADIO_BTN_SYNC_IDS)/sizeof(int)) return E_INVALIDARG;
  nRadioID = RADIO_BTN_SYNC_IDS[nMixingMode];
  lResult = SendMessage(GetDlgItem(m_Dlg, nRadioID),
			     (UINT) BM_SETCHECK,
			     (WPARAM) 1,
			     (LPARAM) 0);

  return S_OK;
}


HRESULT VideoMixingProperties::OnApplyChanges(void)
{
int i;
  for(i=0; i<sizeof(RADIO_BUTTON_IDS)/sizeof(int); ++i)
  {
    const int iCheck = SendMessage(GetDlgItem(m_Dlg, RADIO_BUTTON_IDS[i]), (UINT)BM_GETCHECK, 0, 0);
    if(iCheck != 0)
    {
      std::string sID = toString(i);
      HRESULT hr = m_pSettingsInterface->SetParameter(FILTER_PARAM_ORIENTATION, sID.c_str());
      break;
    }
  }

  for(i=0; i<sizeof(RADIO_BTN_SYNC_IDS)/sizeof(int); ++i)
  {
    const int iCheck = SendMessage(GetDlgItem(m_Dlg, RADIO_BTN_SYNC_IDS[i]), (UINT)BM_GETCHECK, 0, 0);
    if(iCheck != 0)
    {
      std::string sID = toString(i);
      HRESULT hr = m_pSettingsInterface->SetParameter(FILTER_PARAM_SYNCHRONISATION, sID.c_str());
      break;
    }
  }
  return S_OK;
} 

