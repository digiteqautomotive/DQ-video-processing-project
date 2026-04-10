/** @file

MODULE				: DLLSetup

FILE NAME			: DLLSetup.cpp

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik, All rights reserved.

===========================================================================*/
#include "stdafx.h"
#include "VideoCompareFilter.h"
#include "VideoCompareProperties.h"

//###############################  Standard Filter DLL Code ###############################
static const WCHAR g_wszName[] = L"DQ VPP Video Compare";   /// A name for the filter 

// The next bunch of structures define information for the class factory.
AMOVIESETUP_FILTER FilterInfo =
{
  &CLSID_VPP_VideoCompareFilter, // CLSID
  g_wszName,                    // Name
  MERIT_DO_NOT_USE,             // Merit
  0,                            // Number of AMOVIESETUP_PIN structs
  NULL                          // Pin registration information.
};

CFactoryTemplate g_Templates[] =
{
  {
    g_wszName,                          // Name
    &CLSID_VPP_VideoCompareFilter,       // CLSID
    VideoCompareFilter::CreateInstance,  // Method to create an instance of MyComponent
    NULL,                               // Initialization function
    &FilterInfo                         // Set-up information (for filters)
  },
  // This entry is for the property page.
  {
    L"Video Compare Properties",
    &CLSID_VideoCompareProperties,
    VideoCompareProperties::CreateInstance,
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


HRESULT  VideoCompareProperties::ReadSettings(void)
{
int nLength = 0;
char szBuffer[BUFFER_SIZE];
HRESULT hr = m_pSettingsInterface->GetParameter(FILTER_PARAM_ORIENTATION, sizeof(szBuffer), szBuffer, &nLength);
  if(FAILED(hr)) return hr;

  int nCompareMode = atoi(szBuffer);
  if(nCompareMode >= sizeof(RADIO_BUTTON_IDS)/sizeof(int)) return E_INVALIDARG;
  int nRadioID = RADIO_BUTTON_IDS[nCompareMode];
  long lResult = SendMessage(GetDlgItem(m_Dlg, nRadioID),	// handle to destination control
			     (UINT) BM_SETCHECK,		// message ID
			     (WPARAM) 1,			// = 0; not used, must be zero
			     (LPARAM) 0				// = (LPARAM) MAKELONG ((short) nUpper, (short) nLower)
			    );					// returns LRESULT in lResult

  hr = m_pSettingsInterface->GetParameter(FILTER_PARAM_SYNCHRONISATION, sizeof(szBuffer), szBuffer, &nLength);
  if(FAILED(hr)) return hr;
  nCompareMode = atoi(szBuffer);
  if(nCompareMode >= sizeof(RADIO_BTN_SYNC_IDS)/sizeof(int)) return E_INVALIDARG;
  nRadioID = RADIO_BTN_SYNC_IDS[nCompareMode];
  lResult = SendMessage(GetDlgItem(m_Dlg, nRadioID),
			     (UINT) BM_SETCHECK,
			     (WPARAM) 1,
			     (LPARAM) 0);

  return S_OK;
}


HRESULT VideoCompareProperties::OnApplyChanges(void)
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

