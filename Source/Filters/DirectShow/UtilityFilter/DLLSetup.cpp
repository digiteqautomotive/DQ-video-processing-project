/** @file

MODULE				: DLLSetup

FILE NAME			: DLLSetup.cpp

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2015, CSIR
Copyright (c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.

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
#include "UtilityFilter.h"
#include "UtilityProperties.h"
//////////////////////////////////////////////////////////////////////////
//###############################  Standard Filter DLL Code ###############################
static const WCHAR g_wszName[] = L"DQ VPP Utility Filter";

// The next bunch of structures define information for the class factory.
AMOVIESETUP_FILTER FilterInfo =
{
  &CLSID_VPP_UtilityFilter,  // CLSID
  g_wszName,                  // Name
  MERIT_DO_NOT_USE,           // Merit
  0,                          // Number of AMOVIESETUP_PIN structs
  NULL                        // Pin registration information.
};


CFactoryTemplate g_Templates[] =
{
  {
    g_wszName,                      // Name
    &CLSID_VPP_UtilityFilter,       // CLSID
    UtilityFilter::CreateInstance,  // Method to create an instance of MyComponent
    NULL,                           // Initialization function
    &FilterInfo                     // Set-up information (for filters)
  },
  // This entry is for the property page.
  {
    L"Utility Properties",
    &CLSID_UtilityProperties,
    UtilityProperties::CreateInstance,
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


//////////////////////////////////////////////////////////////////////////

HRESULT UtilityProperties::ReadSettings()
{
    initialiseControls();

    int nLength = 0;
    char szBuffer[BUFFER_SIZE];

    // Mode of operation
    HRESULT hr = m_pSettingsInterface->GetParameter("mode", sizeof(szBuffer), szBuffer, &nLength);
    if (SUCCEEDED(hr))
    {
      unsigned uiMode = atoi(szBuffer);
      switch (uiMode)
      {
      case 0:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING,  0, (LPARAM)UF_FPS_TS);
          break;
        }
      case 1:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING,  0, (LPARAM)UF_FPS_ST);
          break;
        }
      case 2:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING,  0, (LPARAM)UF_TIME);
          break;
        }
      case 3:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING,  0, (LPARAM)UF_DATE_TIME);
          break;
        }
      case 4:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING,  0, (LPARAM)UF_FPS_TS_DATE_TIME);
          break;
        }
      case 5:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING,  0, (LPARAM)UF_FPS_ST_DATE_TIME);
          break;
        }
      case 6:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING, 0, (LPARAM)UF_MEASURE_BITRATE);
          break;
      }
      case 7:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING, 0, (LPARAM)UF_DETECT_UPSTREAM_SOURCE);
        break;
        }
      case 8:
        {
        SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING, 0, (LPARAM)UF_NONE);
        break;
        }
      }
    }
    else
    {
      return E_FAIL;
    }

  hr = setEditTextFromIntFilterParameter(UF_PARAM_X,IDC_EDIT_X);
  if(FAILED(hr)) return hr;
  hr = setEditTextFromIntFilterParameter(UF_PARAM_Y,IDC_EDIT_Y);
  if(FAILED(hr)) return hr;
  hr = setEditTextFromIntFilterParameter(UF_FONT_SIZE,IDC_FONT_SIZE);
  if(FAILED(hr)) return hr;
  hr = setCheckBoxFromBoolFilterParameter(UF_WRITE_ON_FRAME,IDC_WRITE_ON_FRAME);
  if(FAILED(hr)) return hr;
  hr = setEditTextFromIntFilterParameter(UF_HISTORY_SIZE,IDC_HISTORY_SIZE);
  if(FAILED(hr)) return hr;
  hr = setEditTextFromIntFilterParameter(UF_PRECISION,IDC_PRECISION);
  return hr;
}


HRESULT UtilityProperties::OnApplyChanges(void)
{
  int nLength = 0;
  char szBuffer[BUFFER_SIZE];

    // mode of operation
  int index = ComboBox_GetCurSel(GetDlgItem(m_Dlg, IDC_CMB_MODE));
  ASSERT (index != CB_ERR);
  _itoa(index, szBuffer, 10);
  m_pSettingsInterface->SetParameter(UF_MODE, szBuffer);
    //nLength = GetDlgItemText(m_Dlg, IDC_CMB_MODE, szBuffer, BUFFER_SIZE);
    //if (strcmp(szBuffer, UF_FPS_TS_DATE_TIME) == 0)
    //{

    //}
  m_pSettingsInterface->SetParameter(UF_MODE, szBuffer);

  HRESULT hr = setIntFilterParameterFromEditText(UF_PARAM_X,IDC_EDIT_X);
  if(FAILED(hr)) return hr;
  hr = setIntFilterParameterFromEditText(UF_PARAM_Y,IDC_EDIT_Y);
  if(FAILED(hr)) return hr;
  hr = setIntFilterParameterFromEditText(UF_FONT_SIZE,IDC_FONT_SIZE);
  if(FAILED(hr)) return hr;
  hr = setBoolFilterParameterFromCheckBox(UF_WRITE_ON_FRAME,IDC_WRITE_ON_FRAME);
  if(FAILED(hr)) return hr;
  hr = setIntFilterParameterFromEditText(UF_HISTORY_SIZE,IDC_HISTORY_SIZE);
  if(FAILED(hr)) return hr;
  hr = setIntFilterParameterFromEditText(UF_PRECISION,IDC_PRECISION);
  return hr;
} 


void UtilityProperties::initialiseControls()
{
  InitCommonControls();

    // mode of operation
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_RESETCONTENT, 0, 0);
    //Add default option
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_ADDSTRING,	 0, (LPARAM)UF_FPS_TS);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING,  0, (LPARAM)UF_FPS_TS);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  1, (LPARAM)UF_FPS_ST);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  2, (LPARAM)UF_TIME);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  3, (LPARAM)UF_DATE_TIME);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  4, (LPARAM)UF_FPS_TS_DATE_TIME);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  5, (LPARAM)UF_FPS_ST_DATE_TIME);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  6, (LPARAM)UF_MEASURE_BITRATE);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  7, (LPARAM)UF_DETECT_UPSTREAM_SOURCE);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING,  8, (LPARAM)UF_NONE);
  SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SETMINVISIBLE, 9, 0);

    // Init UI
  setSpinBoxRange(IDC_SPIN1, 0, SHRT_MAX);
  setSpinBoxRange(IDC_SPIN2, 0, SHRT_MAX);
  setSpinBoxRange(IDC_SPIN3, 0, SHRT_MAX);
  setSpinBoxRange(IDC_SPIN4, 0, SHRT_MAX);
  setSpinBoxRange(IDC_SPIN5, 0, SHRT_MAX);
}

