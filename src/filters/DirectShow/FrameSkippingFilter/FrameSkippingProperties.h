/** @file

MODULE				: FrameSkippingProperties

FILE NAME			: FrameSkippingProperties.h

DESCRIPTION			: Properties for FrameSkipping filter.

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
#pragma once
#include <DirectShowExt/FilterPropertiesBase.h>
#include <DirectShowExt/FilterParameterStringConstants.h>
#include <climits>
#include <sstream>
#include <string>
#include "resource.h"

#define BUFFER_SIZE 256

/**
 * \ingroup DirectShowFilters
 * FrameSkipping Filter Property Dialog
 */
class FrameSkippingProperties : public FilterPropertiesBase
{
public:

  static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
  {
    FrameSkippingProperties *pNewObject = new FrameSkippingProperties(pUnk);
    if (pNewObject == NULL)
    {
      *pHr = E_OUTOFMEMORY;
    }
    return pNewObject;
  }

  FrameSkippingProperties::FrameSkippingProperties(IUnknown *pUnk) :
    FilterPropertiesBase(NAME("Frame Skipping Properties"), pUnk, IDD_FRAME_SKIP_DIALOG, IDS_FRAME_SKIP_CAPTION)
  {
    ;
  }

  BOOL OnReceiveMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
  {
    // Let the parent class handle the message.
    return FilterPropertiesBase::OnReceiveMessage(hwnd, uMsg, wParam, lParam);
  }

  HRESULT ReadSettings()
  {

    initialiseControls();


    int nLength = 0;
    char szBuffer[BUFFER_SIZE];

    // Mode of operation
    HRESULT hr = m_pSettingsInterface->GetParameter(FILTER_PARAM_MODE, sizeof(szBuffer), szBuffer, &nLength);
    if (SUCCEEDED(hr))
    {
      unsigned uiMode = atoi(szBuffer);
      switch (uiMode)
      {
        case 0:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING, 0, (LPARAM)FILTER_PARAM_SKIP_X_EVERY_Y);
          break;
        }
        case 1:
        {
          SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING, 0, (LPARAM)FILTER_PARAM_TARGET_RATE_BASED);
          break;
        }
      }
    }
    else
    {
      return E_FAIL;
    }


    short lower = 0;
    short upper = SHRT_MAX;

    setSpinBoxRange(IDC_SPIN1, lower, upper);
    setSpinBoxRange(IDC_SPIN2, lower, upper);
    setSpinBoxRange(IDC_SPIN3, lower, upper);

    hr = setEditTextFromIntFilterParameter(FILTER_PARAM_SKIP_FRAME, IDC_EDIT_SKIP_FRAME_NUMBER);
    if (FAILED(hr))
    {
      return hr;
    }

    hr = setEditTextFromIntFilterParameter(FILTER_PARAM_TOTAL_FRAMES, IDC_EDIT_SKIP_FRAME_TOTAL);
    if (FAILED(hr))
    {
      return hr;
    }

    hr = setEditTextFromIntFilterParameter(FILTER_PARAM_TARGET_FRAMERATE, IDC_EDIT_TARGET_FRAMERATE);

    return hr;
  }

  HRESULT OnApplyChanges(void)
  {

    int nLength = 0;
    char szBuffer[BUFFER_SIZE];

    // mode of operation
    int index = ComboBox_GetCurSel(GetDlgItem(m_Dlg, IDC_CMB_MODE));
    ASSERT(index != CB_ERR);
    _itoa(index, szBuffer, 10);
    m_pSettingsInterface->SetParameter(FILTER_PARAM_MODE, szBuffer);


    HRESULT hr = setIntFilterParameterFromEditText(FILTER_PARAM_SKIP_FRAME, IDC_EDIT_SKIP_FRAME_NUMBER);
    if (FAILED(hr)) return hr;

    hr = setIntFilterParameterFromEditText(FILTER_PARAM_TOTAL_FRAMES, IDC_EDIT_SKIP_FRAME_TOTAL);
    if (FAILED(hr)) return hr;
    hr = setIntFilterParameterFromEditText(FILTER_PARAM_TARGET_FRAMERATE, IDC_EDIT_TARGET_FRAMERATE);

    return hr;
  }


  void initialiseControls()
  {
    InitCommonControls();

    // mode of operation
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_RESETCONTENT, 0, 0);
    //Add default option
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_ADDSTRING, 0, (LPARAM)"Skip x every y");
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SELECTSTRING, 0, (LPARAM)"Skip x every y");
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_INSERTSTRING, 1, (LPARAM)"Target Fps based");
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE), CB_SETMINVISIBLE, 9, 0);

    short lower = 0;
    short upper = SHRT_MAX;

    // Init UI
    long lResult = SendMessage(			// returns LRESULT in lResult
      GetDlgItem(m_Dlg, IDC_SPIN4),	// handle to destination control
      (UINT)UDM_SETRANGE,			// message ID
      (WPARAM)0,						// = 0; not used, must be zero
      (LPARAM)MAKELONG(upper, lower)      // = (LPARAM) MAKELONG ((short) nUpper, (short) nLower)
      );
    lResult = SendMessage(			// returns LRESULT in lResult
      GetDlgItem(m_Dlg, IDC_SPIN5),	// handle to destination control
      (UINT)UDM_SETRANGE,			// message ID
      (WPARAM)0,						// = 0; not used, must be zero
      (LPARAM)MAKELONG(upper, lower)      // = (LPARAM) MAKELONG ((short) nUpper, (short) nLower)
      );
  }

};
