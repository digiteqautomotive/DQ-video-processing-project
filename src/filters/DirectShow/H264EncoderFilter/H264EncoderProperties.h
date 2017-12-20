/** @file

MODULE				: H264EncoderProperties

FILE NAME			: H264EncoderProperties.h

DESCRIPTION			: Properties for H264 Encoder filter.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2014, Meraka Institute
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the Meraka Institute nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

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
#include <artist/Util/Conversion.h>
#include <cassert>
#include <climits>
#include "resource.h"

#define BUFFER_SIZE 256

const int RADIO_BUTTON_IDS[] = { IDC_RADIO_VPP, IDC_RADIO_H264, IDC_RADIO_AVC1 };

class H264EncoderProperties : public FilterPropertiesBase
{
public:

  static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr) 
  {
    H264EncoderProperties *pNewObject = new H264EncoderProperties(pUnk);
    if (pNewObject == NULL) 
    {
      *pHr = E_OUTOFMEMORY;
    }
    return pNewObject;
  }

  H264EncoderProperties::H264EncoderProperties(IUnknown *pUnk) : 
    FilterPropertiesBase(NAME("H264 Properties"), pUnk, IDD_H264PROP_DIALOG, IDD_H264PROP_CAPTION)
  {;}

  BOOL OnReceiveMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
  {
    // Let the parent class handle the message.
    return FilterPropertiesBase::OnReceiveMessage(hwnd,uMsg,wParam,lParam);
  }

  HRESULT ReadSettings()
  {
    initialiseControls();

    // Mode of operation
    HRESULT hr = setComboTextFromFilterParameter(CODEC_PARAM_MODE_OF_OPERATION, IDC_CMB_MODE_OF_OPERATION);
    
    // frame bit limit
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_FRAME_BIT_LIMIT, IDC_EDIT_BITRATE_LIMIT);
    if (FAILED(hr)) return hr;

    // Quality
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_QUALITY, IDC_EDIT_QUALITY);
    if (FAILED(hr)) return hr;

    // I-Frame period
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_IFRAME_PERIOD, IDC_EDIT_IFRAME_PERIOD);
    if (FAILED(hr)) return hr;

    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_I_PICTURE_MULTIPLIER, IDC_EDIT_IPICTURE_MULT);
    if (FAILED(hr)) return hr;

    // notify
    hr = setCheckBoxFromBoolFilterParameter(CODEC_PARAM_NOTIFYONIFRAME, IDC_CHECK_NOTIFY);
    assert(SUCCEEDED(hr));

    // max bits per frame
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_MAX_BITS_PER_FRAME, IDC_EDIT_MAX_BITS_PER_FRAME);
    if (FAILED(hr)) return hr;

    // num control frames
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_NUM_RATE_CONTROL_FRAMES, IDC_EDIT_NUM_RATE_CONTROL_FRAMES);
    if (FAILED(hr)) return hr;

    // max dist
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_MAX_DISTORTION, IDC_EDIT_MAX_DIST);
    if (FAILED(hr)) return hr;

    // DMAX
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_IPICTURE_DMAX_MULTIPLIER, IDC_EDIT_IPICTURE_DMAX_MULT);
    if (FAILED(hr)) return hr;
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_IPICTURE_DMAX_FRACTION, IDC_EDIT_IPICTURE_DMAX_FRAC);
    if (FAILED(hr)) return hr;

    // intra QP
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_MINIMUM_INTRA_QP, IDC_EDIT_MIN_INTRA_QP);
    if (FAILED(hr)) return hr;

    // inter QP
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_MINIMUM_INTER_QP, IDC_EDIT_MIN_INTER_QP);
    if (FAILED(hr)) return hr;

    // rate overshoot percent
    hr = setEditTextFromIntFilterParameter(CODEC_PARAM_RATE_OVERSHOOT_PERCENT, IDC_EDIT_RATE_OVERSHOOT_PERCENT);
    if (FAILED(hr)) return hr;

    // H264 type
    int nLength = 0;
    char szBuffer[BUFFER_SIZE];
    hr = m_pSettingsInterface->GetParameter(FILTER_PARAM_H264_OUTPUT_TYPE, sizeof(szBuffer), szBuffer, &nLength);
    if (SUCCEEDED(hr))
    {
      int nH264Type = atoi(szBuffer);
      int nRadioID = RADIO_BUTTON_IDS[nH264Type];
      long lResult = SendMessage(				// returns LRESULT in lResult
        GetDlgItem(m_Dlg, nRadioID),		// handle to destination control
        (UINT)BM_SETCHECK,					// message ID
        (WPARAM)1,							// = 0; not used, must be zero
        (LPARAM)0							// = (LPARAM) MAKELONG ((short) nUpper, (short) nLower)
        );
      return S_OK;
    }
    else
    {
      return E_FAIL;
    }

    return hr;
  }

  HRESULT OnApplyChanges(void)
  {
    // mode of operation
    HRESULT hr = setIntFilterParameterFromEditText(CODEC_PARAM_MODE_OF_OPERATION, IDC_CMB_MODE_OF_OPERATION);
    if (FAILED(hr)) return hr;

    // frame bit limit
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_FRAME_BIT_LIMIT, IDC_EDIT_BITRATE_LIMIT);
    if (FAILED(hr)) return hr;

    // quality
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_QUALITY, IDC_EDIT_QUALITY);
    if (FAILED(hr)) return hr;

    // i-frame period
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_IFRAME_PERIOD, IDC_EDIT_IFRAME_PERIOD);
    if (FAILED(hr)) return hr;

    // notify
    hr = setBoolFilterParameterFromCheckBox(CODEC_PARAM_NOTIFYONIFRAME, IDC_CHECK_NOTIFY);
    if (FAILED(hr)) return hr;

    // use MS H264 decoder
    hr = setBoolFilterParameterFromCheckBox(FILTER_PARAM_USE_MS_H264, IDC_USE_MS_H264);

    // max bits per frame
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_MAX_BITS_PER_FRAME, IDC_EDIT_MAX_BITS_PER_FRAME);
    if (FAILED(hr)) return hr;

    // num control frames
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_NUM_RATE_CONTROL_FRAMES, IDC_EDIT_NUM_RATE_CONTROL_FRAMES);
    if (FAILED(hr)) return hr;

    // max dist
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_MAX_DISTORTION, IDC_EDIT_MAX_DIST);
    if (FAILED(hr)) return hr;

    // DMAX
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_IPICTURE_DMAX_MULTIPLIER, IDC_EDIT_IPICTURE_DMAX_MULT);
    if (FAILED(hr)) return hr;
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_IPICTURE_DMAX_FRACTION, IDC_EDIT_IPICTURE_DMAX_FRAC);
    if (FAILED(hr)) return hr;

    // intra QP
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_MINIMUM_INTRA_QP, IDC_EDIT_MIN_INTRA_QP);
    if (FAILED(hr)) return hr;

    // inter QP
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_MINIMUM_INTER_QP, IDC_EDIT_MIN_INTER_QP);
    if (FAILED(hr)) return hr;

    // rate overshoot percent
    hr = setIntFilterParameterFromEditText(CODEC_PARAM_RATE_OVERSHOOT_PERCENT, IDC_EDIT_RATE_OVERSHOOT_PERCENT);
    if (FAILED(hr)) return hr;

    for (int i = 0; i <= 2; ++i)
    {
      int nRadioID = RADIO_BUTTON_IDS[i];
      int iCheck = SendMessage(GetDlgItem(m_Dlg, nRadioID), (UINT)BM_GETCHECK, 0, 0);
      if (iCheck != 0)
      {
        std::string sID = artist::toString(i);
        HRESULT hr = m_pSettingsInterface->SetParameter(FILTER_PARAM_H264_OUTPUT_TYPE, sID.c_str());
        break;
      }
    }
    return hr;
  } 

private:

  void initialiseControls()
  {
    InitCommonControls();

    // mode of operation
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE_OF_OPERATION), CB_RESETCONTENT, 0, 0);
    //Add default option
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE_OF_OPERATION), CB_ADDSTRING,	    0, (LPARAM)"0");
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE_OF_OPERATION), CB_SELECTSTRING,  0, (LPARAM)"0");
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE_OF_OPERATION), CB_INSERTSTRING,  1, (LPARAM)"1");
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE_OF_OPERATION), CB_INSERTSTRING,  2, (LPARAM)"2");
    SendMessage(GetDlgItem(m_Dlg, IDC_CMB_MODE_OF_OPERATION), CB_SETMINVISIBLE, 3, 0);

    // set spin box ranges
    // Frame bit limit
    setSpinBoxRange(IDC_SPIN1, 0, UINT_MAX);
    // I-frame period
    setSpinBoxRange(IDC_SPIN2, 0, UINT_MAX);
    // Quality
    setSpinBoxRange(IDC_SPIN3, 0, D_MAX_QUALITY_H264);
    setSpinBoxRange(IDC_SPIN4, 0, INT_MAX);
    setSpinBoxRange(IDC_SPIN5, 0, 9999); 
    setSpinBoxRange(IDC_SPIN6, 0, INT_MAX);
    // DMAX
    setSpinBoxRange(IDC_SPIN7, 0, 9);
    setSpinBoxRange(IDC_SPIN8, 0, 9);
    // intra QP
    setSpinBoxRange(IDC_SPIN9, 0, D_MAX_QUALITY_H264);
    // inter
    setSpinBoxRange(IDC_SPIN10, 0, D_MAX_QUALITY_H264);
    // rate overshoot percent
    setSpinBoxRange(IDC_SPIN11, 0, 1000);
  }
};

