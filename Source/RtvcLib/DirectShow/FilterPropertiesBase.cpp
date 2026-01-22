/** @file

MODULE				: FilterPropertiesBase

FILE NAME			: FilterPropertiesBase.cpp

DESCRIPTION			: Filter Property base class that has a pointer to the filter's ISettingInterface COM interface.
					Subclasses must then still implement the ReadSettings and OnApplyChanges to read and write the filter settings.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008, Meraka Institute
All rights reserved.
===========================================================================*/
#include <Windows.h>
#include <assert.h>

#include "CommonDefs.h"
#include "FilterPropertiesBase.h"


#ifndef BUFFER_SIZE
  #define BUFFER_SIZE 256
#endif


HRESULT FilterPropertiesBase::OnConnect(IUnknown *pUnk)
{
  if(pUnk == NULL)
  {
	return E_POINTER;
  }
  ASSERT(m_pSettingsInterface == NULL);

  return pUnk->QueryInterface(IID_ISettingsInterface, reinterpret_cast<void**>(&m_pSettingsInterface));
}


HRESULT FilterPropertiesBase::OnActivate(void)
{
  INITCOMMONCONTROLSEX icc;
  icc.dwSize = sizeof(INITCOMMONCONTROLSEX);
  icc.dwICC = ICC_BAR_CLASSES;
  if (InitCommonControlsEx(&icc) == FALSE)
  {
	return E_FAIL;
  }

  ASSERT(m_pSettingsInterface != NULL);

  return ReadSettings();
}


INT_PTR FilterPropertiesBase::OnReceiveMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
		//TODO: only SetDirty if the spin message was intercepted
  SetDirty();
		// Let the parent class handle the message.
  return CBasePropertyPage::OnReceiveMessage(hwnd,uMsg,wParam,lParam);
} 


HRESULT FilterPropertiesBase::OnDisconnect(void)
{
  if (m_pSettingsInterface)
  {
	m_pSettingsInterface->Release();
	m_pSettingsInterface = NULL;
  }
  return S_OK;
}


void FilterPropertiesBase::SetDirty()
{
  m_bDirty = TRUE;
  if (m_pPageSite)
  {
	m_pPageSite->OnStatusChange(PROPPAGESTATUS_DIRTY);
  }
}


void FilterPropertiesBase::setSpinBoxRange(int iSpinBoxId, short lower, short upper)
{
  long lResult = SendMessage(			    // returns LRESULT in lResult
			GetDlgItem(m_Dlg, iSpinBoxId),	    // handle to destination control
			(UINT) UDM_SETRANGE,			        // message ID
			(WPARAM) 0,						            // = 0; not used, must be zero
			(LPARAM) MAKELONG ( upper, lower) // = (LPARAM) MAKELONG ((short) nUpper, (short) nLower)
			);
}


HRESULT FilterPropertiesBase::setEditTextFromIntFilterParameter(const char* szParam, int iEditId)
{
  int nLength = 0;
  char szBuffer[BUFFER_SIZE];
  HRESULT hr = m_pSettingsInterface->GetParameter(szParam, sizeof(szBuffer), szBuffer, &nLength);
  if (SUCCEEDED(hr))
  {
	SetDlgItemText(m_Dlg, iEditId, szBuffer);
        return S_OK;
  }
  else
  {
	return E_FAIL;
  }
}


HRESULT FilterPropertiesBase::setComboTextFromFilterParameter(const char* szParam, int iCmbId)
{
  int nLength = 0;
  char szBuffer[BUFFER_SIZE];
  HRESULT hr = m_pSettingsInterface->GetParameter(szParam, sizeof(szBuffer), szBuffer, &nLength);
  if (SUCCEEDED(hr))
  {
      SendMessage(GetDlgItem(m_Dlg, iCmbId), CB_SELECTSTRING,  0, (LPARAM)szBuffer);
  }
  return hr;
}


HRESULT FilterPropertiesBase::setIntFilterParameterFromEditText(const char* szParam, int iEditId)
{
  int nLength = 0;
  char szBuffer[BUFFER_SIZE];
  nLength = GetDlgItemText(m_Dlg, iEditId, szBuffer, BUFFER_SIZE);
  return m_pSettingsInterface->SetParameter(szParam, szBuffer);
}


HRESULT FilterPropertiesBase::setCheckBoxFromBoolFilterParameter(const char* szParam, int iCheckBoxId)
{
  int nLength = 0;
  char szBuffer[BUFFER_SIZE];
  HRESULT hr = m_pSettingsInterface->GetParameter(szParam, sizeof(szBuffer), szBuffer, &nLength);
  if(SUCCEEDED(hr))
  {
    WPARAM wParam = (szBuffer[0]=='0') ? 0 : 1;
    long lResult = SendMessage(				      // returns LRESULT in lResult
				GetDlgItem(m_Dlg, iCheckBoxId),	// handle to destination control
				(UINT) BM_SETCHECK,		// message ID
				(WPARAM) wParam,		// = 0; not used, must be zero
				(LPARAM) 0			// = (LPARAM) MAKELONG ((short) nUpper, (short) nLower)
				);
    return S_OK;
  }
  else
  {
    return E_FAIL;
  }
}


HRESULT FilterPropertiesBase::setBoolFilterParameterFromCheckBox(const char* szParam, int iCheckBoxId)
{
  HRESULT hr(S_OK);
  int iCheck = SendMessage( GetDlgItem(m_Dlg, iCheckBoxId),	(UINT) BM_GETCHECK,	0, 0);
  if (iCheck == 0)
  {
    hr = m_pSettingsInterface->SetParameter(szParam, "false");
  }
  else
  {
    hr = m_pSettingsInterface->SetParameter(szParam, "true");
  }
  return hr;
}
