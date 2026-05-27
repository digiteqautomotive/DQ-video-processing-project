/** @file

MODULE:			VirtualMicProperties

FILE NAME:		VirtualMicProperties.cpp

DESCRIPTION: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#pragma once

#include "VirtualMicProperties.h"
#include "VirtualMic.h"

#include "resource.h"


CUnknown * WINAPI VirtualMicProperties::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr) 
{
  ASSERT( pHr != NULL );
  VirtualMicProperties *pNewObject = new VirtualMicProperties(pUnk);
  if(pNewObject == NULL) 
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pNewObject;
}


VirtualMicProperties::VirtualMicProperties(IUnknown *pUnk): 
	        CBasePropertyPage(NAME("Virtual microphone properties"), pUnk, IDD_VIRTUAL_MIC_DIALOG, IDS_VMIC_CAPTION),
		m_piConfig(NULL)
{
}


// This routine is called when the property page is connected to the filter,
// but before the property page is displayed. There is an implicit
// design assumption that while we're connected the filter values being
// manipulated through the property pages are not simultaneously being
// manipulated via any of the filter's other interfaces.
HRESULT VirtualMicProperties::OnConnect( IUnknown* piUnknown )
{
  ASSERT( piUnknown != NULL );

  HRESULT hResult;
  if(piUnknown == NULL)
      hResult = VFW_E_NEED_OWNER;
  else
  {
        // We should receive pointer to KS proxy for our driver.
        // Make sure that we got what we expect
    hResult = piUnknown->QueryInterface(IID_ISettingsInterface, reinterpret_cast<void**>(&m_piConfig));
    if(FAILED(hResult))
    {
//        DbgLog((LOG_ERROR | LOG_TRACE, 0,
//                _T( "CBallProxyPluginPage::OnConnect: owner doesn't support IBallProxySampleConfig" ) ));
    }
  }

//  DbgLog(( LOG_TRACE, 5, _T( "CBallProxyPluginPage::OnConnect: return 0x%X" ), hResult ));
  return hResult;
}


VirtualMicProperties::~VirtualMicProperties()
{
  if(m_piConfig)
  {
    m_piConfig->Release();
    m_piConfig = NULL;
  }
}


HRESULT VirtualMicProperties::ReadSettings(void)
{
  SendMessage(GetDlgItem(m_Dlg,IDC_SPIN1), (UINT)UDM_SETRANGE32, 0, 2);

  if(m_piConfig)
  {     
    char szBuffer[BUFFER_SIZE];
    int nLength = 0;
    if(SUCCEEDED(m_piConfig->GetParameter(FILTER_PARAM_PATTERN,sizeof(szBuffer), szBuffer, &nLength)))
      if(nLength>0)
      {
        SetDlgItemText(m_Dlg, IDC_EDIT_PATTERN, szBuffer);
      }
  }
  
  return S_OK;
}


HRESULT VirtualMicProperties::OnApplyChanges(void)
{
  if(m_piConfig==NULL) return E_UNEXPECTED;
  int nLength = 0;
  char szBuffer[BUFFER_SIZE];
  nLength = GetDlgItemText(m_Dlg, IDC_EDIT_PATTERN, szBuffer, BUFFER_SIZE);
  return m_piConfig->SetParameter(FILTER_PARAM_PATTERN, szBuffer);
}


HRESULT VirtualMicProperties::OnActivate(void)
{
  INITCOMMONCONTROLSEX icc;
  icc.dwSize = sizeof(INITCOMMONCONTROLSEX);
  icc.dwICC = ICC_BAR_CLASSES;
  if (InitCommonControlsEx(&icc) == FALSE)
  {
	return E_FAIL;
  }

//  ASSERT(m_pSettingsInterface != NULL);

  return ReadSettings();
}


INT_PTR VirtualMicProperties::OnReceiveMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
		//TODO: only SetDirty if the spin message was intercepted
  SetDirty();
		// Let the parent class handle the message.
  return CBasePropertyPage::OnReceiveMessage(hwnd,uMsg,wParam,lParam);
}


void VirtualMicProperties::SetDirty(void)
{
  m_bDirty = TRUE;
  if (m_pPageSite)
  {
	m_pPageSite->OnStatusChange(PROPPAGESTATUS_DIRTY);
  }
}
