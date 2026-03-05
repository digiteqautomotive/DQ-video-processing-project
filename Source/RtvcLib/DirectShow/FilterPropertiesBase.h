/** @file

MODULE				: FilterPropertiesBase

FILE NAME			: FilterPropertiesBase.h

DESCRIPTION			: Filter Property base class that has a pointer to the filter's ISettingInterface COM interface.
					Subclasses must then still implement the ReadSettings and OnApplyChanges to read and write the filter settings.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008, Meraka Institute
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

#pragma warning(push)     // disable for this header only
#pragma warning(disable:4312)
// DirectShow
#include <streams.h>
#pragma warning(pop)      // restore original warning level

#include <Commctrl.h>

#include <DirectShow/SettingsInterface.h>


#define BUFFER_SIZE 256

/**
* Property dialog base class. This is the base classes for the property pages of filters that provide the ISettingsInterface COM interface.
* Sub classes must implement 
* - ReadSettings in which the filter settings need to be read and the dialog needs to be setup.
* - OnApplyChanges in which the dialog values must be stored in the filter.
* 
* From: http://msdn.microsoft.com/en-us/library/dd375010(VS.85).aspx Creating a Filter Property Page.
* - OnConnect is called when the client creates the property page. It sets the IUnknown pointer to the filter.
* - OnActivate is called when the dialog is created.
* - OnReceiveMessage is called when the dialog receives a window message.
* - OnApplyChanges is called when the user commits the property changes by clicking the OK or Apply button.
* - OnDisconnect is called when the user dismisses the property sheet.
*/
class FilterPropertiesBase: public CBasePropertyPage
{
public:
	HRESULT OnConnect(IUnknown *pUnk);

	HRESULT OnActivate(void);

	INT_PTR OnReceiveMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

	HRESULT OnApplyChanges(void) = 0;

	HRESULT OnDisconnect(void);

protected:

	FilterPropertiesBase::FilterPropertiesBase(char* szName, IUnknown *pUnk, int nDialogId, int nTitleId): 
		CBasePropertyPage(szName, pUnk, nDialogId, nTitleId),
		m_pSettingsInterface(NULL)
	{}

	virtual HRESULT ReadSettings() = 0;


	void SetDirty();

	void setSpinBoxRange(int iSpinBoxId, short lower, short upper);

	HRESULT setEditTextFromIntFilterParameter(const char* szParam, const int iEditId);

	HRESULT setComboTextFromFilterParameter(const char* szParam, const int iCmbId);

	HRESULT setIntFilterParameterFromEditText(const char* szParam, const int iEditId);

	HRESULT setCheckBoxFromBoolFilterParameter(const char* szParam, const int iCheckBoxId);

	HRESULT setBoolFilterParameterFromCheckBox(const char* szParam, const int iCheckBoxId);

	/// Pointer to the filter's custom interface.
	/// This pointer allows subclasses access to the filter for setting configuration.
	ISettingsInterface *m_pSettingsInterface;
};

