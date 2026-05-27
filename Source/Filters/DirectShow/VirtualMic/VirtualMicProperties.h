/** @file

MODULE				: VirtualMicProperties

FILE NAME			: VirtualMicProperties.h

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#pragma once

#include <DirectShow/CommonDefs.h>
#include <DirectShow/FilterPropertiesBase.h>
#include "GeneralUtils/StringUtil.h"


/** \ingroup DirectShowFilters
 * Audio mixing filter property dialog */
class VirtualMicProperties: public CBasePropertyPage
{
public:

	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr);

	VirtualMicProperties(IUnknown *pUnk);
        virtual ~VirtualMicProperties();

        virtual HRESULT OnConnect(IUnknown* piUnknown);
        virtual HRESULT OnActivate(void);
        INT_PTR OnReceiveMessage(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
        void SetDirty(void);

	virtual HRESULT ReadSettings(void);
	virtual HRESULT OnApplyChanges(void);

	ISettingsInterface *m_piConfig;
};

