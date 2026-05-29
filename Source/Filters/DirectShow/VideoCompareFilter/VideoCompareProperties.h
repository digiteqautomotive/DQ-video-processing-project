/** @file

MODULE				: VideoCompareProperties

FILE NAME			: VideoCompareProperties.h

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik, All rights reserved.

===========================================================================*/
#pragma once

#include <DirectShow/CommonDefs.h>
#include <DirectShow/FilterPropertiesBase.h>
#include "GeneralUtils/Conversion.h"

#include "resource.h"
#define BUFFER_SIZE 256

//const int RADIO_BUTTON_IDS[] = {IDC_HORIZONTAL, IDC_VERTICAL};
const int RADIO_BTN_SYNC_IDS[] = {IDC_RAD_INPUT1, IDC_RAD_INPUT2, IDC_RAD_BOTH};


/** \ingroup DirectShowFilters Video Compare filter property dialog */
class VideoCompareProperties: public FilterPropertiesBase
{
public:

	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr) 
	{
		VideoCompareProperties *pNewObject = new VideoCompareProperties(pUnk);
		if (pNewObject == NULL) 
		{
			*pHr = E_OUTOFMEMORY;
		}
		return pNewObject;
	}

	VideoCompareProperties::VideoCompareProperties(IUnknown *pUnk) : 
	FilterPropertiesBase(NAME("Video Compare Properties"), pUnk, IDD_VIDEO_COMPARE_DIALOG, IDS_VM_CAPTION)
	{}


	HRESULT ReadSettings(void);

	HRESULT OnApplyChanges(void);
};

