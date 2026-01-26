/** @file

MODULE				: UtilityProperties

FILE NAME			: UtilityProperties.h

DESCRIPTION			: Properties for FramerateDisplay filter.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2015, Meraka Institute
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

#include <DirectShow/FilterPropertiesBase.h>

#include "resource.h"

#define BUFFER_SIZE 256

#define UF_PARAM_X "x"
#define UF_PARAM_Y "y"
#define UF_ESTIMATED_VIDEO_BITRATE "estimated_videobitrate"
#define UF_ESTIMATED_FRAMERATE     "estimated_framerate"
#define UF_ESTIMATED_BITRATE       "estimated_bitrate"
#define UF_MODE "mode"
#define UF_PRECISION "precision"
#define UF_HISTORY_SIZE "history_size"
#define UF_FPS_TS "FPS Timestamp (TS)"
#define UF_FPS_ST "FPS Timestamp (ST)"
#define UF_TIME "Time"
#define UF_DATE_TIME "DateTime"
#define UF_FPS_TS_DATE_TIME "FPS TS + DateTime"
#define UF_FPS_ST_DATE_TIME "FPS ST + DateTime"
#define UF_MEASURE_BITRATE "Measure bitrate"
#define UF_DETECT_UPSTREAM_SOURCE "Detect upstream source"
#define UF_NONE "None"
#define UF_FONT_SIZE "Font Size"
#define UF_WRITE_ON_FRAME "Write on frame"


class UtilityProperties: public FilterPropertiesBase
{
public:

	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr) 
	{
		UtilityProperties *pNewObject = new UtilityProperties(pUnk);
		if (pNewObject == NULL) 
		{
			*pHr = E_OUTOFMEMORY;
		}
		return pNewObject;
	}

	UtilityProperties::UtilityProperties(IUnknown *pUnk) : 
		FilterPropertiesBase(NAME("Utility Properties"), pUnk, IDD_UTILITY_DIALOG, IDS_UTILITY_CAPTION)
	{}

	virtual HRESULT ReadSettings(void);
	virtual HRESULT UtilityProperties::OnApplyChanges(void);
	virtual void initialiseControls(void);

private:

	unsigned m_uiX;
	unsigned m_uiY;
};

