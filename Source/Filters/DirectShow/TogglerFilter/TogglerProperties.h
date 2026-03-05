/** @file

MODULE				: TogglerProperties

FILE NAME			: TogglerProperties.h

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive; designed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#pragma once

#include "DirectShow/CommonDefs.h"
#include "DirectShow/FilterPropertiesBase.h"
#include "../FilterParameterStringConstants.h"
#include "GeneralUtils/StringUtil.h"

#include "resource.h"



class TogglerProperties: public FilterPropertiesBase
{
public:
  static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr);

  TogglerProperties(IUnknown *pUnk);

  HRESULT ReadSettings();

  HRESULT OnApplyChanges(void);

private:
  void initialiseControls();

};
