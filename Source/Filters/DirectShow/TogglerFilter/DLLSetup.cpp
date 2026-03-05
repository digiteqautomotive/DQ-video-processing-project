/** @file

MODULE				: DLLSetup

FILE NAME			: DLLSetup.cpp

DESCRIPTION			: 
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026 Digiteq Automotive; designed by Jaroslav Fojtik		All rights reserved.
===========================================================================*/
#include "stdafx.h"
#include "TogglerFilter.h"
#include "TogglerProperties.h"
//////////////////////////////////////////////////////////////////////////
//###############################  Standard Filter DLL Code ###############################
static const WCHAR g_wszName[] = L"DQ VPP Toggle Filter";   /// A name for the filter 


// The next bunch of structures define information for the class factory.
AMOVIESETUP_FILTER FilterInfo =
{
  &CLSID_VPP_TogglerFilter,  // CLSID
  g_wszName,                  // Name
  MERIT_NORMAL,		      // Merit
  0,                          // Number of AMOVIESETUP_PIN structs
  NULL                        // Pin registration information.
};

CFactoryTemplate g_Templates[] = 
{
  { 
    g_wszName,                      // Name
    &CLSID_VPP_TogglerFilter,      // CLSID
    TogglerFilter::CreateInstance, // Method to create an instance of MyComponent
    NULL,                           // Initialization function
    &FilterInfo                     // Set-up information (for filters)
  },
  // This entry is for the property page.
  { 
    L"Toggle Properties",
    &CLSID_TogglerProperties,
    TogglerProperties::CreateInstance, 
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
