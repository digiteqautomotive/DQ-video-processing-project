/** @file

MODULE:			SkewingFilter

FILE NAME:		SkewingFilter.cpp

DESCRIPTION:		This filter skips a specified number of frames depending on the parameter denoted by "skipFrame"

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2011, CSIR All rights reserved.
Copyright (c) 2025, Jaroslav Fojtik

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

==========================================================================*/
#include "stdafx.h"

#include "SkewingFilter.h"
#include "GeneralUtils/StringUtil.h"
#include "DirectShow/StatusInterface.h"

#include "../VariantTools.cpp"


SkewingFilter::SkewingFilter(LPUNKNOWN pUnk, HRESULT *pHr): 
    CTransInPlaceFilter(NAME("CSIR RTVC Skewing Filter"), pUnk, CLSID_VPP_SkewingFilter, pHr, false),
    seenFirstFrame(false),
    previousTimestamp(0),
    add_time(0),
    m_DirtyProps(true)
{
  // Init parameters
	initParameters();
}


SkewingFilter::~SkewingFilter()
{;}


CUnknown * WINAPI SkewingFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  SkewingFilter *pFilter = new SkewingFilter(pUnk, pHr);
  if (pFilter== NULL) 
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pFilter;
}


STDMETHODIMP SkewingFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == (IID_ISettingsInterface))	  return GetInterface((ISettingsInterface*) this, ppv);
  if(riid == (IID_IStatusInterface))	  return GetInterface((IStatusInterface*) this, ppv);
  if(riid == (IID_ISpecifyPropertyPages)) return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  if(riid == IID_IPersistPropertyBag)	  return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPropertyBag)		  return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_IPersistStream)	  return GetInterface((IPersistStream*)this, ppv);

  return CTransInPlaceFilter::NonDelegatingQueryInterface(riid, ppv);
}


HRESULT SkewingFilter::Transform(IMediaSample *pSample)
{
  if(pSample)
  { 
    REFERENCE_TIME tStart;
    REFERENCE_TIME tStop;
    REFERENCE_TIME newStart;
    REFERENCE_TIME newStop;

    //int add_time = 100; // parameter to define how many frames to skip

    HRESULT hr = pSample->GetTime(&tStart, &tStop); //get the current time now
       
    REFERENCE_TIME tDiff = tStart - previousTimestamp;
    // Make sure timestamps are increasing
    if (tDiff > 0)
    {           
        //newStart = tStart + (add_time*1000000); //add time to start time in multiples of 1000 000
        //newStop  = tStop  + (add_time*1000000); //add time to stop time in multiples of 1000 000

      newStart = tStart + (add_time); 
      newStop  = tStop  + (add_time);
       
      hr = pSample->SetTime(&newStart, &newStop); //get the current time now
      previousTimestamp = tStart;
    }
  }
  return S_OK;
}


HRESULT SkewingFilter::CheckInputType(const CMediaType *mtIn)
{
  if(mtIn==NULL) return E_INVALIDARG;

	// Check the major type.
  if(mtIn->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  if(mtIn->formattype!=FORMAT_VideoInfo && mtIn->formattype!=FORMAT_VideoInfo2)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

	// Adding advert media type to this method
/*
  if(mtIn->subtype != MEDIASUBTYPE_RGB24 && mtIn->subtype != MEDIASUBTYPE_RGB32 && mtIn->subtype != MEDIASUBTYPE_ARGB32)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }
*/

  return S_OK;
}


HRESULT SkewingFilter::Run(REFERENCE_TIME tStart)
{
  return CTransInPlaceFilter::Run(tStart);
}


HRESULT SkewingFilter::Stop(void)
{
  add_time = 0;
  seenFirstFrame = false;
  previousTimestamp = 0;

  return CTransInPlaceFilter::Stop();
}


HRESULT SkewingFilter::GetPages(CAUUID *pPages)		// For the Skewing Property Page
{
  if(pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if(pPages->pElems == NULL) 
  {
    return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_SkewingProperties;
  return S_OK;
}


//----------------- IPersistPropertyBag------------------


HRESULT SkewingFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_SkewingFilter;
  return S_OK;
}


HRESULT SkewingFilter::InitNew(void)
{
  add_time = 0;
  return S_OK;
}


#define olestr(s) sstr(s)
#if defined(_WIN32) && !defined(OLE2ANSI)
 #define olecmp(__str1,__str2) wcscmp(__str1,__str2)
 #define oleicmp(__str1,__str2) wcsicmp(__str1,__str2)
 #define sstr(s) L###s
#else
 #define olecmp(__str1,__str2) strcmp(__str1,__str2)
 #define oleicmp(__str1,__str2) stricmp(__str1,__str2)
 #define sstr(s) #s
#endif

OLECHAR OLE_PARAM_TIME_SKEW[] = olestr("TIME SKEW");


HRESULT SkewingFilter::Load(IPropertyBag *pPropBag, IErrorLog *pErrorLog)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  memset(&var,0,sizeof(var));

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_TIME_SKEW));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TIME_SKEW,sizeof(OLE_PARAM_TIME_SKEW));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      IntFromVariant(add_time,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  VariantClear(&var);
return S_OK;
}


HRESULT SkewingFilter::Save(IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_TIME_SKEW));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TIME_SKEW,sizeof(OLE_PARAM_TIME_SKEW));
    var.vt = VT_INT;
    var.intVal = add_time;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

return S_OK;
}


//----------------- IPropertyBag------------------


HRESULT SkewingFilter::Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  VariantClear(pVar);

  if(oleicmp(pszPropName, OLE_PARAM_TIME_SKEW) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = add_time;
    return S_OK;
  }

  return E_INVALIDARG;
}


HRESULT SkewingFilter::Write(LPCOLESTR pszPropName, VARIANT *pVar)
{
CheckPointer(pszPropName, E_INVALIDARG);
CheckPointer(pVar, E_INVALIDARG);
  if(oleicmp(pszPropName, OLE_PARAM_TIME_SKEW) == 0)
  {   
    if(IntFromVariant(add_time,pVar)) return S_OK;
  }
  return E_INVALIDARG;
}


//----------------- IPersistStream------------------


HRESULT SkewingFilter::GetSizeMax(ULARGE_INTEGER *pcbSize)
{
CheckPointer(pcbSize, E_INVALIDARG);
  pcbSize->QuadPart = sizeof(add_time);
  return S_OK;
}


HRESULT SkewingFilter::IsDirty(void)
{
  return m_DirtyProps ? S_OK : S_FALSE;
}


HRESULT SkewingFilter::Load(IStream *pStm)
{
ULONG pcbRead;
int iadd_time;
HRESULT hr;
  ASSERT(sizeof(iadd_time)==sizeof(add_time));
  CheckPointer(pStm, E_INVALIDARG);

  if(FAILED(hr=pStm->Read(&iadd_time, sizeof(iadd_time), &pcbRead))) return hr;
  if(add_time!=iadd_time)
  {
    add_time = iadd_time;
    m_DirtyProps = true;
  }
return S_OK;
}


HRESULT SkewingFilter::Save(IStream *pStm, BOOL fClearDirty)
{
HRESULT hr;
CheckPointer(pStm, E_INVALIDARG);
  if(FAILED(hr=pStm->Write(&add_time, sizeof(add_time), NULL))) return hr;
  if(fClearDirty) m_DirtyProps=false;
return S_OK;
}
