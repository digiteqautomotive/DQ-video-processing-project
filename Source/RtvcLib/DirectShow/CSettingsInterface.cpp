/** @file

MODULE				: DirectShow

FILE NAME			: CSettingsInterface.cpp

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2017, CSIR
Copyright (c) 2025-2026, Digiteq automotive, programmed by Jaroslav Fojtik	All rights reserved.

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
===========================================================================*/
#include "stdafx.h"
#include "CSettingsInterface.h"

#include "VariantTools.h"
#include "wxdebug.h"

#include <sstream>

#include "GeneralUtils/StringUtil.h"	// compatiblitity for older MSVC


CSettingsInterface::CSettingsInterface(void): m_DirtyProps(false)
{
}


CSettingsInterface::~CSettingsInterface(void)
{
  for(std::list<ParameterBase*>::iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    delete(*it);
    *it = NULL;
  }
  m_vParams.clear();
}

void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, int* pAddr, int nDefaultValue, bool bReadOnly) //, std::vector<int> vAllowedValues)
{	  // Add the param to the map
  m_vParams.push_back(new ParameterInt(sParamName,oleName,pAddr,nDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, int* pAddr, int nDefaultValue, bool bReadOnly) //, std::vector<int> vAllowedValues)
{	  // Add the param to the map
  m_vParams.push_back(new ParameterInt(sParamName,NULL,pAddr,nDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, int* pAddr, bool bReadOnly) //, std::vector<int> vAllowedValues)
{	  // Add the param to the map
  if(pAddr)
      m_vParams.push_back(new ParameterInt(sParamName,oleName,pAddr,*pAddr,bReadOnly));
}

void CSettingsInterface::addParameter(const char *sParamName, unsigned* pAddr, unsigned uiDefaultValue, bool bReadOnly) //, std::vector<unsigned> vAllowedValues)
{	  // Add the param to the map
  m_vParams.push_back(new ParameterUnsigned(sParamName,NULL,pAddr,uiDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, unsigned* pAddr, unsigned uiDefaultValue, bool bReadOnly) //, std::vector<unsigned> vAllowedValues)
{	  // Add the param to the map
  m_vParams.push_back(new ParameterUnsigned(sParamName,oleName,pAddr,uiDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, unsigned* pAddr, bool bReadOnly)
{	  // Add the param to the map
  if(pAddr)
    m_vParams.push_back(new ParameterUnsigned(sParamName,oleName,pAddr,*pAddr,bReadOnly));
}

void CSettingsInterface::addParameter(const char *sParamName, std::string* pAddr, std::string sDefaultValue, bool bReadOnly) //, std::vector<std::string> vAllowedValues)
{  // Add the param to the map
  m_vParams.push_back(new ParameterString(sParamName,NULL,pAddr,sDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, std::string* pAddr, std::string sDefaultValue, bool bReadOnly) //, std::vector<std::string> vAllowedValues)
{  // Add the param to the map
  m_vParams.push_back(new ParameterString(sParamName,oleName,pAddr,sDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, std::string* pAddr, bool bReadOnly) //, std::vector<std::string> vAllowedValues)
{  // Add the param to the map
  if(pAddr)
    m_vParams.push_back(new ParameterString(sParamName,oleName,pAddr,*pAddr,bReadOnly));
}

void CSettingsInterface::addParameter(const char *sParamName, bool* pAddr, bool bDefaultValue, bool bReadOnly) //, std::vector<bool> vAllowedValues)
{  // Add the param to the map
  m_vParams.push_back(new ParameterBool(sParamName,NULL,pAddr,bDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, bool* pAddr, bool bDefaultValue, bool bReadOnly) //, std::vector<bool> vAllowedValues)
{  // Add the param to the map
  m_vParams.push_back(new ParameterBool(sParamName,oleName,pAddr,bDefaultValue,bReadOnly));
}

void CSettingsInterface::addParameter(const char *sParamName, double* pAddr, double dDefaultValue, bool bReadOnly) //, std::vector<double> vAllowedValues)
{	  // Add the param to the map
  m_vParams.push_back(new ParameterDouble(sParamName,NULL,pAddr,dDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, double* pAddr, double dDefaultValue, bool bReadOnly) //, std::vector<double> vAllowedValues)
{	  // Add the param to the map
  m_vParams.push_back(new ParameterDouble(sParamName,oleName,pAddr,dDefaultValue,bReadOnly));
}
void CSettingsInterface::addParameter(const char *sParamName, const OLECHAR *oleName, double* pAddr, bool bReadOnly) //, std::vector<double> vAllowedValues)
{	  // Add the param to the map
  if(pAddr)
      m_vParams.push_back(new ParameterDouble(sParamName,oleName,pAddr,*pAddr,bReadOnly));
}


STDMETHODIMP CSettingsInterface::GetParameter(const char* type, int nBufferSize, char* value, int* length)
{
  for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if(!strcmp(type,(*it)->Name.c_str()))
    {
      const std::string valS = (*it)->ToStr();
      if(valS.empty())
      {
        if(nBufferSize>0) *value=0;
        *length = 0;
      }
      else
      {
        *length = valS.length();
        if(*length>nBufferSize) *length=nBufferSize;
        strncpy(value,valS.c_str(),nBufferSize);
      }
      return S_OK;
    }
  }
 return E_FAIL;
}


std::string CSettingsInterface::GetParameter(const char* szParamName, bool *Ok)
{
  for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if(!strcmp(szParamName,(*it)->Name.c_str()))
    {
      if(Ok) *Ok = true;
      if((*it)->Type()==VPP_STRING) return (*it)->ToStr();	// Legal string could be empty.

      const std::string valS = (*it)->ToStr();
      if(valS.empty()) break;			// This causes to fail due to empty string.
      return valS;
    }
  }
  if(Ok) *Ok=false;
  return std::string();
}


STDMETHODIMP CSettingsInterface::SetParameter(const char* type, const char* value)
{
  for(std::list<ParameterBase*>::iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if(!strcmp(type,(*it)->Name.c_str()))
    {
      if((*it)->FromStr(value)) return S_OK;
      return E_FAIL;
    }
  }

  return E_FAIL;
}


STDMETHODIMP CSettingsInterface::GetParameterSettings(char* szResult, int nSize)
{
  std::stringstream sstream;

  // Iterate over all the different params and display their current values
  
 for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
 {
    if(*it==NULL) continue;
    sstream << "Parameter name: " << (*it)->Name << " Value: " << (*it)->ToStr() << "\r\n";
 }

  std::string sResult(sstream.str());

  if (sResult.length() < (size_t)nSize)
  {
    memcpy(szResult, sResult.c_str(), sResult.length());
    szResult[sResult.length()] = 0;
    return S_OK;
  }
	// Buffer is too small
  return E_FAIL;
}


bool CSettingsInterface::revertParameter(const char* type)
{
  for(std::list<ParameterBase*>::iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if(!strcmp(type,(*it)->Name.c_str()))
    {
      (*it)->RevertToPreviousValue();
      return true;
    }
  }

  return false;
}

///////////////////////////////////////////////////////////////////////////////////////


HRESULT CSettingsInterface::GetSizeMax(ULARGE_INTEGER *pcbSize)
{
  if(pcbSize==NULL) return E_POINTER;
  pcbSize->QuadPart = 0;
  for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if((*it)->ReadOnly) continue;
    pcbSize->QuadPart += (*it)->Size();
  }
  return S_OK;
}


HRESULT CSettingsInterface::Save(IStream *pStm, BOOL fClearDirty)
{
CheckPointer(pStm, E_INVALIDARG);
 for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if((*it)->ReadOnly) continue;
    HRESULT hr = (*it)->Save(pStm);
    if(FAILED(hr)) return hr;
  }
  return S_OK;
}


HRESULT CSettingsInterface::IsDirty(void)
{
  return m_DirtyProps ? S_OK : S_FALSE;
}


HRESULT CSettingsInterface::Load(IStream *pStm)
{
CheckPointer(pStm, E_INVALIDARG);
 for(std::list<ParameterBase*>::iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if((*it)->ReadOnly) continue;
    HRESULT hr = (*it)->Load(pStm);
    if(FAILED(hr)) return hr;
  }
  return S_OK;
}


//----------------- IPersistPropertyBag------------------


HRESULT CSettingsInterface::InitNew(void)
{
  for(std::list<ParameterBase*>::iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if(*it==NULL) continue;
    if((*it)->ReadOnly) continue;
    (*it)->RevertToDefaultValue();    
  }
  return S_OK;
}


HRESULT STDMETHODCALLTYPE CSettingsInterface::Load(__RPC__in_opt IPropertyBag *pPropBag, __RPC__in_opt IErrorLog *pErrorLog)
{
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;
  memset(&var,0,sizeof(var));
  for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if((*it)->OleName!=NULL && !(*it)->ReadOnly)
    {
      if(SUCCEEDED(pPropBag->Read((*it)->OleName,&var,NULL)))
      {
        (*it)->setVariant(var);
      }
      VariantClear(&var);
    }
  }
  return S_OK;
}


HRESULT STDMETHODCALLTYPE CSettingsInterface::Save(__RPC__in_opt IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties)
{
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;
  memset(&var,0,sizeof(var));
  for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if((*it)->OleName != NULL)
    {
      if((*it)->ReadOnly && !fSaveAllProperties) continue;
      (*it)->getVariant(var);
      pPropBag->Write((*it)->OleName,&var);
      VariantClear(&var);
    }
  }
  if(fClearDirty) m_DirtyProps=false;
  return S_OK;
}


//----------------- IPropertyBag------------------


HRESULT STDMETHODCALLTYPE CSettingsInterface::Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  VariantClear(pVar);
  for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if((*it)->OleName != NULL)
    {
      if(!oleicmp(pszPropName,(*it)->OleName))
      {
        (*it)->getVariant(*pVar);
        return S_OK;
      }
    }
  }
  return E_INVALIDARG;
}


HRESULT STDMETHODCALLTYPE CSettingsInterface::Write(LPCOLESTR pszPropName, VARIANT *pVar)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  VariantClear(pVar);
  for(std::list<ParameterBase*>::const_iterator it=m_vParams.begin(); it!=m_vParams.end(); ++it)
  {
    if((*it)->OleName != NULL)
    {
      if(!oleicmp(pszPropName,(*it)->OleName))
      {
        if((*it)->setVariant(*pVar)) return S_OK;
        return E_INVALIDARG;
      }
    }
  }
  return E_INVALIDARG;
}


///////////////////////////////////////////////////////////////////////////////////////


ParameterBase::ParameterBase(const char *NewName, const OLECHAR *NewOleName, bool NewReadOnly): Name(NewName), ReadOnly(NewReadOnly)
{
  if(NewOleName==NULL || *NewOleName==0)
      OleName = NULL;
  else
  {
      int len = sizeof(OLECHAR)*(olelen(NewOleName)+1);
      OleName = (OLECHAR*)CoTaskMemAlloc(len);
      if(OleName)
          memcpy(OleName,NewOleName,len);
  }
}


ParameterBase::~ParameterBase()
{
  if(OleName)
  {
    CoTaskMemFree(OleName);
    OleName = NULL;
  }
}


///////////////////////////////////////////////////////////////////////////////////////


ParameterInt::ParameterInt(const char *NewName, const OLECHAR *NewOleName, int *ValueAddr, int IniDefaultValue, bool NewReadOnly):
        ParameterBase(NewName,NewOleName,NewReadOnly),
	pValue(ValueAddr),
	DefaultValue(IniDefaultValue),
        PreviousValue(IniDefaultValue)
{
  if(pValue==NULL)
    pValue = &PreviousValue;
  else
   *pValue = IniDefaultValue;
}


std::string ParameterInt::ToStr(void) const
{
ASSERT(pValue!=NULL);
  return std::to_string(*pValue);
}


bool ParameterInt::FromStr(const char *str)
{
ASSERT(pValue!=NULL);
  if(str==NULL) return false; 
  PreviousValue = *pValue;
  *pValue = atoi(str);
return true;
}


void ParameterInt::getVariant(VARIANT &var) const
{
  var.vt = VT_INT;
  var.intVal = *pValue;
}


bool ParameterInt::setVariant(const VARIANT &var)
{
  return getFromVariant(*pValue,&var);

}


///////////////////////////////////////////////////////////////////////////////////////


ParameterUnsigned::ParameterUnsigned(const char *NewName, const OLECHAR *NewOleName, unsigned *ValueAddr, unsigned IniDefaultValue, bool NewReadOnly):
        ParameterBase(NewName,NewOleName,NewReadOnly),
	pValue(ValueAddr),
	DefaultValue(IniDefaultValue),
        PreviousValue(IniDefaultValue)
{
  if(pValue==NULL)
    pValue = &PreviousValue;
  else
    *pValue = IniDefaultValue;
}


std::string ParameterUnsigned::ToStr(void) const
{
ASSERT(pValue!=NULL);
  return std::to_string(*pValue);
}


bool ParameterUnsigned::FromStr(const char *str)
{
ASSERT(pValue!=NULL);
  if(str==NULL) return false; 
  PreviousValue = *pValue;
  *pValue = atoi(str);
return true;
}


void ParameterUnsigned::getVariant(VARIANT &var) const
{
  var.vt = VT_UINT;
  var.uintVal = *pValue;
}


bool ParameterUnsigned::setVariant(const VARIANT &var)
{
  return getFromVariant(*pValue,&var);
}


///////////////////////////////////////////////////////////////////////////////////////


ParameterBool::ParameterBool(const char *NewName, const OLECHAR *NewOleName, bool *ValueAddr, bool IniDefaultValue, bool NewReadOnly):
        ParameterBase(NewName,NewOleName,NewReadOnly),
	pValue(ValueAddr),
	DefaultValue(IniDefaultValue),
        PreviousValue(IniDefaultValue)
{
  if(pValue==NULL)
    pValue = &PreviousValue;
  else
    *pValue = IniDefaultValue;
}


std::string ParameterBool::ToStr(void) const
{
ASSERT(pValue!=NULL);
  return std::to_string(*pValue);
}


bool ParameterBool::FromStr(const char *str)
{
ASSERT(pValue!=NULL);
  if(str==NULL) return false; 
  PreviousValue = *pValue;
  if(!strcmp(str,"true")) *pValue=true;
  else if(!strcmp(str,"false")) *pValue=false;
  else *pValue = atoi(str);
return true;
}


HRESULT ParameterBool::Load(IStream *pStm)
{
char b;
HRESULT hr;
  ASSERT(pValue!=NULL);
  hr = pStm->Read(&b,1,NULL);
  if(SUCCEEDED(hr)) *pValue=b;
  return hr;
}


void ParameterBool::getVariant(VARIANT &var) const
{
  var.vt = VT_BOOL;
  var.boolVal = *pValue;
}


bool ParameterBool::setVariant(const VARIANT &var)
{
  return getFromVariant(*pValue,&var);
}

///////////////////////////////////////////////////////////////////////////////////////


ParameterDouble::ParameterDouble(const char *NewName, const OLECHAR *NewOleName, double *ValueAddr, double IniDefaultValue, double NewReadOnly):
        ParameterBase(NewName,NewOleName,NewReadOnly),
	pValue(ValueAddr),
	DefaultValue(IniDefaultValue),
        PreviousValue(IniDefaultValue)
{
  if(pValue==NULL)
    pValue = &PreviousValue;
  else
    *pValue = IniDefaultValue;
}


std::string ParameterDouble::ToStr(void) const
{
ASSERT(pValue!=NULL);
  return std::to_string(*pValue);
}


bool ParameterDouble::FromStr(const char *str)
{
ASSERT(pValue!=NULL);
  if(str==NULL) return false; 
  PreviousValue = *pValue;
  *pValue = atof(str);
return true;
}


void ParameterDouble::getVariant(VARIANT &var) const
{
  var.vt = VT_R8;
  var.dblVal = *pValue;
}


bool ParameterDouble::setVariant(const VARIANT &var)
{
  return getFromVariant(*pValue,&var);
}


///////////////////////////////////////////////////////////////////////////////////////


ParameterString::ParameterString(const char *NewName, const OLECHAR *NewOleName, std::string *ValueAddr, std::string IniDefaultValue, bool NewReadOnly):
        ParameterBase(NewName,NewOleName,NewReadOnly),
	pValue(ValueAddr),
	DefaultValue(IniDefaultValue),
        PreviousValue(IniDefaultValue)
{
  if(pValue==NULL)
    pValue = &PreviousValue;
  else
    *pValue = IniDefaultValue;
}


std::string ParameterString::ToStr(void) const
{
ASSERT(pValue!=NULL);
  return *pValue;
}


bool ParameterString::FromStr(const char *str)
{
ASSERT(pValue!=NULL);
  if(str==NULL) return false; 
  PreviousValue = *pValue;
  *pValue = str;
return true;
}


HRESULT ParameterString::Save(IStream *pStm) const
{
ASSERT(pValue!=NULL);
  if(pValue->empty())
  {
    static const OLECHAR olec = 0;
    return pStm->Write(&olec, sizeof(olec), NULL);    
  }
  return pStm->Write(pValue->c_str(), pValue->length()+1, NULL);
}


HRESULT ParameterString::Load(IStream *pStm)
{
char c=0;
HRESULT hr;
  ASSERT(pValue!=NULL);
  hr = pStm->Read(&c,1,NULL);
  if(FAILED(hr)) return hr;
  if(c==0)
  {
    pValue->clear();
    return hr;
  }
  while(c!=0)
  {
    *pValue += c;
    hr = pStm->Read(&c,1,NULL);
    if(FAILED(hr)) return hr;
  }
  return hr;
}


void ParameterString::getVariant(VARIANT &var) const
{
  if(pValue->empty())
  {
    var.vt = VT_NULL;
    return;
  }

  var.vt = VT_LPSTR;
  var.pcVal = (CHAR*)CoTaskMemAlloc(pValue->length()+1);
  if(var.pcVal!=NULL)
  {
    strcpy(var.pcVal,pValue->c_str());
  }
}


bool ParameterString::setVariant(const VARIANT &var)
{
  if(var.vt == VT_NULL)
  {
    pValue->clear();
    return true;
  }
  if(var.vt == VT_LPSTR)
  {
    *pValue = var.pcVal;
    return true;
  }
  return false;
}


///////////////////////////////////////////////////////////////////////////////////////


ParameterRefTime::ParameterRefTime(const char *NewName, const OLECHAR *NewOleName, long long *ValueAddr, long long IniDefaultValue, bool NewReadOnly):
        ParameterBase(NewName,NewOleName,NewReadOnly),
	pValue(ValueAddr),
	DefaultValue(IniDefaultValue),
        PreviousValue(IniDefaultValue)
{
  if(pValue==NULL)
    pValue = &PreviousValue;
  else
    *pValue = IniDefaultValue;
}


std::string ParameterRefTime::ToStr(void) const
{
ASSERT(pValue!=NULL);
  const double Val_sec = *pValue / 10000000.0;
  return std::to_string(Val_sec);
}


bool ParameterRefTime::FromStr(const char *str)
{
ASSERT(pValue!=NULL);
  if(str==NULL) return false; 
  PreviousValue = *pValue;
  const double Val_sec = atof(str);
  *pValue = Val_sec * 10000000.0;
return true;
}


void ParameterRefTime::getVariant(VARIANT &var) const
{
  var.vt = VT_I8;
  var.llVal = *pValue;
}


bool ParameterRefTime::setVariant(const VARIANT &var)
{
  return getFromVariant(*pValue,&var);
}

