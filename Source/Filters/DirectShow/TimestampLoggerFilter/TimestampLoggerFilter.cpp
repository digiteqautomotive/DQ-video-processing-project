/** @file

MODULE:		TimestampLoggerFilter

FILE NAME:	TimestampLoggerFilter.cpp

DESCRIPTION:
                     
LICENSE:	Software License Agreement (BSD License)

Copyright (c) 2008-2012, CSIR
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

//**** Adding the TimingFilter
#include "TimestampLoggerFilter.h"
#include "GeneralUtils/StringUtil.h"

//To calculate the time now
#include <iostream>
#include <time.h>
#include <ctime>
#include <process.h>


TimestampLoggerFilter::TimestampLoggerFilter(LPUNKNOWN pUnk, HRESULT *pHr):
		CTransInPlaceFilter(NAME("DQ VPP Timestamp Logger"), pUnk, CLSID_VPP_TimeStampLoggerFilter, pHr, false),
                ThreadStatus(THREAD_NOT_INITIALISED),
		WriteThread(0)
{
  MsgWakeUp = CreateEvent(NULL, TRUE, FALSE, NULL);  
}


TimestampLoggerFilter::~TimestampLoggerFilter()
{
#ifdef _WIN32
  if(ThreadStatus == THREAD_RUNNING)
  {
    ThreadStatus = THREAD_STOPPING;
    //if(StopEvent) SetEvent(StopEvent);
    if(MsgWakeUp) SetEvent(MsgWakeUp);
    if(WriteThread)
    {
      if(WAIT_TIMEOUT==WaitForSingleObject(WriteThread,1000)) // Wait for thread to be finished WIN.
      {     
        TerminateThread(WriteThread,-1);		// Kill thread, that gets locked.
      }
    }
  }
  if(WriteThread) {CloseHandle(WriteThread);WriteThread=0;}
  if(MsgWakeUp) CloseHandle(MsgWakeUp);
  if(m_outStream.is_open()) m_outStream.close();
  while(!m_vTimeInfo.empty())
  {
    TimeInfo *TI = m_vTimeInfo.front();
    m_vTimeInfo.pop_front();
    if(TI) delete TI;
  }
  //if(StopEvent) CloseHandle(StopEvent);
#else
  StopEvent = 1;
  pthread_cond_signal(&MsgWakeUp);
  pthread_join(WriteThread, NULL);	// Wait for thread to be finished LNX.
#endif
}


CUnknown * WINAPI TimestampLoggerFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  TimestampLoggerFilter *pFilter = new TimestampLoggerFilter(pUnk, pHr);
  if(pHr) *pHr = (pFilter==NULL) ? E_OUTOFMEMORY : S_OK;
  return pFilter;
}


THREAD_DECL TimestampLoggerFilter::WorkerThread(void* arg)
{
DWORD waitobj;
  TimestampLoggerFilter *Filter = (TimestampLoggerFilter*)arg;
 // HRESULT hr = CoInitializeEx(0, COINIT_MULTITHREADED|COINIT_DISABLE_OLE1DDE);  // No need for OLE in this thread, no COM call is used here.

  Filter->ThreadStatus = THREAD_RUNNING;
  while(Filter->ThreadStatus == THREAD_RUNNING)
  {
    DWORD waitobj  = WaitForSingleObject(Filter->MsgWakeUp,INFINITE);
    if(Filter->ThreadStatus != THREAD_RUNNING) break;
    ResetEvent(Filter->MsgWakeUp);

    while(!Filter->m_vTimeInfo.empty())
    {
      TimeInfo *TI;
      {
        CAutoLock cAutoLock(&Filter->m_vTimeInfoLock);
        TI = Filter->m_vTimeInfo.front();
        Filter->m_vTimeInfo.pop_front();
      }
      if(TI)
      {
        if(!Filter->m_outStream.is_open())
        {
          if(Filter->m_State != State_Stopped)
            Filter->m_outStream.open(Filter->m_sFileName.c_str(), std::ios_base::out);
          if(!Filter->m_outStream.is_open())
          {
	    delete(TI);
	    break;
          }
        }
        TI->log(Filter->m_outStream);
        delete(TI);
      }      
    }
    if(Filter->m_State == State_Stopped)
    {
      if(Filter->m_outStream.is_open())
          Filter->m_outStream.close();  
    }
  }

  if(Filter->m_outStream.is_open())
      Filter->m_outStream.close();
  //if(SUCCEEDED(hr)) CoUninitialize();
  Filter->ThreadStatus = THREAD_STOPPED;
return 0;
}


STDMETHODIMP TimestampLoggerFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == IID_IFileSourceFilter) return GetInterface((IFileSourceFilter*)this,ppv);
  return CTransInPlaceFilter::NonDelegatingQueryInterface(riid, ppv);
}


HRESULT TimestampLoggerFilter::Transform(IMediaSample *pSample) // Overrriding the receive method. This method receives a media sample, processes it, and delivers it to the downstream filter.
{
  if(pSample)
  {	// Output time to file
    TimeInfo *pTI = new TimeInfo(pSample->GetActualDataLength());
    if(pTI)
    {
      pTI->hrTIME = pSample->GetTime(&pTI->StartTime,&pTI->StopTime);
      {		// locked scope
        CAutoLock cAutoLock(&m_vTimeInfoLock);
        m_vTimeInfo.push_back(pTI);
      }
      SetEvent(MsgWakeUp);		// Write time info to file
    }
  }
  return S_OK;
}


HRESULT TimestampLoggerFilter::CheckInputType(const CMediaType* mtIn)
{
	// Accept any input type
  return S_OK;
}


STDMETHODIMP TimestampLoggerFilter::Load(LPCOLESTR lpwszFileName, const AM_MEDIA_TYPE *pmt)
{
	// Store the filename
  m_sFileName = StringUtil::wideStringToString(lpwszFileName);
  return S_OK;
}


STDMETHODIMP TimestampLoggerFilter::GetCurFile(LPOLESTR * ppszFileName, AM_MEDIA_TYPE *pmt)
{
  if(ppszFileName==NULL) return E_POINTER;
  *ppszFileName = NULL;
  if(!m_sFileName.empty())
  {
    std::wstring wsFileName = StringUtil::stringToWideString(m_sFileName);
    DWORD n = sizeof(WCHAR)*(1 + wsFileName.length());
    *ppszFileName = (LPOLESTR) CoTaskMemAlloc(n);
    if(*ppszFileName!=NULL) {
      memcpy(*ppszFileName, wsFileName.c_str(), n);
    }
    return S_OK;
  }
  else
  {
    return E_FAIL;
  }
}


HRESULT TimestampLoggerFilter::Run(REFERENCE_TIME tStart)
{
  if(ThreadStatus==THREAD_NOT_INITIALISED || ThreadStatus==THREAD_STOPPED)
  {
    unsigned int ThreadID;
    ThreadStatus = THREAD_INITIALISING;
    WriteThread = (HANDLE)_beginthreadex(NULL, 0, WorkerThread, this, 0, &ThreadID);
  }
  return CTransInPlaceFilter::Run(tStart);
}


HRESULT TimestampLoggerFilter::Stop(void)
{
  SetEvent(MsgWakeUp);		// Write time info to file
  return CTransInPlaceFilter::Stop();
}


void TimestampLoggerFilter::TimeInfo::log(std::ofstream &outStream) const
{
  if(FAILED(hrTIME))
    outStream << "GetTime hr=0x" << std::hex << hrTIME << " " << std::dec <<  Size <<  std::endl;
  else
    outStream << StartTime << " " << StopTime <<  " " << StopTime-StartTime << " " <<  Size <<  std::endl;
}

