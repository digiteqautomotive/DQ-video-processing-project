/** @file

MODULE                : TimestampLoggerFilter

FILE NAME             : TimestampLoggerFilter.h

DESCRIPTION           : Filter that logs media sample timestamps and durations to file specified using IFileSourceFilter interface
                     
LICENSE: Software License Agreement (BSD License)

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

===========================================================================
*/
#pragma once
#include <streams.h>

#include <fstream>
#include <list>


// {D04C9742-E0E8-43FB-967F-260EAA156F7E}
static const GUID CLSID_VPP_TimeStampLoggerFilter = 
{ 0xd04c9742, 0xe0e8, 0x43fb, { 0x96, 0x7f, 0x26, 0xe, 0xaa, 0x15, 0x6f, 0x7e } };


#ifdef _WIN32
 #define THREAD_DECL	unsigned WINAPI
#else
 #define THREAD_DECL	void *
#endif


/** \ingroup DirectShowFilters
 * Filter that logs media sample timestamps and durations 
 * to file specified using IFileSourceFilter interface */
class TimestampLoggerFilter: public CTransInPlaceFilter, public IFileSourceFilter
{
public:
	typedef enum
	{
	  THREAD_NOT_INITIALISED =0,
          THREAD_INITIALISING,
          THREAD_RUNNING,
          THREAD_STOPPING,
          THREAD_STOPPED,
          THREAD_ABORTED
        } E_THR_STATUS;

	DECLARE_IUNKNOWN

	/// Constructor
	TimestampLoggerFilter(LPUNKNOWN pUnk, HRESULT *pHr);
	/// Destructor
	~TimestampLoggerFilter();

	/// Static object-creation method (for the class factory)
	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 

	/// override this to publicize our interfaces
	STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

	// From IFileSourceFilter
	STDMETHODIMP Load(LPCOLESTR lpwszFileName, const AM_MEDIA_TYPE *pmt);
	STDMETHODIMP GetCurFile(LPOLESTR * ppszFileName, AM_MEDIA_TYPE *pmt);

	// Overriding various CTransInPlace methods
	HRESULT Transform(IMediaSample *pSample);/* Overrriding the receive method. 
	This method receives a media sample, processes it, and delivers it to the downstream filter.*/

	HRESULT CheckInputType(const CMediaType* mtIn);

	virtual STDMETHODIMP Run(REFERENCE_TIME tStart);
	virtual STDMETHODIMP Stop(void);
        virtual STDMETHODIMP Pause(void);

private:
	std::string m_sFileName;	
	std::ofstream m_outStream;

        volatile char ThreadStatus;
	static THREAD_DECL WorkerThread(void*);
#ifdef _WIN32
	//HANDLE        StopEvent;	///< stop signal for log thread
	HANDLE        MsgWakeUp;	///< new log entry signal for log thread
	HANDLE        WriteThread;	///< log thread
#else
	volatile int StopEvent;
	pthread_cond_t MsgWakeUp;
	pthread_mutex_t mtx_MsgWakeUp;
	pthread_t WriteThread;
#endif

	class TimeInfo
	{
        public:
		TimeInfo(REFERENCE_TIME tStart, REFERENCE_TIME tStop, unsigned uiSize):
			StartTime(tStart), StopTime(tStop), Size(uiSize)
		{}
		TimeInfo(unsigned uiSize): Size(uiSize) {}
		REFERENCE_TIME StartTime;
		REFERENCE_TIME StopTime;
		HRESULT hrTIME;
                unsigned Size;

		void log(std::ofstream &outStream) const;
	};

        CCritSec m_vTimeInfoLock;
	std::list<TimeInfo*> m_vTimeInfo;
};