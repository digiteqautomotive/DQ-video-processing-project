/** @file

MODULE				: AudioMixingFilter

FILE NAME			: AudioMixingFilter.h

DESCRIPTION			: Audio Mixing Filter for PCM
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2010, CSIR
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
#pragma once

#include <DirectShow/MultiIOBaseFilter.h>
#include <DirectShow/CSettingsInterface.h>

#include <deque>

// #define USE_PROPERTY_PAGE

/// {1D91CA5C-C865-46BB-85F5-48ACFAB69585}
static const GUID CLSID_VPP_AudioMixingFilter = 
{ 0x1d91ca5c, 0xc865, 0x46bb, { 0x85, 0xf5, 0x48, 0xac, 0xfa, 0xb6, 0x95, 0x85 } };

#ifdef USE_PROPERTY_PAGE
/// {4549F980-EE2D-4DB7-8603-3212C418CC2B}
static const GUID CLSID_AudioMixingProperties = 
{ 0x4549f980, 0xee2d, 0x4db7, { 0x86, 0x3, 0x32, 0x12, 0xc4, 0x18, 0xcc, 0x2b } };
#endif


/**
 * \ingroup DirectShowFilters
 * Audio Mixing Filter for PCM Mono Media
 */
class AudioMixingFilter: public CMultiIOBaseFilter
#ifdef USE_PROPERTY_PAGE
			 ,public ISpecifyPropertyPages
#endif
{
  typedef std::deque<IMediaSample*> MediaSampleQueue_t; 
public:
	DECLARE_IUNKNOWN;

	/// Constructor
	AudioMixingFilter();
	/// Destructor
	~AudioMixingFilter();

	/// Static object-creation method (for the class factory)
	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 

	///  From CBasePin
	virtual HRESULT CheckOutputType(const CMediaType* pMediaType);

	/// Delegates call to virtual ReceiveFirstSample and ReceiveSecondSample methods.
	virtual HRESULT Receive(IMediaSample *pSample, int nIndex );

	/// From CBaseOutputPin
	virtual HRESULT DecideBufferSize(IMemAllocator* pAlloc, ALLOCATOR_PROPERTIES* pRequestProperties, int m_nIndex);

	/// To Check media type
	virtual HRESULT GetMediaType(int iPosition, CMediaType* pmt, int nOutputPinIndex) ;

	/// Overriding this so that we can set whether this is an RGB24 or an RGB32 Filter and create
	/// the picture concatenator dynamically. This method also tells us if the advert stream is tagged
	HRESULT SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt, int nIndex);

	STDMETHODIMP Stop(void);
	STDMETHODIMP Pause(void);
  
	virtual void InitialiseInputTypes(void)
	{
	  AddInputType(&MEDIATYPE_Audio, &MEDIASUBTYPE_PCM, &FORMAT_WaveFormatEx);
	}
	virtual void InitialiseOutputTypes()
	{
	  AddOutputType(&MEDIATYPE_Audio, &MEDIASUBTYPE_PCM, &FORMAT_WaveFormatEx);
	}

	virtual int InitialNumberOfInputPins()  const	{return 2;}
	virtual int InitialNumberOfOutputPins() const {return 1;}
	virtual bool OnFullCreateMoreInputs() const	{return false;}
	virtual bool OnFullCreateMoreOutputs() const	{return false;}

		// you can also override these if you want to know about streaming
	virtual HRESULT StartStreaming();
	virtual HRESULT StopStreaming();

		// Streaming calls delegated from input pins
	virtual STDMETHODIMP EndOfStream(int nIndex);
	virtual STDMETHODIMP BeginFlush(int nIndex);
	virtual STDMETHODIMP EndFlush(int nIndex);

	HRESULT GenerateOutputSample(IMediaSample *pSample, int nIndex);

	void initParameters(void);

#ifdef USE_PROPERTY_PAGE
	/// For configuration dialog
	STDMETHODIMP GetPages(CAUUID *pPages);
#endif

	virtual STDMETHODIMP GetClassID(CLSID *pClassID);

	/// For configuration dialog
	STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

private:

		/// Pointers to our last received Media Sample
	BYTE* m_pSampleBuffers[2];
	// Sizes
	int m_nSampleSizes[2];
	MediaSampleQueue_t m_qSamples1;
	MediaSampleQueue_t m_qSamples2;

	unsigned m_uiBitsPerSample;
	unsigned m_uiSamplesPerSecond;
	unsigned m_uiChannels;

		// Vars needed for eos behaviour
	unsigned m_uiEndOfStreamCount;
	unsigned m_uiBeginFlushCount;
	unsigned m_uiEndFlushCount;

	HANDLE m_hSyncEvent;
};
