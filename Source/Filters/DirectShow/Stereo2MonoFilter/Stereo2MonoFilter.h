/** @file

MODULE                : Stereo2MonoFilter

FILE NAME             : Stereo2MonoFilter.h

DESCRIPTION           :  This filter outputs mono output from a stereo input

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2014, CSIR
All rights reserved.

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

#include <DirectShow/CustomBaseFilter.h>


// {DE243F19-49C6-4F86-9B46-AAAFFCF89505}
static const GUID CLSID_VPP_Stereo2MonoFilter = 
{ 0xde243f19, 0x49c6, 0x4f86, { 0x9b, 0x46, 0xaa, 0xaf, 0xfc, 0xf8, 0x95, 0x5 } };

// {D4D51F7C-50E7-434A-966C-FBFF889D00B1}
static const GUID CLSID_Stereo2MonoProperties = 
{ 0xd4d51f7c, 0x50e7, 0x434a, { 0x96, 0x6c, 0xfb, 0xff, 0x88, 0x9d, 0x0, 0xb1 } };


// Forward declarations

class Stereo2MonoFilter: public CCustomBaseFilter,
                         public ISpecifyPropertyPages,
                         public IPersistPropertyBag, public IPropertyBag, public IPersistStream
{
public:
    DECLARE_IUNKNOWN

	/// Constructor
    Stereo2MonoFilter(LPUNKNOWN pUnk, HRESULT *pHr);
	/// Destructor
    ~Stereo2MonoFilter();

	/// Static object-creation method (for the class factory)
    static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 

	/// override this to publicize our interfaces
    STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

    HRESULT CheckInputType(const CMediaType *pMediaType);
    HRESULT SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt);
    HRESULT GetMediaType(int iPosition, CMediaType *pMediaType);
    HRESULT DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp);
    HRESULT CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut);

    STDMETHODIMP GetPages(CAUUID *pPages); // For the Property Page

	// Overridden from CSettingsInterface
  virtual void initParameters() { addParameter("channel", &m_channel, 0); }

	// IPersistPropertyBag
  virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);
  virtual HRESULT STDMETHODCALLTYPE InitNew(void);        
  virtual HRESULT STDMETHODCALLTYPE Load(__RPC__in_opt IPropertyBag *pPropBag, __RPC__in_opt IErrorLog *pErrorLog);
  virtual HRESULT STDMETHODCALLTYPE Save(__RPC__in_opt IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties);

	// IPropertyBag
  STDMETHODIMP Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog);
  STDMETHODIMP Write(LPCOLESTR pszPropName, VARIANT *pVar);

	// IPersistStream
  virtual HRESULT STDMETHODCALLTYPE GetSizeMax(ULARGE_INTEGER *pcbSize);
  virtual HRESULT STDMETHODCALLTYPE IsDirty(void);
  virtual HRESULT STDMETHODCALLTYPE Load(IStream *pStm);
  virtual HRESULT STDMETHODCALLTYPE Save(IStream *pStm, BOOL fClearDirty);

private:
  	/// Overridden from CCustomBaseFilter
	  virtual void InitialiseInputTypes();
    virtual HRESULT ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength);

  int m_channel;
  int m_nBitsPerSample;
  int m_nSamplesPerSecond;
};
