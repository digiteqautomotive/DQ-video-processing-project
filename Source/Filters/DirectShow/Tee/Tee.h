/** @file

MODULE				: Tee

FILE NAME			: Tee.h

DESCRIPTION			: Basic Tee filter implementation demonstrating how the usage of the CMultiIOBaseFilter class
					NOTE: This filter will not handle ALL media types and has been created for demonstration purposes.
					Use the standard DirectShow tee in applications.
					  
LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2012, CSIR
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

#include <DirectShow/MultiIOBaseFilter.h>

#define PARAM_COPY "copy"


// {4CCD08A1-0C4E-4367-BED1-38B7D92D24E7}
static const GUID CLSID_VPP_Tee = 
{ 0x4ccd08a1, 0xc4e, 0x4367, { 0xbe, 0xd1, 0x38, 0xb7, 0xd9, 0x2d, 0x24, 0xe7 } };

// {D482DD89-752F-44e5-AF4F-DF02D4E81C00}
DEFINE_GUID(CLSID_VPP_TeeProperties, 
0xd482dd89, 0x752f, 0x44e5, 0xaf, 0x4f, 0xdf, 0x2, 0xd4, 0xe8, 0x1c, 0x0);


/** \ingroup DirectShowFilters
 * Basic Tee filter implementation demonstrating how the usage of the CMultiIOBaseFilter class
 * NOTE: This filter will not handle ALL media types and has been created for demonstration purposes.
 * Use the standard DirectShow tee in applications.
 */
class RtvcTee: public CMultiIOBaseFilter, public ISpecifyPropertyPages
{
public:
	DECLARE_IUNKNOWN

	/// Constructor
	RtvcTee();
	/// Destructor
	~RtvcTee();

	/// Static object-creation method (for the class factory)
	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 

	virtual HRESULT Receive(IMediaSample *pSample, int nIndex );
	
	virtual int InitialNumberOfInputPins() const {return 1;}
	virtual int InitialNumberOfOutputPins() const {return 1;}
	virtual bool OnFullCreateMoreInputs() const {return false;}
	virtual bool OnFullCreateMoreOutputs() const {return true;}
	virtual void InitialiseInputTypes();
	virtual void InitialiseOutputTypes();
	void initParameters(void);

	///  From CBasePin
	virtual HRESULT CheckOutputType(const CMediaType* pMediaType);


	/// From CBaseOutputPin
	virtual HRESULT DecideBufferSize(IMemAllocator* pAlloc, ALLOCATOR_PROPERTIES* pRequestProperties, int m_nIndex);

	/// To Check media type
	virtual HRESULT GetMediaType(int iPosition, CMediaType* pmt, int nOutputPinIndex);

	STDMETHODIMP GetClassID(CLSID *pClassID);

	STDMETHODIMP GetPages(CAUUID *pPages);

        STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

protected:
        bool m_bCopy;
};
