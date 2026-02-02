/** @file

MODULE                : SkewingFilter

FILE NAME             : SkewingFilter.h

DESCRIPTION           :  This filter skips a specified number of frames depending on the parameter denoted by "skipparam"

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2011, CSIR
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

//#include <Filters/DirectShow/DirectShowFilterGuids.h>
#include <DirectShow/CSettingsInterface.h>


// {BCE11DA2-B087-4CF1-B96B-FD186A4B5D32}
static const GUID CLSID_VPP_SkewingFilter = 
{ 0xbce11da2, 0xb087, 0x4cf1, { 0xb9, 0x6b, 0xfd, 0x18, 0x6a, 0x4b, 0x5d, 0x32 } };

// {FFD8870B-832A-49FF-9E7A-03C48029DC9A}
static const GUID CLSID_SkewingProperties = 
{ 0xffd8870b, 0x832a, 0x49ff, { 0x9e, 0x7a, 0x3, 0xc4, 0x80, 0x29, 0xdc, 0x9a } };


// Forward declarations

class SkewingFilter: public CTransInPlaceFilter,
                     public CSettingsInterface,
                     public ISpecifyPropertyPages
{
public:
    DECLARE_IUNKNOWN

	/// Constructor
    SkewingFilter(LPUNKNOWN pUnk, HRESULT *pHr);
	/// Destructor
    ~SkewingFilter();

	/// Static object-creation method (for the class factory)
    static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 

	/// override this to publicize our interfaces
    STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

	//Overriding various CTransInPlace methods
    HRESULT Transform(IMediaSample *pSample);/* Overrriding the receive method. 
                                           This method receives a media sample, processes it, and delivers it to the downstream filter.*/

    HRESULT CheckInputType(const CMediaType* mtIn);

    STDMETHODIMP GetPages(CAUUID *pPages);	///< For the Skewing Property Page

    STDMETHODIMP Run( REFERENCE_TIME tStart );
    STDMETHODIMP Stop(void);

	// IPersistPropertyBag
    virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);
    //virtual HRESULT STDMETHODCALLTYPE InitNew(void) {return CSettingsInterface::InitNew();}    

	/// Overridden from CSettingsInterface
    void initParameters(void);

private:
    bool seenFirstFrame;
    REFERENCE_TIME previousTimestamp;  

    int add_time;		///< For adding time to the timestamp filter.
};
