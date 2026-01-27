/** @file

MODULE				: RotateFilter

FILE NAME			: RotateFilter.h

DESCRIPTION			: Rotate for RGB24 and RGB32 media. 
					Rotation parameters can be set using the COM SettingsInterface SetParameter method.
					  
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

// CSIR includes
#include <DirectShow/CustomBaseFilter.h>
#include "../FilterParameterStringConstants.h"

/// {565DDCE4-F711-46DB-80D8-1AA1C2D0FA8E}
static const GUID CLSID_VPP_RotateFilter = 
{ 0x565ddce4, 0xf711, 0x46db, { 0x80, 0xd8, 0x1a, 0xa1, 0xc2, 0xd0, 0xfa, 0x8e } };

// {DF1ECF47-9233-4D25-98BD-5A01B984EF0E}
static const GUID CLSID_RotateProperties = 
{ 0xdf1ecf47, 0x9233, 0x4d25, { 0x98, 0xbd, 0x5a, 0x1, 0xb9, 0x84, 0xef, 0xe } };


// Forward declarations
class PicRotateBase;


/**
 * \ingroup DirectShowFilters
 * Rotate for RGB24 and RGB32 media. 
 * Rotation parameters can be set using the COM SettingsInterface SetParameter method.
 */
class RotateFilter: public CCustomBaseFilter, public ISpecifyPropertyPages,
                    public IPersistPropertyBag, public IPropertyBag
{
public:
	DECLARE_IUNKNOWN

	/// Constructor
	RotateFilter();
	/// Destructor
	~RotateFilter();

	/// Static object-creation method (for the class factory)
	static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 

	/**
	* Overriding this so that we can set whether this is an RGB24 or an RGB32 Filter
	*/
	HRESULT SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt);

	/**
	* Used for Media Type Negotiation 
	* Returns an HRESULT value. Possible values include those shown in the following table.
	* <table border="0" cols="2"><tr valign="top"><td><b>Value</b></td><td><b>Description</b></td></TR><TR><TD>S_OK</TD><TD>Success</TD></TR><TR><TD>VFW_S_NO_MORE_ITEMS</TD><TD>Index out of range</TD></TR><TR><TD>E_INVALIDARG</TD><TD>Index less than zero</TD></TR></TABLE>
	* The output pin's CTransformOutputPin::GetMediaType method calls this method. The derived class must implement this method. For more information, see CBasePin::GetMediaType.
	* To use custom media types, the media type can be manipulated in this method.
	*/
	HRESULT GetMediaType(int iPosition, CMediaType *pMediaType);

	/// Buffer Allocation
	/**
	* The output pin's CTransformOutputPin::DecideBufferSize method calls this method. The derived class must implement this method. For more information, see CBaseOutputPin::DecideBufferSize. 
	* @param pAlloc Pointer to the IMemAllocator interface on the output pin's allocator.
	* @param pProp Pointer to an ALLOCATOR_PROPERTIES structure that contains buffer requirements from the downstream input pin.
	* @return Value: Returns S_OK or another HRESULT value.
	*/
	HRESULT DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp);

	/**
	* The CheckTransform method checks whether an input media type is compatible with an output media type.
	* <table border="0" cols="2"> <tr valign="top"> <td  width="50%"><b>Value</b></td> <td width="50%"><b>Description</b></td> </tr> <tr valign="top"> <td width="50%">S_OK</td> <td width="50%">The media types are compatible.</td> </tr> <tr valign="top"> <td width="50%">VFW_E_TYPE_NOT_ACCEPTED</td> <td width="50%">The media types are not compatible.</td> </tr> </table>
	*/
	HRESULT CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut);

	/// Overridden from CSettingsInterface
	virtual void initParameters();

	STDMETHODIMP SetParameter( const char* type, const char* value);

	/// Overridden from CCustomBaseFilter
	virtual void InitialiseInputTypes();

	STDMETHODIMP GetPages(CAUUID *pPages);

	STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

		// IPersistPropertyBag
	virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);
	virtual HRESULT STDMETHODCALLTYPE InitNew(void) {return CCustomBaseFilter::InitNew();}
	virtual HRESULT STDMETHODCALLTYPE Load(__RPC__in_opt IPropertyBag *pPropBag, __RPC__in_opt IErrorLog *pErrorLog);
	virtual HRESULT STDMETHODCALLTYPE Save(__RPC__in_opt IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties);

		// IPropertyBag
	STDMETHODIMP Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog);
	STDMETHODIMP Write(LPCOLESTR pszPropName, VARIANT *pVar);


private:
	/**
	* This method converts the input buffer from RGB24 | 32 to YUV420P
	* @param pSource The source buffer
	* @param pDest The destination buffer
	*/
        virtual HRESULT ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength);

	void RecalculateFilterParameters();

	/// Pointer to our picture rotation class
	PicRotateBase* m_pRotate;

	unsigned m_nBitsPerPixel;

	/// 0 = Don't rotate
	/// 1 = 90 deg Clockwise
	/// 2 = 180 deg Clockwise
	/// 3 = 270 deg Clockwise
	/// 4 = Upside down
	/// 5 = Mirror
	/// 6 = Flip diagonally
	int m_nRotation; 
	int m_nOutWidth;
	int m_nOutHeight;

	int m_nStride;
	int m_nPadding;
};