/** @file

MODULE				: CropFilter

FILE NAME			: CropFilter.h

DESCRIPTION			: Cropfilter for RGB24 and RGB32 media.
Crop parameters can be set using the COM SettingsInterface SetParameter method.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008 - 2015, CSIR
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
===========================================================================*/
#pragma once
// CSIR includes
#include <DirectShow/CustomBaseFilter.h>
#include "Filters/DirectShow/FilterParameterStringConstants.h"

// {2AA8007D-F5B9-4255-B4DD-A3BAD40681A8}
static const GUID CLSID_VPP_CropFilter = 
{ 0x2aa8007d, 0xf5b9, 0x4255, { 0xb4, 0xdd, 0xa3, 0xba, 0xd4, 0x6, 0x81, 0xa8 } };

// {AEF6D61E-C1CE-4B54-BC63-363EEB1DC2EB}
static const GUID CLSID_CropProperties = 
{ 0xaef6d61e, 0xc1ce, 0x4b54, { 0xbc, 0x63, 0x36, 0x3e, 0xeb, 0x1d, 0xc2, 0xeb } };



// Forward declarations
class PicCropperBase;

/**
 * @brief Crop filter for RGB24 and RGB32 media. 
 * 
 * Crop parameters can be set using the COM SettingsInterface SetParameter method.
 */
class CropFilter: public CCustomBaseFilter, public ISpecifyPropertyPages, 
                  public IPersistPropertyBag, public IPropertyBag
{
public:
    DECLARE_IUNKNOWN

	// Constructor
    CropFilter();
	// Destructor
    ~CropFilter();

  /** @brief Static object-creation method (for the class factory) */
  static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr);

  /** Overriding this so that we can set whether this is an RGB24 or an RGB32 Filter  */
  HRESULT SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt);

  /** @brief Used for Media Type Negotiation
   * Returns an HRESULT value. Possible values include those shown in the following table.
   * <table border="0" cols="2"><tr valign="top"><td><b>Value</b></td><td><b>Description</b></td></TR><TR><TD>S_OK</TD><TD>Success</TD></TR><TR><TD>VFW_S_NO_MORE_ITEMS</TD><TD>Index out of range</TD></TR><TR><TD>E_INVALIDARG</TD><TD>Index less than zero</TD></TR></TABLE>
   * The output pin's CTransformOutputPin::GetMediaType method calls this method. The derived class must implement this method. For more information, see CBasePin::GetMediaType.
   * To use custom media types, the media type can be manipulated in this method. */
  HRESULT GetMediaType(int iPosition, CMediaType *pMediaType);

  /** @brief The output pin's CTransformOutputPin::DecideBufferSize method calls this method. The derived class must implement this method. For more information, see CBaseOutputPin::DecideBufferSize.
   * @param pAlloc Pointer to the IMemAllocator interface on the output pin's allocator.
   * @param pProp Pointer to an ALLOCATOR_PROPERTIES structure that contains buffer requirements from the downstream input pin.
   * @return Value: Returns S_OK or another HRESULT value. */
  HRESULT DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp);

  /** @brief The CheckTransform method checks whether an input media type is compatible with an output media type.
   * <table border="0" cols="2"> <tr valign="top"> <td  width="50%"><b>Value</b></td> <td width="50%"><b>Description</b></td> </tr> <tr valign="top"> <td width="50%">S_OK</td> <td width="50%">The media types are compatible.</td> </tr> <tr valign="top"> <td width="50%">VFW_E_TYPE_NOT_ACCEPTED</td> <td width="50%">The media types are not compatible.</td> </tr> </table> */
  HRESULT CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut);

	// Overridden from CSettingsInterface
  void initParameters();  
  STDMETHODIMP SetParameter(const char* type, const char* value);

	// Overridden from CCustomBaseFilter
  virtual void InitialiseInputTypes();

	// Overridden from ISpecifyPropertyPages
  STDMETHODIMP GetPages(CAUUID *pPages);

	// Overridden from COM
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
  /** This method applies the crop transform
   * @param pSource The source buffer
   * @param pDest The destination buffer */
  virtual HRESULT ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength);
  /// Sets crop parameters if values are valid
  HRESULT SetCropIfValid(int nTotalDimensionImage, int nNewCrop, int& nOldCrop, int nOppositeCrop);
  /// This method calculates how much should be cropped from the picture based on the crop ratios
  void RecalculateFilterParameters();
  /// Pointer to our picture cropper
  PicCropperBase* m_pCropper;
  //BYTE* m_pCropBuffer;
  unsigned m_nBitsPerPixel;
  //Member variables which affect the transform
  int m_nLeftCrop;
  int m_nRightCrop;
  int m_nTopCrop;
  int m_nBottomCrop;
  //int m_nStride;
  //int m_nPadding;
  bool m_DirtyPropsStream;
};
