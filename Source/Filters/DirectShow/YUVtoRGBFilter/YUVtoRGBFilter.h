/** @file

MODULE				: YUVtoRGBFilter

FILE NAME			: YUVtoRGBFilter.h

DESCRIPTION			: This Filter converts the YUV444 Interleaved media back into RGB24 media.

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive All rights reserved.

===========================================================================
*/
#pragma once

// CSIR includes
#include <DirectShow/CustomBaseFilter.h>

// Parameters
//#define CHROMINANCE_OFFSET "chrominanceoffset"
#define INVERT "invert"


// GUID

// {08F78B88-4A61-495A-9E9F-AAD47ED3EDF4}
DEFINE_GUID(CLSID_VPP_YUVtoRGBColorConverter, 
0x8f78b88, 0x4a61, 0x495a, 0x9e, 0x9f, 0xaa, 0xd4, 0x7e, 0xd3, 0xed, 0xf4);

// {554D85CB-E127-477E-B44D-3CE5A3AC956A}
DEFINE_GUID(CLSID_YUVtoRGBProperties, 
0x554d85cb, 0xe127, 0x477e, 0xb4, 0x4d, 0x3c, 0xe5, 0xa3, 0xac, 0x95, 0x6a);



// Forward declarations
class YUVtoRGBConverter;

/**
* \ingroup DirectShowFilters
* This Filter converts the YUV444 Interleaved media back into RGB24 media.
*/
class YUVtoRGBFilter: public CCustomBaseFilter, public ISpecifyPropertyPages,
		      public IPersistPropertyBag, public IPropertyBag
{
public:
  DECLARE_IUNKNOWN

  /// Constructor
  YUVtoRGBFilter();
  /// Destructor
  ~YUVtoRGBFilter();

  /// Static object-creation method (for the class factory)
  static CUnknown * WINAPI CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr); 

  /** Overriding this so that we can set whether this is an RGB24 or an RGB32 Filter */
  HRESULT SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt);

  /**
  * Used for Media Type Negotiation.
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

  /// Overridden from CCustomBaseFilter
  virtual void InitialiseInputTypes();

  /// Overridden from CSettingsInterface
  STDMETHODIMP SetParameter( const char* type, const char* value);

  virtual HRESULT BreakConnect(PIN_DIRECTION dir);

  /// Overridden from CSettingsInterface
  void initParameters(void);

  STDMETHODIMP GetPages(CAUUID *pPages);

  STDMETHODIMP NonDelegatingQueryInterface(REFIID riid, void **ppv);

protected:

  /** This method converts the input buffer from YUV444I to RGB32
  * @param pSource The source buffer
  * @param pDest The destination buffer */
  virtual HRESULT ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength);

	// IPersistPropertyBag
  virtual HRESULT STDMETHODCALLTYPE GetClassID(CLSID *pClassID);
  virtual HRESULT STDMETHODCALLTYPE InitNew(void) {return CSettingsInterface::InitNew();}
  virtual HRESULT STDMETHODCALLTYPE Load(__RPC__in_opt IPropertyBag *pPropBag, __RPC__in_opt IErrorLog *pErrorLog);
  virtual HRESULT STDMETHODCALLTYPE Save(__RPC__in_opt IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties);

	// IPropertyBag
  STDMETHODIMP Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog);
  STDMETHODIMP Write(LPCOLESTR pszPropName, VARIANT *pVar);

private:

  /// Pointer to our YUV444 to RGB24 converter
  YUVtoRGBConverter* m_pConverter;
  // Size of YUV image
  int m_nSizeUV;
  // Chrominance offset
  //int m_nChrominanceOffset;
  // YUV is always top down: the invert flag determines whether the image
  // is flipped during conversion, or whether the sign is simply set
  bool m_bInvert;

  GUID in_MediaType;
  GUID out_MediaType;
};
