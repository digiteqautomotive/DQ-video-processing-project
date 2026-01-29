/** @file

MODULE				: H264

FILE NAME			: H264DecoderFilter.h

DESCRIPTION			: DirectShow H.264 decoder filter header

LICENSE	: GNU Lesser General Public License

Copyright (c) 2008 - 2012, CSIR
All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

===========================================================================
*/
#pragma once

// Meraka includes
#include <DirectShow/CustomBaseFilter.h>
//#include <DirectShowExt/DirectShowMediaFormats.h>
#include <DirectShow/NotifyCodes.h>


/// {F97F9C42-6DA9-40AE-8434-2DF8AFD8F0DE}
static const GUID CLSID_VPP_H264Decoder = 
{ 0xf97f9c42, 0x6da9, 0x40ae, { 0x84, 0x34, 0x2d, 0xf8, 0xaf, 0xd8, 0xf0, 0xde } };



/// Custom RTVC H264 media format
// {BDF25152-046B-4509-8E55-6C73831C8DC4}
static const GUID MEDIASUBTYPE_H264M = 
{ 0xbdf25152, 0x46b, 0x4509, { 0x8e, 0x55, 0x6c, 0x73, 0x83, 0x1c, 0x8d, 0xc4 } };
/// Custom RTVC H264 media format
// {BDF25152-046B-4509-8E55-6C73831C8DC4}
static const GUID MEDIASUBTYPE_VPP_H264 =
{ 0xbdf25152, 0x46b, 0x4509, { 0x8e, 0x55, 0x6c, 0x73, 0x83, 0x1c, 0x8d, 0xc4 } };


// Forward
class ICodecv2;

class H264DecoderFilter: public CCustomBaseFilter
{
public:
  /// Constructor
  H264DecoderFilter();
  /// Destructor
  ~H264DecoderFilter();

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

  ///Overridden from CSettingsInterface
  void initParameters() {}
  ///Overridden from CSettingsInterface
  STDMETHODIMP GetParameter( const char* type, int* length, void* value );
  ///Overridden from CSettingsInterface
  STDMETHODIMP SetParameter( const char* type, const char* value);

  /// Overridden from CCustomBaseFilter
  virtual void InitialiseInputTypes();

  STDMETHODIMP GetClassID(CLSID *pClassID);

private:
  /**
  * This method decodes the frame from H.264 to RGB24
  * @param[in] pBufferIn The source buffer
  * @param[in] lInBufferSize The size of the in buffer
  * @param[in] lActualDataLength The size of the actual frame inside the in buffer
  * @param[in,out] pBufferOut The destination buffer
  * @param[in] lOutBufferSize The size of the destination buffer
  * @param[in,out] lOutActualDataLength The size of the data encoded into the destination buffer
  */
  virtual HRESULT ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength);

  HRESULT initialiseH264Decoder();
  void resizeEncodedPictureBufferIfNecessary(long lActualDataLength);

  ICodecv2* m_pCodec;

  unsigned char* m_pSeqParamSet;
  unsigned m_uiSeqParamSetLen;
  unsigned char* m_pPicParamSet;
  unsigned m_uiPicParamSetLen;

  // only set if media subtype == MEDIASUBTYPE_AVC1
  // and stores length of length field preceding H264 data
  unsigned m_uiAvc1PrefixLength;
  // buffer in case we have to copy the incoming data
  const BYTE* m_pEncodedPictureBuffer;
  unsigned m_uiEncodedPictureBufferLength;
};
