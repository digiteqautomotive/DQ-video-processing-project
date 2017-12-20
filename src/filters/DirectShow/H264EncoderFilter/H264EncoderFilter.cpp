/** @file

MODULE:
 
TAG: 

FILE NAME: 

DESCRIPTION: 

COPYRIGHT: (c)CSIR 2007-2018 all rights reserved

LICENSE: Software License Agreement (BSD License)

RESTRICTIONS: 
Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation and/or 
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may 
be used to endorse or promote products derived from this software without specific 
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
OF THE POSSIBILITY OF SUCH DAMAGE.
===========================================================================

*/

#include "stdafx.h"
#include "H264EncoderFilter.h"
#include <cassert>
#include <dvdmedia.h>
#include <wmcodecdsp.h>
#include <H264v2/H264v2.h>
#include <CodecUtils/ICodecv2.h>
#include <CodecUtils/CodecConfigurationUtil.h>
#include <GeneralUtils/Conversion.h>

const unsigned char g_startCode[] = { 0, 0, 0, 1};

/// ***: Basic implementation is complete
/// Setting two additional rows and columns to zero values for now due to the large 32 Pixel motion estimation range!

/// Should be
/// 352x288 Flags for auto iframe detection
/// (352/16) * (288/16) = 22 * 18 = 396 16x16 macroblocks
/// The adverts will take up 
/// 96 Pixels (6 macroblocks) on the left of the image
/// 64 Pixels (4 macroblocks) at the bottom of the image
/// Note that the image mask is inverted: the viewed image has the adverts at the left and at the BOTTOM
bool g_pAutoIframeDetectFlags_352x288[396] =	{				  //  ____  Extra zero columns for now - see comment *** above
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Extra zero row for now - see comment *** above
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Extra zero row for now - see comment *** above
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
												};
/// 352x288 Flags for auto iframe detection
/// (176/16) * (144/16) = 11 * 9  = 99  16x16 macroblocks
/// The adverts will take up 
/// 48 Pixels (3 macroblocks) on the left of the image
/// 32 Pixels (2 macroblocks) at the bottom of the image
bool g_pAutoIframeDetectFlags_176x144[99] =		{    //  	 ____	Extra zero columns for now - see comment *** above
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Extra zero row for now - see comment *** above
													0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Extra zero row for now - see comment *** above
													0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
													0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1,
												};

const REFERENCE_TIME FPS_25 = UNITS / 25;

const unsigned MINIMUM_BUFFER_SIZE = 5024;
H264EncoderFilter::H264EncoderFilter()
  : CCustomBaseFilter(NAME("CSIR H264 Encoder"), 0, CLSID_RTVC_H264Encoder),
  m_pCodec(NULL),
  m_nFrameBitLimit(0),
  m_bNotifyOnIFrame(false),
  m_pSeqParamSet(0),
  m_uiSeqParamSetLen(0),
  m_pPicParamSet(0),
  m_uiPicParamSetLen(0),
  m_uiIFramePeriod(0),
  m_uiCurrentFrame(0),
  m_rtFrameLength(FPS_25),
  m_tStart(0),
  m_tStop(m_rtFrameLength)
{
	//Call the initialise input method to load all acceptable input types for this filter
	InitialiseInputTypes();
	initParameters();
	H264v2Factory factory;
	m_pCodec = factory.GetCodecInstance();
	// Set default codec properties 
	if (m_pCodec)
	{
    configureDefaultH264CodecParameters(m_pCodec);
  }
	else
	{
		SetLastError("Unable to create H264 Encoder from Factory.", true);
	}
}

H264EncoderFilter::~H264EncoderFilter()
{
	if (m_pCodec)
	{
		m_pCodec->Close();
		H264v2Factory factory;
		factory.ReleaseCodecInstance(m_pCodec);
	}

  if (m_pSeqParamSet) delete[] m_pSeqParamSet; m_pSeqParamSet = NULL;
  if (m_pPicParamSet) delete[] m_pPicParamSet; m_pPicParamSet = NULL;
}

CUnknown * WINAPI H264EncoderFilter::CreateInstance( LPUNKNOWN pUnk, HRESULT *pHr )
{
	H264EncoderFilter *pFilter = new H264EncoderFilter();
	if (pFilter== NULL) 
	{
		*pHr = E_OUTOFMEMORY;
	}
	return pFilter;
}

 
void H264EncoderFilter::InitialiseInputTypes()
{
	AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24,		&FORMAT_VideoInfo);
	AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUV420P_S,	&FORMAT_VideoInfo);
  // TODO
#if 0
	AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_I420,	&FORMAT_VideoInfo);
#endif
}


HRESULT H264EncoderFilter::SetMediaType( PIN_DIRECTION direction, const CMediaType *pmt )
{
	HRESULT hr = CCustomBaseFilter::SetMediaType(direction, pmt);
	if (direction == PINDIR_INPUT)
	{
    // try close just in case
    m_pCodec->Close();

    if (pmt->subtype == MEDIASUBTYPE_RGB24)
		{
      m_pCodec->SetParameter(CODEC_PARAM_IN_COLOUR, D_IN_COLOUR_RGB24);
		}
		else if (pmt->subtype == MEDIASUBTYPE_YUV420P_S)
		{
      m_pCodec->SetParameter(CODEC_PARAM_IN_COLOUR, D_IN_COLOUR_YUV420P);
		}
    // TODO
#if 0
    else if (pmt->subtype == MEDIASUBTYPE_I420)
    {
      m_pCodec->SetParameter(CODEC_PARAM_IN_COLOUR, D_IN_COLOUR_YUV420P8);
		}
#endif
    m_pCodec->SetParameter(FILTER_PARAM_WIDTH, artist::toString(m_nInWidth).c_str());
    m_pCodec->SetParameter(FILTER_PARAM_HEIGHT, artist::toString(m_nInHeight).c_str());

    // generate sequence and picture parameter sets
    if (m_pSeqParamSet) delete[] m_pSeqParamSet; m_pSeqParamSet = NULL;
    if (m_pPicParamSet) delete[] m_pPicParamSet; m_pPicParamSet = NULL;

    // set the selected parameter set numbers 
    m_pCodec->SetParameter((char *)"seq param set",  "0");
    m_pCodec->SetParameter((char *)"pic param set",  "0");

    m_pCodec->SetParameter((char *)("generate param set on open"),  "0");
    m_pCodec->SetParameter((char *)("picture coding type"),         "2");	///< Seq param set = H264V2_SEQ_PARAM.
    m_pSeqParamSet = new unsigned char[100];
    if(!m_pCodec->Code(NULL, m_pSeqParamSet, 100 * 8))
    {
      if (m_pSeqParamSet) delete[] m_pSeqParamSet; m_pSeqParamSet = NULL;
      SetLastError(m_pCodec->GetErrorStr(), true);
      return hr;
    }
    else
    {
      m_uiSeqParamSetLen = m_pCodec->GetCompressedByteLength();
      m_pSeqParamSet[m_uiSeqParamSetLen] = 0;
      m_sSeqParamSet = std::string((const char*)m_pSeqParamSet, m_uiSeqParamSetLen);
      int nCheck = strlen((char*)m_pSeqParamSet);
    }

    m_pCodec->SetParameter((char *)("picture coding type"),	  "3");	///< Pic param set = H264V2_PIC_PARAM.
    m_pPicParamSet = new unsigned char[100];
    if(!m_pCodec->Code(NULL, m_pPicParamSet, 100 * 8))
    {
      if (m_pSeqParamSet) delete[] m_pSeqParamSet; m_pSeqParamSet = NULL;
      if (m_pPicParamSet) delete[] m_pPicParamSet; m_pPicParamSet = NULL;
      SetLastError(m_pCodec->GetErrorStr(), true);
      return E_FAIL;
    }
    else
    {
      m_uiPicParamSetLen = m_pCodec->GetCompressedByteLength();
      m_pPicParamSet[m_uiPicParamSetLen] = 0;
      m_sPicParamSet = std::string((const char*)m_pPicParamSet, m_uiPicParamSetLen);
      int nCheck = strlen((char*)m_pPicParamSet);
    }

    // RG: copied from codec anayser, do we need this?
    // reset codec for standard operation
    m_pCodec->SetParameter((char *)("picture coding type"),	  "0");	///< I-frame = H264V2_INTRA.
    m_pCodec->SetParameter((char *)("generate param set on open"),  "1");

		if (!m_pCodec->Open())
		{
			//Houston: we have a failure
			char* szErrorStr = m_pCodec->GetErrorStr();
			printf("%s\n", szErrorStr);
			SetLastError(szErrorStr, true);
      return E_FAIL;
		}

		//// Apply the i-frame auto detect flags only if this input is an advert media type
		//if ( pmt->subtype == MEDIASUBTYPE_RGB24_ADVERT || pmt->subtype == MEDIASUBTYPE_YUV420P_ADVERT)
		//{
		//	// Now that the codec is open: get the inner access interface
		//	ICodecInnerAccess* pInnerAccess = dynamic_cast<ICodecInnerAccess*>(dynamic_cast<H264v2Codec*>(m_pCodec));
		//	if (pInnerAccess)
		//	{
		//		bool* pFlags = NULL;
		//		if (m_nInWidth == 352 && m_nInHeight == 288)
		//		{
		//			pFlags = g_pAutoIframeDetectFlags_352x288;
		//		}
		//		else if (m_nInWidth == 176 && m_nInHeight == 144)
		//		{
		//			pFlags = g_pAutoIframeDetectFlags_176x144;
		//		}

		//		if (pFlags)
		//			pInnerAccess->SetMember(AUTO_IFRAME_DETECT_FLAG, static_cast<void*>(pFlags));
		//		// Else the ICodecInnerAccess interface will use a default
		//	}
		//}
	}
	return hr;
}

HRESULT H264EncoderFilter::GetMediaType( int iPosition, CMediaType *pMediaType )
{
	if (iPosition < 0)
	{
		return E_INVALIDARG;
	}
	else if (iPosition == 0)
	{
    if (m_nH264Type == H264_VPP)
    {
      // Get the input pin's media type and return this as the output media type - we want to retain
      // all the information about the image
      HRESULT hr = m_pInput->ConnectionMediaType(pMediaType);
      if (FAILED(hr))
      {
        return hr;
      }

      pMediaType->subtype = MEDIASUBTYPE_VPP_H264;
      ASSERT(pMediaType->formattype == FORMAT_VideoInfo);
      VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
      BITMAPINFOHEADER* pBi = &(pVih->bmiHeader);

      REFERENCE_TIME avgTimePerFrame = pVih->AvgTimePerFrame;
      // For compressed formats, this value is the implied bit 
      // depth of the uncompressed image, after the image has been decoded.
      if (pBi->biBitCount != 24)
        pBi->biBitCount = 24;

      pBi->biSizeImage = DIBSIZE(pVih->bmiHeader);
      pBi->biSizeImage = DIBSIZE(pVih->bmiHeader);

      // COMMENTING TO REMOVE
      // in the case of YUV I420 input to the H264 encoder, we need to change this back to RGB
      //pBi->biCompression = BI_RGB;
      pBi->biCompression = DWORD('1cva');

      // Store SPS and PPS in media format header
      int nCurrentFormatBlockSize = pMediaType->cbFormat;

      if (m_uiSeqParamSetLen + m_uiPicParamSetLen > 0)
      {
        // old size + one int to store size of SPS/PPS + SPS/PPS/prepended by start codes
        int iAdditionalLength = sizeof(int) + m_uiSeqParamSetLen + m_uiPicParamSetLen;
        int nNewSize = nCurrentFormatBlockSize + iAdditionalLength;
        pMediaType->ReallocFormatBuffer(nNewSize);

        BYTE* pFormat = pMediaType->Format();
        BYTE* pStartPos = &(pFormat[nCurrentFormatBlockSize]);
        // copy SPS
        memcpy(pStartPos, m_pSeqParamSet, m_uiSeqParamSetLen);
        pStartPos += m_uiSeqParamSetLen;
        // copy PPS
        memcpy(pStartPos, m_pPicParamSet, m_uiPicParamSetLen);
        pStartPos += m_uiPicParamSetLen;
        // Copy additional header size
        memcpy(pStartPos, &iAdditionalLength, sizeof(int));
      }
      else
      {
        // not configured: just copy in size of 0
        pMediaType->ReallocFormatBuffer(nCurrentFormatBlockSize + sizeof(int));
        BYTE* pFormat = pMediaType->Format();
        memset(pFormat + nCurrentFormatBlockSize, 0, sizeof(int));
      }
    }
    else if (m_nH264Type == H264_H264)
    {
      // Get the input pin's media type and return this as the output media type - we want to retain
      // all the information about the image
      HRESULT hr = m_pInput->ConnectionMediaType(pMediaType);
      if (FAILED(hr))
      {
        return hr;
      }
      VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
      REFERENCE_TIME avgTimePerFrame = pVih->AvgTimePerFrame;
   
      pMediaType->InitMediaType();
      pMediaType->SetType(&MEDIATYPE_Video);
      pMediaType->SetSubtype(&MEDIASUBTYPE_H264);
      pMediaType->SetFormatType(&FORMAT_VideoInfo2);
      VIDEOINFOHEADER2* pvi2 = (VIDEOINFOHEADER2*)pMediaType->AllocFormatBuffer(sizeof(VIDEOINFOHEADER2));
      ZeroMemory(pvi2, sizeof(VIDEOINFOHEADER2));
      pvi2->bmiHeader.biBitCount = 24;
      pvi2->bmiHeader.biSize = 40;
      pvi2->bmiHeader.biPlanes = 1;
      pvi2->bmiHeader.biWidth = m_nInWidth;
      pvi2->bmiHeader.biHeight = m_nInHeight;
      pvi2->bmiHeader.biSize = m_nInWidth * m_nInHeight * 3;
      pvi2->bmiHeader.biSizeImage = DIBSIZE(pvi2->bmiHeader);
      pvi2->bmiHeader.biCompression = DWORD('1cva');
      pvi2->AvgTimePerFrame = avgTimePerFrame;
      SetRect(&pvi2->rcSource, 0, 0, m_nInWidth, m_nInHeight);
      pvi2->rcTarget = pvi2->rcSource;

      pvi2->dwPictAspectRatioX = m_nInWidth;
      pvi2->dwPictAspectRatioY = m_nInHeight;
    }
    else if (m_nH264Type == H264_AVC1)
    {
      // Get the input pin's media type and return this as the output media type - we want to retain
      // all the information about the image
      HRESULT hr = m_pInput->ConnectionMediaType(pMediaType);
      if (FAILED(hr))
      {
        return hr;
      }
      VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
      REFERENCE_TIME avgTimePerFrame = pVih->AvgTimePerFrame;

      pMediaType->InitMediaType();
      pMediaType->SetType(&MEDIATYPE_Video);
      pMediaType->SetSubtype(&MEDIASUBTYPE_AVC1);
      pMediaType->SetFormatType(&FORMAT_MPEG2Video);

      ASSERT(m_uiSeqParamSetLen > 0 && m_uiPicParamSetLen > 0);

      // ps have start codes: for this media type we replace each start code with 2 byte lenght prefix
      int psLen = m_uiSeqParamSetLen + m_uiPicParamSetLen - 4;
      BYTE* pFormatBuffer = pMediaType->AllocFormatBuffer(sizeof(MPEG2VIDEOINFO) + psLen);
      MPEG2VIDEOINFO* pMpeg2Vih = (MPEG2VIDEOINFO*)pFormatBuffer;

      ZeroMemory(pMpeg2Vih, sizeof(MPEG2VIDEOINFO) + psLen);

      pMpeg2Vih->dwFlags = 4;
      pMpeg2Vih->dwProfile = 66;
      pMpeg2Vih->dwLevel = 20;

      pMpeg2Vih->cbSequenceHeader = psLen;
      int iCurPos = 0;

      BYTE* pSequenceHeader = (BYTE*)&pMpeg2Vih->dwSequenceHeader[0];
      // parameter set length includes start code
      pSequenceHeader[iCurPos] = ((m_uiSeqParamSetLen - 4) >> 8);
      pSequenceHeader[iCurPos + 1] = ((m_uiSeqParamSetLen - 4) & 0xFF);
      memcpy(pSequenceHeader + iCurPos + 2, m_pSeqParamSet + 4, m_uiSeqParamSetLen - 4);
      iCurPos += m_uiSeqParamSetLen - 4 + 2;
      pSequenceHeader[iCurPos] = ((m_uiPicParamSetLen - 4) >> 8);
      pSequenceHeader[iCurPos + 1] = ((m_uiPicParamSetLen - 4) & 0xFF);
      memcpy(pSequenceHeader + iCurPos + 2, m_pPicParamSet + 4, m_uiPicParamSetLen - 4);

      VIDEOINFOHEADER2* pvi2 = &pMpeg2Vih->hdr;
      pvi2->bmiHeader.biBitCount = 24;
      pvi2->bmiHeader.biSize = 40;
      pvi2->bmiHeader.biPlanes = 1;
      pvi2->bmiHeader.biWidth = m_nInWidth;
      pvi2->bmiHeader.biHeight = m_nInHeight;
      pvi2->bmiHeader.biSizeImage = DIBSIZE(pvi2->bmiHeader);
      pvi2->bmiHeader.biCompression = DWORD('1cva');
      pvi2->AvgTimePerFrame = avgTimePerFrame;
      //SetRect(&pvi2->rcSource, 0, 0, m_cx, m_cy);
      SetRect(&pvi2->rcSource, 0, 0, m_nInWidth, m_nInHeight);
      pvi2->rcTarget = pvi2->rcSource;

      pvi2->dwPictAspectRatioX = m_nInWidth;
      pvi2->dwPictAspectRatioY = m_nInHeight;
    }
    return S_OK;
	}
	return VFW_S_NO_MORE_ITEMS;
}

HRESULT H264EncoderFilter::DecideBufferSize( IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp )
{
	AM_MEDIA_TYPE mt;
	HRESULT hr = m_pOutput->ConnectionMediaType(&mt);
	if (FAILED(hr))
	{
		return hr;
	}

  if (m_nH264Type == H264_H264)
  {
    //Make sure that the format type is our custom format
    ASSERT(mt.formattype == FORMAT_VideoInfo2);
    VIDEOINFOHEADER2* pVih = (VIDEOINFOHEADER2*)mt.pbFormat;
    BITMAPINFOHEADER *pbmi = &pVih->bmiHeader;
    //TOREVISE: Should actually change mode and see what the maximum size is per frame?
    pProp->cbBuffer = DIBSIZE(*pbmi);
  }
  else if (m_nH264Type == H264_AVC1)
  {
    //Make sure that the format type is our custom format
    ASSERT(mt.formattype == FORMAT_MPEG2Video);
    MPEG2VIDEOINFO* pMpeg2Vih = (MPEG2VIDEOINFO*)mt.pbFormat;
    BITMAPINFOHEADER *pbmi = &pMpeg2Vih->hdr.bmiHeader;
    //TOREVISE: Should actually change mode and see what the maximum size is per frame?
    pProp->cbBuffer = DIBSIZE(*pbmi);
  }
  else if (m_nH264Type == H264_VPP)
  {
    ASSERT(mt.formattype == FORMAT_VideoInfo);
    BITMAPINFOHEADER *pbmi = HEADER(mt.pbFormat);
    //TOREVISE: Should actually change mode and see what the maximum size is per frame?
    pProp->cbBuffer = DIBSIZE(*pbmi);
  }

	if (pProp->cbAlign == 0)
	{
		pProp->cbAlign = 1;
	}
	if (pProp->cBuffers == 0)
	{
		pProp->cBuffers = 1;
	}
	// Release the format block.
	FreeMediaType(mt);

	// Set allocator properties.
	ALLOCATOR_PROPERTIES Actual;
	hr = pAlloc->SetProperties(pProp, &Actual);
	if (FAILED(hr)) 
	{
		return hr;
	}
	// Even when it succeeds, check the actual result.
	if (pProp->cbBuffer > Actual.cbBuffer) 
	{
		return E_FAIL;
	}
	return S_OK;
}

inline unsigned H264EncoderFilter::getParameterSetLength() const
{
  return m_uiSeqParamSetLen + m_uiPicParamSetLen;
}

inline unsigned H264EncoderFilter::copySequenceAndPictureParameterSetsIntoBuffer( BYTE* pBuffer )
{
  // The SPS and PPS contain start code
  assert (m_uiSeqParamSetLen > 0 && m_uiPicParamSetLen > 0);
  memcpy(pBuffer, (void*)m_pSeqParamSet, m_uiSeqParamSetLen);
  pBuffer += m_uiSeqParamSetLen;
  memcpy(pBuffer, m_pPicParamSet, m_uiPicParamSetLen);
  pBuffer += m_uiPicParamSetLen;
  return getParameterSetLength();
}

HRESULT H264EncoderFilter::Transform( IMediaSample *pSource, IMediaSample *pDest )
{
  HRESULT hr = CCustomBaseFilter::Transform(pSource, pDest);
#if 0
  pDest->SetTime( &m_tStart, &m_tStop );
  // inc
  m_tStart = m_tStop;
  m_tStop += m_rtFrameLength;
#endif
  return hr;
}

HRESULT H264EncoderFilter::ApplyTransform(BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{
  // lock filter so that it can not be reconfigured during a code operation
  CAutoLock lck(&m_csCodec);
  lOutActualDataLength = 0;
	//make sure we were able to initialise our Codec
	if (m_pCodec)
	{
		if (m_pCodec->Ready())
		{
      if (m_uiIFramePeriod)
      {
        ++m_uiCurrentFrame;
        if (m_uiCurrentFrame%m_uiIFramePeriod == 0)
        {
          m_pCodec->Restart();
        }
      }

			int nFrameBitLimit = 0;
			if (m_nFrameBitLimit == 0)
				// An encoded frame can never be bigger than an RGB format frame
				nFrameBitLimit = m_nInWidth * m_nInHeight * 3 /* RGB */ * 8 /*8 bits*/ ;
			else
				nFrameBitLimit = m_nFrameBitLimit;

      BYTE* pOutBufferPos = pBufferOut;

      DbgLog((LOG_TRACE, 0, 
        TEXT("H264 Codec Byte Limit: %d"), nFrameBitLimit));
      int nResult = m_pCodec->Code(pBufferIn, pOutBufferPos, nFrameBitLimit);
      if (nResult)
      {
        //Encoding was successful
        lOutActualDataLength += m_pCodec->GetCompressedByteLength();

        // check if an i-frame was encoded
        int nLen = 0;
        char szBuffer[20];
        m_pCodec->GetParameter("last pic coding type",&nLen, szBuffer );
        if (strcmp(szBuffer, "0") == 0 /*H264V2_INTRA*/)
        {
          if (m_bNotifyOnIFrame)
          {
            SetNotificationMessage("I-Frame");
          }
        }

        // if we're outputting AVC1 we need to replace start codes with length prefixes
        // This will have to be inefficient for now
        if (m_nH264Type == H264_AVC1)
        {
          std::vector<int> vStartCodePositions;
          vStartCodePositions.push_back(0);
          const BYTE startCode[4] = { 0, 0, 0, 1 };
          for (int i = 4; i < m_pCodec->GetCompressedByteLength(); ++i)
          {
            if (memcmp(pOutBufferPos + i, startCode, 4) == 0)
            {
              vStartCodePositions.push_back(i);
            }
          }
          int remaining = m_pCodec->GetCompressedByteLength();
          for (size_t i = 0; i < vStartCodePositions.size() - 1; ++i)
          {
            int len = vStartCodePositions[i + 1] - vStartCodePositions[i] - 4;
            // update length in buffer
            int pos = vStartCodePositions[i];
            pOutBufferPos[pos] = (len >> 24);
            pOutBufferPos[pos + 1] = (len >> 16);
            pOutBufferPos[pos + 2] = (len >> 8);
            pOutBufferPos[pos + 3] = (len & 0xFF);
            remaining -= (len + 4);
          }
          // replace last element
          int len = remaining - 4;
          int pos = vStartCodePositions[vStartCodePositions.size() - 1];
          pOutBufferPos[pos] = (len >> 24);
          pOutBufferPos[pos + 1] = (len >> 16);
          pOutBufferPos[pos + 2] = (len >> 8);
          pOutBufferPos[pos + 3] = (len & 0xFF);
        }

#ifdef USE_FILE_SOURCE
				DbgLog((LOG_TRACE, 0, TEXT("H264 Codec Success: Bit Length: %d Byte Length: %d"), m_pCodec->GetCompressedBitLength(), m_pCodec->GetCompressedByteLength()));

                // HACK: replace with data from file
        readNalUnit();

        memcpy( pBufferOut, m_pBuffer, m_uiCurrentNalUnitSize );
        lOutActualDataLength = m_uiCurrentNalUnitSize;
#endif
			}
			else
			{
				//An error has occurred
				DbgLog((LOG_TRACE, 0, TEXT("H264 Codec Error: %s"), m_pCodec->GetErrorStr()));
				std::string sError = m_pCodec->GetErrorStr();
				sError += ". Requested frame bit limit=" + artist::toString(nFrameBitLimit) + ".";
      
#if 1
        m_pCodec->Restart();
        SetLastError(sError.c_str(), true);
        lOutActualDataLength = 0;
#else
        // This alternative is more sophisticated in that it tries to encode the same picture again
        // The other solution above is simpler and cleaner though. Errors are handled in the base class
        // by returning S_FALSE from the Transform method which causes the current frame not to be deliverd
        // downstream

        // Notify outer layer
        SetNotificationMessage(sError.c_str());
        // restart the codec here and try again
        m_pCodec->Restart();
        
        int nResult = m_pCodec->Code(pBufferIn, pOutBufferPos, nFrameBitLimit);
        if (nResult)
        {
          //Encoding was successful
          lOutActualDataLength += m_pCodec->GetCompressedByteLength();

          // check if an i-frame was encoded
          int nLen = 0;
          char szBuffer[20];
          m_pCodec->GetParameter("last pic coding type",&nLen, szBuffer );
          if (strcmp(szBuffer, "0") == 0 /*H264V2_INTRA*/)
          {
            if (m_bNotifyOnIFrame)
            {
              SetNotificationMessage("I-Frame");
            }
          }

          DbgLog((LOG_TRACE, 0, TEXT("H264 Codec Success on second attempt: Bit Length: %d Byte Length: %d"), m_pCodec->GetCompressedBitLength(), m_pCodec->GetCompressedByteLength()));
        }
        else
        {
          //An error has occurred
          DbgLog((LOG_TRACE, 0, TEXT("H264 Codec Error: %s"), m_pCodec->GetErrorStr()));
          std::string sError = m_pCodec->GetErrorStr();
          sError += ". Requested frame bit limit=" + toString(nFrameBitLimit) + ".";
          SetLastError(sError.c_str(), true);
          m_pCodec->Restart();
          lOutActualDataLength = 0;
        }
#endif
      }
		}
	}
  return S_OK;
}

HRESULT H264EncoderFilter::CheckTransform( const CMediaType *mtIn, const CMediaType *mtOut )
{
	// Check the major type.
	if (mtOut->majortype != MEDIATYPE_Video)
	{
		return VFW_E_TYPE_NOT_ACCEPTED;
	}

  if (m_nH264Type == H264_H264)
  {
    if (mtOut->subtype != MEDIASUBTYPE_H264)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }

    if (mtOut->formattype != FORMAT_VideoInfo2)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }
  else if (m_nH264Type == H264_AVC1)
  {
    if (mtOut->subtype != MEDIASUBTYPE_AVC1)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }

    if (mtOut->formattype != FORMAT_MPEG2Video)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }
  else if (m_nH264Type == H264_VPP)
  {
    if (mtOut->subtype != MEDIASUBTYPE_VPP_H264)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }

    if (mtOut->formattype != FORMAT_VideoInfo)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }

  // Everything is good.
	return S_OK;
}

STDMETHODIMP H264EncoderFilter::GetParameter( const char* szParamName, int nBufferSize, char* szValue, int* pLength )
{
	if (SUCCEEDED(CCustomBaseFilter::GetParameter(szParamName, nBufferSize, szValue, pLength)))
	{
		return S_OK;
	}
	else
	{
    CAutoLock lck(&m_csCodec);
		// Check if its a codec parameter
		if (m_pCodec && m_pCodec->GetParameter(szParamName, pLength, szValue))
		{
			return S_OK;
		}
		return E_FAIL;
	}
}

STDMETHODIMP H264EncoderFilter::SetParameter( const char* type, const char* value )
{
	if (SUCCEEDED(CCustomBaseFilter::SetParameter(type, value)))
	{
		return S_OK;
	}
	else
	{
    CAutoLock lck(&m_csCodec);
		// Check if it's a codec parameter
		if (m_pCodec && m_pCodec->SetParameter(type, value))
		{
      // re-open codec if certain parameters have changed
      if ( 
        (_strnicmp(type, CODEC_PARAM_MODE_OF_OPERATION, strlen(type)) == 0) ||
        (_strnicmp(type, CODEC_PARAM_QUALITY, strlen(type)) == 0)
         )
      {
        if (!m_pCodec->Open())
        {
          //Houston: we have a failure
          char* szErrorStr = m_pCodec->GetErrorStr();
          printf("%s\n", szErrorStr);
          SetLastError(szErrorStr, true);
          return E_FAIL;
        }
      }
			return S_OK;
		}
		return E_FAIL;
	}
}

STDMETHODIMP H264EncoderFilter::GetParameterSettings( char* szResult, int nSize )
{
	if (SUCCEEDED(CCustomBaseFilter::GetParameterSettings(szResult, nSize)))
	{
		// Now add the codec parameters to the output:
		int nLen = strlen(szResult);
		char szValue[10];
		int nParamLength = 0;
    CAutoLock lck(&m_csCodec);
		std::string sCodecParams("Codec Parameters:\r\n");
		if( m_pCodec->GetParameter("parameters", &nParamLength, szValue))
		{
			int nParamCount = atoi(szValue);
			char szParamValue[256];
			memset(szParamValue, 0, 256);

			int nLenName = 0, nLenValue = 0;
			for (int i = 0; i < nParamCount; i++)
			{
				// Get parameter name
				const char* szParamName;
				m_pCodec->GetParameterName(i, &szParamName, &nLenName);
				// Now get the value
				m_pCodec->GetParameter(szParamName, &nLenValue, szParamValue);
				sCodecParams += "Parameter: " + std::string(szParamName) + " : Value:" + std::string(szParamValue) + "\r\n";
			}
			// now check if the buffer is big enough:
			int nTotalSize = sCodecParams.length() + nLen;
			if (nTotalSize < nSize)
			{
				memcpy(szResult + nLen, sCodecParams.c_str(), sCodecParams.length());
				// Set null terminator
				szResult[nTotalSize] = 0;
				return S_OK;
			}
			else
			{
				return E_FAIL;
			}
		}
		else
		{
			return E_FAIL;
		}
	}
	else
	{
		return E_FAIL;
	}
}

STDMETHODIMP H264EncoderFilter::GetFramebitLimit(int& iFrameBitLimit)
{
  iFrameBitLimit = m_nFrameBitLimit;
  return S_OK;
}

STDMETHODIMP H264EncoderFilter::SetFramebitLimit(int iFrameBitLimit)
{
  // lock filter so that it can not be reconfigured during a code operation
  CAutoLock lck(&m_csCodec);

  if (iFrameBitLimit <= 0)
  {
    return E_FAIL;
  }
  m_nFrameBitLimit = iFrameBitLimit;
  return S_OK;
}

STDMETHODIMP H264EncoderFilter::GetGroupId(int& iGroupId)
{
  return E_NOTIMPL;
}

STDMETHODIMP H264EncoderFilter::SetGroupId(int iGroupId)
{
  return E_NOTIMPL;
}

STDMETHODIMP H264EncoderFilter::GenerateIdr()
{
  // lock filter so that it can not be reconfigured during a code operation
  CAutoLock lck(&m_csCodec);
  m_pCodec->Restart();
  return S_OK;
}
