/** @file

MODULE				: CropFilter

FILE NAME			: CropFilter.cpp

DESCRIPTION			:

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2008-2017, CSIR
Copyright (c) 2025-2026, Jaroslav Fojtik
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

==========================================================================*/
#include "CropFilter.h"
#include <DirectShow/CommonDefs.h>
#include <PicCropperRGB24Impl.h>
#include <PicCropperRGB32Impl.h>
#include <Dvdmedia.h>			// VIDEOINFOHEADER2

#include "../VariantTools.cpp"


DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);


 CropFilter::CropFilter(): CCustomBaseFilter(NAME("DQ VPP Crop Filter"), 0, CLSID_VPP_CropFilter),
  m_pCropper(NULL),
  m_pCropBuffer(NULL),
  m_nBitsPerPixel(BITS_PER_PIXEL_RGB24),
  m_nStride(0),
  m_nPadding(0),
  m_DirtyPropsStream(true)
{
  //Call the initialise input method to load all acceptable input types for this filter
  InitialiseInputTypes();
  // Init parameters
  initParameters();
}

 CropFilter::~CropFilter()
{
  if (m_pCropper)
  {
    delete m_pCropper;
    m_pCropper = NULL;
  }
  if (m_pCropBuffer)
  {
    delete m_pCropBuffer;
    m_pCropBuffer = NULL;
  }
}

CUnknown * WINAPI CropFilter::CreateInstance(LPUNKNOWN pUnk, HRESULT *pHr)
{
  CropFilter *pFilter = new CropFilter();
  if (pFilter == NULL)
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pFilter;
}


void CropFilter::InitialiseInputTypes()
{
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB24, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_RGB32, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_ARGB32, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_AYUV, &FORMAT_VideoInfo);
  //AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YV24, &FORMAT_VideoInfo);
}


HRESULT CropFilter::SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt)
{
  HRESULT hr = CCustomBaseFilter::SetMediaType(direction, pmt);
  if (direction == PINDIR_INPUT)
  {
    //Set defaults and make sure that the target dimensions are valid
    if ((m_nOutWidth == 0) || (labs(m_nOutWidth) > labs(m_nInWidth)))
    {
      m_nOutWidth = m_nInWidth;
    }
    if ((m_nOutHeight == 0) || (labs(m_nOutHeight) > labs(m_nInHeight)))
    {
      m_nOutHeight = m_nInHeight;
    }
    //Determine whether we are connected to a RGB24 or 32 source
    if (pmt->majortype == MEDIATYPE_Video)
    {
      //The converter might already exist if the filter has been connected previously
      if (m_pCropper)
      {
        delete m_pCropper;
        m_pCropper = NULL;
      }
      if (m_pCropBuffer)
      {
        delete m_pCropBuffer;
        m_pCropBuffer = NULL;
      }
      if(pmt->subtype==MEDIASUBTYPE_RGB24 || pmt->subtype==MEDIASUBTYPE_YV24)
      {
        m_pCropper = new PicCropperRGB24Impl();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB24;
        // Create temp buffer for crop
        m_pCropBuffer = new BYTE[(labs(m_nOutWidth*m_nOutHeight) * m_nBitsPerPixel)/8];
      }
      else if(pmt->subtype==MEDIASUBTYPE_RGB32 || pmt->subtype==MEDIASUBTYPE_ARGB32 || pmt->subtype==MEDIASUBTYPE_AYUV)
      {
        m_pCropper = new PicCropperRGB32Impl();
        m_nBitsPerPixel = BITS_PER_PIXEL_RGB32;
        // Create temp buffer for crop
        m_pCropBuffer = new BYTE[(labs(m_nOutWidth*m_nOutHeight) * m_nBitsPerPixel)/8];
      }
    }
    RecalculateFilterParameters();
  }
  return hr;
}


HRESULT CropFilter::GetMediaType(int iPosition, CMediaType *pMediaType)
{
  if(iPosition < 0) return E_INVALIDARG;
  if(pMediaType == NULL) return E_POINTER;

  if (iPosition == 0)
  {
    // Get the input pin's media type and return this as the output media type - we want to retain
    // all the information about the image
    HRESULT hr = m_pInput->ConnectionMediaType(pMediaType);
    if (FAILED(hr))
    {
      return hr;
    }

    // Get the bitmap info header and adapt the cropped 
    //make sure that it's a video info header
    if(pMediaType->formattype!=FORMAT_VideoInfo && pMediaType->formattype!=FORMAT_VideoInfo2)
        return VFW_E_TYPE_NOT_ACCEPTED;
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
    //Now we need to calculate the size of the output image
    BITMAPINFOHEADER* pBi = &(pVih->bmiHeader);

    pBi->biHeight = m_nOutHeight;
    if(pBi->biHeight == 0) return E_INVALIDARG;

    pBi->biWidth = m_nOutWidth;
    if(pBi->biWidth <= 0) return E_INVALIDARG;

    pBi->biSizeImage = (labs(pBi->biWidth*pBi->biHeight) * m_nBitsPerPixel)/8;

    pVih->rcSource.top = 0;
    pVih->rcSource.left = 0;
    pVih->rcSource.right = pBi->biWidth;
    pVih->rcSource.bottom = pBi->biHeight;

    pVih->rcTarget.top = 0;
    pVih->rcTarget.left = 0;
    pVih->rcTarget.right = pBi->biWidth;
    pVih->rcTarget.bottom = pBi->biHeight;

    pMediaType->lSampleSize = pBi->biSizeImage;

    return S_OK;
  }
  return VFW_S_NO_MORE_ITEMS;
}


HRESULT CropFilter::DecideBufferSize(IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp)
{
  // Adding padding to take stride into account
  pProp->cbBuffer = ((m_nOutWidth + m_nPadding) * m_nOutHeight * m_nBitsPerPixel) / 8;

  if (pProp->cbAlign == 0)
  {
    pProp->cbAlign = 1;
  }
  if (pProp->cBuffers == 0)
  {
    pProp->cBuffers = 1;
  }

  // Set allocator properties.
  ALLOCATOR_PROPERTIES Actual;
  HRESULT hr = pAlloc->SetProperties(pProp, &Actual);
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


HRESULT CropFilter::CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut)
{
  //Make sure the input and output types are related
  if (mtOut->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  	// Video format
  m_Vflip = false;
  if(mtOut->formattype==FORMAT_VideoInfo)
  {
    const VIDEOINFOHEADER * const pVihOut = (const VIDEOINFOHEADER*)mtOut->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(pBiOut->biHeight != m_nOutHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutHeight))
        m_Vflip = true;
      else
      {
        DbgLog((LOG_TRACE, 0, TEXT("Crop - Unaccepted height at last negoitation stage")));
        return VFW_E_TYPE_NOT_ACCEPTED;
      }        
    }
    if(pBiOut->biWidth != m_nOutWidth)
    {
      DbgLog((LOG_TRACE, 0, TEXT("Crop - Unaccepted width at last negoitation stage")));
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }
  else if(mtOut->formattype==FORMAT_VideoInfo2)
  {
    const VIDEOINFOHEADER2 * const pVihOut = (const VIDEOINFOHEADER2*)mtOut->pbFormat;
    const BITMAPINFOHEADER* const pBiOut = &(pVihOut->bmiHeader);
    if(pBiOut->biHeight != m_nOutHeight)
    {
      if(labs(pBiOut->biHeight) == labs(m_nOutHeight))
        m_Vflip = true;
      else
      {
        DbgLog((LOG_TRACE, 0, TEXT("Crop - Unaccepted height at last negoitation stage")));
        return VFW_E_TYPE_NOT_ACCEPTED;
      } 
    }
    if(pBiOut->biWidth != m_nOutWidth)
    {
      DbgLog((LOG_TRACE, 0, TEXT("Crop - Unaccepted width at last negoitation stage")));
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
  }
  else
      return VFW_E_TYPE_NOT_ACCEPTED;

	//Subtypes
  if(mtIn->subtype == MEDIASUBTYPE_RGB24)
  {
    if (mtOut->subtype != MEDIASUBTYPE_RGB24)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  }
  if(mtIn->subtype == MEDIASUBTYPE_YV24)
  {
    if (mtOut->subtype != MEDIASUBTYPE_YV24)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if (mtIn->subtype == MEDIASUBTYPE_AYUV)
  {
    if (mtOut->subtype != MEDIASUBTYPE_AYUV)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 
  if(mtIn->subtype == MEDIASUBTYPE_RGB32 || mtIn->subtype == MEDIASUBTYPE_ARGB32)
  {
    if(mtOut->subtype != MEDIASUBTYPE_RGB32 && mtOut->subtype != MEDIASUBTYPE_ARGB32)
    {
      return VFW_E_TYPE_NOT_ACCEPTED;
    }
    return S_OK;
  } 

  return VFW_E_TYPE_NOT_ACCEPTED;
}


void CropFilter::initParameters()
{
  addParameter(FILTER_PARAM_TARGET_WIDTH, &m_nOutWidth, -1, true);
  addParameter(FILTER_PARAM_TARGET_HEIGHT, &m_nOutHeight, -1, true);
  addParameter(FILTER_PARAM_LEFT_CROP, &m_nLeftCrop, 0);
  addParameter(FILTER_PARAM_RIGHT_CROP, &m_nRightCrop, 0);
  addParameter(FILTER_PARAM_TOP_CROP, &m_nTopCrop, 0);
  addParameter(FILTER_PARAM_BOTTOM_CROP, &m_nBottomCrop, 0);
  RecalculateFilterParameters();
}


STDMETHODIMP CropFilter::SetParameter(const char* type, const char* value)
{
  // For now, one cannot set any parameters once the output has been connected -> this will affect the buffer size
  if (m_pOutput)
  {
    if (m_pOutput->IsConnected())
    {
      return E_FAIL;
    }
  }

  if (SUCCEEDED(CSettingsInterface::SetParameter(type, value)))
  {
    RecalculateFilterParameters();
    return S_OK;
  }
  else
  {
    return E_FAIL;
  }
}

HRESULT CropFilter::ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{
  //make sure we were able to initialise our cropper
  ASSERT(m_pCropper);
  // TODO: Add stride parameter for crop class to avoid this extra mem alloc
  //Call cropping conversion code
  m_pCropper->SetInDimensions(m_nInWidth, m_nInHeight);
  m_pCropper->SetOutDimensions(m_nOutWidth, m_nOutHeight);
  m_pCropper->SetCrop(m_nLeftCrop, m_nRightCrop, m_nTopCrop, m_nBottomCrop);
  int res = m_pCropper->Crop(pBufferIn, m_pCropBuffer, m_Vflip);
  ASSERT(res == 1);
  // Copy the cropped image with stride padding
  BYTE* pFrom = m_pCropBuffer;
  BYTE* pTo = pBufferOut;

  int nBytesPerLine = (m_nOutWidth * m_nBitsPerPixel) / 8;
  for (size_t i = 0; i < (size_t)m_nOutHeight; i++)
  {
    memcpy(pTo, pFrom, nBytesPerLine);
    pFrom += nBytesPerLine;
    pTo += nBytesPerLine;
    for (size_t j = 0; j < (size_t)m_nPadding; j++)
    {
      *pTo = 0;
      pTo++;
    }
  }
  lOutActualDataLength = ((m_nOutWidth + m_nPadding) * m_nOutHeight * m_nBitsPerPixel) / 8;
  return S_OK;
}


void CropFilter::RecalculateFilterParameters(void)
{
  //Set defaults and make sure that the target dimensions are valid
  m_nOutWidth = labs(m_nInWidth) - m_nLeftCrop - m_nRightCrop;
  m_nOutHeight = labs(m_nInHeight) - m_nTopCrop - m_nBottomCrop;
  m_nOutPixels = labs(m_nOutWidth * m_nOutHeight);
#if 0
  if (m_nBytesPerPixel == BYTES_PER_PIXEL_RGB24)
  {
    m_nStride = (m_nOutWidth * (m_nBitCount / 8) + 3) & ~3;
    m_nPadding = m_nStride - (m_nBytesPerPixel * m_nOutWidth);
  }
  else
  {
    m_nStride = 0;
    m_nPadding = 0;
  }
#else
  m_nStride = ((((m_nOutWidth * m_nBitCount) + 31) & ~31) >> 3);
  m_nPadding = m_nStride - (m_nBitsPerPixel * m_nOutWidth)/8;
#endif
}


HRESULT CropFilter::SetCropIfValid(int nTotalDimensionImage, int nNewCrop, int& nOldCrop, int nOppositeCrop)
{
  if (nNewCrop >= 0)
  {
    if (nTotalDimensionImage - nOppositeCrop - nNewCrop > 0)
    {
      nOldCrop = nNewCrop;
      RecalculateFilterParameters();
      return S_OK;
    }
    else
    {
      // Fail: Total crop exceeds image dimension
      return E_FAIL;
    }
  }
  else
  {
    // negative value
    return E_FAIL;
  }
}

HRESULT CropFilter::GetPages(CAUUID *pPages)
{
  if (pPages == NULL) return E_POINTER;
  pPages->cElems = 1;
  pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
  if (pPages->pElems == NULL)
  {
    return E_OUTOFMEMORY;
  }
  pPages->pElems[0] = CLSID_CropProperties;
  return S_OK;
}


HRESULT CropFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == IID_IPersistPropertyBag)
      return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPropertyBag)
      return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_ISpecifyPropertyPages)
      return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  if(riid == IID_IPersistStream)
      return GetInterface((IPersistStream*)this, ppv);
	// Call the parent class.
  return CCustomBaseFilter::NonDelegatingQueryInterface(riid, ppv);  
}


//----------------- IPersistPropertyBag------------------


HRESULT CropFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_CropFilter;
  return S_OK;
}


HRESULT CropFilter::InitNew(void)
{
  m_nLeftCrop = m_nRightCrop = m_nTopCrop = m_nBottomCrop = 0;
  return S_OK;
}


OLECHAR OLE_PARAM_LEFT_CROP[] =  olestr(FILTER_PARAM_LEFT_CROP);
OLECHAR OLE_PARAM_RIGHT_CROP[] = olestr(FILTER_PARAM_RIGHT_CROP);
OLECHAR OLE_PARAM_TOP_CROP[] =  olestr(FILTER_PARAM_TOP_CROP);
OLECHAR OLE_PARAM_BOTTOM_CROP[] = olestr(FILTER_PARAM_BOTTOM_CROP);


HRESULT CropFilter::Load(IPropertyBag *pPropBag, IErrorLog *pErrorLog)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  memset(&var,0,sizeof(var));

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_LEFT_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_LEFT_CROP,sizeof(OLE_PARAM_LEFT_CROP));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      getFromVariant(m_nLeftCrop,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_RIGHT_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_RIGHT_CROP,sizeof(OLE_PARAM_RIGHT_CROP));
    VariantClear(&var);
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      getFromVariant(m_nRightCrop,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_TOP_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TOP_CROP,sizeof(OLE_PARAM_TOP_CROP));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      getFromVariant(m_nTopCrop,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_BOTTOM_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_BOTTOM_CROP,sizeof(OLE_PARAM_BOTTOM_CROP));
    VariantClear(&var);
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      getFromVariant(m_nBottomCrop,&var);
    }
    CoTaskMemFree((void*)PropName);
  }

  VariantClear(&var);
return S_OK;
}


HRESULT CropFilter::Save(IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(FILTER_PARAM_LEFT_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_LEFT_CROP,sizeof(FILTER_PARAM_LEFT_CROP));
    var.vt = VT_INT;
    var.intVal = m_nLeftCrop;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(FILTER_PARAM_RIGHT_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_RIGHT_CROP,sizeof(FILTER_PARAM_RIGHT_CROP));
    var.vt = VT_INT;
    var.intVal = m_nRightCrop;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(FILTER_PARAM_TOP_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_TOP_CROP,sizeof(FILTER_PARAM_TOP_CROP));
    var.vt = VT_INT;
    var.intVal = m_nTopCrop;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(FILTER_PARAM_BOTTOM_CROP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_BOTTOM_CROP,sizeof(FILTER_PARAM_BOTTOM_CROP));
    var.vt = VT_INT;
    var.intVal = m_nBottomCrop;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

return S_OK;
}


//----------------- IPropertyBag------------------

HRESULT CropFilter::Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  VariantClear(pVar);

  if(oleicmp(pszPropName, OLE_PARAM_LEFT_CROP) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nLeftCrop;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_RIGHT_CROP) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nRightCrop;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_TOP_CROP) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nTopCrop;
    return S_OK;
  }
  if(oleicmp(pszPropName, OLE_PARAM_BOTTOM_CROP) == 0)
  {   
    pVar->vt = VT_INT;
    pVar->intVal = m_nBottomCrop;
    return S_OK;
  }

  return E_INVALIDARG;
}


HRESULT CropFilter::Write(LPCOLESTR pszPropName, VARIANT *pVar)
{
CheckPointer(pszPropName, E_INVALIDARG);
CheckPointer(pVar, E_INVALIDARG);
  if(oleicmp(pszPropName, OLE_PARAM_LEFT_CROP) == 0)
  {   
    if(getFromVariant(m_nLeftCrop,pVar)) return S_OK;
  }
  else if(oleicmp(pszPropName, OLE_PARAM_RIGHT_CROP) == 0)
  {
    if(getFromVariant(m_nRightCrop,pVar)) return S_OK;    
  }
  else if(oleicmp(pszPropName, OLE_PARAM_TOP_CROP) == 0)
  {
    if(getFromVariant(m_nTopCrop,pVar)) return S_OK;    
  }
  else if(oleicmp(pszPropName, OLE_PARAM_BOTTOM_CROP) == 0)
  {
    if(getFromVariant(m_nBottomCrop,pVar)) return S_OK;    
  }
  return E_INVALIDARG;
}


//----------------- IPersistStream------------------


HRESULT CropFilter::GetSizeMax(ULARGE_INTEGER *pcbSize)
{
CheckPointer(pcbSize, E_INVALIDARG);
  pcbSize->QuadPart = sizeof(m_nLeftCrop) + sizeof(m_nRightCrop) + sizeof(m_nTopCrop) + sizeof(m_nBottomCrop);
  return S_OK;
}


HRESULT CropFilter::IsDirty(void)
{
  return m_DirtyPropsStream ? S_OK : S_FALSE;
}


HRESULT CropFilter::Load(IStream *pStm)
{
ULONG pcbRead;
int nLeftCrop, nRightCrop, nTopCrop, nBottomCrop;
HRESULT hr;
  ASSERT(sizeof(nLeftCrop)==sizeof(m_nLeftCrop));
  ASSERT(sizeof(nRightCrop)==sizeof(m_nRightCrop));
  ASSERT(sizeof(nTopCrop)==sizeof(m_nTopCrop));
  ASSERT(sizeof(nBottomCrop)==sizeof(m_nBottomCrop));
  CheckPointer(pStm, E_INVALIDARG);

  if(FAILED(hr=pStm->Read(&nLeftCrop, sizeof(nLeftCrop), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nRightCrop, sizeof(nRightCrop), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nTopCrop, sizeof(nTopCrop), &pcbRead))) return hr;
  if(FAILED(hr=pStm->Read(&nBottomCrop, sizeof(nBottomCrop), &pcbRead))) return hr;
  if(m_nLeftCrop!=nLeftCrop || m_nRightCrop!=nRightCrop || m_nTopCrop!=nTopCrop || m_nBottomCrop!=nBottomCrop)
  {
    m_nLeftCrop = nLeftCrop;
    m_nRightCrop = nRightCrop;
    m_nTopCrop = nTopCrop;
    m_nBottomCrop = nBottomCrop;
    m_DirtyPropsStream = true;
  }
return S_OK;
}


HRESULT CropFilter::Save(IStream *pStm, BOOL fClearDirty)
{
HRESULT hr;
CheckPointer(pStm, E_INVALIDARG);
  if(FAILED(hr=pStm->Write(&m_nLeftCrop, sizeof(m_nLeftCrop), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nRightCrop, sizeof(m_nRightCrop), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nTopCrop, sizeof(m_nTopCrop), NULL))) return hr;
  if(FAILED(hr=pStm->Write(&m_nBottomCrop, sizeof(m_nBottomCrop), NULL))) return hr;
  if(fClearDirty) m_DirtyPropsStream=false;
return S_OK;
}

