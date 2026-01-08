/** @file

MODULE				: YUVtoRGBFilter

FILE NAME			: YUVtoRGBFilter.cpp

DESCRIPTION			: 

LICENSE: Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive All rights reserved.

===========================================================================*/
#include "YUVtoRGBFilter.h"
#include "DirectShow/CommonDefs.h"
#include "DirectShow/CustomMediaTypes.h"
#include "DirectShow/DirectShowMediaFormats.h"
#include "AYUV444toRGB24Converter.h"
#include "RealYUV420toRGB24Converter.h"
#include "YUYVtoRGBConverter.h"
#include "YVYUtoRGBConverter.h"

#include <wmcodecdsp.h>			// MEDIASUBTYPE_I420; MEDIASUBTYPE_AVC1
#include <Dvdmedia.h>			// VIDEOINFOHEADER2
#include "../VariantTools.cpp"


DEFINE_GUID(GUID_NULL, 0x00000000, 0x0000, 0x0000, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
DEFINE_GUID(MEDIASUBTYPE_YV24, 0x34325659, 0x0000, 0x0010, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71);


YUVtoRGBFilter::YUVtoRGBFilter():
  CCustomBaseFilter(NAME("DQ VPP YUV to RGB converter"), 0, CLSID_VPP_YUVtoRGBColorConverter),
  m_pConverter(NULL),
  m_nSizeUV(0),
  m_bInvert(false),
  m_DirtyProps(true)
{
  out_MediaType = GUID_NULL;
  in_MediaType = GUID_NULL;

  //Call the initialise input method to load all acceptable input types for this filter
  InitialiseInputTypes();
  initParameters();

  // init converter
  //m_pConverter->SetFlip(m_bInvert);
}


YUVtoRGBFilter::~YUVtoRGBFilter()
{
  if(m_pConverter)
  {
    delete m_pConverter;
    m_pConverter = NULL;
  }
}


CUnknown * WINAPI YUVtoRGBFilter::CreateInstance( LPUNKNOWN pUnk, HRESULT *pHr )
{
  YUVtoRGBFilter *pFilter = new YUVtoRGBFilter();
  if (pFilter== NULL) 
  {
    *pHr = E_OUTOFMEMORY;
  }
  return pFilter;
}


void YUVtoRGBFilter::InitialiseInputTypes()
{
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_AYUV, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YV24, &FORMAT_VideoInfo);

  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUV420P_S, &FORMAT_VideoInfo); // I420 = IYUV = YUV420p (sometimes YUV420p can refer to YV12) https://gist.github.com/Jim-Bar/3cbba684a71d1a9d468a6711a6eddbeb
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_I420, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YV12, &FORMAT_VideoInfo);

  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUYV, &FORMAT_VideoInfo);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUY2, &FORMAT_VideoInfo);

  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YVYU, &FORMAT_VideoInfo);
 
	/////////////////////////////////////////////////
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_AYUV, &FORMAT_VideoInfo2);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YV24, &FORMAT_VideoInfo2);

  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUV420P_S, &FORMAT_VideoInfo2); // I420 = IYUV = YUV420p (sometimes YUV420p can refer to YV12) https://gist.github.com/Jim-Bar/3cbba684a71d1a9d468a6711a6eddbeb
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_I420, &FORMAT_VideoInfo2);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YV12, &FORMAT_VideoInfo2);

  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUYV, &FORMAT_VideoInfo2);
  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YUY2, &FORMAT_VideoInfo2);

  AddInputType(&MEDIATYPE_Video, &MEDIASUBTYPE_YVYU, &FORMAT_VideoInfo2);
}


HRESULT YUVtoRGBFilter::BreakConnect(PIN_DIRECTION dir)
{
  if(m_pConverter) {delete m_pConverter; m_pConverter=NULL;}
  if(dir == PINDIR_INPUT)
    in_MediaType = GUID_NULL;
  else
    out_MediaType = GUID_NULL;
  return CCustomBaseFilter::BreakConnect(dir);
}


HRESULT YUVtoRGBFilter::SetMediaType(PIN_DIRECTION direction, const CMediaType *pmt)
{
  HRESULT hr = CCustomBaseFilter::SetMediaType(direction, pmt);
  if(direction == PINDIR_INPUT)
  {
    in_MediaType = pmt->subtype;
    m_nSizeUV = m_nInPixels >> 2;
  }
  else
  {
    out_MediaType = pmt->subtype;
  }

  if(out_MediaType!=GUID_NULL && in_MediaType!=GUID_NULL)
  {
    if(m_pConverter) {delete m_pConverter; m_pConverter=NULL;}

    if(out_MediaType==MEDIASUBTYPE_RGB32 || out_MediaType==MEDIASUBTYPE_ARGB32)
    {
      if(in_MediaType == MEDIASUBTYPE_AYUV)
          m_pConverter = new AYUV444toARGB32Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType == MEDIASUBTYPE_YV24)
          m_pConverter = new YV24toRGB32Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType==MEDIASUBTYPE_YUV420P_S || in_MediaType==MEDIASUBTYPE_I420 || in_MediaType==MEDIASUBTYPE_YV12)
          m_pConverter = new YUV420toRGB32Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType==MEDIASUBTYPE_YUYV || in_MediaType==MEDIASUBTYPE_YUY2)
          m_pConverter = new YUYVtoARGB32Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType==MEDIASUBTYPE_YVYU)
          m_pConverter = new YVYUtoARGB32Converter(m_nInWidth, m_nInHeight);
    }
    if(out_MediaType == MEDIASUBTYPE_RGB24)
    {
      if(in_MediaType == MEDIASUBTYPE_AYUV)
          m_pConverter = new AYUV444toRGB24Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType == MEDIASUBTYPE_YV24)
          m_pConverter = new YV24toRGB24Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType==MEDIASUBTYPE_YUV420P_S || in_MediaType==MEDIASUBTYPE_I420 || in_MediaType==MEDIASUBTYPE_YV12)
          m_pConverter = new YUV420toRGB24Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType==MEDIASUBTYPE_YUYV || in_MediaType==MEDIASUBTYPE_YUY2)
          m_pConverter = new YUYVtoRGB24Converter(m_nInWidth, m_nInHeight);
      if(in_MediaType==MEDIASUBTYPE_YVYU)
          m_pConverter = new YVYUtoRGB24Converter(m_nInWidth, m_nInHeight);
    }

    if(m_pConverter==NULL) return VFW_E_TYPE_NOT_ACCEPTED;

    if(out_MediaType==MEDIASUBTYPE_RGB32 || out_MediaType==MEDIASUBTYPE_ARGB32 || out_MediaType==MEDIASUBTYPE_RGB24)
    {
      m_pConverter->SetFlip((!m_bInvert) ^ (m_nOutHeight<0));
    }
  }
  return hr;
}


HRESULT YUVtoRGBFilter::GetMediaType(int iPosition, CMediaType *pMediaType)
{
  if (iPosition < 0)
  {
    return E_INVALIDARG;
  }
  if(iPosition > 1)  return VFW_S_NO_MORE_ITEMS;


    // Get the input pin's media type and return this as the output media type - we want to retain
    // all the information about the image
  HRESULT hr = m_pInput->ConnectionMediaType(pMediaType);
  if(FAILED(hr)) return hr;

  BITMAPINFOHEADER *pbmi = NULL;
  if(pMediaType->formattype==FORMAT_VideoInfo)
  {
    VIDEOINFOHEADER * const pVih = (VIDEOINFOHEADER*)pMediaType->pbFormat;
    if(pMediaType->cbFormat >= sizeof(VIDEOINFOHEADER))
        if(pVih) {pbmi=&(pVih->bmiHeader);}
  }
  else if(pMediaType->formattype==FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 * const pVih2 = (VIDEOINFOHEADER2*)pMediaType->pbFormat;
    if(pVih2)
    {
      if(pMediaType->cbFormat >= sizeof(VIDEOINFOHEADER2))
      {
        pbmi = &(pVih2->bmiHeader);
		// This does not work properly, needs to be investigated.
/*
        VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)CoTaskMemAlloc(sizeof(VIDEOINFOHEADER));
        if(pVih)
        {
          //memset(pVih,0,sizeof(VIDEOINFOHEADER));
          pVih->rcSource = pVih2->rcSource;
          pVih->rcTarget = pVih2->rcTarget;
          pVih->dwBitRate = pVih2->dwBitRate;
          pVih->AvgTimePerFrame = pVih2->AvgTimePerFrame;
          pVih->dwBitErrorRate = pVih2->dwBitErrorRate;
          memcpy(&pVih->bmiHeader, pbmi, sizeof(BITMAPINFOHEADER));        

//          AM_MEDIA_TYPE amt;
//	  memcpy(&amt, (AM_MEDIA_TYPE*)pMediaType, sizeof(AM_MEDIA_TYPE));
//          amt.pbFormat = (BYTE*)pVih;
//          amt.cbFormat = sizeof(VIDEOINFOHEADER);
//          pMediaType->Set(amt);
              
          CoTaskMemFree(pMediaType->pbFormat);
          pMediaType->pbFormat = (BYTE*)pVih;
          pMediaType->cbFormat = sizeof(VIDEOINFOHEADER);
          pMediaType->formattype = FORMAT_VideoInfo;
        }
*/
      }
    }
  }
  if(pbmi == NULL)
      return VFW_E_TYPE_NOT_ACCEPTED;


  if(iPosition == 0)
  {
    pbmi->biBitCount = 24;
    pbmi->biSizeImage = m_nInPixels * BITS_PER_PIXEL_RGB24 / 8;
    pMediaType->SetSubtype(&MEDIASUBTYPE_RGB24);
  }
  else
  {
    pbmi->biBitCount = 32;
    pbmi->biSizeImage = m_nInPixels * BITS_PER_PIXEL_RGB32 / 8;
    pMediaType->SetSubtype(&MEDIASUBTYPE_RGB32);
  }


/*
    // negative height of YUV DIBS should be ignored according to 
    // http://msdn.microsoft.com/en-us/library/windows/desktop/dd407212(v=vs.85).aspx
    // we will set our output height negative according to m_bInvert
    if (m_bInvert)
    {
      if (pvi->bmiHeader.biHeight < 0) // make sure height is not already negative
        pvi->bmiHeader.biHeight = -1 * pvi->bmiHeader.biHeight;
    }
    else
    {
      // we will flip the image: make sure height is positive
      if (pvi->bmiHeader.biHeight > 0) // make sure height is not already negative
        pvi->bmiHeader.biHeight = -1 * pvi->bmiHeader.biHeight;
    }
*/

  pMediaType->SetSampleSize(pbmi->biSizeImage);
    // adapt compression in case necessary
  pbmi->biCompression = BI_RGB;
    //Reset the media subtype
    // Change the output format from MEDIASUBTYPE_YUV444P to RGB24
  return S_OK;
}


HRESULT YUVtoRGBFilter::DecideBufferSize( IMemAllocator *pAlloc, ALLOCATOR_PROPERTIES *pProp )
{
  AM_MEDIA_TYPE mt;
  HRESULT hr = m_pOutput->ConnectionMediaType(&mt);
  if (FAILED(hr))
  {
    return hr;
  }
  BITMAPINFOHEADER *pbmi = NULL;

  //Make sure that the format type is our custom format
  if(mt.formattype==FORMAT_VideoInfo)
      pbmi = HEADER(mt.pbFormat);
  if(mt.formattype==FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 * const pVih2 = (VIDEOINFOHEADER2*)mt.pbFormat;
    if(pVih2) pbmi = &(pVih2->bmiHeader);
  }
  if(pbmi==NULL) return E_INVALIDARG;

  pProp->cbBuffer = DIBSIZE(*pbmi);

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
  if(FAILED(hr)) 
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


HRESULT YUVtoRGBFilter::ApplyTransform(const BYTE* pBufferIn, long lInBufferSize, long lActualDataLength, BYTE* pBufferOut, long lOutBufferSize, long& lOutActualDataLength)
{  
 if(m_pConverter==NULL) return E_POINTER;
  ASSERT(pBufferIn);
  ASSERT(pBufferOut);

  lOutActualDataLength = m_nInPixels * m_videoOutHeader.bmiHeader.biBitCount / 8;
  if(lOutBufferSize < lOutActualDataLength) return E_INVALIDARG;

  m_pConverter->Convert(pBufferIn, pBufferOut);
  DbgLog((LOG_TRACE, 0, TEXT("Converted from YUV to RGB Directly")));
  //RGB24 stores 3 Bytes per pixel  
  return S_OK;
}


HRESULT YUVtoRGBFilter::CheckTransform(const CMediaType *mtIn, const CMediaType *mtOut)
{
  //Does this also occur when format changes occur such as when changing the renderer size???
  long lSize = mtOut->GetSampleSize();

  // Check the major type.
  if(mtOut->majortype != MEDIATYPE_Video)
  {
    return VFW_E_TYPE_NOT_ACCEPTED;
  }

  BITMAPINFOHEADER *pBiOut = NULL;
  BITMAPINFOHEADER *pBiIn = NULL;

  if(mtOut->formattype == FORMAT_VideoInfo)
  {
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)mtOut->pbFormat;
    pBiOut = &(pVih->bmiHeader);
  }
  if(mtOut->formattype == FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 *pVih2 = (VIDEOINFOHEADER2*)mtOut->pbFormat;
    pBiOut = &(pVih2->bmiHeader);
  }

  if(mtIn->formattype == FORMAT_VideoInfo)
  {
    VIDEOINFOHEADER *pVih = (VIDEOINFOHEADER*)mtIn->pbFormat;
    pBiIn = &(pVih->bmiHeader);
  }
  if(mtIn->formattype == FORMAT_VideoInfo2)
  {
    VIDEOINFOHEADER2 *pVih2 = (VIDEOINFOHEADER2*)mtIn->pbFormat;
    pBiIn = &(pVih2->bmiHeader);
  }

  if(pBiIn==NULL || pBiOut==NULL) return VFW_E_TYPE_NOT_ACCEPTED;

  if(labs(pBiIn->biWidth)!=labs(pBiOut->biWidth) ||
     labs(pBiIn->biHeight)!=labs(pBiOut->biHeight)) return VFW_E_TYPE_NOT_ACCEPTED;

  bool TransformOK = false;  
  if(mtOut->subtype==MEDIASUBTYPE_RGB32 || mtOut->subtype==MEDIASUBTYPE_ARGB32)
  {
    if(mtIn->subtype == MEDIASUBTYPE_AYUV)
        TransformOK = true;
    if(mtIn->subtype == MEDIASUBTYPE_YV24)
        TransformOK = true;
    if(mtIn->subtype==MEDIASUBTYPE_YUV420P_S || mtIn->subtype==MEDIASUBTYPE_I420 || mtIn->subtype==MEDIASUBTYPE_YV12)
        TransformOK = true;
    if(mtIn->subtype==MEDIASUBTYPE_YUYV || mtIn->subtype==MEDIASUBTYPE_YUY2)
        TransformOK = true;
    if(mtIn->subtype==MEDIASUBTYPE_YVYU)
        TransformOK = true;
  }
  else if(mtOut->subtype == MEDIASUBTYPE_RGB24)
  {
    if(mtIn->subtype == MEDIASUBTYPE_AYUV)
        TransformOK = true;
    if(mtIn->subtype == MEDIASUBTYPE_YV24)
        TransformOK = true;
    if(mtIn->subtype==MEDIASUBTYPE_YUV420P_S || mtIn->subtype==MEDIASUBTYPE_I420 || mtIn->subtype==MEDIASUBTYPE_YV12)
        TransformOK = true;
    if(mtIn->subtype==MEDIASUBTYPE_YUYV || mtIn->subtype==MEDIASUBTYPE_YUY2)
        TransformOK = true;
    if(mtIn->subtype==MEDIASUBTYPE_YVYU)
        TransformOK = true;
  }

  if(!TransformOK)
      return VFW_E_TYPE_NOT_ACCEPTED;  

  return S_OK;	    // Everything is good.
}


STDMETHODIMP YUVtoRGBFilter::SetParameter(const char* type, const char* value)
{
  if (SUCCEEDED(CSettingsInterface::SetParameter(type, value)))
  {
    //m_pConverter->SetChrominanceOffset(m_nChrominanceOffset);
    m_pConverter->SetFlip((!m_bInvert) ^ (m_nOutHeight<0));
    return S_OK;
  }
  else
  {
    return E_FAIL;
  }
}


HRESULT YUVtoRGBFilter::GetPages(CAUUID *pPages)
{
    if (pPages == NULL) return E_POINTER;
    pPages->cElems = 1;
    pPages->pElems = (GUID*)CoTaskMemAlloc(sizeof(GUID));
    if (pPages->pElems == NULL) 
    {
      return E_OUTOFMEMORY;
    }
    pPages->pElems[0] = CLSID_YUVtoRGBProperties;
    return S_OK;
}


 HRESULT YUVtoRGBFilter::NonDelegatingQueryInterface(REFIID riid, void **ppv)
{
  if(riid == IID_IPersistPropertyBag)   return GetInterface(static_cast<IPersistPropertyBag*>(this), ppv);
  if(riid == IID_IPersistStream)	return GetInterface(static_cast<IPersistStream*>(this), ppv);
  if(riid == IID_IPropertyBag)		return GetInterface(static_cast<IPropertyBag*>(this), ppv);
  if(riid == IID_ISpecifyPropertyPages) return GetInterface(static_cast<ISpecifyPropertyPages*>(this), ppv);
  
      // Call the parent class.
  return CCustomBaseFilter::NonDelegatingQueryInterface(riid, ppv);
}


 void YUVtoRGBFilter::initParameters(void)
{
  //addParameter(CHROMINANCE_OFFSET, &m_nChrominanceOffset, 128);
  addParameter(INVERT, &m_bInvert,  m_bInvert);
}


//----------------- IPersistPropertyBag------------------


HRESULT  YUVtoRGBFilter::GetClassID(CLSID *pClassID)
{
  if(pClassID==NULL) return E_POINTER;
  *pClassID = CLSID_VPP_YUVtoRGBColorConverter;
  return S_OK;
}


HRESULT  YUVtoRGBFilter::InitNew(void)
{
  m_nOutWidth = m_nOutHeight = 0;
  return S_OK;
}


OLECHAR OLE_PARAM_FLIP[] =  olestr(FILTER_PARAM_ORIENTATION);


HRESULT YUVtoRGBFilter::Load(IPropertyBag *pPropBag, IErrorLog *pErrorLog)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  memset(&var,0,sizeof(var));

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_FLIP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_FLIP,sizeof(OLE_PARAM_FLIP));
    if(SUCCEEDED(pPropBag->Read(PropName,&var,NULL)))
    {
      int Val;
      IntFromVariant(Val,&var);
      m_bInvert = Val;
    }
    CoTaskMemFree((void*)PropName);
  }

  VariantClear(&var);
return S_OK;
}


HRESULT  YUVtoRGBFilter::Save(IPropertyBag *pPropBag, BOOL fClearDirty, BOOL fSaveAllProperties)
{
LPCOLESTR PropName;
VARIANT var;
  if(pPropBag==NULL) return E_POINTER;

  PropName = (LPCOLESTR)CoTaskMemAlloc(sizeof(OLE_PARAM_FLIP));
  if(PropName)
  {
    memcpy((void*)PropName,OLE_PARAM_FLIP,sizeof(OLE_PARAM_FLIP));
    var.vt = VT_BOOL;
    var.boolVal = m_bInvert;
    pPropBag->Write(PropName,&var);
    CoTaskMemFree((void*)PropName);
  }

return S_OK;
}


//----------------- IPropertyBag------------------


HRESULT  YUVtoRGBFilter::Read(LPCOLESTR pszPropName, VARIANT *pVar, IErrorLog *pErrorLog)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  VariantClear(pVar);

  if(oleicmp(pszPropName, OLE_PARAM_FLIP) == 0)
  {   
    pVar->vt = VT_BOOL;
    pVar->boolVal = m_bInvert;
    return S_OK;
  }

  return E_INVALIDARG;
}


HRESULT  YUVtoRGBFilter::Write(LPCOLESTR pszPropName, VARIANT *pVar)
{
  CheckPointer(pszPropName, E_INVALIDARG);
  CheckPointer(pVar, E_INVALIDARG);

  if(oleicmp(pszPropName, OLE_PARAM_FLIP) == 0)
  { 
    int val;
    if(IntFromVariant(val,pVar))
    {
      m_bInvert = val;
      return S_OK;
    }
  } 
  return E_INVALIDARG;
}


//----------------- IPersistStream------------------


HRESULT  YUVtoRGBFilter::GetSizeMax(ULARGE_INTEGER *pcbSize)
{
CheckPointer(pcbSize, E_INVALIDARG);
  pcbSize->QuadPart = 1;
  return S_OK;
}


HRESULT  YUVtoRGBFilter::IsDirty(void)
{
  return m_DirtyProps ? S_OK : S_FALSE;
}


HRESULT YUVtoRGBFilter::Load(IStream *pStm)
{
ULONG pcbRead;
char flip;
HRESULT hr;
  CheckPointer(pStm, E_INVALIDARG);

  if(FAILED(hr=pStm->Read(&flip,1,&pcbRead))) return hr;
  if(flip!=m_bInvert)
  {
    m_bInvert = flip;
    m_DirtyProps = true;
  }
return S_OK;
}


HRESULT  YUVtoRGBFilter::Save(IStream *pStm, BOOL fClearDirty)
{
HRESULT hr;
CheckPointer(pStm, E_INVALIDARG);
  if(FAILED(hr=pStm->Write(&m_bInvert, 1, NULL))) return hr;
  if(fClearDirty) m_DirtyProps=false;
return S_OK;
}
