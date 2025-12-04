
#define olestr(s) sstr(s)
#if defined(_WIN32) && !defined(OLE2ANSI)
 #define olecmp(__str1,__str2) wcscmp(__str1,__str2)
 #define oleicmp(__str1,__str2) wcsicmp(__str1,__str2)
 #define sstr(s) L###s
#else
 #define olecmp(__str1,__str2) strcmp(__str1,__str2)
 #define oleicmp(__str1,__str2) stricmp(__str1,__str2)
 #define sstr(s) #s
#endif


bool IntFromVariant(int & val, const VARIANT *pVar)
{
  switch(pVar->vt)
  {
    case VT_I1:  val=pVar->cVal; return true;
    case VT_I2:  val=pVar->iVal; return true;
    case VT_I4:  val=pVar->intVal; return true;
    case VT_I8:  val=pVar->llVal; return true;
    case VT_UI1: val=pVar->bVal; return true;
    case VT_UI2: val=pVar->uiVal; return true;
    case VT_UI4: val=pVar->uintVal; return true;
    case VT_UI8: val=pVar->ullVal; return true;
    case VT_R4:	 val=pVar->fltVal; return true;
    case VT_R8:	 val=pVar->dblVal; return true;
    case VT_BOOL:val=pVar->boolVal; return true;
    case VT_INT: val=pVar->intVal; return true;
    case VT_UINT:val=pVar->uintVal; return true;
    case VT_UINT_PTR:if(pVar->puintVal==NULL) return false;
                 val=*pVar->puintVal; return true;
    case VT_INT_PTR:if(pVar->pintVal==NULL) return false;
                 val=*pVar->pintVal; return true;
    case VT_LPSTR:if(pVar->pcVal==NULL) return false;
		  val=atoi(pVar->pcVal); return true;
    case VT_LPWSTR:if(pVar->puiVal==NULL) return false;
		  val=_wtoi((const wchar_t*)pVar->pcVal); return true;
    case VT_BSTR:if(pVar->bstrVal==NULL) return false;
		  val=_wtoi((const wchar_t*)pVar->bstrVal); return true;
    //	VT_VARIANT	= 12,
    //	VT_UNKNOWN	= 13,
    //	VT_DECIMAL	= 14,
    //	VT_CY	= 6,
    //	VT_DATE	= 7,
    //	VT_DISPATCH	= 9,
    //	VT_ERROR	= 10,
    //VT_EMPTY	= 0,
    //VT_NULL	= 1,
    //default: return E_INVALIDARG;	no need to handle here.
  }
  return false;
}


bool UnsignedFromVariant(unsigned int & val, const VARIANT *pVar)
{
  switch(pVar->vt)
  {
    case VT_I1:  val=pVar->cVal; return true;
    case VT_I2:  if(pVar->iVal<0) return false;
                 val=pVar->iVal; return true;
    case VT_I4:  if(pVar->intVal<0) return false;
                 val=pVar->intVal; return true;
    case VT_I8:  if(pVar->llVal<0) return false;
                 val=pVar->llVal; return true;
    case VT_UI1: val=pVar->bVal; return true;
    case VT_UI2: val=pVar->uiVal; return true;
    case VT_UI4: val=pVar->uintVal; return true;
    case VT_UI8: val=pVar->ullVal; return true;
    case VT_R4:	 if(pVar->fltVal<0) return false;
                 val=pVar->fltVal; return true;
    case VT_R8:	 if(pVar->dblVal<0) return false;
                 val=pVar->dblVal; return true;
    case VT_BOOL:val=pVar->boolVal; return true;
    case VT_INT: if(pVar->intVal<0) return false;
                 val=pVar->intVal; return true;
    case VT_UINT:val=pVar->uintVal; return true;
    case VT_UINT_PTR:if(pVar->puintVal==NULL) return false;
                 val=*pVar->puintVal; return true;
    case VT_INT_PTR:if(pVar->pintVal==NULL) return false;
                 if(*pVar->pintVal<0) return false;
                 val=*pVar->pintVal; return true;
    case VT_LPSTR:if(pVar->pcVal==NULL) return false;
		  val=atoi(pVar->pcVal); return true;
    case VT_LPWSTR:if(pVar->puiVal==NULL) return false;
		  val=_wtoi((const wchar_t*)pVar->pcVal); return true;
    case VT_BSTR:if(pVar->bstrVal==NULL) return false;
		  val=_wtoi((const wchar_t*)pVar->bstrVal); return true;
    //	VT_VARIANT	= 12,
    //	VT_UNKNOWN	= 13,
    //	VT_DECIMAL	= 14,
    //	VT_CY	= 6,
    //	VT_DATE	= 7,
    //	VT_DISPATCH	= 9,
    //	VT_ERROR	= 10,
    //VT_EMPTY	= 0,
    //VT_NULL	= 1,
    //default: return E_INVALIDARG;	no need to handle here.
  }
  return false;
}
