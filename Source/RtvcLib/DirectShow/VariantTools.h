#ifndef __VARIANT_TOOLS__
#define __VARIANT_TOOLS__

#define olestr(s) sstr(s)
#if defined(_WIN32) && !defined(OLE2ANSI)
 #define olecmp(__str1,__str2) wcscmp(__str1,__str2)
 #define oleicmp(__str1,__str2) wcsicmp(__str1,__str2)
 #define olelen(__str) wcslen(__str)
 #define sstr(s) L##s
#else
 #define olecmp(__str1,__str2) strcmp(__str1,__str2)
 #define oleicmp(__str1,__str2) stricmp(__str1,__str2)
 #define olelen(__str) strlen(__str)
 #define sstr(s) #s
#endif


bool getFromVariant(bool & val, const VARIANT *pVar);

bool getFromVariant(int & val, const VARIANT *pVar);

bool getFromVariant(unsigned int & val, const VARIANT *pVar);

bool getFromVariant(LONGLONG & val, const VARIANT *pVar);

bool getFromVariant(double & val, const VARIANT *pVar);

#endif