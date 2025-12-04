#include "../../RtvcLib/DirectShow/VersionInfo.h"


/////////////////////////////////////////////////////////////////////////////
//
// Version
//
#define APP_COMPANY_NAME_STRING "Digiteq Automotive s.r.o"
#define PRODUCT_INFO_STRING "Direct show filter for video scalling"

#ifndef STR_HELPER
  #define STR_HELPER(__x) #__x
  #define STR_(__x) STR_HELPER(__x)
 #endif
#define PRODUCT_VERSION STR_(MAJOR_VERSION) "." STR_(MINOR_VERSION) "." STR_(BUILD_VERSION)


VS_VERSION_INFO VERSIONINFO
 FILEVERSION MAJOR_VERSION,MINOR_VERSION,BUILD_VERSION,0
 //PRODUCTVERSION 0,0,0,0
 FILEFLAGSMASK 0x3fL
#ifdef _DEBUG
 FILEFLAGS 0x1L
#else
 FILEFLAGS 0x0L
#endif
 FILEOS 0x40004L
 FILETYPE 0x1L
 FILESUBTYPE 0x0L
BEGIN
    BLOCK "StringFileInfo"
    BEGIN
        BLOCK "040904b0"
        BEGIN
            VALUE "CompanyName", APP_COMPANY_NAME_STRING
            VALUE "FileDescription", PRODUCT_INFO_STRING
            //VALUE "InternalName", "ScaleFilter.dll"
            //VALUE "LegalCopyright", COPYRIGHT_INTRO_STRING " " APP_COMPANY_NAME_STRING " " COPYRIGHT_DISCLAIMER_STRING
            VALUE "OriginalFilename", ORIGINAL_FILENAME
            //VALUE "ProductName", PRODUCT_NAME_STRING
            VALUE "ProductVersion", PRODUCT_VERSION
        END
    END
    BLOCK "VarFileInfo"
    BEGIN
        VALUE "Translation", 0x409, 1200
    END
END
