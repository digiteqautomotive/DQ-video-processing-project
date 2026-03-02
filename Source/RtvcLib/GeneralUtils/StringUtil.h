/**
LICENSE: Software License Agreement(BSD License)

Copyright(c) 2017, Ralf Globisch
Copyright(c) 2025-2026 Digiteq Automotive, programmed by Jaroslav Fojtik		All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met :

* Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and / or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/
#pragma once
#include <string>
#include <cstdlib>


#if _MSC_VER <= 1600
#include "Conversion.h"
 namespace std
 {
   inline string to_string(int value) {return toString((long long)value);}
   inline string to_string(unsigned value) {return toString((unsigned long long)value);}
   inline string to_string(double value) {return toString((double)value);}
 }
#endif


#ifdef ARTIST
namespace artist {
#endif


class StringUtil
{
public:
#ifdef _WIN32

  static std::string StringUtil::wideStringToString(const wchar_t* wszString)
  {
    const unsigned max_len = 255;
    char szBuffer[max_len];
    size_t uiConverted(0);
    errno_t err = wcstombs_s(&uiConverted, szBuffer, max_len, wszString, _TRUNCATE);
    return (err == 0) ? std::string(szBuffer, uiConverted - 1) : ""; // subtract 1 for null terminator
  }

  static std::wstring stringToWideString(const std::string& sString)
  {
    if(sString.empty()) return std::wstring();
    return std::wstring(sString.c_str(), sString.c_str()+sString.length());
  }


#endif

  static bool stringToBool(const std::string& sValue, bool& bValid);

  static std::string boolToString(bool bValue)
  {
    return std::string(bValue ? "1" : "0");
  }

  static std::string doubleToString(double d, int nPrecision = 2);

  static double stringToDouble(std::string sDouble);

#ifdef _WIN32	
  static std::string GetTimeString(std::string sFormat = "");
#endif

  static std::string getZeroPaddedString(const std::string& sString, int iCount, int iTotal);
};


#ifdef ARTIST
} // artist
#endif
