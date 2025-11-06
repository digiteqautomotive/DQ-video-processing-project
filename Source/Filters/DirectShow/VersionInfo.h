#pragma once

#include <sstream>
#include <string>

const unsigned MAJOR_VERSION = 1;
const unsigned MINOR_VERSION = 5;
const unsigned BUILD_VERSION = 11;

/// 0.0.0: - Initial release of filter with version control

class VersionInfo
{
public:
  static std::string toString()
  {
    std::ostringstream ostr;
    ostr << MAJOR_VERSION << "." << MINOR_VERSION << "." << BUILD_VERSION;
    return ostr.str();
  }

};
