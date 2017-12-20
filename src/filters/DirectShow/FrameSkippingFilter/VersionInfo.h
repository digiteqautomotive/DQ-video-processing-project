#pragma once

#include <sstream>
#include <string>

const unsigned MAJOR_VERSION = 1;
const unsigned MINOR_VERSION = 0;
const unsigned BUILD_VERSION = 1;

/// 0.0.0: - Initial release of filter with version control
/// 0.1.0: - Updating frame duration when skipping frames
/// 1.0.0: - Added source and target frame rate concept rather than skip x out of y frames
/// 1.0.1: - Bugfix in average duration per frame calculations
struct VersionInfo
{
  static std::string toString()
  {
    std::ostringstream ostr;
    ostr << MAJOR_VERSION << "." << MINOR_VERSION << "." << BUILD_VERSION;
    return ostr.str();
  }

};
