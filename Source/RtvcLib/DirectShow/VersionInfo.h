#pragma once

#include <string>

const unsigned MAJOR_VERSION = 1;
const unsigned MINOR_VERSION = 6;
const unsigned BUILD_VERSION = 2;

/// 0.0.0: - Initial release of filter with version control
/// 1.6.0: - Refactored CSIR_VPP to DQ_VPP


class VersionInfo
{
public:
  void doGetVersion(std::string& sVersion) const;

  static std::string toString(void);
};
