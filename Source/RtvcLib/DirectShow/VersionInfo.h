#ifndef __VersionInfo_h__
#define __VersionInfo_h__

#define MAJOR_VERSION	1
#define MINOR_VERSION	7
#define BUILD_VERSION	35

/// 0.0.0: - Initial release of filter with version control
/// 1.6.0: - Refactored CSIR_VPP to DQ_VPP


#ifdef __cplusplus
#include <string>

class VersionInfo
{
public:
  void doGetVersion(std::string& sVersion) const;

  static std::string toString(void);
};

#endif

#endif
