#include "VersionInfo.h"
#include <sstream>


void VersionInfo::doGetVersion(std::string& sVersion) const
{
  sVersion = VersionInfo::toString();
}


std::string VersionInfo::toString(void)
{
  std::ostringstream ostr;
  ostr << MAJOR_VERSION << "." << MINOR_VERSION << "." << BUILD_VERSION;
  return ostr.str();
}
