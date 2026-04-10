# CMakeLists.txt for <VideoCompareFilter> 

INCLUDE_DIRECTORIES(
${vppIncludes}
)

# Lib directories
LINK_DIRECTORIES(
${vppLink}
)

SET(FLT_HDRS
resource.h
stdafx.h
VideoCompareFilter.h
VideoCompareProperties.h
)

SET(FLT_SRCS 
DLLSetup.cpp
stdafx.cpp
VideoCompareFilter.cpp
VideoCompareFilter.def
VideoCompareFilter.rc
)

ADD_LIBRARY(
VideoCompareFilter SHARED ${FLT_SRCS} ${FLT_HDRS})

TARGET_LINK_LIBRARIES (
VideoCompareFilter
${vppLibs}
vpp
) 

INSTALL(
  TARGETS VideoCompareFilter
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION bin
)

IF (REGISTER_DS_FILTERS)
ADD_CUSTOM_COMMAND(
TARGET VideoCompareFilter 
POST_BUILD COMMAND 
regsvr32 /s \"$(TargetPath)\"
)
ENDIF(REGISTER_DS_FILTERS)
