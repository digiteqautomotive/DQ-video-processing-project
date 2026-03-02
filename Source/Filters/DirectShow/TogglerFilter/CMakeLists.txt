# CMakeLists.txt for <TogglerFilter> 

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")

INCLUDE_DIRECTORIES(
${vppIncludes}
)

# Lib directories
LINK_DIRECTORIES(
${vppLink}
)

SET(FLT_HDRS
TogglerFilter.h
TogglerProperties.h
resource.h
stdafx.h
)

SET(FLT_SRCS 
DLLSetup.cpp
TogglerFilter.cpp
TogglerFilter.def
TogglerFilter.rc
TogglerProperties.cpp
stdafx.cpp
)

ADD_LIBRARY(
TogglerFilter SHARED ${FLT_SRCS} ${FLT_HDRS})

TARGET_LINK_LIBRARIES (
TogglerFilter
${vppLibs}
vpp
) 

INSTALL(
  TARGETS TogglerFilter
  ARCHIVE DESTINATION bin
  LIBRARY DESTINATION bin
)

IF (REGISTER_DS_FILTERS)
ADD_CUSTOM_COMMAND(
TARGET TogglerFilter 
POST_BUILD COMMAND 
regsvr32 /s \"$(TargetPath)\"
)
ENDIF(REGISTER_DS_FILTERS)
