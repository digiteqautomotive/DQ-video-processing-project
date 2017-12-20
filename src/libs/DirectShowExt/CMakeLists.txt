# CMakeLists.txt in DirectShowExt dir
# Adds a library called DirectShowExt
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")


INCLUDE_DIRECTORIES(
$ENV{DSHOWBASECLASSES}
$ENV{MSSDK}/Include
$ENV{MSSDK}/Include/um
$ENV{MSSDK}/Include/shared
${artist_SOURCE_DIR}/../include
${artist_SOURCE_DIR}/../include/artist/Util
${GLOBAL_VPP_DIR}/include
${GLOBAL_VPP_DIR}/include/CodecUtils
${GLOBAL_VPP_DIR}/include/GeneralUtils
${GLOBAL_VPP_DIR}/include/ImageUtils
${GLOBAL_VPP_DIR}/include/StreamUtils
)

# Lib directories
LINK_DIRECTORIES(
$ENV{MSSDK}/Lib
)

SET(PROJ_HDRS
CodecControlInterface.h
ComTypeDefs.h
Crossbar.h
CSettingsInterface.h
CStatusInterface.h
CustomBaseFilter.h
CustomMediaTypes.h
DirectShowHelper.h
DirectShowHelper.h
DirectShowMediaFormats.h
FilterParameterStringConstants.h
FilterPropertiesBase.h
IFileInfoSourceInterface.h
IMediaBufferDuration.h
MultiIOBaseFilter.h
MultiIOInputPin.h
MultiIOOutputPin.h
NetworkReferenceClock.h
NotifyCodes.h
ParameterConstants.h
SettingsInterface.h
StatusInterface.h
stdafx.h
VideoMixingBase.h
)

SET(PROJ_SRCS
Crossbar.cpp
CSettingsInterface.cpp
CStatusInterface.cpp
CustomBaseFilter.cpp
DirectShowHelper.cpp
DirectShowHelper.cpp
MultiIOBaseFilter.cpp
MultiIOInputPin.cpp
MultiIOOutputPin.cpp
NetworkReferenceClock.cpp
stdafx.cpp
VideoMixingBase.cpp
)

ADD_LIBRARY( RtvcDirectShowExt STATIC ${PROJ_SRCS} ${PROJ_HDRS})


# adding precompiled header support on windows
if (MSVC)
   set_target_properties(RtvcDirectShowExt PROPERTIES COMPILE_FLAGS "/Yustdafx.h")
   set_source_files_properties(stdafx.cpp PROPERTIES COMPILE_FLAGS "/Ycstdafx.h")
endif(MSVC)
#if (MSVC)
#   set_target_properties(RtvcDirectShowExt PROPERTIES COMPILE_FLAGS "/YuDirectShowExtPch.h")
#   set_source_files_properties(DirectShowExtPch.cpp PROPERTIES COMPILE_FLAGS "/YcDirectShowExtPch.h")
#endif(MSVC)

if (CMAKE_BUILD_TYPE STREQUAL Debug)
SET_TARGET_PROPERTIES(RtvcDirectShowExt PROPERTIES DEBUG_POSTFIX "D" )
message("DirectShowExt Debug Build")
else (CMAKE_BUILD_TYPE STREQUAL Debug)
SET_TARGET_PROPERTIES(RtvcDirectShowExt PROPERTIES DEBUG_POSTFIX "")
message("DirectShowExt Release Build")
endif (CMAKE_BUILD_TYPE STREQUAL Debug)

