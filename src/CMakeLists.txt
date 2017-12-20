cmake_minimum_required(VERSION 2.6)

PROJECT( vpp )

SET(EXECUTABLE_OUTPUT_PATH ${vpp_SOURCE_DIR}/../bin)
SET(LIBRARY_OUTPUT_PATH ${vpp_SOURCE_DIR}/../lib)
SET(RUNTIME_OUTPUT_DIRECTORY ${vpp_SOURCE_DIR}/../bin)


IF(WIN32)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${vpp_SOURCE_DIR}/../bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${vpp_SOURCE_DIR}/../bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${vpp_SOURCE_DIR}/../lib)
ELSE(WIN32)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${vpp_SOURCE_DIR}/../bin/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${vpp_SOURCE_DIR}/../bin/build)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${vpp_SOURCE_DIR}/../lib)
ENDIF(WIN32)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

message("ARTIST_CONTRIB_DIR directories:" $ENV{ARTIST_CONTRIB_DIR})
message("BOOST_INCLUDE_DIR:" $ENV{BOOST_INCLUDE_DIR})
message("MS SDK: " $ENV{MSSDK})

IF(WIN32)
IF(MSVC12)
set(ARTIST_CONTRIB_DIR_LIBS ${vpp_SOURCE_DIR}/../contrib/lib/VC12)
ELSEIF(MSVC11)
set(ARTIST_CONTRIB_DIR_LIBS ${vpp_SOURCE_DIR}/../contrib/lib/VC11)
ELSEIF(MSVC10)
set(ARTIST_CONTRIB_DIR_LIBS ${vpp_SOURCE_DIR}/../contrib/lib/VC10)
ELSEIF(MSVC14)
set(ARTIST_CONTRIB_DIR_LIBS ${vpp_SOURCE_DIR}/../contrib/lib/VC14)
ELSE(MSVC12)
set(ARTIST_CONTRIB_DIR_LIBS ${vpp_SOURCE_DIR}/../contrib/lib)
ENDIF(MSVC12)
ENDIF(WIN32)

IF(MSVC)
# set this to avoid VS2013 Win7-SDK compiler warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_USING_V110_SDK71_")
ENDIF(MSVC)

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
# 64bit
  message("Detected 64-bit build - compiling with -fPIC")
#  SET(CMAKE_CXX_FLAGS "-fPIC")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
# 32 bit
message("Detected 32-bit build")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

# variable to differentiate between artist and vpp builds
add_definitions(-DARTIST)

# set this variable to "true" if you need to build the CodecAnalyser project
# CodecAnalyser adds the MFC dependency
SET(BUILD_CODEC_ANALYSER false)

# set this variable to "true" if you need to build the V264 project
SET(BUILD_VCODEC false)

# registering of DS filters
SET(REGISTER_DS_FILTERS false)

SET(BUILD_RTP_PLUS_PLUS_FILTERS false)

SET(ENABLE_X265 false)

# Opus
SET(OPUS_SUPPORT true)

SET(USE_LIVE_MEDIA_EXTENSIONS false)

SET(BUILD_H263 true)

IF(BUILD_CODEC_ANALYSER)
message("Generating solution with CodecAnalyser")
# the codec analyser requires the V264 headers
SET(BUILD_VCODEC false)
ELSE(BUILD_CODEC_ANALYSER)
message("Generating solution without CodecAnalyser")
add_definitions(-DARTIST_CONFIGURATION)
ENDIF(BUILD_CODEC_ANALYSER)

IF(BUILD_VCODEC)
message("Generating solution with V264V2")
ELSE(BUILD_VCODEC)
message("Generating solution without CodecAnalyser")
ENDIF(BUILD_VCODEC)

#define 3rd party libs under windows
# on linux these will have been installed
IF(WIN32)
# Assuming the Foo-root dir is defined in FOO_DIR
# compiling libamr manually now
# add_library( amr UNKNOWN IMPORTED )
#add_library( baseclasses UNKNOWN IMPORTED )
add_library( faac UNKNOWN IMPORTED )
add_library( faad UNKNOWN IMPORTED )
add_library( glog UNKNOWN IMPORTED )
#add_library( zmq UNKNOWN IMPORTED )
add_library( openssllibeay UNKNOWN IMPORTED )
add_library( opensslssleay UNKNOWN IMPORTED )
IF (USE_LIVE555_FROM_CONTRIB)
add_library( liveMedia UNKNOWN IMPORTED )
add_library( BasicUsageEnvironment UNKNOWN IMPORTED )
add_library( groupsock UNKNOWN IMPORTED )
add_library( UsageEnvironment UNKNOWN IMPORTED )
ENDIF (USE_LIVE555_FROM_CONTRIB)
IF(USE_LIVE_MEDIA_EXTENSIONS)
add_library( LiveMediaExt UNKNOWN IMPORTED )
ENDIF(USE_LIVE_MEDIA_EXTENSIONS)
add_library( protobuf UNKNOWN IMPORTED )
add_library( xerces-c UNKNOWN IMPORTED )
add_library( xalan-c UNKNOWN IMPORTED )
add_library( x264 UNKNOWN IMPORTED )
add_library( x265 UNKNOWN IMPORTED )
add_library( zmq UNKNOWN IMPORTED )
IF (BUILD_RTP_PLUS_PLUS_FILTERS)
message("RCLD_ROOT:" $ENV{RCLD_ROOT})
add_library( rtp++ UNKNOWN IMPORTED )
add_library( SipSessionState UNKNOWN IMPORTED )
add_library( scream UNKNOWN IMPORTED )
ENDIF (BUILD_RTP_PLUS_PLUS_FILTERS)

# You can define two import-locations: one for debug and one for release.

# set_target_properties( amr PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libamr_nb_d.lib 
# set_target_properties( amr PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libamr_nb.lib )

#set_target_properties( baseclasses PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/strmbasd.lib )
#set_target_properties( baseclasses PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/strmbase.lib )

set_target_properties( faac PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libfaacD.lib )
set_target_properties( faac PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libfaac.lib )

set_target_properties( faad PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libfaadD.lib )
set_target_properties( faad PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libfaad.lib )

#set_target_properties( glog PROPERTIES IMPORTED_LOCATION_DEBUG $ENV{GLOG_DIR}/lib/Debug/libglog.lib )
#set_target_properties( glog PROPERTIES IMPORTED_LOCATION_RELEASE $ENV{GLOG_DIR}/lib/Release/libglog.lib )

set_target_properties( glog PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libglogD.lib )
set_target_properties( glog PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libglog.lib )


IF (USE_LIVE_MEDIA_EXTENSIONS)
set_target_properties( LiveMediaExt PROPERTIES IMPORTED_LOCATION_DEBUG $ENV{LIVE_MEDIA_EXT_ROOT}/lib/Debug/LiveMediaExt.lib )
set_target_properties( LiveMediaExt PROPERTIES IMPORTED_LOCATION_RELEASE $ENV{LIVE_MEDIA_EXT_ROOT}/lib/Release/LiveMediaExt.lib )
ENDIF(USE_LIVE_MEDIA_EXTENSIONS)

IF (BUILD_RTP_PLUS_PLUS_FILTERS)
message("Building rtp++ filters")
set_target_properties( rtp++ PROPERTIES IMPORTED_LOCATION_DEBUG $ENV{RCLD_ROOT}/rtp++/lib/Debug/Core.lib )
set_target_properties( rtp++ PROPERTIES IMPORTED_LOCATION_RELEASE $ENV{RCLD_ROOT}/rtp++/lib/Release/Core.lib )
set_target_properties( SipSessionState PROPERTIES IMPORTED_LOCATION_DEBUG $ENV{RCLD_ROOT}/rtp++/lib/Debug/SipSessionState.lib )
set_target_properties( SipSessionState PROPERTIES IMPORTED_LOCATION_RELEASE $ENV{RCLD_ROOT}/rtp++/lib/Release/SipSessionState.lib )
set_target_properties( scream PROPERTIES IMPORTED_LOCATION_DEBUG $ENV{RCLD_ROOT}/rtp++/lib/Debug/scream.lib )
set_target_properties( scream PROPERTIES IMPORTED_LOCATION_RELEASE $ENV{RCLD_ROOT}/rtp++/lib/Release/scream.lib )
ENDIF(BUILD_RTP_PLUS_PLUS_FILTERS)

IF (OPUS_SUPPORT)
message("Opus: $ENV{OPUS_ROOT}")
add_library( opus UNKNOWN IMPORTED )
add_library( celt UNKNOWN IMPORTED )
add_library( silk_common UNKNOWN IMPORTED )
add_library( silk_fixed UNKNOWN IMPORTED )
add_library( silk_float UNKNOWN IMPORTED )
set_target_properties( opus PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/opusD.lib )
set_target_properties( opus PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/opus.lib )
set_target_properties( celt PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/celtD.lib )
set_target_properties( celt PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/celt.lib )
set_target_properties( silk_common PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/silk_commonD.lib )
set_target_properties( silk_common PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/silk_common.lib )
set_target_properties( silk_fixed PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/silk_fixedD.lib )
set_target_properties( silk_fixed PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/silk_fixed.lib )
set_target_properties( silk_float PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/silk_floatD.lib )
set_target_properties( silk_float PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/silk_float.lib )
ENDIF(OPUS_SUPPORT)

IF (USE_LIVE555_FROM_CONTRIB)
set_target_properties( liveMedia PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libliveMediaD.lib )
set_target_properties( BasicUsageEnvironment PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libBasicUsageEnvironmentD.lib )
set_target_properties( groupsock PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libgroupsockD.lib )
set_target_properties( UsageEnvironment PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libUsageEnvironmentD.lib )
set_target_properties( liveMedia PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libliveMedia.lib )
set_target_properties( BasicUsageEnvironment PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libBasicUsageEnvironment.lib )
set_target_properties( groupsock PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libgroupsock.lib )
set_target_properties( UsageEnvironment PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libUsageEnvironment.lib )
ENDIF(USE_LIVE555_FROM_CONTRIB)

set_target_properties( protobuf PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libprotobufD.lib )
set_target_properties( protobuf PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libprotobuf.lib )

set_target_properties( xerces-c PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/xerces-c_3D.lib )
set_target_properties( xerces-c PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/xerces-c_3.lib )

set_target_properties( x264 PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libx264d.lib )
set_target_properties( x264 PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libx264.lib )

set_target_properties( x265 PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/x265-staticD.lib )
set_target_properties( x265 PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/x265-static.lib )

set_target_properties( xalan-c PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/Xalan-C_1D.lib )
set_target_properties( xalan-c PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/Xalan-C_1.lib )

set_target_properties( zmq PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libzmqD.lib )
set_target_properties( zmq PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libzmq.lib )

set_target_properties( openssllibeay PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/libeay32D.lib )
set_target_properties( openssllibeay PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/libeay32.lib )

set_target_properties( opensslssleay PROPERTIES IMPORTED_LOCATION_DEBUG ${ARTIST_CONTRIB_DIR_LIBS}/ssleay32D.lib )
set_target_properties( opensslssleay PROPERTIES IMPORTED_LOCATION_RELEASE ${ARTIST_CONTRIB_DIR_LIBS}/ssleay32.lib )
ENDIF(WIN32)

IF(WIN32)
IF (USE_LIVE555_FROM_CONTRIB)
SET(live555Includes
$ENV{ARTIST_CONTRIB_DIR}/live/BasicUsageEnvironment/include
$ENV{ARTIST_CONTRIB_DIR}/live/groupsock/include
$ENV{ARTIST_CONTRIB_DIR}/live/liveMedia/include
$ENV{ARTIST_CONTRIB_DIR}/live/UsageEnvironment/include
)
ELSE(USE_LIVE555_FROM_CONTRIB)
SET(live555Includes
${vpp_SOURCE_DIR}/libs/live/BasicUsageEnvironment/include
${vpp_SOURCE_DIR}/libs/live/groupsock/include
${vpp_SOURCE_DIR}/libs/live/liveMedia/include
${vpp_SOURCE_DIR}/libs/live/UsageEnvironment/include
)
ENDIF (USE_LIVE555_FROM_CONTRIB)
ELSE(WIN32)
SET(live555Includes
${vpp_SOURCE_DIR}/libs/live/BasicUsageEnvironment/include
${vpp_SOURCE_DIR}/libs/live/groupsock/include
${vpp_SOURCE_DIR}/libs/live/liveMedia/include
${vpp_SOURCE_DIR}/libs/live/UsageEnvironment/include
)
ENDIF(WIN32)

IF(WIN32)

SET(artistIncludes
${vpp_SOURCE_DIR}
${vpp_SOURCE_DIR}/lib
${vpp_SOURCE_DIR}/libs
${vpp_SOURCE_DIR}/libs/amapi-proto/include
${vpp_SOURCE_DIR}/../externals
${vpp_SOURCE_DIR}/../externals/protobuf/src
${vpp_SOURCE_DIR}/../externals/glog-0.3.3/src/windows
${vpp_SOURCE_DIR}/../externals/xalan_c-1.11-src/xalan-c-1.11/c/src
${vpp_SOURCE_DIR}/../externals/xerces-c-3.1.1/src
${vpp_SOURCE_DIR}/../externals/zeromq-3.2.3/include
${vpp_SOURCE_DIR}/../externals/openssl-1.0.2/include
${vpp_SOURCE_DIR}/../include
${vpp_SOURCE_DIR}/../include/artist
${vpp_SOURCE_DIR}/../include/artist/LiveMedia
${vpp_SOURCE_DIR}/../include/artist/Media
${vpp_SOURCE_DIR}/../include/artist/Util
${vpp_SOURCE_DIR}/../include/codec_sdk_win32_ia32_release/include
${live555Includes}
$ENV{BOOST_INCLUDE_DIR}
$ENV{DSHOWBASECLASSES}
$ENV{MSSDK}/Include
$ENV{MSSDK}/Include/um
$ENV{MSSDK}/Include/shared
${vpp_SOURCE_DIR}/filters/DirectShow/GMFBridge
)
ELSEIF(UNIX)
#FIND_PATH(LIVEMEDIA_INCLUDE_DIR liveMedia/liveMedia_version.hh
#/usr/local/include
#/usr/include
#)
SET(artistIncludes
${vpp_SOURCE_DIR}/lib
${vpp_SOURCE_DIR}/libs
${vpp_SOURCE_DIR}/libs/amapi-proto/include
${vpp_SOURCE_DIR}/../externals/glog-0.3.3/src
${vpp_SOURCE_DIR}/../externals/protobuf/src
${vpp_SOURCE_DIR}/../externals/xerces-c-3.1.1/src
${vpp_SOURCE_DIR}/../externals/zeromq-3.2.3/include
${vpp_SOURCE_DIR}
${vpp_SOURCE_DIR}/../include
${vpp_SOURCE_DIR}/../include/artist
${vpp_SOURCE_DIR}/../include/artist/LiveMedia
${vpp_SOURCE_DIR}/../include/artist/Media
${vpp_SOURCE_DIR}/../include/artist/Util
${live555Includes}
#${LIVEMEDIA_INCLUDE_DIR}/BasicUsageEnvironment
#${LIVEMEDIA_INCLUDE_DIR}/groupsock
#${LIVEMEDIA_INCLUDE_DIR}/liveMedia
#${LIVEMEDIA_INCLUDE_DIR}/UsageEnvironment
/usr/local/include
/usr/include
)
ENDIF(WIN32)

SET(artistCodecIncludes
${vpp_SOURCE_DIR}/Codecs
${vpp_SOURCE_DIR}/Codecs/H263v2
${vpp_SOURCE_DIR}/Codecs/H263v2Codec
${vpp_SOURCE_DIR}/Codecs/H263v2SwitchCodec
${vpp_SOURCE_DIR}/Codecs/H264v2
${vpp_SOURCE_DIR}/Codecs/H264v2Codec
${vpp_SOURCE_DIR}/Codecs/H264v2SwitchCodec
${vpp_SOURCE_DIR}/Codecs/V264v2
${vpp_SOURCE_DIR}/Codecs/V264v2Codec
)

SET(artistUtilIncludes
${vpp_SOURCE_DIR}/libs/CodecUtils
${vpp_SOURCE_DIR}/libs/GeneralUtils
${vpp_SOURCE_DIR}/libs/ImageUtils
${vpp_SOURCE_DIR}/libs/StreamUtils
)

SET(live555Includes
${vpp_SOURCE_DIR}/libs/LiveMedia/live/BasicUsageEnvironment/include
${vpp_SOURCE_DIR}/libs/LiveMedia/live/groupsock/include
${vpp_SOURCE_DIR}/libs/LiveMedia/live/liveMedia/include
${vpp_SOURCE_DIR}/libs/LiveMedia/live/UsageEnvironment/include
)

# we need ${CMAKE_LIBRARY_ARCHITECTURE} on 64-bit linux systems to locate xerces

# lib directories
IF(WIN32)
SET(artistLink
${vpp_SOURCE_DIR}/../lib
$ENV{BOOST_LIB_DIR}
${ARTIST_CONTRIB_DIR_LIBS}
$ENV{MSSDK}/Lib
)
ELSEIF(UNIX)
SET(artistLink
${vpp_SOURCE_DIR}/../lib
${vpp_SOURCE_DIR}/../externals/glog-0.3.3/.libs
${vpp_SOURCE_DIR}/../externals/protobuf/src/.libs
${vpp_SOURCE_DIR}/../externals/xerces-c-3.1.1/src/.libs
${vpp_SOURCE_DIR}/../externals/zeromq-3.2.3/src/.libs
/usr/local/lib
/usr/lib
/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
)
ENDIF(WIN32)

IF(WIN32)
SET(artistLibs
glog
protobuf
amapi-proto
xalan-c
xerces-c
zmq
ws2_32
psapi
openssllibeay
opensslssleay
)
ELSEIF(UNIX)
SET(artistLibs
boost_date_time boost_filesystem boost_regex boost_signals boost_system boost_thread boost_unit_test_framework boost_program_options
glog
protobuf
amapi-proto
zmq
pthread
rrd
#openssllibeay
#opensslssleay
xerces-c
ssl
crypto
)
ENDIF(WIN32)

SET(LiveMediaLibs
liveMedia
BasicUsageEnvironment
UsageEnvironment
groupsock
)

IF(WIN32)
SET(DsFilterLibs
baseclasses
VppLib
#RtvcDirectShowExt
winmm
comctl32
)
ENDIF(WIN32)

IF(WIN32)
SET(artistLibsDsApp
artist
RtvcDirectShowExt
ClassIDMapper
amapi-proto
baseclasses
${artistLibs}
winmm
)
ELSEIF(UNIX)
SET(artistLibsDsApp
artist
amapi-proto
${artistLibs}
)
ENDIF(WIN32)

# definitions
IF (UNIX)
add_definitions( -std=c++14 )
ENDIF (UNIX)

SET(USE_GLOBAL_VPP true)
SET(GLOBAL_VPP_DIR
${vpp_SOURCE_DIR}/libs/VppLib
)

#IF(BUILD_CODEC_ANALYSER)
#ADD_SUBDIRECTORY( codecs ) 
#ADD_SUBDIRECTORY( libs ) 
#ADD_SUBDIRECTORY( tools ) 
#ELSE(BUILD_CODEC_ANALYSER)
#ADD_SUBDIRECTORY( app ) 
ADD_SUBDIRECTORY( codecs ) 
ADD_SUBDIRECTORY( filters ) 
#ADD_SUBDIRECTORY( lib ) 
ADD_SUBDIRECTORY( libs ) 
#ADD_SUBDIRECTORY( test ) 
#ADD_SUBDIRECTORY( tools ) 
#ENDIF(BUILD_CODEC_ANALYSER)
