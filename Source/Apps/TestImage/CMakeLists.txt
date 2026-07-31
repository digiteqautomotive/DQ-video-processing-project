cmake_minimum_required(VERSION 2.6)


INCLUDE_DIRECTORIES(
${vppIncludes}
)

# Lib directories
LINK_DIRECTORIES(
${vppLink}
)


SET(TEST_SRCS
	TestImage.cpp stdafx.cpp
        reference/ConvRGB24toYUV420_fixpLUT.cpp
        reference/ConvRGB24toYUV420_LUT.cpp
        reference/ConvRGB24toYUV420_Mt.cpp
        reference/ConvRGB24toYUV420_Real.cpp
        reference/ConvRGB24toYUV420_SIMD.cpp
        reference/ConvRGB32toYUV420_Real.cpp
        reference/RealRGB24toYUV420ConverterImpl2.cpp
	reference/RealYUV420toRGB24CCIR601Converter.cpp
	reference/RealYUV420toRGB24CCIR601ConverterVer16.cpp
	reference/RealYUV420toRGB24ConverterImpl2Ver16.cpp
)

SET(TEST_HEADERS 
        ConvertTesters.cpp ConvertTesters.h 
	RotateTesters.cpp RotateTesters.h 
	ScaleTesters.cpp ScaleTesters.h
	reference/ConvRGB24toYUV420RealImpl2.h
	reference/ConvYUV420toRGB24CCIR601Real.h
	reference/ConvYUV420toRGB24CCIR601RealVer16.h
	reference/ConvYUV420toRGB24RealImpl2Ver16.h
)


# Specify the source code files
add_executable(TestImage ${TEST_HEADERS} ${TEST_SRCS})


TARGET_LINK_LIBRARIES (
TestImage
${vppLibs}
vpp
) 

