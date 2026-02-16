cmake_minimum_required(VERSION 2.6)


INCLUDE_DIRECTORIES(
${vppIncludes}
)

# Lib directories
LINK_DIRECTORIES(
${vppLink}
)


# Specify the source code files
add_executable(TestImage
        ConvertTesters.cpp ConvertTesters.h 
	RotateTesters.cpp RotateTesters.h 
	ScaleTesters.cpp ScaleTesters.h
	TestImage.cpp stdafx.cpp
        reference/ConvRGB24toYUV420_fixpLUT.cpp
        reference/ConvRGB24toYUV420_LUT.cpp
        reference/ConvRGB24toYUV420_Mt.cpp
        reference/ConvRGB24toYUV420_Real.cpp
        reference/ConvRGB24toYUV420_SIMD.cpp
        reference/ConvRGB32toYUV420_Real.cpp)


TARGET_LINK_LIBRARIES (
TestImage
${vppLibs}
vpp
) 

