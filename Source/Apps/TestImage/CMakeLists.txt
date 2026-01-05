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
	TestImage.cpp)


TARGET_LINK_LIBRARIES (
TestImage
${vppLibs}
vpp
) 

