# CMakeLists.txt

SET(DS_SRCS 
DirectShow/CSettingsInterface.cpp
DirectShow/CStatusInterface.cpp
DirectShow/CustomBaseFilter.cpp
DirectShow/DirectShowHelper.cpp
DirectShow/FilterPropertiesBase.cpp
DirectShow/FrameQueue.cpp
DirectShow/MultiIOBaseFilter.cpp
DirectShow/MultiIOInputPin.cpp
DirectShow/MultiIOOutputPin.cpp
DirectShow/stdafx.cpp
DirectShow/VideoMixingBase.cpp
DirectShow/VariantTools.cpp
DirectShow/VersionInfo.cpp
)

SET(GU_SRCS
GeneralUtils/MeasurementTable.cpp
GeneralUtils/StringUtil.cpp 
)

SET(IU_SRCS
../libs/VppLib/src/ConvUYVYtoRGB.cpp
../libs/VppLib/src/ConvNV12toRGB.cpp 
../libs/VppLib/src/ConvRGBtoYUYV_FixP.cpp
../libs/VppLib/src/ConvRGB24toYUV420_FixP.cpp
../libs/VppLib/src/ConvRGB32toYUV420_FixP.cpp
../libs/VppLib/src/ConvYUVtoRGB.cpp
../libs/VppLib/src/ConvYUYVtoRGB.cpp
../libs/VppLib/src/ConvYV24toRGB.cpp
../libs/VppLib/src/ConvYUV420toRGB.cpp
../libs/VppLib/src/ConvYVYUtoRGB.cpp
../libs/VppLib/src/ImageHandlerV2.cpp
../libs/VppLib/src/OverlayExtMem2Dv2.cpp
../libs/VppLib/src/OverlayMem2Dv2.cpp
../libs/VppLib/src/PicCompareBase.cpp
../libs/VppLib/src/PicConcatRGB24Impl.cpp
../libs/VppLib/src/PicConcatRGB32Impl.cpp
../libs/VppLib/src/PicCropperRGB24Impl.cpp
../libs/VppLib/src/PicCropperRGB32Impl.cpp
../libs/VppLib/src/PicCropperRGB565Impl.cpp
../libs/VppLib/src/PicInPicRGB24Impl.cpp
../libs/VppLib/src/PicInPicRGB32Impl.cpp
../libs/VppLib/src/PicInPicRGB565Impl.cpp
../libs/VppLib/src/PicInPicYV24Impl.cpp
../libs/VppLib/src/PicRotateBPP16Impl.cpp
../libs/VppLib/src/PicRotateRGB24Impl.cpp
../libs/VppLib/src/PicRotateRGB32Impl.cpp
../libs/VppLib/src/PicRotateRGBBase.cpp
../libs/VppLib/src/PicScalerARGB32Impl.cpp
../libs/VppLib/src/PicScalerRGB565Impl.cpp
../libs/VppLib/src/PicScalerUYVYImpl.cpp
../libs/VppLib/src/PicScalerYUYVImpl.cpp
../libs/VppLib/src/PicScalerYUV420PImpl.cpp
../libs/VppLib/src/PicScalerYV24Impl.cpp
../libs/VppLib/src/RealRGB24toYUV420CCIR601Converter.cpp
../libs/VppLib/src/RealRGB24toYUV420CCIR601ConverterVer16.cpp
../libs/VppLib/src/RealRGB24toYUV420ConverterImpl2.cpp
../libs/VppLib/src/RealRGB24toYUV420ConverterImpl2Ver16.cpp
../libs/VppLib/src/RealYUV420toRGB24CCIR601Converter.cpp
../libs/VppLib/src/RealYUV420toRGB24CCIR601ConverterVer16.cpp
../libs/VppLib/src/RealYUV420toRGB24ConverterImpl2Ver16.cpp
)
IF (MMX_SSE_ACCELERATORS)
  SET(IU_SRCS ${IU_SRCS}
      ../libs/VppLib/src/PicScalerRGB24ImplA.cpp
      ../libs/VppLib/src/PicScalerRGB32ImplA.cpp
      ../libs/VppLib/src/PicCompRGB24ImplA.cpp
      ../libs/VppLib/src/PicCompRGB32ImplA.cpp
      ../libs/VppLib/src/PicCompYUYV_ImplA.cpp
      ../libs/VppLib/src/PicScalerARGB32MMX.cpp
      ../libs/VppLib/src/PicScalerARGB32SSE.cpp
      ../libs/VppLib/src/ConvAYUV444toRGB24_ASM.cpp
      ../libs/VppLib/src/ConvYUV420toRGB24_ASM.cpp
  )
  if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
    SET(IU_SRCS ${IU_SRCS}
      ../libs/VppLib/src/raster_MSVC.asm
      ../libs/VppLib/src/RowConvert_Win32.asm
      ../libs/VppLib/src/RowScallerARGB32_Win32.asm
      ../libs/VppLib/src/RowScallerMMX_Win32.asm
      ../libs/VppLib/src/RowScallerSSE_Win32.asm
      ../libs/VppLib/src/ToolsCPU_Win32.asm
    )
  elseif ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "x64")
    SET(IU_SRCS ${IU_SRCS}
      ../libs/VppLib/src/raster_MSVC64.asm
      ../libs/VppLib/src/RowConvert_Win64.asm
      ../libs/VppLib/src/RowScallerARGB32_x64.asm
      ../libs/VppLib/src/RowScallerMMX_x64.asm
      ../libs/VppLib/src/RowScallerSSE_x64.asm
      ../libs/VppLib/src/ToolsCPU_x64.asm
    )
  else ()
    message(SEND_ERROR "No assembly accelerators for this platform!")
  endif ()
ELSE (MMX_SSE_ACCELERATORS)
  SET(IU_SRCS ${IU_SRCS}
      ../libs/VppLib/src/PicScalerRGB24Impl.cpp
      ../libs/VppLib/src/PicScalerRGB32Impl.cpp
      ../libs/VppLib/src/ConvAYUV444toRGB24.cpp
      ../libs/VppLib/src/PicCompRGB24Impl.cpp
      ../libs/VppLib/src/PicCompRGB32Impl.cpp
      ../libs/VppLib/src/PicCompYUYV_Impl.cpp
  )
ENDIF(MMX_SSE_ACCELERATORS)

SET(S_SRCS
Shared/MediaSample.cpp
)

SET(DS_HEADERS
DirectShow/CommonDefs.h
DirectShow/CSettingsInterface.h
DirectShow/CStatusInterface.h
DirectShow/CustomBaseFilter.h
DirectShow/CustomMediaTypes.h
DirectShow/DirectShowHelper.h
DirectShow/FilterPropertiesBase.h
DirectShow/FrameQueue.h
DirectShow/MultiIOBaseFilter.h
DirectShow/MultiIOInputPin.h
DirectShow/MultiIOOutputPin.h
DirectShow/NotifyCodes.h
DirectShow/SettingsInterface.h
DirectShow/StatusInterface.h
DirectShow/stdafx.h
DirectShow/VariantTools.h
DirectShow/VideoMixingBase.h
)
SET(GU_HEADERS
GeneralUtils/MeasurementTable.h
GeneralUtils/StringUtil.h
)
SET(IU_HEADERS
../libs/VppLib/include/ConvUYVYtoRGB.h
../libs/VppLib/include/ConvNV12toRGB.h
../libs/VppLib/include/ConvAYUV444toRGB.h
../libs/VppLib/include/ConvRGB24toYUV420.h
../libs/VppLib/include/ConvRGB24toYUV420CCIR601Real.h
../libs/VppLib/include/ConvRGB24toYUV420CCIR601Ver16.h
../libs/VppLib/include/ConvRGB24toYUV420RealImpl2.h
../libs/VppLib/include/ConvRGB24toYUV420RealImpl2Ver16.h
../libs/VppLib/include/ConvRGB32toYUV420.h
../libs/VppLib/include/ConvRGBtoYUV420.h
../libs/VppLib/include/ConvRGBtoYUYV.h
../libs/VppLib/include/ConvYUV420toRGB.h
../libs/VppLib/include/ConvYUV420toRGB24CCIR601Real.h
../libs/VppLib/include/ConvYUV420toRGB24CCIR601RealVer16.h
../libs/VppLib/include/ConvYUV420toRGB24RealImpl2.h
../libs/VppLib/include/ConvYUV420toRGB24RealImpl2Ver16.h
../libs/VppLib/include/ConvYUVtoRGB.h
../libs/VppLib/include/ConvYUYVtoRGB.h
../libs/VppLib/include/ConvYVYUtoRGB.h
../libs/VppLib/include/ImageHandlerV2.h
../libs/VppLib/include/OverlayExtMem2Dv2.h
../libs/VppLib/include/OverlayMem2Dv2.h
../libs/VppLib/include/PicCompareBase.h
../libs/VppLib/include/PicCompRGB24Impl.h
../libs/VppLib/include/PicCompRGB32Impl.h
../libs/VppLib/include/PicCompYUYV_Impl.h
../libs/VppLib/include/PicConcatBase.h
../libs/VppLib/include/PicConcatRGB24Impl.h
../libs/VppLib/include/PicConcatRGB32Impl.h
../libs/VppLib/include/PicCropperBase.h
../libs/VppLib/include/PicCropperRGB24Impl.h
../libs/VppLib/include/PicCropperRGB32Impl.h
../libs/VppLib/include/PicInPicBase.h
../libs/VppLib/include/PicInPicRGB24Impl.h
../libs/VppLib/include/PicInPicRGB32Impl.h
../libs/VppLib/include/PicRotateBase.h
../libs/VppLib/include/PicRotateRGB24Impl.h
../libs/VppLib/include/PicRotateRGB32Impl.h
../libs/VppLib/include/PicRotateRGBBase.h
../libs/VppLib/include/PicScalerBase.h
../libs/VppLib/include/PicScalerRGB24Impl.h
../libs/VppLib/include/PicScalerYUV420PImpl.h
)
SET(S_HEADERS
Shared/CommonDefs.h
GeneralUtils/Conversion.h
Shared/MediaSample.h
GeneralUtils/StringUtil.h
Shared/TimerUtil.h
)

SET(VPP_SRCS
${DS_SRCS} 
${GU_SRCS}
${IU_SRCS}
${S_SRCS}
)
SET(VPP_HEADERS 
${DS_HEADERS}
${GU_HEADERS}
${IU_HEADERS}
${S_HEADERS}
)

INCLUDE_DIRECTORIES(
${vppIncludes}
)

SOURCE_GROUP("Source Files\\DirectShow" FILES ${DS_SRCS})
SOURCE_GROUP("Source Files\\GeneralUtils" FILES ${GU_SRCS})
SOURCE_GROUP("Source Files\\Image" FILES ${IU_SRCS})
SOURCE_GROUP("Source Files\\Shared" FILES ${S_SRCS})

SOURCE_GROUP("Header Files\\DirectShow" FILES ${DS_HEADERS})
SOURCE_GROUP("Header Files\\GeneralUtils" FILES ${GU_HEADERS})
SOURCE_GROUP("Header Files\\Image" FILES ${IU_HEADERS})
SOURCE_GROUP("Header Files\\Shared" FILES ${S_HEADERS})

ADD_LIBRARY( vpp STATIC ${VPP_SRCS} ${VPP_HEADERS})

# adding precompiled header support on windows
#if (MSVC)
#   set_target_properties(Core PROPERTIES COMPILE_FLAGS "/YuCorePch.h")
#   set_source_files_properties(CorePch.cpp PROPERTIES COMPILE_FLAGS "/YcCorePch.h")
#endif(MSVC)


#ADD_SUBDIRECTORY( TestImage )