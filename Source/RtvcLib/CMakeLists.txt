# CMakeLists.txt

SET(DS_SRCS 
DirectShow/CSettingsInterface.cpp
DirectShow/CStatusInterface.cpp
DirectShow/CustomBaseFilter.cpp
DirectShow/DirectShowHelper.cpp
DirectShow/MultiIOBaseFilter.cpp
DirectShow/MultiIOInputPin.cpp
DirectShow/MultiIOOutputPin.cpp
DirectShow/stdafx.cpp
DirectShow/VideoMixingBase.cpp
)
SET(GU_SRCS
GeneralUtils/MeasurementTable.cpp
)
SET(IU_SRCS
../libs/VppLib/src/FastFixedPointRGB24toYUV420Converter.cpp
../libs/VppLib/src/FastGpuRGB24toYUV420Converter.cpp
../libs/VppLib/src/FastLookupTableRGB24toYUV420Converter.cpp
../libs/VppLib/src/FastRGB24toYUV420Converter.cpp
../libs/VppLib/src/FastSimdRGB24toYUV420Converter.cpp
../libs/VppLib/src/ImageHandlerV2.cpp
../libs/VppLib/src/MtRGB24toYUV420Converter.cpp
../libs/VppLib/src/OverlayExtMem2Dv2.cpp
../libs/VppLib/src/OverlayMem2Dv2.cpp
../libs/VppLib/src/PicConcatRGB24Impl.cpp
../libs/VppLib/src/PicConcatRGB32Impl.cpp
../libs/VppLib/src/PicCropperRGB24Impl.cpp
../libs/VppLib/src/PicCropperRGB32Impl.cpp
../libs/VppLib/src/PicInPicRGB24Impl.cpp
../libs/VppLib/src/PicInPicRGB32Impl.cpp
../libs/VppLib/src/PicRotateRGB24Impl.cpp
../libs/VppLib/src/PicRotateRGB32Impl.cpp
../libs/VppLib/src/PicRotateRGBBase.cpp
../libs/VppLib/src/PicScalerRGB24Impl.cpp
../libs/VppLib/src/PicScalerRGB32Impl.cpp
../libs/VppLib/src/PicScalerUYVYImpl.cpp
../libs/VppLib/src/PicScalerYUYVImpl.cpp
../libs/VppLib/src/PicScalerYUV420PImpl.cpp
../libs/VppLib/src/RealRGB24toYUV420CCIR601Converter.cpp
../libs/VppLib/src/RealRGB24toYUV420CCIR601ConverterVer16.cpp
../libs/VppLib/src/RealRGB24toYUV420Converter.cpp
../libs/VppLib/src/RealRGB24toYUV420ConverterImpl2.cpp
../libs/VppLib/src/RealRGB24toYUV420ConverterImpl2Ver16.cpp
../libs/VppLib/src/RealRGB32toYUV420Converter.cpp
../libs/VppLib/src/RealYUV420toRGB24CCIR601Converter.cpp
../libs/VppLib/src/RealYUV420toRGB24CCIR601ConverterVer16.cpp
../libs/VppLib/src/RealYUV420toRGB24Converter.cpp
../libs/VppLib/src/RealYUV420toRGB24ConverterImpl2.cpp
../libs/VppLib/src/RealYUV420toRGB24ConverterImpl2Ver16.cpp
../libs/VppLib/src/RealYUV444toRGB24Converter.cpp
)
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
DirectShow/MultiIOBaseFilter.h
DirectShow/MultiIOInputPin.h
DirectShow/MultiIOOutputPin.h
DirectShow/NotifyCodes.h
DirectShow/SettingsInterface.h
DirectShow/StatusInterface.h
DirectShow/stdafx.h
DirectShow/VideoMixingBase.h
)
SET(GU_HEADERS
GeneralUtils/MeasurementTable.h
)
SET(IU_HEADERS
../libs/VppLib/include/FastFixedPointRGB24toYUV420Converter.h
../libs/VppLib/include/FastGpuRGB24toYUV420Converter.h
../libs/VppLib/include/FastLookupTableRGB24toYUV420Converter.h
../libs/VppLib/include/FastRGB24toYUV420Converter.h
../libs/VppLib/include/FastSimdRGB24toYUV420Converter.h
../libs/VppLib/include/ImageHandlerV2.h
../libs/VppLib/include/MtRGB24toYUV420Converter.h
../libs/VppLib/include/OverlayExtMem2Dv2.h
../libs/VppLib/include/OverlayMem2Dv2.h
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
../libs/VppLib/include/RealRGB24toYUV420CCIR601Converter.h
../libs/VppLib/include/RealRGB24toYUV420CCIR601ConverterVer16.h
../libs/VppLib/include/RealRGB24toYUV420Converter.h
../libs/VppLib/include/RealRGB24toYUV420ConverterImpl2.h
../libs/VppLib/include/RealRGB24toYUV420ConverterImpl2Ver16.h
../libs/VppLib/include/RealRGB32toYUV420Converter.h
../libs/VppLib/include/RealYUV420toRGB24CCIR601Converter.h
../libs/VppLib/include/RealYUV420toRGB24CCIR601ConverterVer16.h
../libs/VppLib/include/RealYUV420toRGB24Converter.h
../libs/VppLib/include/RealYUV420toRGB24ConverterImpl2.h
../libs/VppLib/include/RealYUV420toRGB24ConverterImpl2Ver16.h
../libs/VppLib/include/RGBtoRGBConverter.h
../libs/VppLib/include/RGBtoYUV420Converter.h
../libs/VppLib/include/YUV420toRGBConverter.h
../libs/VppLib/include/YUV444toRGBConverter.h
../libs/VppLib/include/RealYUV444toRGB24Converter.h
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
