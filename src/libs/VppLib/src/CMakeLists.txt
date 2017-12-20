cmake_minimum_required(VERSION 2.6)

PROJECT( VppLib )

message("Source Dir:" ${VppLib_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${VppLib_SOURCE_DIR}/../bin)
SET(LIBRARY_OUTPUT_PATH ${VppLib_SOURCE_DIR}/../lib)
SET(RUNTIME_OUTPUT_DIRECTORY ${VppLib_SOURCE_DIR}/../bin)

SET(VppLib_BIN ${VppLib_SOURCE_DIR}/../bin)

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
# 64bit
  message("Detected 64-bit build - compiling with -fPIC")
  SET(CMAKE_CXX_FLAGS "-fPIC")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
# 32 bit
message("Detected 32-bit build")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )


# CMakeLists.txt in Codec Utils dir

INCLUDE_DIRECTORIES(
${VppLib_SOURCE_DIR}/../include
${VppLib_SOURCE_DIR}/../include/CodecUtils
${VppLib_SOURCE_DIR}/../include/GeneralUtils
${VppLib_SOURCE_DIR}/../include/ImageUtils
${VppLib_SOURCE_DIR}/../include/StreamUtils
)

# Adds a library called Shared (libRtvcCodecUtils.a under Linux)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")

IF(BUILD_VCODEC)
SET(VCODEC_SRCS
CodecUtils/VectorQuantiserBaseImpl1.cpp
CodecUtils/VectorQuantiserCbImpl1.cpp
CodecUtils/VectorQuantiserCrImpl1.cpp
CodecUtils/VectorQuantiserLumImpl1.cpp
)
ELSE(BUILD_VCODEC)
SET(VCODEC_SRCS
)
ENDIF(BUILD_VCODEC)

SET(CODEC_UTIL_HDRS
../include/CodecUtils/BitStreamBase.h
../include/CodecUtils/BitStreamBaseMSB.h
../include/CodecUtils/BitStreamReaderMSB.h
../include/CodecUtils/BitStreamWriter.h
../include/CodecUtils/BitStreamWriterMSB.h
../include/CodecUtils/BlockH264.h
../include/CodecUtils/CAVLCH264Impl2.h
../include/CodecUtils/CAVLCH264Impl.h
../include/CodecUtils/CodecDistortionDef.h
../include/CodecUtils/CodedBlkPatternH264VlcDecoder.h
../include/CodecUtils/CodedBlkPatternH264VlcEncoder.h
../include/CodecUtils/CoeffTokenH264VlcDecoder.h
../include/CodecUtils/CoeffTokenH264VlcEncoder.h
../include/CodecUtils/ExpGolombSignedVlcDecoder.h
../include/CodecUtils/ExpGolombSignedVlcEncoder.h
../include/CodecUtils/ExpGolombTruncVlcDecoder.h
../include/CodecUtils/ExpGolombTruncVlcEncoder.h
../include/CodecUtils/ExpGolombUnsignedVlcDecoder.h
../include/CodecUtils/ExpGolombUnsignedVlcEncoder.h
../include/CodecUtils/FastForward4x4ITImpl1.h
../include/CodecUtils/FastForward4x4ITImpl2.h
../include/CodecUtils/FastForward4x4On16x16ITImpl1.h
../include/CodecUtils/FastForwardDC2x2ITImpl1.h
../include/CodecUtils/FastForwardDC4x4ITImpl1.h
../include/CodecUtils/FastInverse4x4ITImpl1.h
../include/CodecUtils/FastInverse4x4On16x16ITImpl1.h
../include/CodecUtils/FastInverseDC2x2ITImpl1.h
../include/CodecUtils/FastInverseDC4x4ITImpl1.h
../include/CodecUtils/H263MotionVectorPredictorImpl1.h
../include/CodecUtils/H264MbImgCache.h
../include/CodecUtils/H264MotionVectorPredictorImpl1.h
../include/CodecUtils/H264RawFileHandler.h
../include/CodecUtils/IBitStreamReader.h
../include/CodecUtils/IBitStreamWriter.h
../include/CodecUtils/ICodecv2.h
../include/CodecUtils/IContextAwareRunLevelCodec.h
../include/CodecUtils/IForwardDct.h
../include/CodecUtils/IForwardTransform.h
../include/CodecUtils/IInverseTransform.h
../include/CodecUtils/IMotionEstimator.h
../include/CodecUtils/IMotionVectorPredictor.h
../include/CodecUtils/IRateControl.h
../include/CodecUtils/IRunLengthCodec.h
../include/CodecUtils/IVlcDecoder.h
../include/CodecUtils/IVlcEncoder.h
../include/CodecUtils/MacroBlockH264.h
../include/CodecUtils/MotionCompensatorH264ImplMultires.h
../include/CodecUtils/MotionCompensatorH264ImplStd.h
../include/CodecUtils/MotionEstimatorH263ImplMultiresCross.h
../include/CodecUtils/MotionEstimatorH263ImplMultiresCrossVer2.h
../include/CodecUtils/MotionEstimatorH264ImplCross.h
../include/CodecUtils/MotionEstimatorH264ImplFull.h
../include/CodecUtils/MotionEstimatorH264ImplMultiresCross.h
../include/CodecUtils/MotionEstimatorH264ImplMultiresCrossVer2.h
../include/CodecUtils/MotionEstimatorH264ImplMultires.h
../include/CodecUtils/MotionEstimatorH264ImplTest.h
../include/CodecUtils/NalHeaderH264.h
../include/CodecUtils/PicParamSetH264.h
../include/CodecUtils/PrefixH264VlcDecoderImpl1.h
../include/CodecUtils/PrefixH264VlcEncoderImpl1.h
../include/CodecUtils/RateControlImpl1.h
../include/CodecUtils/RateControlImplLog.h
../include/CodecUtils/RateControlImplMultiModel.h
../include/CodecUtils/RateControlImplPow.h
../include/CodecUtils/RateControlImplQuad.h
../include/CodecUtils/RawFileHandlerBase.h
../include/CodecUtils/RunBeforeH264VlcDecoder.h
../include/CodecUtils/RunBeforeH264VlcEncoder.h
../include/CodecUtils/SeqParamSetH264.h
../include/CodecUtils/SliceHeaderH264.h
../include/CodecUtils/TotalZeros2x2H264VlcDecoder.h
../include/CodecUtils/TotalZeros2x2H264VlcEncoder.h
../include/CodecUtils/TotalZeros2x4H264VlcDecoder.h
../include/CodecUtils/TotalZeros2x4H264VlcEncoder.h
../include/CodecUtils/TotalZeros4x4H264VlcDecoder.h
../include/CodecUtils/TotalZeros4x4H264VlcEncoder.h
../include/CodecUtils/VectorStructList.h
../include/CodecUtils/YuvRawFileHandler.h
)

SET(CODEC_UTIL_SRCS
CodecUtils/BitStreamReaderMSB.cpp
CodecUtils/BitStreamWriterMSB.cpp
CodecUtils/BlockH264.cpp
CodecUtils/CAVLCH264Impl2.cpp
CodecUtils/CAVLCH264Impl.cpp
CodecUtils/CodedBlkPatternH264VlcDecoder.cpp
CodecUtils/CodedBlkPatternH264VlcEncoder.cpp
#CodecUtils/CodeFragmentForH264BlockLayerTest.cpp
CodecUtils/CoeffTokenH264VlcDecoder.cpp
CodecUtils/CoeffTokenH264VlcEncoder.cpp
CodecUtils/ExpGolombUnsignedVlcDecoder.cpp
CodecUtils/ExpGolombUnsignedVlcEncoder.cpp
CodecUtils/FastForward4x4ITImpl1.cpp
CodecUtils/FastForward4x4ITImpl2.cpp
CodecUtils/FastForward4x4On16x16ITImpl1.cpp
CodecUtils/FastForwardDC2x2ITImpl1.cpp
CodecUtils/FastForwardDC4x4ITImpl1.cpp
CodecUtils/FastInverse4x4ITImpl1.cpp
CodecUtils/FastInverse4x4On16x16ITImpl1.cpp
CodecUtils/FastInverseDC2x2ITImpl1.cpp
CodecUtils/FastInverseDC4x4ITImpl1.cpp
CodecUtils/H263MotionVectorPredictorImpl1.cpp
CodecUtils/H264MbImgCache.cpp
CodecUtils/H264RawFileHandler.cpp
CodecUtils/MacroBlockH264.cpp
CodecUtils/MotionCompensatorH264ImplMultires.cpp
CodecUtils/MotionCompensatorH264ImplStd.cpp
CodecUtils/MotionEstimatorH263ImplMultiresCross.cpp
CodecUtils/MotionEstimatorH263ImplMultiresCrossVer2.cpp
CodecUtils/MotionEstimatorH264ImplCross.cpp
CodecUtils/MotionEstimatorH264ImplFull.cpp
CodecUtils/MotionEstimatorH264ImplMultires.cpp
CodecUtils/MotionEstimatorH264ImplMultiresCross.cpp
CodecUtils/MotionEstimatorH264ImplMultiresCrossVer2.cpp
CodecUtils/MotionEstimatorH264ImplTest.cpp
CodecUtils/NalHeaderH264.cpp
CodecUtils/PicParamSetH264.cpp
CodecUtils/RateControlImpl1.cpp
CodecUtils/RateControlImplLog.cpp
CodecUtils/RateControlImplMultiModel.cpp
CodecUtils/RateControlImplPow.cpp
CodecUtils/RateControlImplQuad.cpp
CodecUtils/RawFileHandlerBase.cpp
CodecUtils/RunBeforeH264VlcDecoder.cpp
CodecUtils/RunBeforeH264VlcEncoder.cpp
CodecUtils/SeqParamSetH264.cpp
CodecUtils/SliceHeaderH264.cpp
CodecUtils/TotalZeros2x2H264VlcDecoder.cpp
CodecUtils/TotalZeros2x2H264VlcEncoder.cpp
CodecUtils/TotalZeros2x4H264VlcDecoder.cpp
CodecUtils/TotalZeros2x4H264VlcEncoder.cpp
CodecUtils/TotalZeros4x4H264VlcDecoder.cpp
CodecUtils/TotalZeros4x4H264VlcEncoder.cpp
CodecUtils/VectorStructList.cpp
CodecUtils/YuvRawFileHandler.cpp
#CodecUtils/AdvancedIntraDctQuantiserImpl.cpp
#CodecUtils/AdvancedIntraDctQuantiserImplLookUp.cpp
#CodecUtils/AdvancedIntraModeH263VlcDecoder.cpp
#CodecUtils/AdvancedIntraModeH263VlcEncoder.cpp
#CodecUtils/BitStreamReader.cpp
#CodecUtils/BitStreamReaderMSB.cpp
#CodecUtils/BitStreamWriter.cpp
#CodecUtils/BitStreamWriterMSB.cpp
#CodecUtils/BlockH263.cpp
#CodecUtils/BlockH264.cpp
#CodecUtils/CAVLCH264Impl.cpp
#CodecUtils/CodecUtilsPch.cpp
#CodecUtils/CodedBlkPatternH264VlcDecoder.cpp
#CodecUtils/CodedBlkPatternH264VlcEncoder.cpp
#CodecUtils/#CodeFragmentForH264BlockLayerTest.cpp
#CodecUtils/CoeffTokenH264VlcDecoder.cpp
#CodecUtils/CoeffTokenH264VlcEncoder.cpp
#CodecUtils/ColourPlaneDecoding.cpp
#CodecUtils/ColourPlaneEncoding.cpp
#CodecUtils/DQuantH263VlcDecoder.cpp
#CodecUtils/DQuantH263VlcEncoder.cpp
#CodecUtils/ExpGolombUnsignedVlcDecoder.cpp
#CodecUtils/ExpGolombUnsignedVlcEncoder.cpp
#CodecUtils/FastForward4x4ITImpl1.cpp
#CodecUtils/FastForward4x4ITImpl2.cpp
#CodecUtils/FastForward4x4On16x16ITImpl1.cpp
#CodecUtils/FastForwardDC2x2ITImpl1.cpp
#CodecUtils/FastForwardDC4x4ITImpl1.cpp
#CodecUtils/FastForwardDctImpl1.cpp
#CodecUtils/FastForwardDctImpl2.cpp
#CodecUtils/FastInterLastRunLevelH263VlcEncoder.cpp
#CodecUtils/FastIntraLastRunLevelH263VlcEncoder.cpp
#CodecUtils/FastInverse4x4ITImpl1.cpp
#CodecUtils/FastInverse4x4On16x16ITImpl1.cpp
#CodecUtils/FastInverseDC2x2ITImpl1.cpp
#CodecUtils/FastInverseDC4x4ITImpl1.cpp
#CodecUtils/FastInverseDctImpl1.cpp
#CodecUtils/FastInverseDctImplZDet2.cpp
#CodecUtils/FastInverseDctImplZDet.cpp
#CodecUtils/FastMotionVectorVlcDecoderImpl1.cpp
#CodecUtils/FastMotionVectorVlcDecoderImpl2.cpp
#CodecUtils/FastRunLengthVlcDecoderImpl1.cpp
#CodecUtils/FastRunLengthVlcDecoderImpl2.cpp
#CodecUtils/FastVectorQuantiserVlcDecoderImpl1.cpp
#CodecUtils/FastVectorQuantiserVlcDecoderImpl2.cpp
#CodecUtils/H264MbImgCache.cpp
#CodecUtils/H264RawFileHandler.cpp
#CodecUtils/ImagePlaneDecoder.cpp
#CodecUtils/ImagePlaneDecoderIntraImpl.cpp
#CodecUtils/ImagePlaneDecoderStdImpl.cpp
#CodecUtils/ImagePlaneEncoderConstQImpl.cpp
#CodecUtils/ImagePlaneEncoder.cpp
#CodecUtils/ImagePlaneEncoderFastImpl1.cpp
#CodecUtils/ImagePlaneEncoderFastImpl2.cpp
#CodecUtils/ImagePlaneEncoderIntraImpl.cpp
#CodecUtils/ImagePlaneEncoderStdImpl.cpp
#CodecUtils/InterPCBPYH263VlcDecoder.cpp
#CodecUtils/InterPCBPYH263VlcEncoder.cpp
#CodecUtils/InterPDctQuantiserImpl.cpp
#CodecUtils/InterPDctQuantiserImplLookUp.cpp
#CodecUtils/InterPLastRunLevH263Impl.cpp
#CodecUtils/InterPMCBPCH263VlcDecoder.cpp
#CodecUtils/InterPMCBPCH263VlcEncoder.cpp
#CodecUtils/IntraCBPYH263VlcDecoder.cpp
#CodecUtils/IntraCBPYH263VlcEncoder.cpp
#CodecUtils/IntraDCH263VlcDecoder.cpp
#CodecUtils/IntraDCH263VlcEncoder.cpp
#CodecUtils/IntraDctQuantiserImpl.cpp
#CodecUtils/IntraDctQuantiserImplLookUp.cpp
#CodecUtils/IntraLastRunLevelH263VlcDecoder.cpp
#CodecUtils/IntraLastRunLevelH263VlcEncoder.cpp
#CodecUtils/IntraLastRunLevH263Impl.cpp
#CodecUtils/IntraMCBPCH263VlcDecoder.cpp
#CodecUtils/IntraMCBPCH263VlcEncoder.cpp
#CodecUtils/LastRunLevelH263List.cpp
#CodecUtils/LastRunLevelH263VlcDecoder.cpp
#CodecUtils/LastRunLevelH263VlcEncoder.cpp
#CodecUtils/MacroBlockH263.cpp
#CodecUtils/MacroBlockH264.cpp
#CodecUtils/ModDQuantH263VlcDecoder.cpp
#CodecUtils/ModDQuantH263VlcEncoder.cpp
#CodecUtils/ModQuantExtEscLevelH263VlcDecoder.cpp
#CodecUtils/ModQuantExtEscLevelH263VlcEncoder.cpp
#CodecUtils/MotionCompensatorH263ImplStd.cpp
#CodecUtils/MotionCompensatorH263ImplUnres.cpp
#CodecUtils/MotionCompensatorH264ImplStd.cpp
#CodecUtils/MotionCompensatorImpl1.cpp
#CodecUtils/MotionEstimatorH263ImplE3SS.cpp
#CodecUtils/MotionEstimatorH263ImplMultiresCross.cpp
#CodecUtils/MotionEstimatorH263ImplStd.cpp
#CodecUtils/MotionEstimatorH263ImplUnres.cpp
#CodecUtils/MotionEstimatorH264ImplTest.cpp
#CodecUtils/MotionEstimatorH264ImplCross.cpp
#CodecUtils/MotionEstimatorH264ImplFull.cpp
#CodecUtils/MotionEstimatorH264ImplMultires.cpp
#CodecUtils/MotionEstimatorH264ImplMultiresCross.cpp
#CodecUtils/MotionEstimatorH264ImplMultiresCrossVer2.cpp
#CodecUtils/MotionEstimatorImpl1.cpp
#CodecUtils/MotionEstimatorImpl2.cpp
#CodecUtils/MotionVectorH263VlcDecoderImplRev.cpp
#CodecUtils/MotionVectorH263VlcDecoderImplStd.cpp
#CodecUtils/MotionVectorH263VlcEncoderImplRev.cpp
#CodecUtils/MotionVectorH263VlcEncoderImplStd.cpp
#CodecUtils/MotionVectorVlcDecoder.cpp
#CodecUtils/MotionVectorVlcEncoder.cpp
#CodecUtils/NalHeaderH264.cpp
#CodecUtils/PicParamSetH264.cpp
#CodecUtils/RateControlImpl1.cpp
#CodecUtils/RateControlImplPow.cpp
#CodecUtils/RateControlImplLog.cpp
#CodecUtils/RateControlImplMultiModel.cpp
#CodecUtils/RateControlImplQuad.cpp
#CodecUtils/RawFileHandlerBase.cpp
#CodecUtils/RleMotionVectorCodec.cpp
#CodecUtils/RleMotionVectorList.cpp
#CodecUtils/RunBeforeH264VlcDecoder.cpp
#CodecUtils/RunBeforeH264VlcEncoder.cpp
#CodecUtils/RunLengthVlcDecoder.cpp
#CodecUtils/RunLengthVlcEncoder.cpp
#CodecUtils/SeqParamSetH264.cpp
#CodecUtils/SimpleMotionVectorList.cpp
#CodecUtils/SliceHeaderH264.cpp
#CodecUtils/StreamSwitcherVer3.cpp
#CodecUtils/SwitchCodecHeaderVer1.cpp
#CodecUtils/TotalZeros2x2H264VlcDecoder.cpp
#CodecUtils/TotalZeros2x2H264VlcEncoder.cpp
#CodecUtils/TotalZeros2x4H264VlcDecoder.cpp
#CodecUtils/TotalZeros2x4H264VlcEncoder.cpp
#CodecUtils/TotalZeros4x4H264VlcDecoder.cpp
#CodecUtils/TotalZeros4x4H264VlcEncoder.cpp
#CodecUtils/VectorList.cpp
#CodecUtils/VectorQuantiserDim16Impl1.cpp
#CodecUtils/VectorQuantiserDim16Impl2.cpp
#CodecUtils/VectorQuantiserDim16Impl3.cpp
#CodecUtils/VectorQuantiserVlcDecoder.cpp
#CodecUtils/VectorQuantiserVlcEncoder.cpp
#CodecUtils/VectorStructList.cpp
#CodecUtils/H263MotionVectorPredictorImpl1.cpp
#CodecUtils/MotionEstimatorH263ImplMultiresCrossVer2.cpp
#CodecUtils/YuvRawFileHandler.cpp
#CodecUtils/${VCODEC_SRCS}
)

#ADD_LIBRARY( RtvcCodecUtils STATIC ${CODEC_UTILS_SRCS} ${CODEC_UTIL_HDRS})

#if (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcCodecUtils PROPERTIES DEBUG_POSTFIX "D" )
#message("CodecUtils Debug Build")
#else (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcCodecUtils PROPERTIES DEBUG_POSTFIX "")
#message("CodecUtils Release Build")
#endif (CMAKE_BUILD_TYPE STREQUAL Debug)

# CMakeLists.txt in GeneralUtils dir
# Adds a library called GeneralUtils(libRtvcGeneralUtils.a under Linux)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")
# get rid of some compiler warnings
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_CRT_SECURE_NO_WARNINGS")

SET(GEN_UTIL_HDRS
#../include/GeneralUtils/DataProfileImplBase.h
#../include/GeneralUtils/DataProfileImplCongest.h
#../include/GeneralUtils/DataProfileImplRandom.h
#../include/GeneralUtils/DataProfileImplSetQ8Intra.h
#../include/GeneralUtils/IDataProfile.h
../include/GeneralUtils/MeasurementTable.h
#stdafx.h
)

SET(GEN_UTIL_SRCS
#GeneralUtils/DataProfileImplBase.cpp
#GeneralUtils/DataProfileImplCongest.cpp
#GeneralUtils/DataProfileImplRandom.cpp
#GeneralUtils/DataProfileImplSetQ8Intra.cpp
GeneralUtils/MeasurementTable.cpp
#stdafx.cpp
)

#ADD_LIBRARY( RtvcGeneralUtils STATIC ${GEN_UTILS_SRCS} ${GEN_UTILS_HDRS})
#
#if (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcGeneralUtils PROPERTIES DEBUG_POSTFIX "D" )
#message("GeneralUtils Debug Build")
#else (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcGeneralUtils PROPERTIES DEBUG_POSTFIX "")
#message("GeneralUtils Release Build")
#endif (CMAKE_BUILD_TYPE STREQUAL Debug)

# CMakeLists.txt in ImageUtils dir
# Adds a library called ImageUtils (libRtvcImageUtils.a under Linux)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")
# get rid of some compiler warnings
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_CRT_SECURE_NO_WARNINGS")

#INCLUDE_DIRECTORIES(
#${artistIncludes}
#${artist_SOURCE_DIR}/libs/GeneralUtils
#)

## Lib directories
#LINK_DIRECTORIES(
#${artistLink}
#)
# see (http://stackoverflow.com/questions/11580748/using-cmake-for-making-a-project-which-includes-mfc)
# see http://stackoverflow.com/questions/16698882/error-c2039-setdefaultdlldirectoriesis-not-a-member-of-global-namespace

IF (BUILD_CODEC_ANALYSER)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_MFC_IMAGE_UTILS -D_AFXDLL -D_USING_V110_SDK71_")
SET(MFC_HDRS
../include/ImageUtils/AutoCrop.h
../include/ImageUtils/AviFile.h
../include/ImageUtils/AviFileHandler.h
../include/ImageUtils/AviFileHandlerUsingCImage.h
../include/ImageUtils/Block.h
../include/ImageUtils/CImage.h
../include/ImageUtils/Graph.h
../include/ImageUtils/ImageHandler.h
../include/ImageUtils/ImageHandlerNET.h
)
ELSE(BUILD_CODEC_ANALYSER)
SET(MFC_HDRS
)
ENDIF(BUILD_CODEC_ANALYSER)

SET(IMAGE_UTIL_HDRS
${MFC_HDRS}
../include/ImageUtils/FastFixedPointRGB24toYUV420Converter.h
../include/ImageUtils/FastGpuRGB24toYUV420Converter.h
../include/ImageUtils/FastLookupTableRGB24toYUV420Converter.h
#../include/ImageUtils/FastSimdRGB24toYUV420Converter.h
../include/ImageUtils/MtRGB24toYUV420Converter.h
../include/ImageUtils/OverlayExtMem2Dv2.h
../include/ImageUtils/OverlayMem2Dv2.h
../include/ImageUtils/PicRotateBase.h
../include/ImageUtils/PicRotateRGB24Impl.h
../include/ImageUtils/PicRotateRGB32Impl.h
../include/ImageUtils/PicRotateRGBBase.h
../include/ImageUtils/RealRGB24toYUV420CCIR601ConverterVer16.h
../include/ImageUtils/RealRGB24toYUV420Converter.h
../include/ImageUtils/RealRGB24toYUV420ConverterImpl2.h
../include/ImageUtils/RealRGB24toYUV420ConverterImpl2Ver16.h
../include/ImageUtils/RealRGB24toYUV420ConverterStl.h
../include/ImageUtils/RealRGB32toYUV420ConverterStl.h
../include/ImageUtils/RealYUV420toRGB24CCIR601ConverterVer16.h
../include/ImageUtils/RealYUV420toRGB24Converter.h
../include/ImageUtils/RealYUV420toRGB24ConverterImpl2.h
../include/ImageUtils/RealYUV420toRGB24ConverterImpl2Ver16.h
../include/ImageUtils/RealYUV420toRGB24ConverterStl.h
../include/ImageUtils/RealYUV444toRGB24Converter.h
../include/ImageUtils/RGBtoYUV420Converter.h
../include/ImageUtils/RGBtoYUV420ConverterStl.h
../include/ImageUtils/YUV420toRGBConverter.h
../include/ImageUtils/YUV420toRGBConverterStl.h
../include/ImageUtils/YUV444toRGBConverter.h
##AviFileHandlerNET.h
##Block.h
#../include/ImageUtils/BlockMotionVector.h
#../include/ImageUtils/BlockPatternConstants.h
#../include/ImageUtils/BlockPatternVlcEncoder.h
#../include/ImageUtils/Codec.h
#../include/ImageUtils/Dct2dodd.h
#../include/ImageUtils/Dwtcodec.h
#../include/ImageUtils/DwtQnt.h
#../include/ImageUtils/EncBlockMotionVector.h
#../include/ImageUtils/EncMotionFactory.h
#../include/ImageUtils/EncMotionVector.h
#../include/ImageUtils/EncMotionVectorPattern.h
#../include/ImageUtils/ExtMem2D.h
#../include/ImageUtils/ExtMemBlock2D.h
#../include/ImageUtils/Fast6BitYUV420toRGB24Converter.h
#../include/ImageUtils/Fast6BitYUV420toRGB24ZoomConverter.h
#../include/ImageUtils/FastRGB24toYUV420Converter.h
#../include/ImageUtils/FastYUV420toInvRGB12Converter.h
#../include/ImageUtils/FastYUV420toInvRGB16Converter.h
#../include/ImageUtils/FastYUV420toInvRGB24Converter.h
#../include/ImageUtils/FastYUV420toInvRGB32Converter.h
#../include/ImageUtils/FastYUV420toInvRGB32Converter_ByteWrite.h
#../include/ImageUtils/FastYUV420toRGB16Converter.h
#../include/ImageUtils/FastYUV420toRGB24Converter.h
#../include/ImageUtils/FastYUV420toRGB24ZoomConverter.h
#../include/ImageUtils/FastYUV420toRGB32Converter.h
#../include/ImageUtils/FastYUV420toRGB32Converter_ByteWrite.h
#../include/ImageUtils/FastYUV420toRGBConverter.h
#../include/ImageUtils/H263Codec.h
#../include/ImageUtils/IExistance.h
#../include/ImageUtils/ImageDefs.h
#../include/ImageUtils/ImageHandlerV2.h
#../include/ImageUtils/ImageObjectHandler.h
#../include/ImageUtils/ImgCodec.h
#../include/ImageUtils/ImgSmp.h
#../include/ImageUtils/IMotionFactory.h
#../include/ImageUtils/Mem2D.h
#../include/ImageUtils/MemBlock2D.h
#../include/ImageUtils/MotionVector.h
#../include/ImageUtils/MotionVectorPattern.h
#../include/ImageUtils/MotionVectorTree.h
#../include/ImageUtils/OverlayExtMem2D.h
#../include/ImageUtils/OverlayExtMem2Dv2.h
#../include/ImageUtils/OverlayMem2D.h
#../include/ImageUtils/OverlayMem2Dv2.h
#../include/ImageUtils/PicConcatBase.h
#../include/ImageUtils/PicConcatRGB24Impl.h
#../include/ImageUtils/PicConcatRGB32Impl.h
#../include/ImageUtils/PicCropperBase.h
#../include/ImageUtils/PicCropperRGB24Impl.h
#../include/ImageUtils/PicCropperRGB32Impl.h
#../include/ImageUtils/PicInPicBase.h
#../include/ImageUtils/PicInPicRGB24Impl.h
#../include/ImageUtils/PicInPicRGB32Impl.h
#../include/ImageUtils/PicRotateBase.h
#../include/ImageUtils/PicRotateRGB24Impl.h
#../include/ImageUtils/PicRotateRGB32Impl.h
#../include/ImageUtils/PicRotateRGBBase.h
#../include/ImageUtils/PicScalerBase.h
#../include/ImageUtils/PicScalerRGB24Impl.h
#../include/ImageUtils/PicScalerYUV420PImpl.h
#../include/ImageUtils/PsyQnt.h
#../include/ImageUtils/Real6BitRGB24toYUV420Converter.h
#../include/ImageUtils/Real6BitYUV420toRGB24Converter.h
#../include/ImageUtils/RealRGB24toYUV420CCIR601Converter.h
#../include/ImageUtils/RealRGB24toYUV420CCIR601ConverterVer16.h
#../include/ImageUtils/RealRGB24toYUV420Converter.h
#../include/ImageUtils/RealRGB24toYUV420ConverterImpl2.h
#../include/ImageUtils/RealRGB24toYUV420ConverterImpl2Ver16.h
#../include/ImageUtils/RealRGB24toYUV420ConverterStl.h
#../include/ImageUtils/RealRGB32toYUV420Converter.h
#../include/ImageUtils/RealRGB32toYUV420ConverterStl.h
#../include/ImageUtils/RealYUV420toRGB24CCIR601Converter.h
#../include/ImageUtils/RealYUV420toRGB24CCIR601ConverterVer16.h
#../include/ImageUtils/RealYUV420toRGB24Converter.h
#../include/ImageUtils/RealYUV420toRGB24ConverterImpl2.h
#../include/ImageUtils/RealYUV420toRGB24ConverterImpl2Ver16.h
#../include/ImageUtils/RealYUV420toRGB24ConverterStl.h
#../include/ImageUtils/RealYUV444toRGB24Converter.h
#../include/ImageUtils/RGB24toRGB32Converter.h
#../include/ImageUtils/RGB32toRGB24Converter.h
#../include/ImageUtils/RGBtoRGBConverter.h
#../include/ImageUtils/RGBtoYUV420Converter.h
#../include/ImageUtils/RGBtoYUV420ConverterStl.h
#../include/ImageUtils/Rlcodec.h
#../include/ImageUtils/Sampler.h
#../include/ImageUtils/SampleSet.h
#../include/ImageUtils/stdafx.h
#../include/ImageUtils/VectorQuantiser.h
#../include/ImageUtils/VectorQuantiserVlcEncoder.h
#../include/ImageUtils/VectorSpace.h
#../include/ImageUtils/VidCodec.h
#../include/ImageUtils/VideoCodecVer01.h
#../include/ImageUtils/VideoDim16VectorQuantiser.h
#../include/ImageUtils/VQcodec.h
#../include/ImageUtils/WaveletCompress.h
#../include/ImageUtils/WImage.h
#../include/ImageUtils/WImgPce.h
#../include/ImageUtils/YUV420toRGBConverter.h
#../include/ImageUtils/YUV420toRGBConverterStl.h
#../include/ImageUtils/YUV444toRGBConverter.h
)

# for now just check if we're building the codec analyser
# as a check whether to include the MFC-related classes
IF (BUILD_CODEC_ANALYSER)
SET(MFC_SRCS
ImageUtils/AutoCrop.cpp
ImageUtils/AviFile.cpp
ImageUtils/AviFileHandler.cpp
ImageUtils/CImage.cpp
ImageUtils/Graph.cpp
ImageUtils/ImageHandler.cpp
)
ELSE(BUILD_CODEC_ANALYSER)
SET(MFC_SRCS
)
ENDIF(BUILD_CODEC_ANALYSER)

SET(IMAGE_UTIL_SRCS
#${MFC_SRCS}
ImageUtils/FastFixedPointRGB24toYUV420Converter.cpp
ImageUtils/FastGpuRGB24toYUV420Converter.cpp
ImageUtils/FastLookupTableRGB24toYUV420Converter.cpp
#ImageUtils/FastSimdRGB24toYUV420Converter.cpp
ImageUtils/MtRGB24toYUV420Converter.cpp
ImageUtils/OverlayExtMem2Dv2.cpp
ImageUtils/OverlayMem2Dv2.cpp
ImageUtils/PicRotateRGB24Impl.cpp
ImageUtils/PicRotateRGB32Impl.cpp
ImageUtils/PicRotateRGBBase.cpp
ImageUtils/RealRGB24toYUV420CCIR601ConverterVer16.cpp
ImageUtils/RealRGB24toYUV420Converter.cpp
ImageUtils/RealRGB24toYUV420ConverterImpl2.cpp
ImageUtils/RealRGB24toYUV420ConverterImpl2Ver16.cpp
ImageUtils/RealYUV420toRGB24CCIR601ConverterVer16.cpp
ImageUtils/RealYUV420toRGB24Converter.cpp
ImageUtils/RealYUV420toRGB24ConverterImpl2.cpp
ImageUtils/RealYUV420toRGB24ConverterImpl2Ver16.cpp
ImageUtils/RealYUV444toRGB24Converter.cpp
##AviFileHandlerNET.cpp
##AviFileHandlerUsingCImage.cpp
##BlockMotionVector.cpp
##BlockPatternVlcEncoder.cpp
##Dct2dodd.cpp
##Dwtcodec.cpp
##DwtQnt.cpp
##EncBlockMotionVector.cpp
##EncMotionFactory.cpp
##EncMotionVector.cpp
##EncMotionVectorPattern.cpp
##ExtMem2D.cpp
##ExtMemBlock2D.cpp
##Fast6BitYUV420toRGB24Converter.cpp
##Fast6BitYUV420toRGB24ZoomConverter.cpp
##FastRGB24toYUV420Converter.cpp
##FastYUV420toInvRGB12Converter.cpp
##FastYUV420toInvRGB16Converter.cpp
##FastYUV420toInvRGB24Converter.cpp
##FastYUV420toInvRGB32Converter_ByteWrite.cpp
##FastYUV420toInvRGB32Converter.cppadd_definitions(
##FastYUV420toRGB16Converter.cpp
##FastYUV420toRGB24Converter.cpp
##FastYUV420toRGB24ZoomConverter.cpp
##FastYUV420toRGB32Converter_ByteWrite.cpp
##FastYUV420toRGB32Converter.cpp
##H263Codec.cpp
##ImageHandlerNET.cpp
##ImageHandlerV2.cpp
##ImageObjectHandler.cpp
##ImgCodec.cpp
##ImgSmp.cpp
##Mem2D.cpp
##MemBlock2D.cpp
##MotionVector.cpp
##MotionVectorPattern.cpp
##MotionVectorTree.cpp
#ImageUtils/OverlayExtMem2D.cpp
#ImageUtils/OverlayExtMem2Dv2.cpp
#ImageUtils/OverlayMem2D.cpp
#ImageUtils/OverlayMem2Dv2.cpp
#ImageUtils/PicConcatRGB24Impl.cpp
#ImageUtils/PicConcatRGB32Impl.cpp
#ImageUtils/PicCropperRGB24Impl.cpp
#ImageUtils/PicCropperRGB32Impl.cpp
#ImageUtils/PicInPicRGB24Impl.cpp
#ImageUtils/PicInPicRGB32Impl.cpp
#ImageUtils/PicRotateRGB24Impl.cpp
#ImageUtils/PicRotateRGB32Impl.cpp
#ImageUtils/PicRotateRGBBase.cpp
#ImageUtils/PicScalerRGB24Impl.cpp
#ImageUtils/PicScalerYUV420PImpl.cpp
##PsyQnt.cpp
##Real6BitRGB24toYUV420Converter.cpp
##Real6BitYUV420toRGB24Converter.cpp
##RealRGB24toYUV420CCIR601Converter.cpp
#ImageUtils/RealRGB24toYUV420Converter.cpp
#ImageUtils/RealRGB24toYUV420ConverterImpl2.cpp
#ImageUtils/RealRGB24toYUV420ConverterImpl2Ver16.cpp
#ImageUtils/RealRGB32toYUV420Converter.cpp
##RealYUV420toRGB24CCIR601Converter.cpp
#ImageUtils/RealYUV420toRGB24Converter.cpp
#ImageUtils/RealYUV420toRGB24ConverterImpl2.cpp
#ImageUtils/RealYUV420toRGB24ConverterImpl2Ver16.cpp
#ImageUtils/RealYUV444toRGB24Converter.cpp
#ImageUtils/RGB24toRGB32Converter.cpp
#ImageUtils/RGB32toRGB24Converter.cpp
##Rlcodec.cpp
##Sampler.cpp
##SampleSet.cpp
##stdafx.cpp
##VectorQuantiserVlcEncoder.cpp
##VectorSpace.cpp
##VidCodec.cpp
##VideoDim16VectorQuantiser.cpp
##VQcodec.cpp
##WaveletCompress.cpp
##WImage.cpp
##WImgPce.cpp
)

#ADD_DEFINITIONS( -D_BUILD_FOR_SHORT )
#
#ADD_LIBRARY( RtvcImageUtils STATIC ${IMAGE_UTILS_SRCS} ${IMAGE_UTILS_HDRS})
#
#if (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcImageUtils PROPERTIES DEBUG_POSTFIX "D" )
#message("Shared Debug Build")
#else (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcImageUtils PROPERTIES DEBUG_POSTFIX "")
#message("Shared Release Build")
#endif (CMAKE_BUILD_TYPE STREQUAL Debug)

# CMakeLists.txt in StreamUtils dir
# Adds a library called StreamUtils(libRtvcStreamUtils.a under Linux)
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_BUILD_FOR_SHORT")

IF (BUILD_CODEC_ANALYSER)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_MFC_IMAGE_UTILS -D_AFXDLL -D_USING_V110_SDK71_")
SET(MFC_HDRS
../include/StreamUtils/CircularStream.h
)
ELSE(BUILD_CODEC_ANALYSER)
SET(MFC_HDRS
)
ENDIF(BUILD_CODEC_ANALYSER)

SET(STREAM_UTIL_HDRS
#${MFC_HDRS}
#../include/StreamUtils/ByteStreamBase.h
#../include/StreamUtils/ByteStreamReader.h
#../include/StreamUtils/ByteStreamWriter.h
#../include/StreamUtils/IStreamBase.h
#../include/StreamUtils/IStreamReader.h
#../include/StreamUtils/IStreamWriter.h
#../include/StreamUtils/StreamChunkHeader.h
)

IF(BUILD_CODEC_ANALYSER)
SET(STREAM_UTIL_SRCS
#StreamUtils/ByteStreamReader.cpp
#StreamUtils/ByteStreamWriter.cpp
#StreamUtils/CircularStream.cpp
#StreamUtils/StreamChunkHeader.cpp
)
ELSE(BUILD_CODEC_ANALYSER)
SET(STREAM_UTIL_SRCS
#StreamUtils/ByteStreamReader.cpp
#StreamUtils/ByteStreamWriter.cpp
#StreamUtils/StreamChunkHeader.cpp
)
ENDIF(BUILD_CODEC_ANALYSER)


#ADD_LIBRARY( RtvcStreamUtils STATIC ${STREAM_UTILS_SRCS} ${STREAM_UTIL_HDRS})
#
#if (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcStreamUtils PROPERTIES DEBUG_POSTFIX "D" )
#message("StreamUtils Debug Build")
#else (CMAKE_BUILD_TYPE STREQUAL Debug)
#SET_TARGET_PROPERTIES(RtvcStreamUtils PROPERTIES DEBUG_POSTFIX "")
#message("StreamUtils Release Build")
#endif (CMAKE_BUILD_TYPE STREQUAL Debug)


SET(VPP_HDRS
${CODEC_UTIL_HDRS}
${GEN_UTIL_HDRS}
${IMAGE_UTIL_HDRS}
${STREAM_UTIL_HDRS}
)

SET(VPP_SRCS
${CODEC_UTIL_SRCS}
${GEN_UTIL_SRCS}
${IMAGE_UTIL_SRCS}
${STREAM_UTIL_SRCS}
)


ADD_LIBRARY( VppLib STATIC ${VPP_SRCS} ${VPP_HDRS})

if (CMAKE_BUILD_TYPE STREQUAL Debug)
SET_TARGET_PROPERTIES(VppLib PROPERTIES DEBUG_POSTFIX "D" )
message("VppLib Debug Build")
else (CMAKE_BUILD_TYPE STREQUAL Debug)
SET_TARGET_PROPERTIES(VppLib PROPERTIES DEBUG_POSTFIX "")
message("VppLib Release Build")
endif (CMAKE_BUILD_TYPE STREQUAL Debug)

