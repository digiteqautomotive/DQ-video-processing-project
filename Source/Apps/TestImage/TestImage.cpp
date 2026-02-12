/** Copyright (c) 2025-2026 Jaroslav Fojtik
    All rights reserved.
**/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "PicScalerRGB24Impl.h"
#include "PicScalerARGB32Impl.h"
#include "PicScalerRGB32Impl.h"
#include "PicScalerARGB32MMX.h"
#include "PicScalerARGB32SSE.h"
#include "PicScalerYUYVImpl.h"
#include "PicScalerYUV420PImpl.h"
#include "ScaleTesters.h"

#include "PicRotateRGB32Impl.h"
#include "PicRotateRGB24Impl.h"
#include "RotateTesters.h"

#include "ConvAYUV444toRGB.h"
#include "RealYUV420toRGB24Converter.h"
#include "ConvRGB24toYUV420_Real.h "
#include "ConvRGB32toYUV420_Real.h "
#include "RGB24toYUV420Converter.h"
#include "RGB32toYUV420Converter.h"
#include "ConvRGB24toYUV420_LUT.h"
#include "ConvRGB24toYUV420_SIMD.h"
#include "ConvertTesters.h"


extern "C"
{
 unsigned long long GetTickCount_us(void);
#if defined(USE_SSE) || defined(USE_MMX)
 unsigned GetFeaturesCPU(void);
#else
 unsigned GetFeaturesCPU(void) {return 0;}
#endif 
}

unsigned char FeaturesCPU = 0x80;


#ifndef int32_t
 #define int32_t int
#endif

#define BITS_PER_PIXEL_YUV420P	12


///////////////////////////////////////////////////////////////////////////////////////////////////


unsigned long BenchConverter(BaseVideoConverter &converter, const void *BlobIn, void *BlobOut)
{
 unsigned long long TimeStampB, TimeStampE;
 unsigned long DurationMin = ~0;
 unsigned long Duration = ~0;
 for(int i=0; i<10; i++)
 {
   TimeStampB = GetTickCount_us();
   converter.Convert(BlobIn,BlobOut);
   TimeStampE = GetTickCount_us();
   Duration = TimeStampE - TimeStampB;
   if(DurationMin > Duration) DurationMin=Duration;
 }
 return DurationMin;
}



unsigned long BenchScaller(PicScalerBase &pScaller, const void *BlobIn, void *BlobOut)
{
 unsigned long long TimeStampB, TimeStampE;
 unsigned long DurationMin = ~0;
 unsigned long Duration = ~0;
 for(int i=0; i<10; i++)
 {
   TimeStampB = GetTickCount_us();
   pScaller.Scale(BlobOut,BlobIn);
   TimeStampE = GetTickCount_us();
   Duration = TimeStampE - TimeStampB;
   if(DurationMin > Duration) DurationMin=Duration;
 }
 return DurationMin;
}


void BenchRotator(const char *RotName, PicRotateRGBBase &pRotator, const void *BlobIn, void *BlobOut)
{
unsigned long long TimeStampB, TimeStampE, Duration, DurationMin;
int i;

  pRotator.SetInDimensions(1920,1080);
  pRotator.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);		
  DurationMin = ~0;
  for(i=0; i<10; i++)
  {
    TimeStampB = GetTickCount_us();
    pRotator.Rotate(BlobIn,BlobOut);
    TimeStampE = GetTickCount_us();
    Duration = TimeStampE - TimeStampB;
    if(DurationMin > Duration) DurationMin=Duration;
  }
  printf("\n%s 1920,1080 time +90deg %.3f", RotName, DurationMin/1000.0);

  pRotator.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
  DurationMin = ~0;
  for(i=0; i<10; i++)
  {
    TimeStampB = GetTickCount_us();
    pRotator.Rotate(BlobIn,BlobOut);
    TimeStampE = GetTickCount_us();
    Duration = TimeStampE - TimeStampB;
    if(DurationMin > Duration) DurationMin=Duration;
  }
  printf(" +180deg %.3f", DurationMin/1000.0);

  pRotator.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
  DurationMin = ~0;
  for(i=0; i<10; i++)
  {
    TimeStampB = GetTickCount_us();
    pRotator.Rotate(BlobIn,BlobOut);
    TimeStampE = GetTickCount_us();
    Duration = TimeStampE - TimeStampB;
    if(DurationMin > Duration) DurationMin=Duration;
  }
  printf(" +270deg %.3f", DurationMin/1000.0);

  pRotator.SetRotateMode(ROTATE_FLIP_VERTICAL);
  DurationMin = ~0;
  for(i=0; i<10; i++)
  {
    TimeStampB = GetTickCount_us();
    pRotator.Rotate(BlobIn,BlobOut);
    TimeStampE = GetTickCount_us();
    Duration = TimeStampE - TimeStampB;
    if(DurationMin > Duration) DurationMin=Duration;
  }
  printf(" FLIP_V %.3f", DurationMin/1000.0);
  pRotator.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
  DurationMin = ~0;
  for(i=0; i<10; i++)
  {
    TimeStampB = GetTickCount_us();
    pRotator.Rotate(BlobIn,BlobOut);
    TimeStampE = GetTickCount_us();
    Duration = TimeStampE - TimeStampB;
    if(DurationMin > Duration) DurationMin=Duration;
  }
  printf(" FLIP_H %.3f", DurationMin/1000.0);
  pRotator.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
  DurationMin = ~0;
  for(i=0; i<10; i++)
  {
    TimeStampB = GetTickCount_us();
    pRotator.Rotate(BlobIn,BlobOut);
    TimeStampE = GetTickCount_us();
    Duration = TimeStampE - TimeStampB;
    if(DurationMin > Duration) DurationMin=Duration;
  }
  printf(" FLIP_D %.3f[ms]", DurationMin/1000.0);
}


int main(int argc, char *argv[])
{
PicScalerRGB32Impl picScalerRGB32;
unsigned char *BlobIn, *BlobOut, *BlobTest;
int i;
bool Pause = false;

  printf("<TEST> Testing scale computing %u[bits] MSVC: %u Build: %s %s", 8*sizeof(void*), _MSC_VER, __DATE__, __TIME__);

  if(argc>1)
  {
    for(i=1; i<argc; i++)
    {
      if(!strcmp(argv[i],"-pause")) Pause=true;
    }
  }

  FeaturesCPU = GetFeaturesCPU();
  BlobIn = (unsigned char *)malloc(16384);
  BlobOut = (unsigned char *)malloc(16384);
  BlobTest = (unsigned char *)malloc(16384);
  
  {
    BlobIn[0] = 0xFF;
    BlobIn[1] = 0xFF;
    BlobIn[2] = 0xFF;
    PicScalerRGB24Impl picScalerRGB24(1,1,1,1);

    picScalerRGB24.Scale(BlobOut,BlobIn);
    if(BlobOut[0]!=0xFF || BlobOut[1]!= 0xFF || BlobOut[2]!=0xFF)
    {
      printf("\nRGB24 picScalerRGB24 on 0xFF overflows [%2.2X,%2.2X,%2.2X].", BlobOut[0], BlobOut[1], BlobOut[2]);
      goto ReturnErr;
    }

    BlobIn[3] = 0xFF;
    PicScalerRGB32Impl picScalerRGB32(1,1,1,1);

    picScalerRGB32.Scale(BlobOut,BlobIn);
    if(BlobOut[0]!=0xFF || BlobOut[1]!= 0xFF || BlobOut[2]!=0xFF || BlobOut[3]!=0xFF)
    {
      printf("\nRGB32 picScalerRGB32 on 0xFF overflows [%2.2X,%2.2X,%2.2X(%2.2X)].", BlobOut[0], BlobOut[1], BlobOut[2], BlobOut[3]);
      goto ReturnErr;
    }

    PicScalerARGB32Impl picScalerARGB32(1,1,1,1);
    picScalerARGB32.Scale(BlobOut,BlobIn);
    if(BlobOut[0]!=0xFF || BlobOut[1]!= 0xFF || BlobOut[2]!=0xFF || BlobOut[3]!=0xFF)
    {
      printf("\nARGB32 picScalerARGB32 on 0xFF overflows [%2.2X,%2.2X,%2.2X,%2.2X].", BlobOut[0], BlobOut[1], BlobOut[2], BlobOut[3]);
      goto ReturnErr;
    }

#ifdef USE_MMX
    if(FeaturesCPU & 1)
    {
      PicScalerARGB32MMX picScalerARGB32mmx(1,1,1,1);
      picScalerARGB32mmx.Scale(BlobOut,BlobIn);
      if(BlobOut[0]!=0xFF || BlobOut[1]!= 0xFF || BlobOut[2]!=0xFF || BlobOut[3]!=0xFF)
      {
        printf("\nARGB32 picScalerARGB32_MMX on 0xFF overflows [%2.2X,%2.2X,%2.2X,%2.2X].", BlobOut[0], BlobOut[1], BlobOut[2], BlobOut[3]);
        goto ReturnErr;
      }
    }
#endif
#ifdef USE_SSE
    if(FeaturesCPU & 2)
    {
      PicScalerARGB32SSE picScalerARGB32sse(1,1,1,1);
      picScalerARGB32sse.Scale(BlobOut,BlobIn);
      if(BlobOut[0]!=0xFF || BlobOut[1]!= 0xFF || BlobOut[2]!=0xFF || BlobOut[3]!=0xFF)
      {
        printf("\nARGB32 picScalerARGB32_SSE on 0xFF overflows [%2.2X,%2.2X,%2.2X,%2.2X].", BlobOut[0], BlobOut[1], BlobOut[2], BlobOut[3]);
        goto ReturnErr;
      }
    }
#endif
  }

  for(i=0; i<16384; i++)
  {
    BlobIn[i] = i;
  }

  for(int WithIn=0; WithIn<6; WithIn++)
   for(int HeightIn=0; HeightIn<6; HeightIn++)
    for(int WithOut=0; WithOut<6; WithOut++)
     for(int HeightOut=0; HeightOut<6; HeightOut++)
     {
       {
         int InSize = 3 * WithIn * HeightIn;
         int OutSize = 3 * WithOut * HeightOut;
         if(InSize==0) OutSize=0;
         PicScalerRGB24Impl picScalerRGB24(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerRGB24Impl origScalerRGB24(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
         memcpy(BlobOut+16+OutSize,BlobIn,16);
         memcpy(BlobTest+16+OutSize,BlobIn,16);

         picScalerRGB24.Scale(BlobOut+16,BlobIn+16);
         origScalerRGB24.Scale(BlobTest+16,BlobIn+16);

         int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nRGB24 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }
         //printf("\nOK Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
       }


#ifdef USE_MMX
       if(FeaturesCPU & 1)
       {
	 int InSize = 4 * WithIn * HeightIn;
         int OutSize = 4 * WithOut * HeightOut;
         if(InSize==0) OutSize=0;
         PicScalerARGB32MMX picScalerARGB32mmx(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerARGB32Impl origScalerARGB32(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
	 memset(BlobOut+16,0xFE,OutSize);
         memcpy(BlobOut+16+OutSize,BlobIn,16);
         memcpy(BlobTest+16+OutSize,BlobIn,16);

         picScalerARGB32mmx.Scale(BlobOut+16,BlobIn+16);
         origScalerARGB32.Scale(BlobTest+16,BlobIn+16);

         int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nARGB32_MMX Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }
         //printf("\nOK ARGB32 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
       }
#endif
#ifdef USE_SSE
       if(FeaturesCPU & 2)
       {
	 int InSize = 4 * WithIn * HeightIn;
         int OutSize = 4 * WithOut * HeightOut;
         if(InSize==0) OutSize=0;
         PicScalerARGB32SSE picScalerARGB32sse(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerARGB32Impl origScalerARGB32(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
	 memset(BlobOut+16,0xFE,OutSize);
         memcpy(BlobOut+16+OutSize,BlobIn,16);
         memcpy(BlobTest+16+OutSize,BlobIn,16);

         picScalerARGB32sse.Scale(BlobOut+16,BlobIn+16);
         origScalerARGB32.Scale(BlobTest+16,BlobIn+16);

         int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nARGB32_SSE Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }
         //printf("\nOK ARGB32 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
       }
#endif

       {
         int InSize = 4 * WithIn * HeightIn;
         int OutSize = 4 * WithOut * HeightOut;
         if(InSize==0) OutSize=0;
         PicScalerARGB32Impl picScalerARGB32(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerARGB32Impl origScalerARGB32(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
         memset(BlobOut+16,0xFE,OutSize);
         memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
         memcpy(BlobTest+16+OutSize,BlobIn,16); // reference tail

         picScalerARGB32.Scale(BlobOut+16,BlobIn+16);
         origScalerARGB32.Scale(BlobTest+16,BlobIn+16);

         int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nARGB32 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }
         //printf("\nOK ARGB32 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
       }

       {
         int InSize = 4 * WithIn * HeightIn;
         int OutSize = 4 * WithOut * HeightOut;
         if(InSize==0) OutSize=0;
         PicScalerRGB32Impl picScalerRGB32(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerRGB32Impl origScalerRGB32(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
         memcpy(BlobOut+16+OutSize,BlobIn,16);
         memcpy(BlobTest+16+OutSize,BlobIn,16);

         picScalerRGB32.Scale(BlobOut+16,BlobIn+16);
         origScalerRGB32.Scale(BlobTest+16,BlobIn+16);

         int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nRGB32 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }
         //printf("\nOK RGB32 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
       }

     }

//-----------------------------------YUYV-----------------------------------------------

  for(int WithIn=0; WithIn<6; WithIn+=2)
   for(int HeightIn=0; HeightIn<5; HeightIn++)
    for(int WithOut=0; WithOut<6; WithOut+=2)
     for(int HeightOut=0; HeightOut<5; HeightOut++)
     {
       {
         int InSize = 2 * WithIn * HeightIn;
         int OutSize = 2 * WithOut * HeightOut;
         if(InSize==0) OutSize=0;
         PicScalerYUYVImpl picScalerYUYV(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerYUYVImpl origScalerYUYV(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
         memset(BlobOut+16,0xFE,OutSize);
         memset(BlobTest+16,0xFC,OutSize);
         memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
         memcpy(BlobTest+16+OutSize,BlobIn,16); // reference tail

         picScalerYUYV.Scale(BlobOut+16,BlobIn+16);
         origScalerYUYV.Scale(BlobTest+16,BlobIn+16);

         const int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nYUYV Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }

       }
     }

//-----------------------------------YV420-----------------------------------------------

  for(int WithIn=0; WithIn<12; WithIn+=2)
   for(int HeightIn=0; HeightIn<10; HeightIn+=2)
    for(int WithOut=0; WithOut<12; WithOut+=2)
     for(int HeightOut=0; HeightOut<10; HeightOut+=2)
     {
       {
         int InSize = (BITS_PER_PIXEL_YUV420P * WithIn * HeightIn) / 8;
         int OutSize = (BITS_PER_PIXEL_YUV420P * WithOut * HeightOut) / 8;
         if(InSize==0) OutSize=0;
         PicScalerYUV420PImpl picScalerYUV420(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerYUV420Impl origScalerYUV420(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
         memset(BlobOut+16,0xFE,OutSize);
         memset(BlobTest+16,0xFC,OutSize);
         memcpy(BlobOut+16+OutSize,BlobIn,16);		// tail
         memcpy(BlobTest+16+OutSize,BlobIn,16);		// reference tail

         picScalerYUV420.Scale(BlobOut+16,BlobIn+16);
         origScalerYUV420.Scale(BlobTest+16,BlobIn+16);

         const int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nYUV420 Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }

       }
     }


//-----------------------------Rotate-RGB32---------------------------------------------

  for(int WithIn=0; WithIn<6; WithIn++)
   for(int HeightIn=0; HeightIn<6; HeightIn++)
   {
     const int WithOut = HeightIn;
     const int HeightOut = WithIn;
     
     int InSize = 4 * WithIn * HeightIn;
     int OutSize = 4 * WithOut * HeightOut;
     if(InSize==0) OutSize=0;
     PicRotateRGB32Impl picRotateARGB32;
     picRotateARGB32.SetInDimensions(WithIn,HeightIn);
     OrigRotateRGB32Impl origRotateARGB32;
     origRotateARGB32.SetInDimensions(WithIn,HeightIn);

     memcpy(BlobOut,BlobIn,16);
     memcpy(BlobTest,BlobIn,16);
     memset(BlobOut+16,0xFE,OutSize);
     memset(BlobTest+16,0xFC,OutSize);
     memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
     memcpy(BlobTest+16+OutSize,BlobIn,16);	// reference tail

     picRotateARGB32.SetRotateMode(ROTATE_NONE);
     origRotateARGB32.SetRotateMode(ROTATE_NONE);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     int pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_NONE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateARGB32.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     origRotateARGB32.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_90_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateARGB32.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     origRotateARGB32.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_90_DEGS_CCK_VFLIP; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }


     picRotateARGB32.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     origRotateARGB32.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_180_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateARGB32.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     origRotateARGB32.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_270_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateARGB32.SetRotateMode(ROTATE_FLIP_VERTICAL);
     origRotateARGB32.SetRotateMode(ROTATE_FLIP_VERTICAL);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_FLIP_VERTICAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateARGB32.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     origRotateARGB32.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_FLIP_HORIZONTAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateARGB32.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     origRotateARGB32.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     picRotateARGB32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_FLIP_DIAGONALLY; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }
   }


//-----------------------------Rotate-RGB24---------------------------------------------

  for(int WithIn=0; WithIn<6; WithIn++)
   for(int HeightIn=0; HeightIn<6; HeightIn++)
   {
     const int WithOut = HeightIn;
     const int HeightOut = WithIn;
     
     int InSize = 3 * WithIn * HeightIn;
     int OutSize = 3 * WithOut * HeightOut;
     if(InSize==0) OutSize=0;
     PicRotateRGB24Impl picRotateRGB24;
     picRotateRGB24.SetInDimensions(WithIn,HeightIn);
     OrigRotateRGB24Impl origRotateRGB24;
     origRotateRGB24.SetInDimensions(WithIn,HeightIn);

     memcpy(BlobOut,BlobIn,16);
     memcpy(BlobTest,BlobIn,16);
     memset(BlobOut+16,0xFE,OutSize);
     memset(BlobTest+16,0xFC,OutSize);
     memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
     memcpy(BlobTest+16+OutSize,BlobIn,16);	// reference tail

     picRotateRGB24.SetRotateMode(ROTATE_NONE);
     origRotateRGB24.SetRotateMode(ROTATE_NONE);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     int pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_NONE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateRGB24.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     origRotateRGB24.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_90_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateRGB24.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     origRotateRGB24.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_90_DEGS_CCK_VFLIP; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateRGB24.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     origRotateRGB24.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_180_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateRGB24.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     origRotateRGB24.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_270_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateRGB24.SetRotateMode(ROTATE_FLIP_VERTICAL);
     origRotateRGB24.SetRotateMode(ROTATE_FLIP_VERTICAL);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_FLIP_VERTICAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateRGB24.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     origRotateRGB24.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_FLIP_HORIZONTAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateRGB24.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     origRotateRGB24.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     picRotateRGB24.Rotate(BlobIn+16, BlobOut+16);
     origRotateRGB24.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nRGB24 Rotate error %ux%u: Op:ROTATE_FLIP_DIAGONALLY; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }
   }


//-----------------------------------AYUV to RGB-------------------------------------------
 for(int Width=35; Width<39; Width++)
   for(int Height=35; Height<39; Height++)
   {
     AYUV444toRGB24Converter picConvertedAYUV2RGB24(Width,Height);
     OrigRealYUV444toRGB24Converter OrigPicConvertedAYUVtoRGB24(Width,Height);

     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);
     OrigPicConvertedAYUVtoRGB24.Convert(BlobIn+16, BlobTest+16);
     int pos = memcmp(BlobTest, BlobOut, Width*Height*3+32);
     if(pos)
     {       
       int diffs1 = 0;
       for(int x=0; x<Width; x++)
         for(int y=0; y<Height; y++)
         {
           pos = 16 + 3*(x + Width*y);
           if(BlobOut[pos]!=BlobTest[pos] || BlobOut[pos+1]!=BlobTest[pos+1] || BlobOut[pos+2]!=BlobTest[pos+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1 || labs(BlobOut[pos+1]-BlobTest[pos+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos+2])>1)
           {
             printf("\nAYUV444toRGB x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", x,y, BlobOut[pos],BlobTest[pos], BlobOut[pos+1],BlobTest[pos+1], BlobOut[pos+2],BlobTest[pos+2]);
             goto ReturnErr;
           }
         }
#ifdef _DEBUG
       printf("\nAYUV to RGB24 %ux%u: 1_diffs=%d.", Width, Height, pos);
#endif
       //goto ReturnErr;
     }

     picConvertedAYUV2RGB24.SetFlip(true);
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);

     {       
       int diffs1 = 0;
       for(int y=0; y<Height; y++)
         for(int x=0; x<Width; x++)
         {
           pos = 16 + 3*(x + Width*y);
	   int posR = 16 + 3*(x + Width*(Height-y-1));
           if(BlobOut[pos]!=BlobTest[posR] || BlobOut[pos+1]!=BlobTest[posR+1] || BlobOut[pos+2]!=BlobTest[posR+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[posR])>1 || labs(BlobOut[pos+1]-BlobTest[posR+1])>1 || labs(BlobOut[pos+2]-BlobTest[posR+2])>1)
           {
             printf("\nAYUV444toRGB flip %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[posR], BlobOut[pos+1],BlobTest[posR+1], BlobOut[pos+2],BlobTest[posR+2]);
             goto ReturnErr;
           }
         }     
     }

     AYUV444toARGB32Converter picConvertedAYUV2ARGB32(Width,Height);

     picConvertedAYUV2ARGB32.Convert(BlobIn+16, BlobOut+16);
     {       
       int diffs1 = 0;
       for(int y=0; y<Height; y++)
         for(int x=0; x<Width; x++)
         {
           pos = 16 + 4*(x + Width*y);			// BGRA
	   const int posR = 16 + 3*(x + Width*y);	// BGR
           if(BlobOut[pos]!=BlobTest[posR] || BlobOut[pos+1]!=BlobTest[posR+1] || BlobOut[pos+2]!=BlobTest[posR+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[posR])>1 || labs(BlobOut[pos+1]-BlobTest[posR+1])>1 || labs(BlobOut[pos+2]-BlobTest[posR+2])>1)
           {
             printf("\nAYUV444toRGB32 %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Width,Height, x,y, BlobOut[pos+1],BlobTest[posR], BlobOut[pos+2],BlobTest[posR+1], BlobOut[pos+3],BlobTest[posR+2]);
             goto ReturnErr;
           }
           if(BlobIn[pos] != BlobOut[pos+3])
           {
             printf("\nAYUV444toRGB32 %ux%u: x=%d y=%d ... A=%d|%d.", Width,Height, x,y, BlobIn[pos], BlobOut[pos+3]);
             goto ReturnErr;
           }
         }     
     }

     picConvertedAYUV2ARGB32.SetFlip(true);
     picConvertedAYUV2ARGB32.Convert(BlobIn+16, BlobOut+16);
     {       
       int diffs1 = 0;
       for(int y=0; y<Height; y++)
         for(int x=0; x<Width; x++)
         {
           pos = 16 + 4*(x + Width*y);				// BGRA
	   const int posR = 16 + 3*(x + Width*(Height-y-1));	// BGR
           if(BlobOut[pos]!=BlobTest[posR] || BlobOut[pos+1]!=BlobTest[posR+1] || BlobOut[pos+2]!=BlobTest[posR+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[posR])>1 || labs(BlobOut[pos+1]-BlobTest[posR+1])>1 || labs(BlobOut[pos+2]-BlobTest[posR+2])>1)           
           {
             printf("\nAYUV444toRGB32 flip %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Width,Height, x,y, BlobOut[pos+1],BlobTest[posR], BlobOut[pos+2],BlobTest[posR+1], BlobOut[pos+3],BlobTest[posR+2]);
             goto ReturnErr;
           }
           const int posRI = 16 + 4*(x + Width*(Height-y-1));
           if(BlobIn[posRI] != BlobOut[pos+3])
           {
             printf("\nAYUV444toRGB32 flip %ux%u: x=%d y=%d ... A=%d|%d.", Width,Height, x,y, BlobIn[posRI], BlobOut[pos+3]);
             goto ReturnErr;
           }
         }     
     }
   }


//-----------------------------------YUV420 to RGB-------------------------------------------
 for(int Width=2; Width<28; Width+=2)
   for(int Height=2; Height<28; Height+=2)
   {
     YUV420toRGB24Converter picConvertedAYUV2RGB24(Width,Height);
     OrigRealYUV420toRGB24Converter OrigPicConvertedAYUV2RGB24(Width,Height);
     
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);
     OrigPicConvertedAYUV2RGB24.Convert(BlobIn+16, BlobTest+16);

     int pos = memcmp(BlobTest, BlobOut, Width*Height*3+32);
     if(pos)   
     {       
       int diffs1 = 0;
       for(int x=0; x<Width; x++)
         for(int y=0; y<Height; y++)
         {
           pos = 16 + 3*(x + Width*y);
           if(BlobOut[pos]!=BlobTest[pos] || BlobOut[pos+1]!=BlobTest[pos+1] || BlobOut[pos+2]!=BlobTest[pos+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1 || labs(BlobOut[pos+1]-BlobTest[pos+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos+2])>1)
           {
             printf("\nYUV420toRGB24 %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos], BlobOut[pos+1],BlobTest[pos+1], BlobOut[pos+2],BlobTest[pos+2]);
             goto ReturnErr;
           }
         }
#ifdef _DEBUG
       printf("\nYUV420 to RGB24 %ux%u: 1_diffs=%d.", Width, Height, pos);
#endif
       //goto ReturnErr;
     }

     picConvertedAYUV2RGB24.SetFlip(true);
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);

     {       
       int diffs1 = 0;
       for(int y=0; y<Height; y++)
         for(int x=0; x<Width; x++)
         {
           pos = 16 + 3*(x + Width*y);
	   int posR = 16 + 3*(x + Width*(Height-y-1));
           if(BlobOut[pos]!=BlobTest[posR] || BlobOut[pos+1]!=BlobTest[posR+1] || BlobOut[pos+2]!=BlobTest[posR+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[posR])>1 || labs(BlobOut[pos+1]-BlobTest[posR+1])>1 || labs(BlobOut[pos+2]-BlobTest[posR+2])>1)
           {
             printf("\nYUV420toRGB flip %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[posR], BlobOut[pos+1],BlobTest[posR+1], BlobOut[pos+2],BlobTest[posR+2]);
             goto ReturnErr;
           }
         }     
     }
   }

//-----------------------------------RGB to YUV420-------------------------------------------
 for(int Width=2; Width<28; Width+=2)
   for(int Height=2; Height<28; Height+=2)
   {
     OrigRealRGB32toYUV420Converter OrigPicConverterRGB32toYUV420(Width,Height);
     RealRGB32toYUV420Converter PicConverterRGB32toYUV420(Width,Height);
     //ConvRGB24toYUV42_float_LUT LutConverterRGB32toYUV420(Width,Height);
     ConvRGB32toYUV420FixP fpConverterRGB32toYUV420(Width,Height);
     //FastRGB32toYUV420Converter PicConverterRGB32toYUV420(Width,Height); //	-- nefunguje!!!

     PicConverterRGB32toYUV420.Convert(BlobIn+16, BlobOut+16);
     OrigPicConverterRGB32toYUV420.Convert(BlobIn+16, BlobTest+16);

     int pos = memcmp(BlobTest, BlobOut, Width*Height+2*(Width)*(Height)/4);
     if(pos)   
     {       
       int diffs1 = 0;
       for(int x=0; x<Width; x++)
         for(int y=0; y<Height; y++)
         {
           pos = 16 + (x + Width*y);
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nRGB32toYUV420 %ux%u: x=%d y=%d ... Y=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }
         }
#ifdef _DEBUG
       printf("\nRGB32 to YUV420 %ux%u: 1_diffs=%d.", Width, Height, pos);
#endif
       unsigned Upos = Width * Height;
       unsigned Vpos = Upos*5 / 4;
       for(int x=0; x<Width/2; x++)
         for(int y=0; y<Height/2; y++)
         {
           pos = 16 + Upos + (x + Width*y/2);
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nRGB32toYUV420 %ux%u: x/2=%d y/2=%d ... U=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }

           pos = 16 + Vpos + x + Width*y/2;
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nRGB32toYUV420 %ux%u: x/2=%d y/2=%d ... V=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }
         }
       //goto ReturnErr;
     }

     fpConverterRGB32toYUV420.Convert(BlobIn+16, BlobOut+16);
     pos = memcmp(BlobTest, BlobOut, Width*Height+2*(Width)*(Height)/4);
     if(pos)   
     {       
       int diffs1 = 0;
       for(int x=0; x<Width; x++)
         for(int y=0; y<Height; y++)
         {
           pos = 16 + (x + Width*y);
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nFixP RGB32toYUV420 %ux%u: x=%d y=%d ... Y=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }
         }
#ifdef _DEBUG
       printf("\nLUT RGB32 to YUV420 %ux%u: 1_diffs=%d.", Width, Height, pos);
#endif
       unsigned Upos = Width * Height;
       unsigned Vpos = Upos*5 / 4;
       for(int x=0; x<Width/2; x++)
         for(int y=0; y<Height/2; y++)
         {
           pos = 16 + Upos + (x + Width*y/2);
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nFixP RGB32toYUV420 %ux%u: x/2=%d y/2=%d ... U=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }

           pos = 16 + Vpos + x + Width*y/2;
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nFixP RGB32toYUV420 %ux%u: x/2=%d y/2=%d ... V=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }
         }
       //goto ReturnErr;
     }

/*
     picConvertedAYUV2RGB24.SetFlip(true);
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);

     {       
       int diffs1 = 0;
       for(int y=0; y<Height; y++)
         for(int x=0; x<Width; x++)
         {
           pos = 16 + 3*(x + Width*y);
	   int posR = 16 + 3*(x + Width*(Height-y-1));
           if(BlobOut[pos]!=BlobTest[posR] || BlobOut[pos+1]!=BlobTest[posR+1] || BlobOut[pos+2]!=BlobTest[posR+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[posR])>1 || labs(BlobOut[pos+1]-BlobTest[posR+1])>1 || labs(BlobOut[pos+2]-BlobTest[posR+2])>1)
           {
             printf("\nYUV420toRGB flip %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[posR], BlobOut[pos+1],BlobTest[posR+1], BlobOut[pos+2],BlobTest[posR+2]);
             goto ReturnErr;
           }
         }     
     }
*/
   }

////////////////////////////////////////////////////////////////////////////////

 for(int Width=2; Width<28; Width+=2)
   for(int Height=2; Height<28; Height+=2)
   {
     OrigRealRGB24toYUV420Converter OrigPicConverterRGB24toYUV420(Width,Height);
     RealRGB24toYUV420Converter PicConverterRGB24toYUV420(Width,Height);
     //ConvRGB24toYUV42_float_LUT LutConverterRGB32toYUV420(Width,Height);
     ConvRGB24toYUV420FixP fpConverterRGB24toYUV420(Width,Height);
     //FastRGB32toYUV420Converter PicConverterRGB32toYUV420(Width,Height); //	-- nefunguje!!!

     PicConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);
     OrigPicConverterRGB24toYUV420.Convert(BlobIn+16, BlobTest+16);

     int pos = memcmp(BlobTest, BlobOut, 16+Width*Height+2*Width*Height/4);
     if(pos)   
     {       
       int diffs1 = 0;
       for(int x=0; x<Width; x++)
         for(int y=0; y<Height; y++)
         {
           pos = 16 + (x + Width*y);
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nRGB24toYUV420 %ux%u: x=%d y=%d ... Y=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }
         }
#ifdef _DEBUG
       printf("\nRGB32 to YUV420 %ux%u: 1_diffs=%d.", Width, Height, pos);
#endif
       unsigned Upos = Width * Height;
       unsigned Vpos = Upos*5 / 4;
       for(int x=0; x<Width/2; x++)
         for(int y=0; y<Height/2; y++)
         {
           pos = 16 + Upos + (x + y*(Width/2));
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nRGB24toYUV420 %ux%u: x/2=%d y/2=%d ... U=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }

           pos = 16 + Vpos + x + Width*y/2;
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nRGB24toYUV420 %ux%u: x/2=%d y/2=%d ... V=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }
         }
       //goto ReturnErr;
     }

     fpConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);
     pos = memcmp(BlobTest, BlobOut, 16+Width*Height+2*(Width*Height/4));
     if(pos)   
     {       
       int diffs1 = 0;
       for(int x=0; x<Width; x++)
         for(int y=0; y<Height; y++)
         {
           pos = 16 + (x + Width*y);
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nFixP RGB24toYUV420 %ux%u: x=%d y=%d ... Y=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             goto ReturnErr;
           }
         }
#ifdef _DEBUG
       printf("\nLUT RGB32 to YUV420 %ux%u: 1_diffs=%d.", Width, Height, pos);
#endif
       unsigned Upos = Width * Height;
       unsigned Vpos = Upos*5 / 4;
       for(int x=0; x<Width/2; x++)
         for(int y=0; y<Height/2; y++)
         {
           pos = 16 + Upos + (x + Width*y/2);
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nFixP RGB24toYUV420 %ux%u: x/2=%d y/2=%d ... U=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             //goto ReturnErr;
           }

           pos = 16 + Vpos + x + Width*y/2;
           if(BlobOut[pos]!=BlobTest[pos])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[pos])>1)
           {
             printf("\nFixP RGB24toYUV420 %ux%u: x/2=%d y/2=%d ... V=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
             //goto ReturnErr;
           }
         }
       //goto ReturnErr;
     }
/*
     picConvertedAYUV2RGB24.SetFlip(true);
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);

     {       
       int diffs1 = 0;
       for(int y=0; y<Height; y++)
         for(int x=0; x<Width; x++)
         {
           pos = 16 + 3*(x + Width*y);
	   int posR = 16 + 3*(x + Width*(Height-y-1));
           if(BlobOut[pos]!=BlobTest[posR] || BlobOut[pos+1]!=BlobTest[posR+1] || BlobOut[pos+2]!=BlobTest[posR+2])
		diffs1++;
	   if(labs(BlobOut[pos]-BlobTest[posR])>1 || labs(BlobOut[pos+1]-BlobTest[posR+1])>1 || labs(BlobOut[pos+2]-BlobTest[posR+2])>1)
           {
             printf("\nYUV420toRGB flip %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Width,Height, x,y, BlobOut[pos],BlobTest[posR], BlobOut[pos+1],BlobTest[posR+1], BlobOut[pos+2],BlobTest[posR+2]);
             goto ReturnErr;
           }
         }     
     }
*/
   }


  printf("\nAll tests passed OK.");
  free(BlobIn);
  free(BlobOut);
  free(BlobTest);

  BlobIn = (unsigned char *)malloc(1920*1080*4);
  BlobOut = (unsigned char *)malloc(1920*1080*4);
  BlobTest = NULL;

  for(int i=0; i<1920*1080*4; i++)
  {
    BlobIn[i] = rand() % 255;
  }

  printf("\n\nBenchmarks:");
  unsigned long long TimeStampB, TimeStampE, Duration, DurationMin;
  {
    {
      RealRGB32toYUV420Converter picConverterRGB32toYUV420(1920,1080);
      DurationMin = BenchConverter(picConverterRGB32toYUV420, BlobIn, BlobOut);
      printf("\nConvert RGB32 to YUV res 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      //picConvertedAYUV2RGB24.SetFlip(true);
      //DurationMin = BenchConverter(picConvertedAYUV2RGB24, BlobIn, BlobOut);
    }
/*    {
      FastRGB32toYUV420Converter fpConverterRGB32toYUV420(1920,1080);
      DurationMin = BenchConverter(fpConverterRGB32toYUV420, BlobIn, BlobOut);
      printf(" fast %.3f[ms]", DurationMin/1000.0);
    } */
    {
      ConvRGB24toYUV42_float_LUT LutConverterRGB24toYUV420(1920,1080);
      DurationMin = BenchConverter(LutConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" LUT %.3f[ms]", DurationMin/1000.0);
    }
    {
      ConvRGB32toYUV420FixP fpConverterRGB32toYUV420(1920,1080);
      DurationMin = BenchConverter(fpConverterRGB32toYUV420, BlobIn, BlobOut);
      printf(" fix point %.3f[ms]", DurationMin/1000.0);
    }
    {
      OrigRealRGB32toYUV420Converter origConverterRGB32toYUV420(1920,1080);
      DurationMin = BenchConverter(origConverterRGB32toYUV420, BlobIn, BlobOut);
      printf(" orig %.3f[ms]", DurationMin/1000.0);
    }

    {
      RealRGB24toYUV420Converter picConverterRGB24toYUV420(1920,1080);
      DurationMin = BenchConverter(picConverterRGB24toYUV420, BlobIn, BlobOut);
      printf("\nConvert RGB24 to YUV res 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      //picConvertedAYUV2RGB24.SetFlip(true);
      //DurationMin = BenchConverter(picConvertedAYUV2RGB24, BlobIn, BlobOut);
    }
    {
      ConvRGB24toYUV420FixP fpConverterRGB24toYUV420(1920,1080);
      DurationMin = BenchConverter(fpConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" fix point %.3f[ms]", DurationMin/1000.0);
    }
    {
      OrigRealRGB24toYUV420Converter origConverterRGB24toYUV420(1920,1080);
      DurationMin = BenchConverter(origConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" orig %.3f[ms]", DurationMin/1000.0);
    }
#ifdef USE_SSE
    if(FeaturesCPU & 2)
    {
      ConvRGB24toYUV420_SIMD simdConverterRGB24toYUV420(1920,1080);
      DurationMin = BenchConverter(simdConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" SIMD %.3f[ms]", DurationMin/1000.0);
    }
#endif

    {
      AYUV444toRGB24Converter picConverterAYUV2RGB24(1920,1080);
      DurationMin = BenchConverter(picConverterAYUV2RGB24, BlobIn, BlobOut);
      printf("\nConvert AYUV to RGB24 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      picConverterAYUV2RGB24.SetFlip(true);
      DurationMin = BenchConverter(picConverterAYUV2RGB24, BlobIn, BlobOut);
    }
    printf(" Flip: %.3f[ms]", DurationMin/1000.0);
    {
      OrigRealYUV444toRGB24Converter picConvertedAYUV2RGB24(1920,1080);
      DurationMin = BenchConverter(picConvertedAYUV2RGB24, BlobIn, BlobOut);
    }
    printf("\nConvert Orig AYUV to RGB24 resolution 1920,1080 time %.3f[ms].", DurationMin/1000.0);

   {
      AYUV444toARGB32Converter picConvertedAYUV2ARGB32(1920,1080);
      DurationMin = BenchConverter(picConvertedAYUV2ARGB32, BlobIn, BlobOut);
      printf("\nConvert AYUV to ARGB32 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      picConvertedAYUV2ARGB32.SetFlip(true);
      DurationMin = BenchConverter(picConvertedAYUV2ARGB32, BlobIn, BlobOut);
    }
    printf(" Flip: %.3f[ms]", DurationMin/1000.0);

    {
      YUV420toRGB24Converter picConvertedYUV420toRGB24(1920,1080);
      DurationMin = BenchConverter(picConvertedYUV420toRGB24, BlobIn, BlobOut);
      printf("\nConvert YUV420 to RGB24 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      picConvertedYUV420toRGB24.SetFlip(true);
      DurationMin = BenchConverter(picConvertedYUV420toRGB24, BlobIn, BlobOut);
    }
    printf(" Flip: %.3f[ms]", DurationMin/1000.0);
    
    {
      OrigRealYUV420toRGB24Converter picConvertedYUV420toRGB24(1920,1080);
      DurationMin = BenchConverter(picConvertedYUV420toRGB24, BlobIn, BlobOut);
    }
    printf("\nConvert Orig YUV420 to RGB24 resolution 1920,1080 time %.3f[ms].", DurationMin/1000.0);

	/////////////////////////////////////////////////////////

    {
      PicScalerRGB24Impl picScalerRGB24(1920,1080,800,600);
      DurationMin = BenchScaller(picScalerRGB24, BlobIn, BlobOut);
    }
    printf("\nScale RGB24 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    {
      PicScalerRGB32Impl picScalerRGB32(1920,1080,800,600);
      DurationMin = BenchScaller(picScalerRGB32, BlobIn, BlobOut);
    }
    printf("\nScale RGB32 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    {
      PicScalerARGB32Impl picScalerARGB32(1920,1080,800,600);     
      DurationMin = BenchScaller(picScalerARGB32, BlobIn, BlobOut);
    }
    printf("\nScale ARGB32 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

#ifdef USE_MMX
    if(FeaturesCPU & 1)
    {
      PicScalerARGB32MMX picScalerARGB32mmx(1920,1080,800,600);     
      DurationMin = BenchScaller(picScalerARGB32mmx, BlobIn, BlobOut);      
      printf("\nScale ARGB32_MMX from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);
    }
#endif

#ifdef USE_SSE
    if(FeaturesCPU & 2)
    {
      PicScalerARGB32SSE picScalerARGB32sse(1920,1080,800,600);
      DurationMin = BenchScaller(picScalerARGB32sse, BlobIn, BlobOut);
      printf("\nScale ARGB32_SSE from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);
    }
#endif

    {
      OrigScalerRGB32Impl origScalerRGB32(1920,1080,800,600);
      DurationMin = BenchScaller(origScalerRGB32, BlobIn, BlobOut);
    }
    printf("\nScale Orig RGB32 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    {
      PicScalerYUYVImpl PicScalerYUYV(1920,1080,800,600);
      DurationMin = BenchScaller(PicScalerYUYV, BlobIn, BlobOut);
    }
    printf("\nScale YUYV from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    {
      PicScalerYUV420PImpl ScalerYUV420(1920,1080,800,600);
      DurationMin = BenchScaller(ScalerYUV420, BlobIn, BlobOut);
    }
    printf("\nScale YUV420 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    {
      OrigScalerYUV420Impl origScalerYUV420(1920,1080,800,600);
      DurationMin = BenchScaller(origScalerYUV420, BlobIn, BlobOut);
    }
    printf(" Orig %.3f[ms].", DurationMin/1000.0);

	/////////////////////////////////////////////////////////

    {
      PicRotateRGB24Impl PicRotateRGB24;
      BenchRotator("Rotate RGB24", PicRotateRGB24, BlobIn, BlobOut);
    }
    {
      OrigRotateRGB24Impl OrigRotateRGB24;
      BenchRotator("OrigRot RGB24", OrigRotateRGB24, BlobIn, BlobOut);
    }

    {
      PicRotateRGB32Impl PicRotateRGB32;
      BenchRotator("Rotate RGB32", PicRotateRGB32, BlobIn, BlobOut);
    }

    {
      OrigRotateRGB32Impl OrigRotateRGB32;
      BenchRotator("OrigRot RGB32", OrigRotateRGB32, BlobIn, BlobOut);
    }
  }

  free(BlobIn);
  free(BlobOut);
  free(BlobTest);
  if(Pause)
  {
    printf("\nPress any key to exit.");
    getchar();
  }
  return 0;

ReturnErr:
  free(BlobIn);
  free(BlobOut);
  free(BlobTest);
  if(Pause)
  {
    printf("\nPress any key to exit.");
    getchar();
  }
  return -1;
}