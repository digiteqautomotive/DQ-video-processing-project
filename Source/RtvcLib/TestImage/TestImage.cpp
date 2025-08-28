/** Copyright (c) 2025 Jaroslav Fojtik
    All rights reserved.
**/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Image/PicScalerRGB24Impl.h"
#include "Image/PicScalerARGB32Impl.h"
#include "Image/PicScalerRGB32Impl.h"
#include "Image/PicScalerARGB32MMX.h"
#include "Image/PicScalerARGB32SSE.h"
#include "Image/PicScalerYUYVImpl.h"
#include "ScaleTesters.h"

#include "Image/PicRotateRGB32Impl.h"
#include "Image/PicRotateRGB24Impl.h"


extern "C"
{
 unsigned long long GetTickCount_us(void);
 unsigned GetFeaturesCPU(void);
 void EmitEMMS(void);
}

unsigned char FeaturesCPU = 0x80;


///////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
PicScalerRGB32Impl picScalerRGB32;
unsigned char *BlobIn, *BlobOut, *BlobTest;
int i;

  printf("<TEST> Testing scale computing %u[bits]", 8*sizeof(void*));

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

//--------------------------------------------------------------------------------------

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

         int pos = memcmp(BlobTest, BlobOut, OutSize+32);
         if(pos)
         {
           printf("\nYUYV Compare error %ux%u -> %ux%u; pos=%d.", WithIn, HeightIn, WithOut, HeightOut, pos);
           goto ReturnErr;
         }

       }
     }


  printf("\nAll tests passed OK.");
  free(BlobIn);
  free(BlobOut);
  free(BlobTest);

  BlobIn = (unsigned char *)malloc(1920*1080*4);
  BlobOut = (unsigned char *)malloc(1920*1080*4);
  BlobTest = NULL;

  printf("\n\nBenchmarks:");
  unsigned long long TimeStampB, TimeStampE, Duration, DurationMin;
  {
    PicScalerRGB24Impl picScalerRGB24(1920,1080,800,600);
    DurationMin = ~0;
    for(i=0; i<10; i++)
    {
      TimeStampB = GetTickCount_us();
      picScalerRGB24.Scale(BlobOut,BlobIn);
      TimeStampE = GetTickCount_us();
      Duration = TimeStampE - TimeStampB;
      if(DurationMin > Duration) DurationMin=Duration;
    }
    printf("\nScale RGB24 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    PicScalerRGB32Impl picScalerRGB32(1920,1080,800,600);
    DurationMin = ~0;
    for(i=0; i<10; i++)
    {
      TimeStampB = GetTickCount_us();
      picScalerRGB32.Scale(BlobOut,BlobIn);
      TimeStampE = GetTickCount_us();
      Duration = TimeStampE - TimeStampB;
      if(DurationMin > Duration) DurationMin=Duration;
    }
    printf("\nScale RGB32 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    PicScalerARGB32Impl picScalerARGB32(1920,1080,800,600);
    DurationMin = ~0;
    for(i=0; i<10; i++)
    {
      TimeStampB = GetTickCount_us();
      picScalerARGB32.Scale(BlobOut,BlobIn);
      TimeStampE = GetTickCount_us();
      Duration = TimeStampE - TimeStampB;
      if(DurationMin > Duration) DurationMin=Duration;
    }
    printf("\nScale ARGB32 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

#ifdef USE_MMX
    if(FeaturesCPU & 1)
    {
      PicScalerARGB32MMX picScalerARGB32mmx(1920,1080,800,600);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        picScalerARGB32mmx.Scale(BlobOut,BlobIn);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      EmitEMMS();
      printf("\nScale ARGB32_MMX from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);
    }
#endif

#ifdef USE_SSE
    if(FeaturesCPU & 2)
    {
      PicScalerARGB32SSE picScalerARGB32sse(1920,1080,800,600);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        picScalerARGB32sse.Scale(BlobOut,BlobIn);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      EmitEMMS();
      printf("\nScale ARGB32_SSE from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);
    }
#endif

    OrigScalerRGB32Impl origScalerRGB32(1920,1080,800,600);
    DurationMin = ~0;
    for(i=0; i<10; i++)
    {
      TimeStampB = GetTickCount_us();
      origScalerRGB32.Scale(BlobOut,BlobIn);
      TimeStampE = GetTickCount_us();
      Duration = TimeStampE - TimeStampB;
      if(DurationMin > Duration) DurationMin=Duration;
    }
    printf("\nScale Orig RGB32 from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);

    {
      PicScalerYUYVImpl PicScalerYUYV(1920,1080,800,600);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicScalerYUYV.Scale(BlobOut,BlobIn);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf("\nScale YUYV from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);
    }

    {
      PicRotateRGB32Impl PicRotateRGB32;
      PicRotateRGB32.SetInDimensions(1920,1080);
      PicRotateRGB32.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);		
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB32.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf("\nRotate RGB32 1920,1080 time +90deg %.3f", DurationMin/1000.0);
      PicRotateRGB32.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB32.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" +180deg %.3f", DurationMin/1000.0);
      PicRotateRGB32.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB32.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" +270deg %.3f", DurationMin/1000.0);
      PicRotateRGB32.SetRotateMode(ROTATE_FLIP_VERTICAL);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB32.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" FLIP_V %.3f", DurationMin/1000.0);
      PicRotateRGB32.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB32.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" FLIP_H %.3f", DurationMin/1000.0);
      PicRotateRGB32.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB32.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" FLIP_D %.3f[ms]", DurationMin/1000.0);
    }

    {
      PicRotateRGB24Impl PicRotateRGB24;
      PicRotateRGB24.SetInDimensions(1920,1080);
      PicRotateRGB24.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);		
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB24.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf("\nRotate RGB24 1920,1080 time +90deg %.3f", DurationMin/1000.0);
      PicRotateRGB24.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB24.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" +180deg %.3f", DurationMin/1000.0);
      PicRotateRGB24.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB24.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" +270deg %.3f", DurationMin/1000.0);
      PicRotateRGB24.SetRotateMode(ROTATE_FLIP_VERTICAL);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB24.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" FLIP_V %.3f", DurationMin/1000.0);
      PicRotateRGB24.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB24.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" FLIP_H %.3f", DurationMin/1000.0);
      PicRotateRGB24.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
      DurationMin = ~0;
      for(i=0; i<10; i++)
      {
        TimeStampB = GetTickCount_us();
        PicRotateRGB24.Rotate(BlobIn,BlobOut);
        TimeStampE = GetTickCount_us();
        Duration = TimeStampE - TimeStampB;
        if(DurationMin > Duration) DurationMin=Duration;
      }
      printf(" FLIP_D %.3f[ms]", DurationMin/1000.0);
    }
  }

  free(BlobIn);
  free(BlobOut);
  free(BlobTest);
  return 0;

ReturnErr:
  free(BlobIn);
  free(BlobOut);
  free(BlobTest);
  return -1;
}