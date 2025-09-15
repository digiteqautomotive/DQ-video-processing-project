/** Copyright (c) 2025 Jaroslav Fojtik
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
#include "ScaleTesters.h"

#include "PicRotateRGB32Impl.h"
#include "PicRotateRGB24Impl.h"
#include "RotateTesters.h"


extern "C"
{
 unsigned long long GetTickCount_us(void);
 unsigned GetFeaturesCPU(void);
 void EmitEMMS(void);
}

unsigned char FeaturesCPU = 0x80;


///////////////////////////////////////////////////////////////////////////////////////////////////

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


int main(void)
{
PicScalerRGB32Impl picScalerRGB32;
unsigned char *BlobIn, *BlobOut, *BlobTest;
int i;

  printf("<TEST> Testing scale computing %u[bits] %s %s", 8*sizeof(void*), __DATE__, __TIME__);

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
      EmitEMMS();
      printf("\nScale ARGB32_MMX from 1920,1080->800,600 time %.3f[ms].", DurationMin/1000.0);
    }
#endif

#ifdef USE_SSE
    if(FeaturesCPU & 2)
    {
      PicScalerARGB32SSE picScalerARGB32sse(1920,1080,800,600);
      DurationMin = BenchScaller(picScalerARGB32sse, BlobIn, BlobOut);
      EmitEMMS();
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
  return 0;

ReturnErr:
  free(BlobIn);
  free(BlobOut);
  free(BlobTest);
  return -1;
}