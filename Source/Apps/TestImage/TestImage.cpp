/** @file

MODULE:			TestImage

FILE NAME:		TestImage.cpp

DESCRIPTION:

LICENSE:		Software License Agreement (BSD License)

Copyright (c) 2025-2026, Digiteq Automotive, programmed by Jaroslav Fojtik	All rights reserved.
*/
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

#include "PicRotateBPP16Impl.h"
#include "PicRotateBPP24Impl.h"
#include "PicRotateBPP32Impl.h"
#include "PicRotateBPP48Impl.h"
#include "PicRotateBPP64Impl.h"
#include "RotateTesters.h"

#include "PicCompRGB32Impl.h"
#include "PicCompRGB24Impl.h"
#include "PicCompYUYV_Impl.h"
#include "PicCompRGB32_SSE2.h"

#include "ConvAYUV444toRGB.h"
#include "ConvYUV420toRGB.h"
#include "ConvRGB24toYUV420.h"
#include "ConvRGB32toYUV420.h"
#include "ConvRGBtoYUYV.h"
#include "ConvYUV420_16toRGB.h"
#ifdef USE_ASM
#include "ConvYUV420toRGB_ASM.h"
#endif
#include "reference/ConvRGB24toYUV420_LUT.h"
#include "reference/ConvRGB24toYUV420_fixpLUT.h"
#include "reference/ConvRGB24toYUV420_SIMD.h"
#include "reference/ConvRGB24toYUV420_Real.h "
#include "reference/ConvRGB32toYUV420_Real.h "
#include "reference/ConvYUV420toRGB24CCIR601Real.h"
#include "reference/ConvYUV420toRGB24CCIR601RealVer16.h"
#include "reference/ConvYUV420toRGB24RealImpl2Ver16.h"
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


unsigned long BenchCompare(PicCompareBase &compare, const void *BlobIn, const void *BlobIn2, void *BlobOut, bool VFlip=false)
{
 unsigned long long TimeStampB, TimeStampE;
 unsigned long DurationMin = ~0;
 unsigned long Duration = ~0;
 for(int i=0; i<10; i++)
 {
   TimeStampB = GetTickCount_us();
   compare.Compare(BlobIn,BlobIn2,BlobOut,VFlip);
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


int I420compare(const char *Alg, int Width, int Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{
  int pos = memcmp(BlobTest, BlobOut, 16+Width*Height+2*Width*Height/4);
  if(pos)   
  {       
    int diffs1 = 0;
    for(int x=0; x<Width; x++)
      for(int y=0; y<Height; y++)
      {
        unsigned pos = 16 + (x + Width*y);
        if(BlobOut[pos]!=BlobTest[pos])
	    diffs1++;
	if(labs(BlobOut[pos]-BlobTest[pos])>1)
        {
           printf("\n%s %ux%u: x=%d y=%d ... Y=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
           return 10000;
        }
      }
#ifdef _DEBUG
       printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, pos);
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
           printf("\n%s %ux%u: x/2=%d y/2=%d ... U=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
           return 10001;
         }

      pos = 16 + Vpos + x + Width*y/2;
      if(BlobOut[pos]!=BlobTest[pos])
          diffs1++;
      if(labs(BlobOut[pos]-BlobTest[pos])>1)
      {
        printf("\n%s %ux%u: x/2=%d y/2=%d ... V=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos]);
        return 10002;
      }
    }
       //goto ReturnErr;
     
    return diffs1;
  }
  return 0;
}


int RGBcompareFlip(const char *Alg, int Width, int Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{  
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x++)
    {
      unsigned pos = 16 + 3*(x + Width*y);
      unsigned posR = 16 + 3*(x + Width*(Height-y-1));
      if(BlobOut[pos]!=BlobTest[posR] || BlobOut[pos+1]!=BlobTest[posR+1] || BlobOut[pos+2]!=BlobTest[posR+2])
          diffs1++;
      if(labs(BlobOut[pos]-BlobTest[posR])>1 || labs(BlobOut[pos+1]-BlobTest[posR+1])>1 || labs(BlobOut[pos+2]-BlobTest[posR+2])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[posR], BlobOut[pos+1],BlobTest[posR+1], BlobOut[pos+2],BlobTest[posR+2]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int YUYVcompare(const char *Alg, int Width, int Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{
  int pos = memcmp(BlobTest, BlobOut, Width*Height*2+32);
  if(pos==0) return 0;
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x+=2)
    {
      pos = 16 + 2*(x + Width*y);
      if(BlobOut[pos]!=BlobTest[pos] || BlobOut[pos+1]!=BlobTest[pos+1] || BlobOut[pos+2]!=BlobTest[pos+2] || BlobOut[pos+3]!=BlobTest[pos+3])
          diffs1++;
      if(labs(BlobOut[pos]-BlobTest[pos])>1 || labs(BlobOut[pos+1]-BlobTest[pos+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos+2])>1 || labs(BlobOut[pos+3]-BlobTest[pos+3])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... Y=%d|%d U=%d|%d Y=%d|%d V=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos], BlobOut[pos+1],BlobTest[pos+1], BlobOut[pos+2],BlobTest[pos+2], BlobOut[pos+3],BlobTest[pos+3]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int YUYVcompareFlip(const char *Alg, int Width, int Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x+=2)
    {
      const int pos = 16 + 2*(x + Width*y);
      const int pos2 = 16 + 2*(x + Width*(Height-y-1));
      if(BlobOut[pos]!=BlobTest[pos2] || BlobOut[pos+1]!=BlobTest[pos2+1] || BlobOut[pos+2]!=BlobTest[pos2+2] || BlobOut[pos+3]!=BlobTest[pos2+3])
          diffs1++;
      if(labs(BlobOut[pos]-BlobTest[pos2])>1 || labs(BlobOut[pos+1]-BlobTest[pos2+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos2+2])>1 || labs(BlobOut[pos+3]-BlobTest[pos2+3])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... Y=%d|%d U=%d|%d Y2=%d|%d V=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos2], BlobOut[pos+1],BlobTest[pos2+1], BlobOut[pos+2],BlobTest[pos2+2], BlobOut[pos+3],BlobTest[pos2+3]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int ARGB32compare(const char *Alg, unsigned Width, unsigned Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{
  int pos = memcmp(BlobTest, BlobOut, Width*Height*4+32);
  if(pos==0) return 0;
  int diffs1 = 0;
  for(unsigned y=0; y<Height; y++)
    for(unsigned x=0; x<Width; x++)
    {
      pos = 16 + 4*(x + Width*y);
      if(BlobOut[pos]!=BlobTest[pos] || BlobOut[pos+1]!=BlobTest[pos+1] || BlobOut[pos+2]!=BlobTest[pos+2])
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d A=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos], BlobOut[pos+1],BlobTest[pos+1], BlobOut[pos+2],BlobTest[pos+2], BlobOut[pos+3],BlobTest[pos+3]);
        diffs1++;
      }
      if(labs(BlobOut[pos]-BlobTest[pos])>1 || labs(BlobOut[pos+1]-BlobTest[pos+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos+2])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d A=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos], BlobOut[pos+1],BlobTest[pos+1], BlobOut[pos+2],BlobTest[pos+2], BlobOut[pos+3],BlobTest[pos+3]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int ARGB32compareFlip(const char *Alg, int Width, int Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x++)
    {
      const int pos = 16 + 4*(x + Width*y);
      const int pos2 = 16 + 4*(x + Width*(Height-y-1));
      if(BlobOut[pos]!=BlobTest[pos2] || BlobOut[pos+1]!=BlobTest[pos2+1] || BlobOut[pos+2]!=BlobTest[pos2+2] || BlobOut[pos+3]!=BlobTest[pos2+3])
          diffs1++;
      if(labs(BlobOut[pos]-BlobTest[pos2])>1 || labs(BlobOut[pos+1]-BlobTest[pos2+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos2+2])>1 || labs(BlobOut[pos+3]-BlobTest[pos2+3])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d A=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos2], BlobOut[pos+1],BlobTest[pos2+1], BlobOut[pos+2],BlobTest[pos2+2], BlobOut[pos+3],BlobTest[pos2+3]);
        return 10000;
      }
    }
//#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
//#endif
  return diffs1;
}


int RGB24compare(const char *Alg, int Width, int Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{
  int pos = memcmp(BlobTest, BlobOut, Width*Height*3+32);
  if(pos==0) return 0;
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x++)
    {
      pos = 16 + 3*(x + Width*y);
      if(BlobOut[pos]!=BlobTest[pos] || BlobOut[pos+1]!=BlobTest[pos+1] || BlobOut[pos+2]!=BlobTest[pos+2])
          diffs1++;
      if(labs(BlobOut[pos]-BlobTest[pos])>1 || labs(BlobOut[pos+1]-BlobTest[pos+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos+2])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos], BlobOut[pos+1],BlobTest[pos+1], BlobOut[pos+2],BlobTest[pos+2]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int RGB24compareVFlip(const char *Alg, int Width, int Height, const unsigned char *BlobOut, const unsigned char *BlobTest)
{
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x++)
    {
      const int pos = 16 + 3*(x + Width*y);
      const int pos2 = 16 + 3*(x + Width*(Height-y-1));
      if(BlobOut[pos]!=BlobTest[pos2] || BlobOut[pos+1]!=BlobTest[pos2+1] || BlobOut[pos+2]!=BlobTest[pos2+2])
          diffs1++;
      if(labs(BlobOut[pos]-BlobTest[pos2])>1 || labs(BlobOut[pos+1]-BlobTest[pos2+1])>1 || labs(BlobOut[pos+2]-BlobTest[pos2+2])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Alg, Width,Height, x,y, BlobOut[pos],BlobTest[pos2], BlobOut[pos+1],BlobTest[pos2+1], BlobOut[pos+2],BlobTest[pos2+2]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int RGB24toRGB32compare(const char *Alg, int Width, int Height, const unsigned char *BlobRef24, const unsigned char *BlobTest)
{
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x++)
    {
      int pos = 16 + 3*(x + Width*y);
      int pos2 = 16 + 4*(x + Width*y);
      if(BlobRef24[pos]!=BlobTest[pos2] || BlobRef24[pos+1]!=BlobTest[pos2+1] || BlobRef24[pos+2]!=BlobTest[pos2+2])
          diffs1++;
      if(labs(BlobRef24[pos]-BlobTest[pos2])>1 || labs(BlobRef24[pos+1]-BlobTest[pos2+1])>1 || labs(BlobRef24[pos+2]-BlobTest[pos2+2])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Alg, Width,Height, x,y, BlobRef24[pos],BlobTest[pos2], BlobRef24[pos+1],BlobTest[pos2+1], BlobRef24[pos+2],BlobTest[pos2+2]);
        return 10000;
      }
      if(BlobTest[pos2+3]!=0)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d A=%d.", Alg, Width,Height, x,y, BlobRef24[pos],BlobTest[pos2], BlobRef24[pos+1],BlobTest[pos2+1], BlobRef24[pos+2],BlobTest[pos2+2],BlobTest[pos2+3]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int RGB24toRGB32compareFlip(const char *Alg, int Width, int Height, const unsigned char *BlobRef24, const unsigned char *BlobTest)
{
  int diffs1 = 0;
  for(int y=0; y<Height; y++)
    for(int x=0; x<Width; x++)
    {
      unsigned posR = 16 + 3*(x + Width*(Height-y-1));      
      int pos2 = 16 + 4*(x + Width*y);
      if(BlobRef24[posR]!=BlobTest[pos2] || BlobRef24[posR+1]!=BlobTest[pos2+1] || BlobRef24[posR+2]!=BlobTest[pos2+2])
          diffs1++;
      if(labs(BlobRef24[posR]-BlobTest[pos2])>1 || labs(BlobRef24[posR+1]-BlobTest[pos2+1])>1 || labs(BlobRef24[posR+2]-BlobTest[pos2+2])>1)
      {
        printf("\n%s %ux%u: x=%d y=%d ... R=%d|%d G=%d|%d B=%d|%d.", Alg, Width,Height, x,y, BlobRef24[posR],BlobTest[pos2], BlobRef24[posR+1],BlobTest[pos2+1], BlobRef24[posR+2],BlobTest[pos2+2]);
        return 10000;
      }
    }
#ifdef _DEBUG
  if(diffs1 > 0)
      printf("\n%s %ux%u: 1_diffs=%d.", Alg, Width, Height, diffs1);
#endif
  return diffs1;
}


int main(int argc, char *argv[])
{
PicScalerRGB32Impl picScalerRGB32;
unsigned char *BlobIn, *BlobIn2, *BlobOut, *BlobTest;
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
  BlobIn2 = (unsigned char *)malloc(16384);
  BlobOut = (unsigned char *)malloc(16384);
  BlobTest = (unsigned char *)malloc(16384);
  if(BlobIn==NULL || BlobIn2==NULL || BlobOut==NULL || BlobTest==NULL)
  {
    printf("\nError: Memory allocation problem.");
    goto ReturnErr;
  }

  for(int i=0; i<16384; i++)
  {
    BlobIn[i] = rand() % 255;
    BlobIn2[i] = rand() % 255;
  }

//-----------------------------Compare-RGB32---------------------------------------------

  for(unsigned Width=0; Width<33; Width++)
   for(unsigned Height=0; Height<33; Height++)
   {     
     int InSize = 4 * Width * Height;
     int OutSize = InSize;
     if(InSize==0) OutSize=0;
     PicCompareRGB32Impl picCompareRGB32;
     picCompareRGB32.SetDimensions(Width,Height);

     memcpy(BlobOut,BlobIn,16);
     memcpy(BlobTest,BlobIn,16);
     memset(BlobOut+16,0xFE,OutSize);
     memset(BlobTest+16,0xFC,OutSize);
     memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
     memcpy(BlobTest+16+OutSize,BlobIn,16);	// reference tail

     picCompareRGB32.Compare(BlobIn+16,BlobIn2+16,BlobOut+16);

     for(int x=0; x<Width; x++)
       for(int y=0; y<Height; y++)
       {
         unsigned offset = 16 + 4*((y*Width)+x);
         
         int16_t val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;
       }

     if(ARGB32compare("picCompareRGB32",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

     picCompareRGB32.Compare(BlobIn+16,BlobIn2+16,BlobOut+16,true);

     if(ARGB32compareFlip("picCompareRGB32(Vflip)",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

#ifdef USE_SSE
     if(FeaturesCPU & 4)
     {
       int diffs;
       PicCompareRGB32_SSE2 compareRGB32_SSE2(Width,Height);

       compareRGB32_SSE2.Compare(BlobIn+16,BlobIn2+16,BlobOut+16);
       if(diffs=ARGB32compare("picCompareRGB32_SSE2",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

       compareRGB32_SSE2.Compare(BlobIn+16,BlobIn2+16,BlobOut+16,true);
       if(ARGB32compareFlip("picCompareRGB32_SSE2",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;
     }
#endif

   }


//-----------------------------Compare-RGB24---------------------------------------------

  for(int Width=0; Width<33; Width++)
   for(int Height=0; Height<33; Height++)
   {     
     int InSize = 3 * Width * Height;
     int OutSize = InSize;
     if(InSize==0) OutSize=0;
     PicCompareRGB24Impl picCompareRGB24;
     picCompareRGB24.SetDimensions(Width,Height);

     memcpy(BlobOut,BlobIn,16);
     memcpy(BlobTest,BlobIn,16);
     memset(BlobOut+16,0xFE,OutSize);
     memset(BlobTest+16,0xFC,OutSize);
     memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
     memcpy(BlobTest+16+OutSize,BlobIn,16);	// reference tail

     picCompareRGB24.Compare(BlobIn+16,BlobIn2+16,BlobOut+16);

     for(int x=0; x<Width; x++)
       for(int y=0; y<Height; y++)
       {
         unsigned offset = 16 + 3*((y*Width)+x);
         
         int16_t val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;
       }

     if(RGB24compare("picCompareRGB24",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

     picCompareRGB24.Compare(BlobIn+16,BlobIn2+16,BlobOut+16,true);

     if(RGB24compareVFlip("picCompareRGB24(Vflip)",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

#ifdef USE_SSE
     if(FeaturesCPU & 4)
     {
       int diffs;
       PicCompareRGB24_SSE2 compareRGB24_SSE2(Width,Height);

       compareRGB24_SSE2.Compare(BlobIn+16,BlobIn2+16,BlobOut+16);
       if(diffs=RGB24compare("picCompareRGB24_SSE2",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

       compareRGB24_SSE2.Compare(BlobIn+16,BlobIn2+16,BlobOut+16,true);
       if(RGB24compareVFlip("picCompareRGB24_SSE2",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;
     }
#endif
   }


//-----------------------------Compare-YUYV---------------------------------------------

  for(int Width=0; Width<43; Width+=2)
   for(int Height=0; Height<33; Height++)
   {     
     int InSize = 3 * Width * Height;
     int OutSize = InSize;
     if(InSize==0) OutSize=0;
     PicCompareYUYV_Impl picCompareYUYV;
     picCompareYUYV.SetDimensions(Width,Height);

     memcpy(BlobOut,BlobIn,16);
     memcpy(BlobTest,BlobIn,16);
     memset(BlobOut+16,0xFE,OutSize);
     memset(BlobTest+16,0xFC,OutSize);
     memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
     memcpy(BlobTest+16+OutSize,BlobIn,16);	// reference tail

     picCompareYUYV.Compare(BlobIn+16,BlobIn2+16,BlobOut+16);

     for(int x=0; x<Width; x+=2)
       for(int y=0; y<Height; y++)
       {
         unsigned offset = 16 + 2*((y*Width)+x);
         
         int16_t val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;	// Y
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;		// U
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;		// Y
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;

         offset++;
         val = (int16_t)BlobIn[offset] - (int16_t)BlobIn2[offset] + 128;		// V
         if(val>=255) BlobTest[offset]=255;
         else if(val<=0) BlobTest[offset]=0;
         else BlobTest[offset]=val;
       }

     if(YUYVcompare("picCompareYUYV",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

     picCompareYUYV.Compare(BlobIn+16,BlobIn2+16,BlobOut+16,true);

     if(YUYVcompareFlip("picCompareYUYV(Vflip)",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

#ifdef USE_SSE
     if(FeaturesCPU & 4)
     {
       int diffs;
       PicCompareYUYV_SSE2 compareYUYV_SSE2(Width,Height);

       compareYUYV_SSE2.Compare(BlobIn+16,BlobIn2+16,BlobOut+16);
       if(diffs=YUYVcompare("picCompareYUYV_SSE2",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;

       compareYUYV_SSE2.Compare(BlobIn+16,BlobIn2+16,BlobOut+16,true);
       if(YUYVcompareFlip("picCompareYUYV_SSE2",Width,Height,BlobOut,BlobTest) != 0)
         goto ReturnErr;
     }
#endif
   }

//-----------------------------Scale-RGB24 & RGB32---------------------------------------
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

//----------------------------------Scale-YUYV-----------------------------------------------

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

//-----------------------------Rotate-BPP16---------------------------------------------

  for(int WithIn=0; WithIn<6; WithIn++)
   for(int HeightIn=0; HeightIn<6; HeightIn++)
   {
     const int WithOut = HeightIn;
     const int HeightOut = WithIn;
     
     int InSize = 2 * WithIn * HeightIn;
     int OutSize = 2 * WithOut * HeightOut;
     if(InSize==0) OutSize=0;
     PicRotateBPP16Impl picRotateBPP16;
     picRotateBPP16.SetInDimensions(WithIn,HeightIn);
     OrigRotateBPP16Impl origRotateBPP16;
     origRotateBPP16.SetInDimensions(WithIn,HeightIn);

     memcpy(BlobOut,BlobIn,16);
     memcpy(BlobTest,BlobIn,16);
     memset(BlobOut+16,0xFE,OutSize);
     memset(BlobTest+16,0xFC,OutSize);
     memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
     memcpy(BlobTest+16+OutSize,BlobIn,16);	// reference tail

     picRotateBPP16.SetRotateMode(ROTATE_NONE);
     origRotateBPP16.SetRotateMode(ROTATE_NONE);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     int pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_NONE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP16.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     origRotateBPP16.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_90_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP16.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     origRotateBPP16.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_90_DEGS_CCK_VFLIP; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }


     picRotateBPP16.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     origRotateBPP16.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_180_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP16.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     origRotateBPP16.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_270_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP16.SetRotateMode(ROTATE_FLIP_VERTICAL);
     origRotateBPP16.SetRotateMode(ROTATE_FLIP_VERTICAL);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_FLIP_VERTICAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP16.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     origRotateBPP16.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_FLIP_HORIZONTAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP16.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     origRotateBPP16.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     picRotateBPP16.Rotate(BlobIn+16, BlobOut+16);
     origRotateBPP16.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nBPP16 Rotate error %ux%u: Op:ROTATE_FLIP_DIAGONALLY; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }
   }

//-----------------------------Rotate-BPP32---------------------------------------------

  for(int WithIn=0; WithIn<6; WithIn++)
   for(int HeightIn=0; HeightIn<6; HeightIn++)
   {
     const int WithOut = HeightIn;
     const int HeightOut = WithIn;
     
     int InSize = 4 * WithIn * HeightIn;
     int OutSize = 4 * WithOut * HeightOut;
     if(InSize==0) OutSize=0;
     PicRotateBPP32Impl picRotateBPP32;
     picRotateBPP32.SetInDimensions(WithIn,HeightIn);
     OrigRotateRGB32Impl origRotateARGB32;
     origRotateARGB32.SetInDimensions(WithIn,HeightIn);

     memcpy(BlobOut,BlobIn,16);
     memcpy(BlobTest,BlobIn,16);
     memset(BlobOut+16,0xFE,OutSize);
     memset(BlobTest+16,0xFC,OutSize);
     memcpy(BlobOut+16+OutSize,BlobIn,16);	// tail
     memcpy(BlobTest+16+OutSize,BlobIn,16);	// reference tail

     picRotateBPP32.SetRotateMode(ROTATE_NONE);
     origRotateARGB32.SetRotateMode(ROTATE_NONE);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     int pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_NONE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP32.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     origRotateARGB32.SetRotateMode(ROTATE_90_DEGREES_CLOCKWISE);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_90_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP32.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     origRotateARGB32.SetRotateMode(ROTATE_90_DEGS_CCK_VFLIP);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_90_DEGS_CCK_VFLIP; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }


     picRotateBPP32.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     origRotateARGB32.SetRotateMode(ROTATE_180_DEGREES_CLOCKWISE);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_180_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP32.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     origRotateARGB32.SetRotateMode(ROTATE_270_DEGREES_CLOCKWISE);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_270_DEGREES_CLOCKWISE; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP32.SetRotateMode(ROTATE_FLIP_VERTICAL);
     origRotateARGB32.SetRotateMode(ROTATE_FLIP_VERTICAL);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_FLIP_VERTICAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP32.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     origRotateARGB32.SetRotateMode(ROTATE_FLIP_HORIZONTAL);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_FLIP_HORIZONTAL; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }

     picRotateBPP32.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     origRotateARGB32.SetRotateMode(ROTATE_FLIP_DIAGONALLY);
     picRotateBPP32.Rotate(BlobIn+16, BlobOut+16);
     origRotateARGB32.Rotate(BlobIn+16, BlobTest+16);

     pos = memcmp(BlobTest, BlobOut, OutSize+32);
     if(pos)
     {
       printf("\nARGB32 Rotate error %ux%u: Op:ROTATE_FLIP_DIAGONALLY; pos=%d.", WithIn, HeightIn, pos);
       goto ReturnErr;
     }
   }


//-----------------------------Rotate-BPP24---------------------------------------------

  for(int WithIn=0; WithIn<6; WithIn++)
   for(int HeightIn=0; HeightIn<6; HeightIn++)
   {
     const int WithOut = HeightIn;
     const int HeightOut = WithIn;
     
     int InSize = 3 * WithIn * HeightIn;
     int OutSize = 3 * WithOut * HeightOut;
     if(InSize==0) OutSize=0;
     PicRotateBPP24Impl picRotateRGB24;
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
     ConvAYUV444toRGB24 picConvertedAYUV2RGB24(Width,Height);
     OrigRealYUV444toRGB24Converter OrigPicConvertedAYUVtoRGB24(Width,Height);

     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);
     OrigPicConvertedAYUVtoRGB24.Convert(BlobIn+16, BlobTest+16);

     int pos = RGB24compare("AYUV444toRGB", Width,Height, BlobOut, BlobTest);
     if(pos >= 10000) goto ReturnErr;

     picConvertedAYUV2RGB24.SetFlip(true);
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);

     pos = RGBcompareFlip("AYUV444toRGB Flip", Width,Height, BlobOut, BlobTest);

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
     ConvYUV420toRGB24 picConvertedAYUV2RGB24(Width,Height);
#ifdef USE_ASM
     ConvYUV420toRGB24_ASM asmConvertedAYUV2RGB24(Width,Height);
#endif
     OrigRealYUV420toRGB24Converter OrigPicConvertedAYUV2RGB24(Width,Height);
#ifdef USE_ASM
     ConvYUV420toRGB32_ASM asmConvertedAYUV2RGB32(Width,Height);
#endif
     YUV420toRGB32Converter picConvertedYUV420toRGB32(Width,Height);
     
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);
     OrigPicConvertedAYUV2RGB24.Convert(BlobIn+16, BlobTest+16);

     int pos = RGB24compare("YUV420toRGB24", Width,Height, BlobTest, BlobOut);
     if(pos >= 10000) goto ReturnErr;

#ifdef USE_ASM
    asmConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);
    pos = RGB24compare("YUV420toRGB24_ASM", Width,Height, BlobTest, BlobOut);
    if(pos >= 10000) goto ReturnErr;
#endif

     picConvertedYUV420toRGB32.Convert(BlobIn+16, BlobOut+16);
     pos = RGB24toRGB32compare("YUV420toRGB32", Width,Height, BlobTest, BlobOut);
     if(pos >= 10000) goto ReturnErr;

#ifdef USE_ASM
    //memset(BlobOut+16,0xAA,Width*Height*4);
    asmConvertedAYUV2RGB32.Convert(BlobIn+16, BlobOut+16);
    pos = RGB24toRGB32compare("YUV420toRGB32_ASM", Width,Height, BlobTest, BlobOut);
    if(pos >= 10000) goto ReturnErr;
#endif

     picConvertedAYUV2RGB24.SetFlip(true);
     picConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);

     pos = RGBcompareFlip("YUV420toRGB24 Flip", Width,Height, BlobTest, BlobOut);
     if(pos >= 10000) goto ReturnErr;

#ifdef USE_ASM
    asmConvertedAYUV2RGB24.SetFlip(true);
    asmConvertedAYUV2RGB24.Convert(BlobIn+16, BlobOut+16);
    pos = RGBcompareFlip("YUV420toRGB24_ASM Flip", Width,Height, BlobTest, BlobOut);
    if(pos >= 10000) goto ReturnErr;

    asmConvertedAYUV2RGB32.SetFlip(true);
    asmConvertedAYUV2RGB32.Convert(BlobIn+16, BlobOut+16);
    pos = RGB24toRGB32compareFlip("YUV420toRGB32_ASM Flip", Width,Height, BlobTest, BlobOut);
    if(pos >= 10000) goto ReturnErr;
#endif

    picConvertedYUV420toRGB32.SetFlip(true);
    picConvertedYUV420toRGB32.Convert(BlobIn+16, BlobOut+16);
    pos = RGB24toRGB32compareFlip("YUV420toRGB32 Flip", Width,Height, BlobTest, BlobOut);
    if(pos >= 10000) goto ReturnErr;
  }


//-------------------------------YUV420 to RGB CCIR601---------------------------------------
 for(int Width=2; Width<28; Width+=2)
   for(int Height=2; Height<28; Height+=2)
   {
     RealYUV420toBGR24CCIR601Converter OrigPicConvertYUV2RGB24_CCIR601(Width,Height);
     ConvYUV420toBGR24_CCIR601 picConvertYUV420toRGB24_CCIR601(Width,Height);
     
     picConvertYUV420toRGB24_CCIR601.Convert(BlobIn+16, BlobOut+16);
     OrigPicConvertYUV2RGB24_CCIR601.Convert(BlobIn+16, BlobTest+16);

     int pos = RGB24compare("YUV420toRGB24_CCIR601", Width,Height, BlobTest, BlobOut);
     if(pos >= 10000)
	goto ReturnErr;

     picConvertYUV420toRGB24_CCIR601.SetFlip(true);
     picConvertYUV420toRGB24_CCIR601.Convert(BlobIn+16, BlobOut+16);

     pos = RGBcompareFlip("YUV420toRGB24_CCIR601 Flip", Width,Height, BlobTest, BlobOut);
     if(pos >= 10000)
	goto ReturnErr;
   }

//-----------------------------------RGB to YUV420-------------------------------------------
 for(int Width=2; Width<28; Width+=2)
   for(int Height=2; Height<28; Height+=2)
   {
     OrigRealRGB32toYUV420Converter OrigPicConverterRGB32toYUV420(Width,Height);
     RealRGB32toYUV420Converter PicConverterRGB32toYUV420(Width,Height);
     ConvRGB32toYUV420FixP fpConverterRGB32toYUV420(Width,Height);
     //FastRGB32toYUV420Converter PicConverterRGB32toYUV420(Width,Height); //	-- nefunguje!!!

     PicConverterRGB32toYUV420.Convert(BlobIn+16, BlobOut+16);
     OrigPicConverterRGB32toYUV420.Convert(BlobIn+16, BlobTest+16);

     int res = I420compare("RGB32toYUV420_Real", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;

     fpConverterRGB32toYUV420.Convert(BlobIn+16, BlobOut+16);

     res = I420compare("RGB32toYUV420_FixedPoint", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;

     OrigPicConverterRGB32toYUV420.SetFlip(true);
     PicConverterRGB32toYUV420.SetFlip(true);

     PicConverterRGB32toYUV420.Convert(BlobIn+16, BlobOut+16);
     OrigPicConverterRGB32toYUV420.Convert(BlobIn+16, BlobTest+16);

     res = I420compare("RGB32toYUV420_Real flip", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;

     fpConverterRGB32toYUV420.SetFlip(true);
     fpConverterRGB32toYUV420.Convert(BlobIn+16, BlobOut+16);

     res = I420compare("RGB32toYUV420_FixedPoint flip", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;
   }

////////////////////////////////////////////////////////////////////////////////

 for(int Width=2; Width<28; Width+=2)
   for(int Height=2; Height<28; Height+=2)
   {
     OrigRealRGB24toYUV420Converter OrigPicConverterRGB24toYUV420(Width,Height);
     ConvRGB24toYUV420Real PicConverterRGB24toYUV420(Width,Height);
     ConvRGB24toYUV42_float_LUT LutConverterRGB24toYUV420(Width,Height);
     ConvRGB24toYUV420FixP fpConverterRGB24toYUV420(Width,Height);
     //FastRGB32toYUV420Converter PicConverterRGB32toYUV420(Width,Height); //	-- not working!!!

     PicConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);
     OrigPicConverterRGB24toYUV420.Convert(BlobIn+16, BlobTest+16);

     int res = I420compare("RGB24toYUV420_Real", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;     

     fpConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);

     res = I420compare("RGB24toYUV420_Fixed Point", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;

     LutConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);

     res = I420compare("RGB24toYUV420_LUT", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;     

     OrigPicConverterRGB24toYUV420.SetFlip(true);
     PicConverterRGB24toYUV420.SetFlip(true);

     PicConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);
     OrigPicConverterRGB24toYUV420.Convert(BlobIn+16, BlobTest+16);

     res = I420compare("RGB24toYUV420_Real flip", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;

     fpConverterRGB24toYUV420.SetFlip(true);
     fpConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);

     res = I420compare("RGB24toYUV420_FixedPoint flip", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;

     LutConverterRGB24toYUV420.SetFlip(true);
     LutConverterRGB24toYUV420.Convert(BlobIn+16, BlobOut+16);

     res = I420compare("RGB24toYUV420_LUT flip", Width,Height, BlobOut,BlobTest);
     if(res>=10000) goto ReturnErr;
   }


	////////////////////////////////////////////////////////////////////////////////

//-----------------------------------YUV420_16 to BGR-------------------------------------------

	// Feed with 0 every 2nd byte.
   for(int pix=0; pix<28*28+28*28*2/4; pix++)
   {
     if(pix%10)
       BlobIn[2*pix+1+16] = 1;		// feed some artifical overflow
     else
       BlobIn[2*pix+1+16] = 0;
   }

   for(int Width=2; Width<28; Width+=2)
     for(int Height=2; Height<28; Height+=2)
     {	
       ConvYUV420_16toBGR24 picConvertedYUV420_16toBGR24(Width,Height);
       RealYUV420toBGR24ConverterImpl2Ver16 origConvertedYUV420_16toBGR24(Width,Height);

       picConvertedYUV420_16toBGR24.Convert(BlobIn+16, BlobOut+16);
       origConvertedYUV420_16toBGR24.Convert(BlobIn+16, BlobTest+16);

       int pos = RGB24compare("YUV420_16toBGR24", Width,Height, BlobTest, BlobOut);
       if(pos >= 10000)
          goto ReturnErr;

       picConvertedYUV420_16toBGR24.SetFlip(true);
       origConvertedYUV420_16toBGR24.SetFlip(true);

       picConvertedYUV420_16toBGR24.Convert(BlobIn+16, BlobOut+16);
       origConvertedYUV420_16toBGR24.Convert(BlobIn+16, BlobTest+16);

       pos = RGB24compare("inv YUV420_16toBGR24", Width,Height, BlobTest, BlobOut);
       if(pos >= 10000)
          goto ReturnErr;

       ConvYUV420_16toBGR24_CCIR601 picConvYUV420_16toBGR24_CCIR601(Width,Height);
       RealYUV420toRGB24CCIR601ConverterVer16 origConvYUV420_16toBGR24_CCIR601(Width,Height);

       picConvYUV420_16toBGR24_CCIR601.Convert(BlobIn+16, BlobOut+16);
       origConvYUV420_16toBGR24_CCIR601.Convert(BlobIn+16, BlobTest+16);

       pos = RGB24compare("YUV420_16toBGR24_CCIR601", Width,Height, BlobTest, BlobOut);
       if(pos >= 10000)
          goto ReturnErr;

       picConvYUV420_16toBGR24_CCIR601.SetFlip(true);
       origConvYUV420_16toBGR24_CCIR601.SetFlip(true);

       picConvYUV420_16toBGR24_CCIR601.Convert(BlobIn+16, BlobOut+16);
       origConvYUV420_16toBGR24_CCIR601.Convert(BlobIn+16, BlobTest+16);

       pos = RGB24compare("YUV420_16toBGR24_CCIR601", Width,Height, BlobTest, BlobOut);
       if(pos >= 10000)
          goto ReturnErr;
     }


  printf("\nAll tests passed OK.");
  free(BlobIn);
  free(BlobIn2);
  free(BlobOut);
  free(BlobTest);

  BlobIn = (unsigned char *)malloc(1920*1080*4);
  BlobIn2 = (unsigned char *)malloc(1920*1080*4);
  BlobOut = (unsigned char *)malloc(1920*1080*4);
  BlobTest = NULL;
  if(BlobIn==NULL || BlobIn2==NULL || BlobOut==NULL)
  {
    printf("\nError: Memory allocation problem.");
    goto ReturnErr;
  }


  for(int i=0; i<1920*1080*4; i++)
  {
    BlobIn[i] = rand() % 255;
    BlobIn2[i] = rand() % 255;
  }

  printf("\n\nBenchmarks:");
  unsigned long long DurationMin;
  {
    {
      PicCompareRGB32Impl compareRGB32Impl(1920,1080);
      DurationMin = BenchCompare(compareRGB32Impl,BlobIn,BlobIn2,BlobOut);
      printf("\nCompare RGB32 compare 1920x1080 time %.3f[ms]", DurationMin/1000.0);
      DurationMin = BenchCompare(compareRGB32Impl,BlobIn,BlobIn2,BlobOut,true);
      printf(" VFlip %.3f[ms]", DurationMin/1000.0);
    }
#ifdef USE_SSE
    if(FeaturesCPU & 4)
    {
      PicCompareRGB32_SSE2 compareRGB32_SSE2(1920,1080);
      DurationMin = BenchCompare(compareRGB32_SSE2,BlobIn,BlobIn2,BlobOut);
      printf(" SSE2 %.3f[ms]", DurationMin/1000.0);
      DurationMin = BenchCompare(compareRGB32_SSE2,BlobIn,BlobIn2,BlobOut,true);
      printf(" VFlip %.3f[ms]", DurationMin/1000.0);
    }
#endif

    {
      PicCompareRGB24Impl compareRGB24Impl(1920,1080);
      DurationMin = BenchCompare(compareRGB24Impl,BlobIn,BlobIn2,BlobOut);
      printf("\nCompare RGB24 compare 1920x1080 time %.3f[ms]", DurationMin/1000.0);
      DurationMin = BenchCompare(compareRGB24Impl,BlobIn,BlobIn2,BlobOut,true);
      printf(" VFlip %.3f[ms]", DurationMin/1000.0);
    }
#ifdef USE_SSE
    if(FeaturesCPU & 4)
    {
      PicCompareRGB24_SSE2 compareRGB24_SSE2(1920,1080);
      DurationMin = BenchCompare(compareRGB24_SSE2,BlobIn,BlobIn2,BlobOut);
      printf(" SSE2 %.3f[ms]", DurationMin/1000.0);
      DurationMin = BenchCompare(compareRGB24_SSE2,BlobIn,BlobIn2,BlobOut,true);
      printf(" VFlip %.3f[ms]", DurationMin/1000.0);
    }
#endif

    {
      PicCompareYUYV_Impl compareYUYV_Impl(1920,1080);
      DurationMin = BenchCompare(compareYUYV_Impl,BlobIn,BlobIn2,BlobOut);
      printf("\nCompare YUYV compare 1920x1080 time %.3f[ms]", DurationMin/1000.0);
      DurationMin = BenchCompare(compareYUYV_Impl,BlobIn,BlobIn2,BlobOut,true);
      printf(" VFlip %.3f[ms]", DurationMin/1000.0);
    }
#ifdef USE_SSE
    if(FeaturesCPU & 4)
    {
      PicCompareYUYV_SSE2 compareYUYV_SSE2(1920,1080);
      DurationMin = BenchCompare(compareYUYV_SSE2,BlobIn,BlobIn2,BlobOut);
      printf(" SSE2 %.3f[ms]", DurationMin/1000.0);
      DurationMin = BenchCompare(compareYUYV_SSE2,BlobIn,BlobIn2,BlobOut,true);
      printf(" VFlip %.3f[ms]", DurationMin/1000.0);
    }
#endif

    {
      RealRGB32toYUV420Converter picConverterRGB32toYUV420(1920,1080);
      DurationMin = BenchConverter(picConverterRGB32toYUV420, BlobIn, BlobOut);
      printf("\nConvert RGB32 to YUV 1920x1080 time %.3f[ms]", DurationMin/1000.0);
    }
/*    {
      FastRGB32toYUV420Converter fpConverterRGB32toYUV420(1920,1080);
      DurationMin = BenchConverter(fpConverterRGB32toYUV420, BlobIn, BlobOut);
      printf(" fast %.3f[ms]", DurationMin/1000.0);
    } */
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
	/////////////////////////////////////////////////////////////////////////////
    {
      RealRGB32toYUV420Converter picConverterRGB32toYUV420(1920,1080);
      picConverterRGB32toYUV420.SetFlip(true);
      DurationMin = BenchConverter(picConverterRGB32toYUV420, BlobIn, BlobOut);
      printf("\nFlipConv RGB32 to YUV res 1920,1080 time %.3f[ms]", DurationMin/1000.0);
    }
    {
      ConvRGB32toYUV420FixP fpConverterRGB32toYUV420(1920,1080);
      fpConverterRGB32toYUV420.SetFlip(true);
      DurationMin = BenchConverter(fpConverterRGB32toYUV420, BlobIn, BlobOut);
      printf(" fix point %.3f[ms]", DurationMin/1000.0);
    }
    {
      OrigRealRGB32toYUV420Converter origConverterRGB32toYUV420(1920,1080);
      origConverterRGB32toYUV420.SetFlip(true);
      DurationMin = BenchConverter(origConverterRGB32toYUV420, BlobIn, BlobOut);
      printf(" orig %.3f[ms]", DurationMin/1000.0);
    }
	/////////////////////////////////////////////////////////////////////////////
    {
      ConvRGB24toYUV420Real picConverterRGB24toYUV420(1920,1080);
      DurationMin = BenchConverter(picConverterRGB24toYUV420, BlobIn, BlobOut);
      printf("\nConvert RGB24 to YUV 1920x1080 time %.3f[ms]", DurationMin/1000.0);
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
      ConvRGB24toYUV42_float_LUT LutConverterRGB24toYUV420(1920,1080);
      DurationMin = BenchConverter(LutConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" LUT %.3f[ms]", DurationMin/1000.0);
    }

	/////////////////////////////////////////////////////////////////////////////
    {
      ConvRGB24toYUV420Real picConverterRGB24toYUV420(1920,1080);
      picConverterRGB24toYUV420.SetFlip(true);
      DurationMin = BenchConverter(picConverterRGB24toYUV420, BlobIn, BlobOut);
      printf("\nFlipConv RGB24 to YUV 1920x1080 time %.3f[ms]", DurationMin/1000.0);
    }
    {
      ConvRGB24toYUV420FixP fpConverterRGB24toYUV420(1920,1080);
      fpConverterRGB24toYUV420.SetFlip(true);
      DurationMin = BenchConverter(fpConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" fix point %.3f[ms]", DurationMin/1000.0);
    }
    {
      OrigRealRGB24toYUV420Converter origConverterRGB24toYUV420(1920,1080);
      origConverterRGB24toYUV420.SetFlip(true);
      DurationMin = BenchConverter(origConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" orig %.3f[ms]", DurationMin/1000.0);
    }
    {
      ConvRGB24toYUV42_float_LUT LutConverterRGB24toYUV420(1920,1080);
      LutConverterRGB24toYUV420.SetFlip(true);
      DurationMin = BenchConverter(LutConverterRGB24toYUV420, BlobIn, BlobOut);
      printf(" LUT %.3f[ms]", DurationMin/1000.0);
    }
    {
      ConvRGB24toYUV42_fixp_LUT LutConverterRGB24toYUV420(1920,1080);
      LutConverterRGB24toYUV420.SetFlip(true);
      DurationMin = BenchConverter(LutConverterRGB24toYUV420, BlobIn, BlobOut);
      printf("+Fix %.3f[ms]", DurationMin/1000.0);
    }

	/////////////////////////////////////////////////////////////////////////////
    {
      ConvAYUV444toRGB24 picConverterAYUV2RGB24(1920,1080);
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
    printf(" orig %.3f[ms].", DurationMin/1000.0);

   {
      AYUV444toARGB32Converter picConvertedAYUV2ARGB32(1920,1080);
      DurationMin = BenchConverter(picConvertedAYUV2ARGB32, BlobIn, BlobOut);
      printf("\nConvert AYUV to ARGB32 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      picConvertedAYUV2ARGB32.SetFlip(true);
      DurationMin = BenchConverter(picConvertedAYUV2ARGB32, BlobIn, BlobOut);
    }
    printf(" Flip: %.3f[ms]", DurationMin/1000.0);

    {
      ConvYUV420toRGB24 picConvertedYUV420toRGB24(1920,1080);
      DurationMin = BenchConverter(picConvertedYUV420toRGB24, BlobIn, BlobOut);
      printf("\nConvert YUV420 to RGB24 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      picConvertedYUV420toRGB24.SetFlip(true);
      DurationMin = BenchConverter(picConvertedYUV420toRGB24, BlobIn, BlobOut);
    }
    printf(" Flip: %.3f[ms]", DurationMin/1000.0);

#ifdef USE_ASM
    {
      ConvYUV420toRGB24_ASM picConvertedYUV420toRGB24(1920,1080);
      DurationMin = BenchConverter(picConvertedYUV420toRGB24, BlobIn, BlobOut);
      printf(" ASM %.3f[ms]", DurationMin/1000.0);
      picConvertedYUV420toRGB24.SetFlip(true);
      DurationMin = BenchConverter(picConvertedYUV420toRGB24, BlobIn, BlobOut);
    }
    printf(" ASM_Flip: %.3f[ms]", DurationMin/1000.0);
#endif
    
    {
      OrigRealYUV420toRGB24Converter origConvertedYUV420toRGB24(1920,1080);
      DurationMin = BenchConverter(origConvertedYUV420toRGB24, BlobIn, BlobOut);
      printf("\nConvert YUV420 to RGB24 resolution 1920,1080 time Orig %.3f[ms]", DurationMin/1000.0);
      origConvertedYUV420toRGB24.SetFlip(true);
      DurationMin = BenchConverter(origConvertedYUV420toRGB24, BlobIn, BlobOut);
    }
    printf(" orig flip %.3f[ms].", DurationMin/1000.0);


    {
      YUV420toRGB32Converter picConvertedYUV420toRGB32(1920,1080);
      DurationMin = BenchConverter(picConvertedYUV420toRGB32, BlobIn, BlobOut);
      printf("\nConvert YUV420 to RGB32 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      picConvertedYUV420toRGB32.SetFlip(true);
      DurationMin = BenchConverter(picConvertedYUV420toRGB32, BlobIn, BlobOut);
    }
    printf(" Flip: %.3f[ms]", DurationMin/1000.0);

#ifdef USE_ASM
    {
      ConvYUV420toRGB32_ASM picConvertedYUV420toRGB32(1920,1080);
      DurationMin = BenchConverter(picConvertedYUV420toRGB32, BlobIn, BlobOut);
      printf(" ASM %.3f[ms]", DurationMin/1000.0);
      picConvertedYUV420toRGB32.SetFlip(true);
      DurationMin = BenchConverter(picConvertedYUV420toRGB32, BlobIn, BlobOut);
    }
    printf(" ASM_Flip: %.3f[ms]", DurationMin/1000.0);
#endif

	///////////////////////////////////////////////////////

    {
      ConvRGB24toYUYV_FixP ConvRGB24toYUYV(1920,1080);
      DurationMin = BenchConverter(ConvRGB24toYUYV, BlobIn, BlobOut);
      printf("\nConvert RGB24 to YUYV resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      ConvRGB24toYUYV.SetFlip(true);
      DurationMin = BenchConverter(ConvRGB24toYUYV, BlobIn, BlobOut);
    }
    printf(" flip %.3f[ms]", DurationMin/1000.0);

    {
      ConvRGB32toYUYV_FixP ConvRGB32toYUYV(1920,1080);
      DurationMin = BenchConverter(ConvRGB32toYUYV, BlobIn, BlobOut);
      printf("\nConvert RGB32 to YUYV resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      ConvRGB32toYUYV.SetFlip(true);
      DurationMin = BenchConverter(ConvRGB32toYUYV, BlobIn, BlobOut);
    }
    printf(" flip %.3f[ms]", DurationMin/1000.0);

	/////////////////////////////////////////////////////////

   {
      ConvYUV420_16toBGR24 convYUV420_16toBGR24(1920,1080);
      DurationMin = BenchConverter(convYUV420_16toBGR24, BlobIn, BlobOut);
      printf("\nConvert YUV420_16 to BGR24 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      convYUV420_16toBGR24.SetFlip(true);
      DurationMin = BenchConverter(convYUV420_16toBGR24, BlobIn, BlobOut);
    }
    printf(" flip %.3f[ms]", DurationMin/1000.0);

   {
      RealYUV420toBGR24ConverterImpl2Ver16 origYUV420_16toBGR24(1920,1080);
      DurationMin = BenchConverter(origYUV420_16toBGR24, BlobIn, BlobOut);
      printf(" orig %.3f[ms]", DurationMin/1000.0);
      origYUV420_16toBGR24.SetFlip(true);
      DurationMin = BenchConverter(origYUV420_16toBGR24, BlobIn, BlobOut);
    }
    printf(" flip %.3f[ms]", DurationMin/1000.0);


   {
      ConvYUV420_16toBGR24_CCIR601 convYUV420_16toBGR24(1920,1080);
      DurationMin = BenchConverter(convYUV420_16toBGR24, BlobIn, BlobOut);
      printf("\nConvert CCIR601 YUV420_16 to BGR24 resolution 1920,1080 time %.3f[ms]", DurationMin/1000.0);
      convYUV420_16toBGR24.SetFlip(true);
      DurationMin = BenchConverter(convYUV420_16toBGR24, BlobIn, BlobOut);
    }
    printf(" flip %.3f[ms]", DurationMin/1000.0);

   {
      RealYUV420toRGB24CCIR601ConverterVer16 origYUV420_16toBGR24(1920,1080);
      DurationMin = BenchConverter(origYUV420_16toBGR24, BlobIn, BlobOut);
      printf(" orig %.3f[ms]", DurationMin/1000.0);
      origYUV420_16toBGR24.SetFlip(true);
      DurationMin = BenchConverter(origYUV420_16toBGR24, BlobIn, BlobOut);
    }
    printf(" flip %.3f[ms]", DurationMin/1000.0);


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
    printf("\nScale RGB32 from 1920,1080->800,600 time %.3f[ms]", DurationMin/1000.0);
    {
      OrigScalerRGB32Impl origScalerRGB32(1920,1080,800,600);
      DurationMin = BenchScaller(origScalerRGB32, BlobIn, BlobOut);
    }
    printf(" Orig %.3f[ms].", DurationMin/1000.0);

    {
      PicScalerARGB32Impl picScalerARGB32(1920,1080,800,600);     
      DurationMin = BenchScaller(picScalerARGB32, BlobIn, BlobOut);
    }
    printf("\nScale ARGB32 from 1920,1080->800,600 time %.3f", DurationMin/1000.0);

#ifdef USE_MMX
    if(FeaturesCPU & 1)
    {
      PicScalerARGB32MMX picScalerARGB32mmx(1920,1080,800,600);     
      DurationMin = BenchScaller(picScalerARGB32mmx, BlobIn, BlobOut);      
      printf(" MMX %.3f", DurationMin/1000.0);
    }
#endif

#ifdef USE_SSE
    if(FeaturesCPU & 2)
    {
      PicScalerARGB32SSE picScalerARGB32sse(1920,1080,800,600);
      DurationMin = BenchScaller(picScalerARGB32sse, BlobIn, BlobOut);
      printf(" SSE %.3f", DurationMin/1000.0);
    }
#endif
    printf("[ms].");

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
      PicRotateBPP24Impl PicRotateRGB24;
      BenchRotator("Rotate RGB24", PicRotateRGB24, BlobIn, BlobOut);
    }
    {
      OrigRotateRGB24Impl OrigRotateRGB24;
      BenchRotator("RotOrig RGB24", OrigRotateRGB24, BlobIn, BlobOut);
    }

    {
      PicRotateBPP32Impl PicRotateBPP32;
      BenchRotator("Rotate RGB32", PicRotateBPP32, BlobIn, BlobOut);
    }

    {
      OrigRotateRGB32Impl OrigRotateRGB32;
      BenchRotator("RotOrig RGB32", OrigRotateRGB32, BlobIn, BlobOut);
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
  free(BlobIn2);
  free(BlobOut);
  free(BlobTest);
  if(Pause)
  {
    printf("\nPress any key to exit.");
    getchar();
  }
  return -1;
}