/** Copyright (c) 2025 Jaroslav Fojtik
    All rights reserved.
**/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Image/PicScalerRGB24Impl.h"
#include "Image/PicScalerARGB32Impl.h"
#include "Image/PicScalerRGB32Impl.h"


class OrigScalerRGB24Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerRGB24Impl(void) { }
	OrigScalerRGB24Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerRGB24Impl(void) {}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

};//end PicScalerRGB24Impl.

int OrigScalerRGB24Impl::Scale(void* pOutImg, const void* pInImg)
{
  if(pOutImg==NULL || pInImg == NULL || _widthIn==0 || _heightIn==0)
      return(0);

  unsigned char*	pSrc		= (unsigned char*)pInImg;
  unsigned char*	pDst		= (unsigned char*)pOutImg;
	
  //double scalex = (_widthIn<=1||_widthOut<=1) ? ((double)_widthIn)/((double)_widthOut) : ((double)_widthIn-1)/((double)_widthOut-1);
  //double scaley = (_heightIn<=1 || _heightOut<=1) ? ((double)_heightIn)/((double)_heightOut) : ((double)_heightIn-1)/((double)_heightOut-1);
  double scalex = ((double)_widthIn) / ((double)_widthOut);
  double scaley = ((double)_heightIn)/((double)_heightOut);

  int x,y,posx,posy,i,j;

  for(y=1; y<=_heightOut; y++)
  {
    posy = (int)((scaley * (double)y) - 1e-5);
    if(posy < 0) posy = 0;
    else if(posy >= _heightIn) posy = _heightIn-1;

    for(x=1; x<=_widthOut; x++)
    {
      posx = (int)((scalex * (double)x) - 1e-5);
      if(posx < 0) posx = 0;
      else if(posx >= _widthIn) posx = _widthIn-1;

			/// Apply a weighted 3x3 FIR filter.
      int ai = (posy*_widthIn*3) + (posx*3);
      int b = 7 * (int)(*(pSrc + ai));
      int g = 7 * (int)(*(pSrc + (ai+1)));
      int r = 7 * (int)(*(pSrc + (ai+2)));
      for(i = -1; i <= 1; i++)
      {
        int row = posy + i;
        if(row < 0)	row = 0;
        else if(row >= _heightIn) row = _heightIn-1;
        for(j = -1; j <= 1; j++)
        {
          int col = posx + j;
          if(col < 0) col = 0;
          else if(col >= _widthIn) col = _widthIn-1;

          int aii = (row*_widthIn*3) + (col*3);
          b += (int)(*(pSrc + aii));
          g += (int)(*(pSrc + (aii+1)));
          r += (int)(*(pSrc + (aii+2)));

        }//end for j...
      }//end for i...

			/// Round before scaling.
      int ao = ((y-1)*_widthOut*3) + ((x-1)*3);
      *(pDst + ao)     	= (unsigned char)((b + 8) >> 4);
      *(pDst + (ao+1))	= (unsigned char)((g + 8) >> 4);
      *(pDst + (ao+2))	= (unsigned char)((r + 8) >> 4);
    }//end for x...
		
  }//end for y...

  return(1);
}//end Scale.


class OrigScalerARGB32Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerARGB32Impl(void) { }
	OrigScalerARGB32Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerARGB32Impl(void) {}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

};//end PicScalerRGB32Impl.

int OrigScalerARGB32Impl::Scale(void* pOutImg, const void* pInImg)
{
	if(pOutImg==NULL || pInImg == NULL || _widthIn==0 || _heightIn==0)
		return(0);

	unsigned char*	pSrc		= (unsigned char*)pInImg;
	unsigned char*	pDst		= (unsigned char*)pOutImg;
	
        double scalex = ((double)_widthIn) / ((double)_widthOut);
        double scaley = ((double)_heightIn)/((double)_heightOut);

	int x,y,posx,posy,i,j;

	for(y=1; y<=_heightOut; y++)
	{
		posy = (int)((scaley * (double)y) - 1e-5);
		if(posy < 0)	posy = 0;
		else if(posy >= _heightIn) posy = _heightIn-1;

		for(x=1; x<=_widthOut; x++)
		{
			posx = (int)((scalex * (double)x) - 1e-5);
			if(posx < 0) posx = 0;
			else if(posx >= _widthIn) posx = _widthIn-1;

			/// Apply a weighted 3x3 FIR filter.
			int ai = (posy*_widthIn*4) + (posx*4);
			int b = 7 * (int)(*(pSrc + ai));
			int g = 7 * (int)(*(pSrc + (ai+1)));
			int r = 7 * (int)(*(pSrc + (ai+2)));
		        int a = 7 * (int)(*(pSrc + (ai+3)));
			for(i = -1; i <= 1; i++)
			{
				int row = posy + i;
				if(row < 0)	row = 0;
				else if(row >= _heightIn) row = _heightIn-1;
				for(j = -1; j <= 1; j++)
				{
					int col = posx + j;
					if(col < 0) col = 0;
					else if(col >= _widthIn) col = _widthIn-1;

					int aii = (row*_widthIn*4) + (col*4);
					b += (int)(*(pSrc + aii));
					g += (int)(*(pSrc + (aii+1)));
					r += (int)(*(pSrc + (aii+2)));
					a += (int)(*(pSrc + (aii+3)));
				}//end for j...
			}//end for i...

			/// Round before scaling.
			int ao = ((y-1)*_widthOut*4) + ((x-1)*4);
			*(pDst + ao)		= (unsigned char)((b + 8) >> 4);
			*(pDst + (ao+1))	= (unsigned char)((g + 8) >> 4);
			*(pDst + (ao+2))	= (unsigned char)((r + 8) >> 4);
			*(pDst + (ao+3))	= (unsigned char)((a + 8) >> 4);
		}//end for x...
		
	}//end for y...

	return(1);
}//end Scale.


class OrigScalerRGB32Impl: public PicScalerBase
{
public:
	// Construction and destruction.
	OrigScalerRGB32Impl(void) { }
	OrigScalerRGB32Impl(int width, int height, int subWidth, int subHeight): PicScalerBase(width,height,subWidth,subHeight) { }
	virtual ~OrigScalerRGB32Impl(void) {}

	// Interface.
	int Scale(void* pOutImg, const void* pInImg);

};//end PicScalerRGB32Impl.

int OrigScalerRGB32Impl::Scale(void* pOutImg, const void* pInImg)
{
	if(pOutImg==NULL || pInImg == NULL || _widthIn==0 || _heightIn==0)
		return(0);

	unsigned char*	pSrc		= (unsigned char*)pInImg;
	unsigned char*	pDst		= (unsigned char*)pOutImg;

        double scalex = ((double)_widthIn) / ((double)_widthOut);
        double scaley = ((double)_heightIn)/((double)_heightOut);

	int x,y,posx,posy,i,j;

	for(y=1; y<=_heightOut; y++)
	{
		posy = (int)((scaley * (double)y) - 1e-5);
		if(posy < 0)	posy = 0;
		else if(posy >= _heightIn) posy = _heightIn-1;

		for(x=1; x<=_widthOut; x++)
		{
			posx = (int)((scalex * (double)x) - 1e-5);
			if(posx < 0) posx = 0;
			else if(posx >= _widthIn) posx = _widthIn-1;

			/// Apply a weighted 3x3 FIR filter.
			int ai = (posy*_widthIn*4) + (posx*4);
			int b = 7 * (int)(*(pSrc + ai));
			int g = 7 * (int)(*(pSrc + (ai+1)));
			int r = 7 * (int)(*(pSrc + (ai+2)));
		        int a = (int)(*(pSrc + (ai+3)));
			for(i = -1; i <= 1; i++)
			{
				int row = posy + i;
				if(row < 0)	row = 0;
				else if(row >= _heightIn) row = _heightIn-1;
				for(j = -1; j <= 1; j++)
				{
					int col = posx + j;
					if(col < 0) col = 0;
					else if(col >= _widthIn) col = _widthIn-1;

					int aii = (row*_widthIn*4) + (col*4);
					b += (int)(*(pSrc + aii));
					g += (int)(*(pSrc + (aii+1)));
					r += (int)(*(pSrc + (aii+2)));
				}//end for j...
			}//end for i...

			/// Round before scaling.
			int ao = ((y-1)*_widthOut*4) + ((x-1)*4);
			*(pDst + ao)		= (unsigned char)((b + 8) >> 4);
			*(pDst + (ao+1))	= (unsigned char)((g + 8) >> 4);
			*(pDst + (ao+2))	= (unsigned char)((r + 8) >> 4);
			*(pDst + (ao+3))	= a;
		}//end for x...
		
	}//end for y...

	return(1);
}//end Scale.



///////////////////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
PicScalerRGB32Impl picScalerRGB32;
unsigned char *BlobIn, *BlobOut, *BlobTest;
int i;

  BlobIn = (unsigned char *)malloc(16384);
  BlobOut = (unsigned char *)malloc(16384);
  BlobTest = (unsigned char *)malloc(16384);

  for(i=0; i<16384; i++)
  {
    BlobIn[i] = i;
  }

  for(int WithIn=0; WithIn<5; WithIn++)
   for(int HeightIn=0; HeightIn<5; HeightIn++)
    for(int WithOut=0; WithOut<5; WithOut++)
     for(int HeightOut=0; HeightOut<5; HeightOut++)
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

       {
         int InSize = 4 * WithIn * HeightIn;
         int OutSize = 4 * WithOut * HeightOut;
         if(InSize==0) OutSize=0;
         PicScalerARGB32Impl picScalerARGB32(WithOut,HeightOut,WithIn,HeightIn);
         OrigScalerARGB32Impl origScalerARGB32(WithOut,HeightOut,WithIn,HeightIn);

         memcpy(BlobOut,BlobIn,16);
         memcpy(BlobTest,BlobIn,16);
         memcpy(BlobOut+16+OutSize,BlobIn,16);
         memcpy(BlobTest+16+OutSize,BlobIn,16);

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

  printf("\nAll tests passed OK.");
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