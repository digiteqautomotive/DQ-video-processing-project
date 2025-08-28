#include <stdio.h>
#include <stdlib.h>

#include "Image/PicScalerBase.h"
#include "ScaleTesters.h"


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


///////////////////////////////////////////////////////////////////////////


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
} //end Scale.


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
		} //end for x...
		
	} //end for y...

	return(1);
} //end Scale.


///////////////////////////////////////////////////////////////////////////////////////////////////

int OrigScalerUYVYImpl::Scale(void* pOutImg, const void* pInImg)
{
  if(pOutImg == NULL || pInImg == NULL || _widthIn==0 || _heightIn==0)
      return(0);

  const unsigned char*	pSrc	= (unsigned char*)pInImg;
  unsigned char*	pDst	= (unsigned char*)pOutImg;
	
  int x, y, posx, posy, i;
  int accuX, accuY;

  accuY = -1;
  posy = 0;
  y = _heightOut;
  while(y-- > 0)	
  {
    accuY += _heightIn;				// DDA integer only algorithm
    posy += accuY / _heightOut;
    accuY = accuY % _heightOut;

    const unsigned pRow[3] = {						// Calculate row starts only once per row
			((posy==0) ? 0 : (2*_widthIn*(posy-1))),
			2*_widthIn*posy,
			((posy+1>=_heightIn) ? (2*_widthIn*(_heightIn-1)) : (2*_widthIn*(posy+1))) };

    accuX = -1;
    posx = 0;
    for(x=0; x<_widthOut; x++)	// Y compound
    {
      accuX += _widthIn;			// DDA integer only algorithm
      posx += accuX / _widthOut;
      accuX = accuX % _widthOut;

			/// Apply a weighted 3x3 FIR filter.
      unsigned Y = 8;			// rounding offset +8
      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + 1 + 2*((posx==0) ? (0) : (posx - 1));
	Y += (*(pSrc + aii));

        if(posx>0) aii+=2;
        if(i==1)
        {
          Y += 8 * (*(pSrc + aii));
	}
	else
	{
          Y += (*(pSrc + aii));
	}
	if(posx+1 < _widthIn) aii+=2;
	Y += (*(pSrc + aii));
      } //end for i...
			/// Round before scaling.
      //pDst = (unsigned char*)pOutImg + (y*_widthOut*4) + (x*4);
      pDst[x*2+1]	= (unsigned char)(Y >> 4);
    }

    accuX = -1;
    posx = 0;
    for(x=0; x<_widthOut/2; x++)	// U and V compounds
    {
      accuX += _widthIn/2;			// DDA integer only algorithm
      posx += accuX / (_widthOut/2);
      accuX = accuX % (_widthOut/2);
      unsigned U = 8;
      unsigned V = 8;

      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + 4 * ((posx==0) ? (0) : (posx - 1));
	U += (*(pSrc + aii+0));
	V += (*(pSrc + aii+2));

        if(posx>0) aii+=4;
        if(i==1)
        {
          U += 8 * (*(pSrc + aii+0));
          V += 8 * (*(pSrc + aii+2));
	}
	else
	{
          U += (*(pSrc + aii+0));
          V += (*(pSrc + aii+2));
	}
	if(posx+1 < _widthIn/2) aii+=4;
	U += (*(pSrc + aii+0));
        V += (*(pSrc + aii+2));
      } //end for i...

      (pDst)[x*4+0] = U >> 4;
      (pDst)[x*4+2] = V >> 4;
    } //end for x...

    pDst += _widthOut * 2;
  } //end for y...

  return(1);
} //end Scale.


///////////////////////////////////////////////////////////////////////////////////////////////////


int OrigScalerYUYVImpl::Scale(void* pOutImg, const void* pInImg)
{
  if(pOutImg == NULL || pInImg == NULL || _widthIn==0 || _heightIn==0)
      return(0);

  const unsigned char*	pSrc	= (unsigned char*)pInImg;
  unsigned char*	pDst	= (unsigned char*)pOutImg;
	
  int x, y, posx, posy, i;
  int accuX, accuY;

  accuY = -1;
  posy = 0;
  y = _heightOut;
  while(y-- > 0)	
  {
    accuY += _heightIn;				// DDA integer only algorithm
    posy += accuY / _heightOut;
    accuY = accuY % _heightOut;

    const unsigned pRow[3] = {						// Calculate row starts only once per row
			((posy==0) ? 0 : (2*_widthIn*(posy-1))),
			2*_widthIn*posy,
			((posy+1>=_heightIn) ? (2*_widthIn*(_heightIn-1)) : (2*_widthIn*(posy+1))) };

    accuX = -1;
    posx = 0;
    for(x=0; x<_widthOut; x++)	// X compound
    {
      accuX += _widthIn;			// DDA integer only algorithm
      posx += accuX / _widthOut;
      accuX = accuX % _widthOut;

			/// Apply a weighted 3x3 FIR filter.
      unsigned Y = 8;			// rounding offset +8
      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + 2 * ((posx==0) ? (0) : (posx - 1));
	Y += (*(pSrc + aii));

        if(posx>0) aii+=2;
        if(i==1)
        {
          Y += 8 * (*(pSrc + aii));
	}
	else
	{
          Y += (*(pSrc + aii));
	}
	if(posx+1 < _widthIn) aii+=2;
	Y += (*(pSrc + aii));
      } //end for i...
			/// Round before scaling.
      //pDst = (unsigned char*)pOutImg + (y*_widthOut*4) + (x*4);
      pDst[x*2]	= (unsigned char)(Y >> 4);
    }

    accuX = -1;
    posx = 0;
    for(x=0; x<_widthOut/2; x++)	// U and V compounds
    {
      accuX += _widthIn/2;			// DDA integer only algorithm
      posx += accuX / (_widthOut/2);
      accuX = accuX % (_widthOut/2);
      unsigned U = 8;
      unsigned V = 8;

      for(i=0; i<=2; i++)
      {				
        int aii = pRow[i] + 4 * ((posx==0) ? (0) : (posx - 1));
	U += (*(pSrc + aii+1));
	V += (*(pSrc + aii+3));

        if(posx>0) aii+=4;
        if(i==1)
        {
          U += 8 * (*(pSrc + aii+1));
          V += 8 * (*(pSrc + aii+3));
	}
	else
	{
          U += (*(pSrc + aii+1));
          V += (*(pSrc + aii+3));
	}
	if(posx+1 < _widthIn/2) aii+=4;
	U += (*(pSrc + aii+1));
        V += (*(pSrc + aii+3));
      } //end for i...

      (pDst)[x*4+1] = U >> 4;
      (pDst)[x*4+3] = V >> 4;
    } //end for x...

    pDst += _widthOut * 2;
  } //end for y...

  return(1);
} //end Scale.

