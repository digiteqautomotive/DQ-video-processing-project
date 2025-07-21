;***************************************************************************
; Copyright: (c) 2021-2025 Jaroslav Fojtik                                 *
;***************************************************************************

.586              ;Target processor.  Use instructions for Pentium class machines
.MMX
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.


; https://docs.oracle.com/cd/E19120-01/open.solaris/817-5477/eojdc/index.html
; https://www.officedaytime.com/simd512e/

;*************************************************************************************


;void ScaleRowAsmARGB32MMX(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmARGB32MMX
ScaleRowAsmARGB32MMX proc \
        uses edi esi ebx \
        pDst:ptr byte, \
        _widthOut:DWORD, \
        pSrcRows:DWORD, \
        _widthIn:ptr byte
local	OutCouter: DWORD
local	StopPtr: DWORD
local	RestDDA: DWORD

	clc
	mov	edi,pDst	; pDst edi=destination pointer
	or	edi,edi
	jz	toend		; NULL pointer test

	mov	eax,_widthOut
	or	eax,eax
	jz	toend
	mov	OutCouter,eax

	mov	ebx,pSrcRows
	or	ebx,ebx
	jz	toend
	mov	ecx,[ebx]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+4]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+8]	; 3rd row pointer
	or	ecx,ecx
	jz	toend

	mov	eax,_widthIn
	or	eax,eax
	jz	toend
	dec	eax
	shl	eax,2		; 4*with_in
	add	eax,ecx		; END PTR
	mov	StopPtr,EAX

	pxor	mm0,mm0
	mov	eax,08080808h
	movd	mm4,eax
	punpcklbw mm4,mm0

	mov	eax,-1
	jmp	CorrectionDDA0

LoopPix0:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]		; Middle line
	mov	edx,[ebx+8]		; Bottom line
	mov	ebx,[ebx]		; Top line

	sub	OutCouter,1
	jc	toend			; <0
	
	movd	mm1,dword ptr [ecx]	; 2*center point
	punpcklbw mm1,mm0
	movq	mm3,mm1
	psllw	mm1,2
	
	movd	mm2,dword ptr [ebx]	; left top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [edx]	; left bottom point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	psllw	mm1,1
	
	paddusw	mm1,mm3			; left middle point

	cmp	StopPtr,edx
	jle	LastColumn0
	
	movd	mm2,dword ptr [ebx+4]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx+4]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [edx+4]	
	jmp	AddCorrection0

LastColumn0:
	movd	mm2,dword ptr [ebx]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [edx]	

AddCorrection0:
	punpcklbw mm2,mm0
	paddusw	mm1,mm2

	paddusw	mm1,mm4
	
	psraw	mm1,4
	
	packuswb mm1,mm0
	movd	dword ptr [edi],mm1
	add	edi,4	

		; DDA integer only algorithm
	mov	eax,RestDDA
CorrectionDDA0:
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
		;edx already set	accuX = accuX % _widthOut;
	or	eax,eax			; zet Z flag
	jz	LoopPix0

	dec	eax
	mov	ebx,StopPtr
	sub	ebx,4
	mov	StopPtr,ebx
	jmp	Col1andUp


;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]
	add	ecx,eax
	mov	[ebx+4],ecx
	mov	edx,[ebx+8]
	add	edx,eax
	mov	[ebx+8],edx
	add	eax,[ebx]
	mov	[ebx],eax
	mov	ebx,eax

	movd	mm1,dword ptr [ecx+4]	; 2*center point
	punpcklbw mm1,mm0
	psllw	mm1,3
	
	movd	mm2,dword ptr [ebx]	; left top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx]	; left middle point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [edx]	; left bottom point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [ebx+4]	; middle top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [edx+4]	; middle bottom point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	cmp	StopPtr,edx
	jle	LastColumn
	
	movd	mm2,dword ptr [ebx+8]	; right top point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx+8]	; right middle point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [edx+8] ; right bottom point
	jmp	AddCorrection

LastColumn:
	movd	mm2,dword ptr [ebx+4]	; right top point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx+4]	; right middle point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [edx+4] ; right bottom point
	
AddCorrection:
	punpcklbw mm2,mm0
	paddusw	mm1,mm2

	paddusw	mm1,mm4
	
	psraw	mm1,4
	
	packuswb mm1,mm0
	movd	dword ptr [edi],mm1
	add	edi,4

			; DDA integer only algorithm
	mov	eax,RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
	;edx already set	accuX = accuX % _widthOut;
Col1andUp:
	shl	eax,2		; *4

NoIncSrc:sub	OutCouter,1
	jnc	LoopCol1


toend:
        ret                     ; _cdecl return

ScaleRowAsmARGB32MMX endp



;*************************************************************************************


        end
