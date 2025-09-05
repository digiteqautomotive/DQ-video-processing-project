;***************************************************************************
;               Copyright: (c) 2021-2025 Jaroslav Fojtik                   *
;* This code could be redistributed under LGPL licency.
;***************************************************************************

.586              ;Target processor.  Use instructions for Pentium class machines
.XMM		  ; https://learn.microsoft.com/en-us/cpp/assembler/masm/dot-xmm?view=msvc-170
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.


;*************************************************************************************


;void ScaleRowAsmARGB32SSE(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmARGB32SSE
ScaleRowAsmARGB32SSE proc \
        uses edi esi ebx \
        pDst:ptr byte, \
        _widthOut:DWORD, \
        pSrcRows:DWORD, \
        _widthIn:ptr byte
local	OutCounter: DWORD
local	StopPtr: DWORD

	mov	edi,pDst	; pDst edi=destination pointer
	or	edi,edi
	jz	toend		; NULL pointer test

	mov	eax,_widthOut
	or	eax,eax
	jz	toend
	inc	eax
	mov	OutCounter,eax

	mov	edx,pSrcRows
	or	edx,edx		; Zero array ptr must be tested first
	jz	toend	
	mov	ecx,[edx+4]	; Middle line
	or	ecx,ecx
	jz	toend
	mov	esi,[edx+8]	; Bottom line - 3rd row pointer
	or	esi,esi
	jz	toend
	mov	ebx,[edx]	; Top line
	or	ebx,ebx
	jz	toend

	mov	eax,_widthIn
	or	eax,eax
	jz	toend
	dec	eax
	shl	eax,2		; 4*(with_in-1)
	add	eax,esi		; END PTR
	mov	StopPtr,EAX

	pxor	xmm0,xmm0
	mov	eax,08080808h
	movd	xmm4,eax
	punpcklbw xmm4,xmm0

	mov	eax,-1
	jmp	CorrectionDDA0

LoopPix0:
	dec	OutCounter
	jz	toend			; <0
	
	movd	xmm1,dword ptr [ecx]	; 2*center point
	punpcklbw xmm1,xmm0
	movq	xmm3,xmm1
	psllw	xmm1,2
	
	movd	xmm2,dword ptr [ebx]	; left top point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [esi]	; left bottom point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	psllw	xmm1,1
	
	paddusw	xmm1,xmm3			; left middle point same as center point

	cmp	StopPtr,esi
	jle	LastColumn0
	
	movd	xmm2,dword ptr [ebx+4]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [ecx+4]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [esi+4]	
	jmp	AddCorrection0

LastColumn0:
	movd	xmm2,dword ptr [ebx]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [ecx]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [esi]	

AddCorrection0:
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2

	paddusw	xmm1,xmm4
	
	psraw	xmm1,4
	
	packuswb xmm1,xmm0
	movd	dword ptr [edi],xmm1
	add	edi,4	

		; DDA integer only algorithm
	mov	eax,edx			;RestDDA
CorrectionDDA0:
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
		;edx already set	accuX = accuX % _widthOut;
	or	eax,eax			; zet Z flag
	jz	LoopPix0

	dec	eax
	sub	StopPtr,4
	jmp	Col1andUp


;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:

	add	ebx,eax
	add	ecx,eax
	add	esi,eax

	movd	xmm1,dword ptr [ecx+4]	; 2*center point
	punpcklbw xmm1,xmm0
	psllw	xmm1,3
	
	movd	xmm2,dword ptr [ebx]	; left top point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [ecx]	; left middle point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	movd	xmm2,dword ptr [esi]	; left bottom point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	movd	xmm2,dword ptr [ebx+4]	; middle top point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	movd	xmm2,dword ptr [esi+4]	; middle bottom point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	cmp	StopPtr,esi
	jle	LastColumn
	
	movd	xmm2,dword ptr [ebx+8]	; right top point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [ecx+8]	; right middle point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [esi+8] ; right bottom point
	jmp	AddCorrection

LastColumn:
	movd	xmm2,dword ptr [ebx+4]	; right top point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [ecx+4]	; right middle point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [esi+4] ; right bottom point
	
AddCorrection:
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2

	paddusw	xmm1,xmm4
	
	psraw	xmm1,4
	
	packuswb xmm1,xmm0
	movd	dword ptr [edi],xmm1
	add	edi,4

			; DDA integer only algorithm
	mov	eax,edx			;RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
	;edx already set	accuX = accuX % _widthOut;
Col1andUp:
	shl	eax,2		; *4

NoIncSrc:dec	OutCounter
	jnz	LoopCol1


toend:
        ret                     ; _cdecl return

ScaleRowAsmARGB32SSE endp



;*************************************************************************************


        end
