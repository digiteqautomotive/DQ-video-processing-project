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

	pxor	mm0,mm0
	mov	eax,08080808h
	movd	mm4,eax
	punpcklbw mm4,mm0

	mov	eax,-1
	jmp	CorrectionDDA0

LoopPix0:
	dec	OutCounter
	jz	toend			; <0
	
	movd	mm1,dword ptr [ecx]	; 2*center point
	punpcklbw mm1,mm0
	movq	mm3,mm1
	psllw	mm1,2
	
	movd	mm2,dword ptr [ebx]	; left top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [esi]	; left bottom point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	psllw	mm1,1
	
	paddusw	mm1,mm3			; left middle point same as center point

	cmp	StopPtr,esi
	jle	LastColumn0
	
	movd	mm2,dword ptr [ebx+4]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx+4]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [esi+4]	
	jmp	AddCorrection0

LastColumn0:
	movd	mm2,dword ptr [ebx]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [esi]	

AddCorrection0:
	punpcklbw mm2,mm0
	paddusw	mm1,mm2

	paddusw	mm1,mm4
	
	psraw	mm1,4
	
	packuswb mm1,mm0
	movd	dword ptr [edi],mm1
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

	movd	mm1,dword ptr [ecx+4]	; 2*center point
	punpcklbw mm1,mm0
	psllw	mm1,3
	
	movd	mm2,dword ptr [ebx]	; left top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx]	; left middle point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [esi]	; left bottom point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [ebx+4]	; middle top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [esi+4]	; middle bottom point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	cmp	StopPtr,esi
	jle	LastColumn
	
	movd	mm2,dword ptr [ebx+8]	; right top point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx+8]	; right middle point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [esi+8] ; right bottom point
	jmp	AddCorrection

LastColumn:
	movd	mm2,dword ptr [ebx+4]	; right top point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [ecx+4]	; right middle point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [esi+4] ; right bottom point
	
AddCorrection:
	punpcklbw mm2,mm0
	paddusw	mm1,mm2

	paddusw	mm1,mm4
	
	psraw	mm1,4
	
	packuswb mm1,mm0
	movd	dword ptr [edi],mm1
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

ScaleRowAsmARGB32MMX endp



;*************************************************************************************


        end
