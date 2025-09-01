;***************************************************************************************
;*                       (c)2025 Digiteq Automotive - Jaroslav Fojtik
;* This code could be redistributed under LGPL licency.
;****************************************************************************************/

.CODE _text             ;Indicates the start of a code segment.


; https://docs.oracle.com/cd/E19120-01/open.solaris/817-5477/eojdc/index.html
; https://www.officedaytime.com/simd512e/



; MMX is currently deprecated on Windows 64 platform and should not be used.


;void ScaleRowAsmARGB32(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmARGB32MMX
ScaleRowAsmARGB32MMX proc \
        uses rdi rsi rbx R12 R13
;       pDst: ptr byte,		RCX
;       _widthOut: unsigned,	RDX
;       pSrcRows: ptr byte	R8
;	_widthIn: unsigned	R9

	mov     rdi,rcx		; pDst rdi=destination pointer
	or	rcx,rcx
	jz	toend		; NULL pointer test
	mov	rcx,rdx		; rdx=amount of output pixels

	mov	R11,R9		; _widthIn
	or	R11,R11
	jz	toend
	mov	R12,rdx		; _widthOut
	or	rdx,rdx
	jz	toend		; _widthOut==0

	or	R8,R8
	jz	toend
	mov	R9, qword ptr [R8+8]		; Middle line
	mov	R10, qword ptr [R8+16]		; Bottom line
	mov	R8, qword ptr [R8]		; Top line
	or	R8,R8
	jz	toend
	or	R9,R9
	jz	toend
	or	R10,R10
	jz	toend

	mov	R13,R11		; with_in
	dec	R13
	shl	R13,2		; 4*with_in
	add	R13,R10		; END PTR

	pxor	mm0,mm0
	mov	eax,08080808h
	movd	mm4,eax
	punpcklbw mm4,mm0

	mov	rdx,-1		; AccuX
	jmp	CorrectionDDA0

LoopPix0:sub	rcx,1
	jc	toend			; <0

	movd	mm1,dword ptr [R9]	; 2*center point
	punpcklbw mm1,mm0
	movq	mm3,mm1
	psllw	mm1,2
	
	movd	mm2,dword ptr [R8]	; left top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R10]	; left bottom point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	psllw	mm1,1
	
	paddusw	mm1,mm3			; left middle point

	cmp	R13,R10
	jle	LastColumn0
	
	movd	mm2,dword ptr [R8+4]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R9+4]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R10+4]	
	jmp	AddCorrection0

LastColumn0:
	movd	mm2,dword ptr [R8]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R9]
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R10]	

AddCorrection0:
	punpcklbw mm2,mm0
	paddusw	mm1,mm2

	paddusw	mm1,mm4
	
	psraw	mm1,4
	
	packuswb mm1,mm0
	movd	dword ptr [rdi],mm1
	add	rdi,4	

		; DDA integer only algorithm
CorrectionDDA0:
	add	rdx,R11		;accuX += _widthIn;	
	mov	rax,rdx
	xor	rdx,rdx
	div	R12		;posx += accuX / _widthOut;
		;rdx already set	accuX = accuX % _widthOut;
	or	rax,rax			; zet Z flag
	jz	LoopPix0
	dec	rax
	sub	R13,4
	jmp	Col1andUp
	

;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:
	movd	mm1,dword ptr [R9+4]	; 2*center point
	punpcklbw mm1,mm0
	psllw	mm1,3
	
	movd	mm2,dword ptr [R8]	; left top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R9]	; left middle point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [R10]	; left bottom point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [R8+4]	; middle top point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	movd	mm2,dword ptr [R10+4]	; middle bottom point
	punpcklbw mm2,mm0	
	paddusw	mm1,mm2

	cmp	R13,R10
	jle	LastColumn
	
	movd	mm2,dword ptr [R8+8]	; right top point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R9+8]	; right middle point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R10+8] ; right bottom point
	jmp	AddCorrection

LastColumn:
	movd	mm2,dword ptr [R8+4]	; right top point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R9+4]	; right middle point
	punpcklbw mm2,mm0
	paddusw	mm1,mm2
	
	movd	mm2,dword ptr [R10+4] ; right bottom point
	
AddCorrection:
	punpcklbw mm2,mm0
	paddusw	mm1,mm2

	paddusw	mm1,mm4
	
	psraw	mm1,4
	
	packuswb mm1,mm0
	movd	dword ptr [rdi],mm1
	add	rdi,4

			; DDA integer only algorithm
	add	rdx,R11		;accuX += _widthIn;	
	mov	rax,rdx
	xor	rdx,rdx
	div	R12		;posx += accuX / _widthOut;
	;rdx already set	accuX = accuX % _widthOut;
	or	rax,rax			; zet Z flag
	jz	NoIncSrc
Col1andUp:
	shl	rax,2		; *4
	add	R8,rax
	add	R9,rax
	add	R10,rax

NoIncSrc:sub	rcx,1
	jnc	LoopCol1


toend:
        ret                     ; _cdecl return


ScaleRowAsmARGB32MMX endp



        end
