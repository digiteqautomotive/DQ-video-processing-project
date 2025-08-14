;***************************************************************************************
;*                       (c)2025 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.CODE _text             ;Indicates the start of a code segment.


; https://docs.oracle.com/cd/E19120-01/open.solaris/817-5477/eojdc/index.html
; https://www.officedaytime.com/simd512e/
; https://softpixel.com/~cwright/programming/simd/sse.php



;void ScaleRowAsmARGB32(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmARGB32SSE
ScaleRowAsmARGB32SSE proc \
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

	xorps	xmm0,xmm0
	mov	eax,08080808h		; rounding offset 8
	movd	xmm4,eax
	punpcklbw xmm4,xmm0

	mov	rdx,-1		; AccuX
	jmp	CorrectionDDA0

LoopPix0:sub	rcx,1
	jc	toend			; <0

	movd	xmm1,dword ptr [R9]	; 2*center point
	punpcklbw xmm1,xmm0
	movq	xmm3,xmm1
	psllw	xmm1,2
	
	movd	xmm2,dword ptr [R8]	; left top point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R10]	; left bottom point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	psllw	xmm1,1
	
	paddusw	xmm1,xmm3			; left middle point

	cmp	R13,R10
	jle	LastColumn0
	
	movd	xmm2,dword ptr [R8+4]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R9+4]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R10+4]	
	jmp	AddCorrection0

LastColumn0:
	movd	xmm2,dword ptr [R8]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R9]
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R10]	

AddCorrection0:
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2

	paddusw	xmm1,xmm4
	
	psraw	xmm1,4
	
	packuswb xmm1,xmm0
	movd	dword ptr [rdi],xmm1
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
	movd	xmm1,dword ptr [R9+4]	; center point
	punpcklbw xmm1,xmm0
	psllw	xmm1,3			; 8*center point

	movd	xmm2,dword ptr [R9]	; left middle point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	;mov	rax,[R8]
	movd	xmm2,dword ptr [R8]		; left top point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	movd	xmm2,dword ptr [R8+4]	; middle top point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	movd	xmm2,dword ptr [R10]	; left bottom point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

	movd	xmm2,dword ptr [R10+4]	; middle bottom point
	punpcklbw xmm2,xmm0	
	paddusw	xmm1,xmm2

;	movlps	xmm3,qword ptr [R8]	; left top point + middle top point	
;	punpcklbw xmm3,xmm0
	
;	movlps	xmm2,qword ptr [R10]	; left bottom point + middle bottom point
;	punpcklbw xmm2,xmm0	
;	paddusw	xmm2,xmm3

;	paddusw	xmm1,xmm2
;	movhlps	xmm2,xmm2		; get upper half to lower half
;	paddusw	xmm1,xmm2

	cmp	R13,R10
	jle	LastColumn
	
	movd	xmm2,dword ptr [R8+8]	; right top point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R9+8]	; right middle point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R10+8] ; right bottom point
	jmp	AddCorrection

LastColumn:
	movd	xmm2,dword ptr [R8+4]	; right top point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R9+4]	; right middle point
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2
	
	movd	xmm2,dword ptr [R10+4] ; right bottom point
	
AddCorrection:
	punpcklbw xmm2,xmm0
	paddusw	xmm1,xmm2

	paddusw	xmm1,xmm4
	
	psraw	xmm1,4
	
	packuswb xmm1,xmm0
	movd	dword ptr [rdi],xmm1
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


ScaleRowAsmARGB32SSE endp



        end
