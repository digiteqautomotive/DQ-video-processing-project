;***************************************************************************************
;*                       (c)2025 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.CODE _text             ;Indicates the start of a code segment.

; https://learn.microsoft.com/en-us/cpp/build/x64-software-conventions?view=msvc-170#x64-register-usage
; Integer arguments are passed in registers RCX, RDX, R8, and R9
; These registers, and RAX, R10, R11, XMM4, and XMM5, are considered volatile,


;void ScaleRowAsmARGB32(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmARGB32
ScaleRowAsmARGB32 proc \
        uses rdi rsi rbx R12 R13
;       pDst: ptr byte,		RCX
;       _widthOut: unsigned,	RDX
;       pSrcRows: ptr byte	R8
;	_widthIn: unsigned	R9

	cld
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

	mov	rdx,-1		; AccuX
	jmp	CorrectionDDA0

LoopPix0:xor	rsi,rsi
	sub	rcx,1
	jc	toend			; <0
	cmp	R13,R10
	jle	LastColumn0
	mov	rsi,4
LastColumn0:
			; R compound
	movzx	ax, byte ptr[R9]
	shl	ax,2			; 2*center point
	movzx	bx, byte ptr[R8]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	ror	rax,12

		; G compound
	inc	rsi
	movzx	ax, byte ptr[R9+1]
	shl	ax,2			; 2*center point
	mov	bl, byte ptr[R8+1]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+1]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+1]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	ror	rax,8

		; B compound
	inc	rsi
	movzx	ax, byte ptr[R9+2]
	shl	ax,2			; 2*center point
	mov	bl, byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	ror	rax,8

		; A compound
	inc	rsi
	movzx	ax, byte ptr[R9+3]
	shl	ax,2			; 2*center point
	mov	bl, byte ptr[R8+3]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+3]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+3]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4

	rol	rax,24
	stosd

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
	mov	rsi,4
	cmp	R13,R10
	jle	LastColumn
	mov	rsi,8
LastColumn:
			; R compound
	movzx	ax, byte ptr[R9+4]
	shl	ax,3			; 4*center point
	movzx	bx, byte ptr[R8]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+4]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+4]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	ror	rax,12

		; G compound
	inc	rsi
	movzx	ax, byte ptr[R9+5]
	shl	ax,3			; 4*center point
	mov	bl, byte ptr[R8+1]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+1]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+1]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+5]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+5]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	ror	rax,8

		; B compound
	inc	rsi
	movzx	ax, byte ptr[R9+6]
	shl	ax,3			; 4*center point
	mov	bl, byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+6]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+6]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	ror	rax,8

		; A compound
	inc	rsi
	movzx	ax, byte ptr[R9+7]
	shl	ax,3			; 4*center point
	mov	bl, byte ptr[R8+3]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+3]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+3]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+7]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+7]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	rol	rax,24
	
	stosd
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

ScaleRowAsmARGB32 endp



;void ScaleRowAsmRGB32(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmRGB32
ScaleRowAsmRGB32 proc \
        uses rdi rsi rbx R12 R13
;       pDst: ptr byte,		RCX
;       _widthOut: unsigned,	RDX
;       pSrcRows: ptr byte	R8
;	_widthIn: unsigned	R9

	cld
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

	mov	rdx,-1		; AccuX
	jmp	CorrectionDDA0

LoopPix0:xor	rsi,rsi
	sub	rcx,1
	jc	toend			; <0
	cmp	R13,R10
	jle	LastColumn0
	mov	rsi,4
LastColumn0:
			; R compound
	movzx	ax, byte ptr[R9]
	shl	ax,2			; 2*center point
	movzx	bx, byte ptr[R8]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shl	eax,20			; Rxxx

		; G compound
	inc	rsi
	movzx	ax, byte ptr[R9+1]
	shl	ax,2			; 2*center point
	mov	bl, byte ptr[R8+1]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+1]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+1]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	ror	eax,8			; GRxx

		; B compound
	inc	rsi
	movzx	ax, byte ptr[R9+2]
	shl	ax,2			; 2*center point
	mov	bl, byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4			; GRxB

		; A compound ... missing in RGB32
	mov	bl, byte ptr[R9+3]	; center point
	mov	ah, bl

	rol	eax,16			; ABGR
	stosd

CorrectionDDA0:		; DDA integer only algorithm
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
	mov	rsi,4
	cmp	R13,R10
	jle	LastColumn
	mov	rsi,8
LastColumn:
			; R compound
	movzx	ax, byte ptr[R9+4]
	shl	ax,3			; 4*center point
	movzx	bx, byte ptr[R8]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+4]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+4]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shl	eax,20			; Rxxx

		; G compound
	inc	rsi
	movzx	ax, byte ptr[R9+5]
	shl	ax,3			; 4*center point
	mov	bl, byte ptr[R8+1]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+1]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+1]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+5]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+5]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	ror	eax,8			; GRxx

		; B compound
	inc	rsi
	movzx	ax, byte ptr[R9+6]
	shl	ax,3			; 4*center point
	mov	bl, byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+6]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+6]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4			; GRxB

		; A compound ... missing in RGB32
	mov	bl, byte ptr[R9+7]	; GRAB
	mov	ah,bl
	
	rol	eax,16			; ABGR
	stosd

			; DDA integer only algorithm
	add	rdx,R11		;accuX += _widthIn;	
	mov	rax,rdx
	xor	rdx,rdx
	div	R12		;posx += accuX / _widthOut;
	;			accuX = accuX % _widthOut;	rdx already set
	or	rax,rax			; set Z flag
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

ScaleRowAsmRGB32 endp



;void ScaleRowAsmRGB24(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmRGB24
ScaleRowAsmRGB24 proc \
        uses rdi rsi rbx R12 R13
;       pDst: ptr byte,		RCX
;       _widthOut: unsigned,	RDX
;       pSrcRows: ptr byte	R8
;	_widthIn: unsigned	R9

	cld
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

	mov	rax,R11		; with_in
	dec	rax
	imul	rax,rax,3	; 3*with_in
	add	rax,R10
	mov	R13,rax		; END PTR

	mov	rdx,-1		; AccuX
	jmp	CorrectionDDA0


LoopPix0:xor	rsi,rsi
	sub	rcx,1
	jc	toend			; <0
	cmp	R13,R10
	jle	LastColumn0
	mov	rsi,3
LastColumn0:
			; R compound
	movzx	ax, byte ptr[R9]
	shl	ax,2			; 2*center point
	movzx	bx, byte ptr[R8]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi],al

		; G compound
	inc	rsi
	movzx	ax, byte ptr[R9+1]
	shl	ax,2			; 2*center point
	movzx	bx, byte ptr[R8+1]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+1]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+1]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi+1],al

		; B compound
	inc	rsi
	movzx	ax, byte ptr[R9+2]
	shl	ax,2			; 2*center point
	movzx	bx, byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi+2],al
	add	rdi,3

CorrectionDDA0:		; DDA integer only algorithm
	add	rdx,R11		;accuX += _widthIn;	
	mov	rax,rdx
	xor	rdx,rdx
	div	R12		;posx += accuX / _widthOut;
		;rdx already set	accuX = accuX % _widthOut;
	or	rax,rax			; zet Z flag
	jz	LoopPix0
	dec	rax
	sub	R13,3
	jmp	Col1andUp
	

;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:
	mov	rsi,3
	cmp	R13,R10
	jle	LastColumn
	mov	rsi,6
LastColumn:
			; R compound
	movzx	ax, byte ptr[R9+3]
	shl	ax,3			; 4*center point
	movzx	bx,byte ptr[R8]		; left top point
	add	ax,bx
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+3]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+3]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	stosb

		; G compound
	inc	rsi
	movzx	ax, byte ptr[R9+4]
	shl	ax,3			; 4*center point
	movzx	bx, byte ptr[R8+1]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+1]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+1]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+4]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+4]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	stosb

		; B compound
	inc	rsi
	movzx	ax, byte ptr[R9+5]
	shl	ax,3			; 4*center point
	movzx	bx, byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+5]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+5]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	stosb

			; DDA integer only algorithm
	add	rdx,R11		;accuX += _widthIn;	
	mov	rax,rdx
	xor	rdx,rdx
	div	R12		;posx += accuX / _widthOut;
	;rdx already set	accuX = accuX % _widthOut;
	or	rax,rax			; zet Z flag
	jz	NoIncSrc
Col1andUp:
	imul	rax,rax,3		; *3
	add	R8,rax
	add	R9,rax
	add	R10,rax

NoIncSrc:sub	rcx,1
	jnc	LoopCol1


toend:
        ret                     ; _cdecl return

ScaleRowAsmRGB24 endp


;

;void ScaleRowAsmYp2(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmYp2
ScaleRowAsmYp2 proc \
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

	mov	rax,R11		; with_in
	dec	rax
	shl	rax,1		; 2*with_in
	add	rax,R10
	mov	R13,rax		; END PTR

	mov	rdx,-1		; AccuX
	jmp	CorrectionDDA0


LoopPix0:xor	rsi,rsi
	sub	rcx,1
	jc	toend			; <0
	cmp	R13,R10
	jle	LastColumn0
	mov	rsi,2
LastColumn0:
			; Y compound
	movzx	ax, byte ptr[R9]
	shl	ax,2			; 2*center point
	movzx	bx, byte ptr[R8]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi],al

	add	rdi,2

CorrectionDDA0:		; DDA integer only algorithm
	add	rdx,R11		;accuX += _widthIn;	
	mov	rax,rdx
	xor	rdx,rdx
	div	R12		;posx += accuX / _widthOut;
		;rdx already set	accuX = accuX % _widthOut;
	or	rax,rax			; zet Z flag
	jz	LoopPix0
	dec	rax
	sub	R13,2
	jmp	Col1andUp
	

;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:
	mov	rsi,2
	cmp	R13,R10
	jle	LastColumn
	mov	rsi,4
LastColumn:
			; Y compound
	movzx	ax, byte ptr[R9+2]
	shl	ax,3			; 4*center point
	movzx	bx, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl,byte ptr[R8]		; left top point
	add	ax,bx
	mov	bl, byte ptr[R8+2]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx	
	mov	bl, byte ptr[R10+2]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx	
	
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi],al
	add	rdi,2
			; DDA integer only algorithm
	add	rdx,R11		;accuX += _widthIn;	
	mov	rax,rdx
	xor	rdx,rdx
	div	R12		;posx += accuX / _widthOut;
	;rdx already set	accuX = accuX % _widthOut;
	or	rax,rax			; zet Z flag
	jz	NoIncSrc
Col1andUp:
	shl	rax,1		; *2
	add	R8,rax
	add	R9,rax
	add	R10,rax

NoIncSrc:sub	rcx,1
	jnc	LoopCol1


toend:
        ret                     ; _cdecl return

ScaleRowAsmYp2 endp



;void ScaleRowAsmUVp4(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmUVp4
ScaleRowAsmUVp4 proc \
        uses rdi rsi rbx R12 R13
;       pDst: ptr byte,		RCX
;       _widthOut: unsigned,	RDX
;       pSrcRows: ptr byte	R8
;	_widthIn: unsigned	R9

	;shr	rdx,1
	;shr	R9,1

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

	mov	rax,R11		; with_in
	dec	rax
	shl	rax,2		; 4*with_in
	add	rax,R10
	mov	R13,rax		; END PTR

	mov	rdx,-1		; AccuX
	jmp	CorrectionDDA0


LoopPix0:xor	rsi,rsi
	sub	rcx,1
	jc	toend			; <0
	cmp	R13,R10
	jle	LastColumn0
	mov	rsi,4
LastColumn0:
			; U compound
	movzx	ax, byte ptr[R9]
	shl	ax,2			; 2*center point
	movzx	bx, byte ptr[R8]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi],al

				; V compound
	add	rsi,2
	movzx	ax, byte ptr[R9+2]
	shl	ax,2			; 2*center point
	mov	bl, byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	shl	ax,1			; 4*center point + 2*(.....)
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R8+RSI]
	add	ax,bx
	mov	bl, byte ptr[R9+RSI]
	add	ax,bx
	mov	bl, byte ptr[R10+RSI]
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi+2],al

	add	rdi,4

CorrectionDDA0:		; DDA integer only algorithm
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
	mov	rsi,4
	cmp	R13,R10
	jle	LastColumn
	mov	rsi,8
LastColumn:
			; U compound
	movzx	ax, byte ptr[R9+4]
	shl	ax,3			; 4*center point
	movzx	bx,byte ptr[R8]		; left top point
	add	ax,bx
	mov	bl, byte ptr[R9]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+4]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+4]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi],al

				; V compound
	add	rsi,2
	movzx	ax, byte ptr[R9+6]
	shl	ax,3			; 4*center point
	movzx	bx,byte ptr[R8+2]	; left top point
	add	ax,bx
	mov	bl, byte ptr[R9+2]	; left middle point
	add	ax,bx
	mov	bl, byte ptr[R10+2]	; left bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+6]	; middle top point
	add	ax,bx
	mov	bl, byte ptr[R10+6]	; middle bottom point
	add	ax,bx
	mov	bl, byte ptr[R8+rsi]	; right top point
	add	ax,bx
	mov	bl, byte ptr[R9+rsi]	; right middle point
	add	ax,bx
	mov	bl, byte ptr[R10+rsi]	; right bottom point
	add	ax,bx
	add	ax,8			; rounding correction
	shr	ax,4
	mov	[rdi+2],al

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

ScaleRowAsmUVp4 endp




        end
