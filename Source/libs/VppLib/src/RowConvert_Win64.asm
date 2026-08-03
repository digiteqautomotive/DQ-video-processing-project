;***************************************************************************************
;*                    (c)2025-2026 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.CODE             ;Indicates the start of a code segment.


KOEF_U2B	EQU	520
KOEF_U2G	EQU	101
KOEF_V2R	EQU	292
KOEF_V2G	EQU	149


;void ConvertRowAsmAYUV_ARGB24(void *pDst, void *pSrc, unsigned Pixels);
        public  ConvertRowAsmAYUV_ARGB24
ConvertRowAsmAYUV_ARGB24 proc \
        uses rdi rsi rbx
;        pDst:ptr byte, \	RCX
;        pSrc:ptr byte, \	RDX
;        Pixels:DWORD		R8

	mov	rsi,rdx
	or	rsi,rsi
	jz	toend
	mov	rdi,rcx
	or	rdi,rdi
	jz	toend
	or	R8,R8
	jz	toend

	mov	R10d,0FFFF0000h
LoopPx: movzx	ecx,byte ptr [rsi+3]	; V
	xor	edx,edx		
	sub	ecx,128

	mov	dh,byte ptr [rsi+1]

	imul	eax,ecx,KOEF_V2R	; 292*V	
	movzx	R9d,byte ptr [rsi+2]	; U	
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx

	sub	R9d,128			; U-128

	test	eax,R10d
	cmovnz	ax,bx
	mov	[rdi+2],ah		;  STORE R ******

	imul	eax,R9d,KOEF_U2B	; 520*V	
	add	eax,edx

	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,R10d
	cmovnz	ax,bx

	mov	[rdi],ah		; STORE B ******

	imul	eax,ecx,KOEF_V2G	; 149*V
	sub	edx,eax	
	imul	ecx,R9d,KOEF_U2G	; 101*U		
	sub	edx,ecx			; 255*Y - 102*U - 148*V
					;EDX = 255*Y	
	mov	ebx,edx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx		

	test	edx,R10d
	cmovnz	dx,bx
	mov	[rdi+1],dh		; STORE  G  ******
	add	rdi,3
	add	rsi,4	
	dec	R8
	jnz	LoopPx

toend:
        ret                     ; _cdecl return

ConvertRowAsmAYUV_ARGB24 endp


;void ConvertRowAsmAYUV_ARGB32(void *pDst, void *pSrc, unsigned Pixels);
        public  ConvertRowAsmAYUV_ARGB32
ConvertRowAsmAYUV_ARGB32 proc \
        uses rdi rsi rbx
;        pDst:ptr byte, \	RCX
;        pSrc:ptr byte, \	RDX
;        Pixels:DWORD		R8

	mov	rsi,rdx
	or	rsi,rsi
	jz	toend
	mov	rdi,rcx
	or	rdi,rdi
	jz	toend
	or	R8,R8
	jz	toend

	mov	R10d,0FFFF0000h
LoopPx: mov	al,byte ptr [rsi]	; A
	mov	byte ptr [rdi+3],al

	movzx	ecx,byte ptr [rsi+3]	; V
	xor	edx,edx		
	sub	ecx,128

	mov	dh,byte ptr [rsi+1]	; Y

	imul	eax,ecx,KOEF_V2R	; 292*V	
	movzx	R9d,byte ptr [rsi+2]	; U	
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx

	sub	R9d,128			; U-128

	test	eax,R10d
	cmovnz	ax,bx
	mov	[rdi+2],ah		;  STORE R ******

	imul	eax,R9d,KOEF_U2B	; 520*V	
	add	eax,edx

	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,R10d
	cmovnz	ax,bx

	mov	[rdi],ah		; STORE B ******

	imul	eax,ecx,KOEF_V2G	; 149*V
	sub	edx,eax	
	imul	ecx,R9d,KOEF_U2G	; 101*U		
	sub	edx,ecx			; 255*Y - 102*U - 148*V
					;EDX = 255*Y	
	mov	ebx,edx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx		

	test	edx,R10d
	cmovnz	dx,bx
	mov	[rdi+1],dh		; STORE  G  ******
	add	rdi,4
	add	rsi,4	
	dec	R8
	jnz	LoopPx

toend:
        ret                     ; _cdecl return

ConvertRowAsmAYUV_ARGB32 endp



;void YUV_RGB(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public YUV_RGB	
IF (@Version LT 1200) OR (@Version GE 1300)
YUV_RGB proc \
        uses rdi rsi rbx,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,
	v: ptr BYTE,
	PixelCount: DWORD	; on stack
ELSE
;; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
YUV_RGB proc \
        uses rdi rsi rbx
ENDIF

	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	ecx,PixelCount
ELSE
	mov	ecx,[rsp+28h+3*8]	;28h stays for 5th arg; rdi,rsi,rbx are now on the stack, i.e. 3*8
ENDIF

	sub	ecx,2
	jb	exit

LoopPx:	xor	edx,edx
	mov	dh,[rsi]		; 255*Y	

	xor	eax,eax
	mov	al,[R9]			; V
	sub	eax,128
	mov	R10,rax	
	imul	eax,KOEF_V2R		; 292*V	
	add	eax,edx
	
	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[rdi],ah		; R1

	movzx	eax,byte ptr [R8]	; U
	sub	eax,128
	mov	R11d,eax
	imul	eax,KOEF_U2B		; 520*U
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[rdi+2],ah		; B1
				;EDX = 255*Y
	mov	rax,R10			; V	
	imul	eax,KOEF_V2G
	sub	edx,eax
	
	mov	eax,R11d		; U
	imul	eax,KOEF_U2G		; 101*U
	sub	edx,eax			; 255*Y - 101*U - 149*V

	test	edx,0FFFF0000h
	jz	NormG1
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx	
NormG1:	mov	[rdi+1],dh	; G1	

	inc	rsi	

		; 2nd pixel
	xor	edx,edx
	mov	dh,[rsi]		; 255*Y		

	mov	rax,R10			; V	
	imul	eax,KOEF_V2R		; 292*V	
	add	eax,edx
	
	test	eax,0FFFF0000h
	jz	NormR2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR2:	mov	[rdi+3],ah	; R2

	mov	eax,R11d		; U
	imul	eax,KOEF_U2B		; 520*U	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB2:	mov	[rdi+5],ah		; B2

				;EDX = 255*Y
	mov	rax,R10			; V	
	imul	eax,KOEF_V2G
	sub	edx,eax
	
	mov	eax,R11d		; U
	imul	eax,KOEF_U2G		; 101*U
	sub	edx,eax		; 255*Y - 101*U - 149*V

	test	edx,0FFFF0000h
	jz	NormG2
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx
NormG2:	mov	[rdi+4],dh	; G2

	inc	R8			; inc U every 2nd pixel
	inc	R9			; inc V every 2nd pixel
	add	rdi,6			; Shift to R3

	inc	rsi			; inc Y every pixel
	sub	ecx,2
	jae	LoopPx

Exit:	
	ret
YUV_RGB endp


;void PelYUV_RGB24(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB24_CMOV
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB24_CMOV proc \
        uses rdi rsi rbx R12 R13 R14,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB24_CMOV proc \
        uses rdi rsi rbx R12 R13 R14
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+6*8]	;28h stays for 5th arg; rdi,rsi,rbx are now on the stack, i.e. 3*8
ENDIF
	mov	R13,R12

	shr	R12,1
	or	R12,R12
	jz	Exit

	xor	edx,edx
LoopPel:mov	al,[R9]
	inc	R9
	sub	al,128
	mov	ch,al			; V
	movsx	eax,al
	imul	eax,KOEF_V2R		; 292*V	
	mov	R11d,eax		; ValV_R

	mov	dh,[rsi]		; 255*y11
	mov	R14d,edx
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R11

	mov	eax,R11d
	mov	dh,[rsi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+3],ah		; R12

	mov	al,[R8]			; u
	inc	R8
	sub	al,128
	mov	cl,al			; U
	movsx	eax,al
	imul	eax,KOEF_U2B		; 520*U
	mov	R10d,eax		; ValU_B

	add	eax,R14d		; 256*y11
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B11

	mov	eax,R10d
	add	eax,edx			; edx = 256*y12
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+5],ah		; R12

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax			; -KOEF_U2G -KOEF_V2G
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,R14d		; Y11
	mov	R14d,ebx		; R14d now contains -KOEF_U2G -KOEF_V2G

	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G11

	mov	eax,R14d
	add	eax,edx			; 256*Y12

	mov	rbx,rax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+4],ah		; G12

;---------------------------------------
	add	rdi,R13
	mov	cx, word ptr [rsi+R13]
	add	rdi,R13
	add	rdi,R13

	mov	eax,R14d		; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21
	mov	ebx,eax

	and	ecx,00000FF00h

	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G21

	add	R14d,ecx		; 256*Y22 + ValUV_G
	mov	eax,R14d
	sar	R14d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R14d
	test	eax,0FFFF0000h
	cmovnz	ax,R14w
	mov	[rdi+4],ah		; G22

	mov	eax,R11d		;[ValV_R]
	add	eax,edx			; 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R21

	add	R11d,ecx		; + 255*y22
	mov	eax,R11d
	sar	R11d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R11d
	test	eax,0FFFF0000h
	cmovnz	ax,R11w
	mov	[rdi+3],ah		; R22

	mov	eax,R10d		;[ValU_B]
	add	eax,edx			; +256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B21

	add	R10d,ecx		; + 255*y22
	mov	eax,R10d
	sar	R10d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R10d
	test	eax,0FFFF0000h
	cmovnz	ax,R10w	
	mov	[rdi+5],ah		; B22

	sub	rdi,R13
	sub	rdi,R13
	add	rsi,2
	sub	rdi,R13
	add	rdi,6

	dec	R12
	jnz	LoopPel
Exit:
	ret
PelYUV_RGB24_CMOV endp


; AMD 64 architecture has always CMOV instruction present.
;ALIAS <PelYUV_RGB24> = <PelYUV_RGB24_CMOV>


;void PelYUV_RGB32_CMOV(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB32_CMOV
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB32_CMOV proc \
        uses rdi rsi rbx R12 R13 R14,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB32_CMOV proc \
        uses rdi rsi rbx R12 R13 R14
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+6*8]	;28h stays for 5th arg; rdi,rsi,rbx are now on the stack, i.e. 3*8
ENDIF
	mov	R13,R12

	shr	R12,1
	or	R12,R12
	jz	Exit

	xor	edx,edx
LoopPel:mov	al,[R9]
	inc	R9
	sub	al,128
	mov	ch,al			; V
	movsx	eax,al
	imul	eax,KOEF_V2R		; 292*V	
	mov	R11d,eax		; ValV_R

	mov	dh,[rsi]		; 255*y11
	mov	R14d,edx
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R11

	mov	eax,R11d
	mov	dh,[rsi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+4],ah		; R12

	mov	al,[R8]			; u
	inc	R8
	sub	al,128
	mov	cl,al			; U
	movsx	eax,al
	imul	eax,KOEF_U2B		; 520*U
	mov	R10d,eax		; ValU_B

	add	eax,R14d		; 256*y11
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B11
	mov	[rdi+3],dl		; A11=0

	mov	eax,R10d
	add	eax,edx			; edx = 256*y12
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+6],ah		; R12
	mov	[rdi+7],dl		; A12=0

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax			; -KOEF_U2G -KOEF_V2G
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,R14d		; Y11
	mov	R14d,ebx		; R14d now contains -KOEF_U2G -KOEF_V2G

	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G11

	mov	eax,R14d
	add	eax,edx			; 256*Y12

	mov	rbx,rax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+5],ah		; G12

;---------------------------------------
	mov	rax,R13
	shl	rax,2
	add	rdi,rax
	mov	cx, word ptr [rsi+R13]

	mov	eax,R14d		; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21
	mov	ebx,eax

	and	ecx,00000FF00h

	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G21

	add	R14d,ecx		; 256*Y22 + ValUV_G
	mov	eax,R14d
	sar	R14d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R14d
	test	eax,0FFFF0000h
	cmovnz	ax,R14w
	mov	[rdi+5],ah		; G22

	mov	eax,R11d		;[ValV_R]
	add	eax,edx			; 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R21

	add	R11d,ecx		; + 255*y22
	mov	eax,R11d
	sar	R11d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R11d
	test	eax,0FFFF0000h
	cmovnz	ax,R11w
	mov	[rdi+4],ah		; R22

	mov	eax,R10d		;[ValU_B]
	add	eax,edx			; +256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B21
	mov	[rdi+3],dl		; A21=0

	add	R10d,ecx		; + 255*y22
	mov	eax,R10d
	sar	R10d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R10d
	test	eax,0FFFF0000h
	cmovnz	ax,R10w	
	mov	[rdi+6],ah		; B22

	mov	rax,r13
	mov	[rdi+7],dl		; A22=0
	shl	rax,2
	sub	rdi,rax
	add	rsi,2
	add	rdi,8

	dec	R12
	jnz	LoopPel
Exit:
	ret
PelYUV_RGB32_CMOV endp


;void PelYUV_RGB24_FlipCMOV(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB24_FlipCMOV
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB24_FlipCMOV proc \
        uses rdi rsi rbx R12 R13 R14,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB24_FlipCMOV proc \
        uses rdi rsi rbx R12 R13 R14
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+6*8]	;28h stays for 5th arg; rdi,rsi,rbx are now on the stack, i.e. 3*8
ENDIF
	mov	R13,R12

	shr	R12,1
	or	R12,R12
	jz	Exit

	xor	edx,edx
LoopPel:mov	al,[R9]
	inc	R9
	sub	al,128
	mov	ch,al			; V
	movsx	eax,al
	imul	eax,KOEF_V2R		; 292*V	
	mov	R11d,eax		; ValV_R

	mov	dh,[rsi]		; 255*y11
	mov	R14d,edx
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R11

	mov	eax,R11d
	mov	dh,[rsi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+3],ah		; R12

	mov	al,[R8]			; u
	inc	R8
	sub	al,128
	mov	cl,al			; U
	movsx	eax,al
	imul	eax,KOEF_U2B		; 520*U
	mov	R10d,eax		; ValU_B

	add	eax,R14d		; 256*y11
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B11

	mov	eax,R10d
	add	eax,edx			; edx = 256*y12
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+5],ah		; R12

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax			; -KOEF_U2G -KOEF_V2G
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,R14d		; Y11
	mov	R14d,ebx		; R14d now contains -KOEF_U2G -KOEF_V2G

	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G11

	mov	eax,R14d
	add	eax,edx			; 256*Y12

	mov	rbx,rax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+4],ah		; G12

;---------------------------------------
	sub	rdi,R13
	mov	cx, word ptr [rsi+R13]
	sub	rdi,R13
	sub	rdi,R13

	mov	eax,R14d		; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21
	mov	ebx,eax

	and	ecx,00000FF00h

	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G21

	add	R14d,ecx		; 256*Y22 + ValUV_G
	mov	eax,R14d
	sar	R14d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R14d
	test	eax,0FFFF0000h
	cmovnz	ax,R14w
	mov	[rdi+4],ah		; G22

	mov	eax,R11d		;[ValV_R]
	add	eax,edx			; 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R21

	add	R11d,ecx		; + 255*y22
	mov	eax,R11d
	sar	R11d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R11d
	test	eax,0FFFF0000h
	cmovnz	ax,R11w
	mov	[rdi+3],ah		; R22

	mov	eax,R10d		;[ValU_B]
	add	eax,edx			; +256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B21

	add	R10d,ecx		; + 255*y22
	mov	eax,R10d
	sar	R10d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R10d
	test	eax,0FFFF0000h
	cmovnz	ax,R10w	
	mov	[rdi+5],ah		; B22

	add	rdi,R13
	add	rdi,R13
	add	rsi,2
	add	rdi,R13
	add	rdi,6

	dec	R12
	jnz	LoopPel
Exit:
	ret
PelYUV_RGB24_FlipCMOV endp



;void PelYUV_RGB32_FlipCMOV(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB32_FlipCMOV
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB32_FlipCMOV proc \
        uses rdi rsi rbx R12 R13 R14,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB32_FlipCMOV proc \
        uses rdi rsi rbx R12 R13 R14
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+6*8]	;28h stays for 5th arg; rdi,rsi,rbx are now on the stack, i.e. 3*8
ENDIF
	mov	R13,R12

	shr	R12,1
	or	R12,R12
	jz	Exit

	xor	edx,edx
LoopPel:mov	al,[R9]
	inc	R9
	sub	al,128
	mov	ch,al			; V
	movsx	eax,al
	imul	eax,KOEF_V2R		; 292*V	
	mov	R11d,eax		; ValV_R

	mov	dh,[rsi]		; 255*y11
	mov	R14d,edx
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R11

	mov	eax,R11d
	mov	dh,[rsi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+4],ah		; R12

	mov	al,[R8]			; u
	inc	R8
	sub	al,128
	mov	cl,al			; U
	movsx	eax,al
	imul	eax,KOEF_U2B		; 520*U
	mov	R10d,eax		; ValU_B

	add	eax,R14d		; 256*y11
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B11
	mov	[rdi+3],dl		; A11=0

	mov	eax,R10d
	add	eax,edx			; edx = 256*y12
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+6],ah		; R12
	mov	[rdi+7],dl		; A12=0

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax			; -KOEF_U2G -KOEF_V2G
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,R14d		; Y11
	mov	R14d,ebx		; R14d now contains -KOEF_U2G -KOEF_V2G

	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G11

	mov	eax,R14d
	add	eax,edx			; 256*Y12

	mov	rbx,rax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+5],ah		; G12

;---------------------------------------
	mov	rax,R13
	shl	rax,2
	sub	rdi,rax
	mov	cx, word ptr [rsi+R13]

	mov	eax,R14d		; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21
	mov	ebx,eax

	and	ecx,00000FF00h

	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+1],ah		; G21

	add	R14d,ecx		; 256*Y22 + ValUV_G
	mov	eax,R14d
	sar	R14d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R14d
	test	eax,0FFFF0000h
	cmovnz	ax,R14w
	mov	[rdi+5],ah		; G22

	mov	eax,R11d		;[ValV_R]
	add	eax,edx			; 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi],ah		; R21

	add	R11d,ecx		; + 255*y22
	mov	eax,R11d
	sar	R11d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R11d
	test	eax,0FFFF0000h
	cmovnz	ax,R11w
	mov	[rdi+4],ah		; R22

	mov	eax,R10d		;[ValU_B]
	add	eax,edx			; +256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[rdi+2],ah		; B21
	mov	[rdi+3],dl		; A21=0

	add	R10d,ecx		; + 255*y22
	mov	eax,R10d
	sar	R10d,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	R10d
	test	eax,0FFFF0000h
	cmovnz	ax,R10w	
	mov	[rdi+6],ah		; B22

	mov	rax,R13
	mov	[rdi+7],dl		; A22=0
	shl	rax,2
	add	rdi,rax			; 4*width
	add	rsi,2
	add	rdi,8

	dec	R12
	jnz	LoopPel
Exit:
	ret
PelYUV_RGB32_FlipCMOV endp


;void YUYV_BGR(unsigned char *OutData, const unsigned char *yuyv, unsigned PixelCount);
	public YUYV_BGR
YUYV_BGR proc \
        uses rdi rsi rbx
;        OutData: ptr BYTE, \
;        yuyv: ptr BYTE, \
;	PixelCount: DWORD

	mov	rsi,rdx			; source data
	mov	rdi,rcx			; destination data
	mov	rcx,R8			; rcx=amount of pixels
	
	sub	ecx,2
	jb	Exit

LoopPx:	mov	ebx,[rsi]	; V_Y2_U_Y1
	mov	eax,ebx
	
	xor	edx,edx
	mov	dh,bl		; 255*Y
	
	rol	eax,8
	and	eax,0FFh	; V
	sub	eax,128
	mov	r8d,eax		; store V  ---> to R8D
	imul	eax,KOEF_V2R	; 291*V
	mov	r9d,eax		; store 291*V to r9d
	add	eax,edx
	
	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[rdi+2],ah	; R1

	movzx	eax,bh		; U
	sub	eax,128
	mov	r11d,eax
	imul	eax,KOEF_U2B	; 520*U
	mov	r10d,eax
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[rdi],ah	; B1

				;EDX = 255*Y	
	imul	r8d,r8d,KOEF_V2G ; V normalised *148 ---> R8D
	sub	edx,r8d
	
	imul	r11d,r11d,KOEF_U2G	; 101*U
	sub	edx,r11d	; 255*Y - 101*U - 148*V

	test	edx,0FFFF0000h
	jz	NormG1
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx	
NormG1:	mov	[rdi+1],dh	; G1

		; 2nd pixel
	ror	ebx,8
	mov	eax,r9d		; 291*V
	and	ebx,0FF00h	; 255*Y2	
	add	eax,ebx

	test	eax,0FFFF0000h
	jz	NormR2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR2:	mov	[rdi+5],ah	; R2

	mov	eax,r10d	; 520*U
	add	eax,ebx

	test	eax,0FFFF0000h
	jz	NormB2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB2:	mov	[rdi+3],ah	; B2

				;EDX = 255*Y
	sub	ebx,r8d		; V normalised * 148
				; r11d - 101*U normalised
	sub	ebx,r11d	; 255*Y - 101*U - 148*V
	test	ebx,0FFFF0000h
	jz	NormG2
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
NormG2:	mov	[rdi+4],bh	; **** G2 ****

	add	rsi,4		; inc Y per 2 pixels
	add	rdi,6		; Shift to R3
	sub	ecx,2
	jae	LoopPx

Exit:	
	ret
YUYV_BGR endp


;void YUYV_BGR_CMOV(unsigned char *OutData, const unsigned char *yuyv, unsigned PixelCount);
	public YUYV_BGR_CMOV
YUYV_BGR_CMOV proc \
        uses rdi rsi rbx
;        OutData: ptr BYTE, \
;        yuyv: ptr BYTE, \
;	PixelCount: DWORD

	mov	rsi,rdx			; source data
	mov	rdi,rcx			; destination data
	mov	rcx,R8			; rcx=amount of pixels
	
	sub	ecx,2
	jb	Exit

LoopPx:	mov	ebx,[rsi]	; V_Y2_U_Y1
	mov	eax,ebx
	
	xor	edx,edx
	mov	dh,bl		; 255*Y1
	
	rol	eax,8
	and	eax,0FFh	; V
	sub	eax,128
	mov	r8d,eax		; store V  ---> to R8D
	imul	eax,KOEF_V2R	; 291*V
	mov	r9d,eax		; store 291*V to r9d
	add	eax,edx
	mov	R11d,eax
	sar	R11d,24		; signed DWORD <0 AH=1; >0 AH=0;
	not	R11d
	test	eax,0FFFF0000h
	cmovnz  ax,R11w
	mov	[rdi+2],ah	; <<<< R1 ****

	movzx	eax,bh		; U
	sub	eax,128
	mov	r11d,eax
	imul	eax,KOEF_U2B	; 520*U
	mov	r10d,eax
	add	eax,edx
	mov	edx,eax
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx
	test	eax,0FFFF0000h
	cmovnz	ax,dx
	mov	[rdi],ah	; <<<< B1 ****

	xor	edx,edx		; restore EDX = 255*Y1 from BL
	mov	dh,bl		; 255*Y1

	imul	r8d,r8d,KOEF_V2G ; V normalised *148 ---> R8D
	sub	edx,r8d
	
	imul	r11d,r11d,KOEF_U2G	; 101*U
	sub	edx,r11d	; 255*Y - 101*U - 148*V
	mov	eax,edx
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
	test	edx,0FFFF0000h
	cmovnz	dx,ax	
	mov	[rdi+1],dh	; <<<< G1 ****

		; 2nd pixel
	ror	ebx,8
	mov	eax,r9d		; 291*V
	and	ebx,0FF00h	; 255*Y2	
	add	eax,ebx
	mov	edx,eax
	sar	edx,24		; signed DWORD <0 AH=1; >0 AH=0;
	not	edx
	test	eax,0FFFF0000h
	cmovnz	ax,dx	
	mov	[rdi+5],ah	; <<<< R2 ****

	mov	eax,r10d	; 520*U
	add	eax,ebx
	mov	edx,eax
	sar	edx,24		; signed DWORD <0 AH=1; >0 AH=0;
	not	edx
	test	eax,0FFFF0000h
	cmovnz	ax,dx
	mov	[rdi+3],ah	; <<<< B2 **** 

				;EDX = 255*Y
	sub	ebx,r8d		; V normalised * 148
				; r11d - 101*U normalised
	sub	ebx,r11d	; 255*Y - 101*U - 148*V
	mov	eax,ebx
	sar	eax,24		; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ebx,0FFFF0000h
	cmovnz	bx,ax
	mov	[rdi+4],bh	; <<<< G2 ****

	add	rsi,4		; inc Y per 2 pixels
	add	rdi,6		; Shift to R3
	sub	ecx,2
	jae	LoopPx

Exit:	
	ret
YUYV_BGR_CMOV endp


        end
