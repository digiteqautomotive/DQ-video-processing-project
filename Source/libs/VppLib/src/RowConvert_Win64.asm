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


;void PelYUV_RGB(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB proc \
        uses rdi rsi rbx R12 R13 R14,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB proc \
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
PelYUV_RGB endp



        end
