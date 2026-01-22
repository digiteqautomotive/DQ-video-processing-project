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



;void YUV_BGR(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public YUV_BGR
	
IF (@Version LT 1200) OR (@Version GE 1300)
YUV_BGR proc \
        uses rdi rsi rbx,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,
	v: ptr BYTE,
	PixelCount: DWORD	; on stack
ELSE
;; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
YUV_BGR proc \
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
NormR1:	mov	[rdi+2],ah		; R1

	movzx	eax,byte ptr [R8]	; U
	sub	eax,128
	mov	R11d,eax
	imul	eax,KOEF_U2B		; 520*U
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[rdi],ah		; B1
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
NormR2:	mov	[rdi+5],ah	; R2

	mov	eax,R11d		; U
	imul	eax,KOEF_U2B		; 520*U	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB2:	mov	[rdi+3],ah		; B2

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
YUV_BGR endp



        end
