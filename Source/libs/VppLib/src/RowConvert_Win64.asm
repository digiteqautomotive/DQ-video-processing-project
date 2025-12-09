;***************************************************************************************
;*                       (c)2025 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.CODE             ;Indicates the start of a code segment.



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

	imul	eax,ecx,359		; 359*V	
	movzx	R9d,byte ptr [rsi+2]	; U	
	add	eax,edx
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx

	sub	R9d,128			; U-128

	test	eax,R10d
	cmovnz	ax,bx
	mov	[rdi+2],ah		;  STORE R ******

	imul	eax,R9d,454		; 454*V	
	add	eax,edx

	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,R10d
	cmovnz	ax,bx

	mov	[rdi],ah		; STORE B ******

	imul	eax,ecx,183		; 183*V
	sub	edx,eax	
	imul	ecx,R9d,88		; 88*U		
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


        end
