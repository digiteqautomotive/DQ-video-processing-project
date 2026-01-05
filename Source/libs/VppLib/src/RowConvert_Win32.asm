;***************************************************************************************
;*                       (c)2025 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.486              ;Target processor.  Use instructions for Pentium class machines
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.


;void ConvertRowAsmAYUV_ARGB24(void *pDst, void *pSrc, unsigned Pixels);
        public  ConvertRowAsmAYUV_ARGB24
ConvertRowAsmAYUV_ARGB24 proc \
        uses edi esi ebx ebp \
        pDst:ptr byte, \
        pSrc:ptr byte, \
        Pixels:DWORD

	mov	esi,pSrc
	or	esi,esi
	jz	toend
	mov	edi,pDst
	or	edi,edi
	jz	toend
	mov	ecx,Pixels
	jecxz	toend

LoopPx: xor	edx,edx
	mov	dh,[esi+1]	; 255*Y	

	movzx	ebx,byte ptr [esi+3]	; V
	sub	ebx,128	
	imul	eax,ebx,292		; 292*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[edi+2],ah		; R

	movzx	ebp,byte ptr [esi+2]	; U
	sub	ebp,128
	imul	eax,ebp,520		; 520*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[edi],ah		; B

				;EDX = 255*Y
	imul	eax,ebx,149		; 149*V
	sub	edx,eax

	imul	eax,ebp,101		; 101*U
	sub	edx,eax		; 255*Y - 101*U - 149*V
	
	test	edx,0FFFF0000h
	jz	NormG1
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx	
NormG1:	mov	[edi+1],dh		; G
	add	edi,3
	add	esi,4	
	dec	ecx
	jnz	LoopPx

toend:
        ret                     ; _cdecl return

ConvertRowAsmAYUV_ARGB24 endp


;void ConvertRowAsmAYUV_ARGB32(void *pDst, void *pSrc, unsigned Pixels);
        public  ConvertRowAsmAYUV_ARGB32
ConvertRowAsmAYUV_ARGB32 proc \
        uses edi esi ebx ebp \
        pDst:ptr byte, \
        pSrc:ptr byte, \
        Pixels:DWORD

	mov	esi,pSrc
	or	esi,esi
	jz	toend
	mov	edi,pDst
	or	edi,edi
	jz	toend
	mov	ecx,Pixels
	jecxz	toend

LoopPx: mov	al,byte ptr [esi]	; A
	xor	edx,edx
	mov	byte ptr [edi+3],al

	mov	dh,[esi+1]	; 255*Y	

	movzx	ebx,byte ptr [esi+3]	; V
	sub	ebx,128	
	imul	eax,ebx,359		; 359*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[edi+2],ah	; R1

	movzx	ebp,byte ptr [esi+2]	; U
	sub	ebp,128
	imul	eax,ebp,454		; 454*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[edi],ah	; B1

				;EDX = 255*Y
	imul	eax,ebx,183			; 183*V
	sub	edx,eax

	imul	eax,ebp,88		; 88*U
	sub	edx,eax		; 255*Y - 102*U - 148*V
	
	test	edx,0FFFF0000h
	jz	NormG1
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx	
NormG1:	mov	[edi+1],dh	; G1	
	add	edi,4
	add	esi,4	
	dec	ecx
	jnz	LoopPx

toend:
        ret                     ; _cdecl return

ConvertRowAsmAYUV_ARGB32 endp


;void YUV_BGR(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public YUV_BGR
YUV_BGR proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
	v: ptr BYTE, \
	PixelCount: DWORD
	
	mov	edi,[OutData]
	mov	esi,[y]
	mov	ebx,[u]
	mov	ecx,[PixelCount]
	push	ebp
	mov	ebp,[v]
	
	sub	ecx,2
	jb	Exit

LoopPx:	xor	edx,edx
	mov	dh,[esi]		; 255*Y	

	movzx	eax,byte ptr [ebp]	; V
	sub	eax,128	
	imul	eax,292			; 292*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[edi+2],ah		; R1

	movzx	eax,byte ptr [ebx]	; U
	sub	eax,128
	imul	eax,520			; 520*U
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[edi],ah	; B1

				;EDX = 255*Y
	movzx	eax,byte ptr [ebp]	; V
	sub	eax,128	
	imul	eax,149			; 149*V
	sub	edx,eax

	movzx	eax,byte ptr [ebx]	; U
	sub	eax,128
	imul	eax,101			; 101*U
	sub	edx,eax			; 255*Y - 101*U - 149*V
	
	test	edx,0FFFF0000h
	jz	NormG1
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx	
NormG1:	mov	[edi+1],dh	; G1	

	inc	esi	

		; 2nd pixel
	xor	edx,edx
	mov	dh,[esi]	; 255*Y		

	movzx	eax,byte ptr [ebp]	; V
	sub	eax,128	
	imul	eax,292			; 292*V	
	add	eax,edx
	
	test	eax,0FFFF0000h
	jz	NormR2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR2:	mov	[edi+5],ah	; R2	

	movzx	eax,byte ptr[ebx]	; U
	sub	eax,128
	imul	eax,520			; 520*U
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB2:	mov	[edi+3],ah	; B2

				;EDX = 255*Y
	movzx	eax,byte ptr [ebp]	; V
	sub	eax,128	
	imul	eax,149
	sub	edx,eax

	movzx	eax,byte ptr [ebx]	; U
	sub	eax,128
	imul	eax,101			; 101*U
	sub	edx,eax		; 255*Y - 101*U - 149*V

	test	edx,0FFFF0000h
	jz	NormG2
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx
NormG2:	mov	[edi+4],dh	; G2

	add	edi,6

	inc	ebx		; inc U every 2nd pixel
	inc	ebp		; inc V every 2nd pixel

	inc	esi		; inc Y every pixel
	sub	ecx,2
	jae	LoopPx

Exit:	
	pop	ebp
	ret
YUV_BGR endp


        end
