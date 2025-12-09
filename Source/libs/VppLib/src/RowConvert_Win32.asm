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
    add	edi,3
	add	esi,4	
	dec	ecx
	jnz	LoopPx

toend:
        ret                     ; _cdecl return

ConvertRowAsmAYUV_ARGB24 endp


        end
