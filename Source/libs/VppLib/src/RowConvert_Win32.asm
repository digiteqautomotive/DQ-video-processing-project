;***************************************************************************************
;*                       (c)2025-2026 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.486              ;Target processor.  Use instructions for Pentium class machines
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.


KOEF_U2B	EQU	520
KOEF_U2G	EQU	101
KOEF_V2R	EQU	292
KOEF_V2G	EQU	149


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
	imul	eax,ebx,KOEF_V2R	; 292*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[edi+2],ah		; R

	movzx	ebp,byte ptr [esi+2]	; U
	sub	ebp,128
	imul	eax,ebp,KOEF_U2B	; 520*U
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[edi],ah		; B

				;EDX = 255*Y
	imul	eax,ebx,KOEF_V2G	; 149*V
	sub	edx,eax

	imul	eax,ebp,KOEF_U2G	; 101*U
	sub	edx,eax			; 255*Y - 101*U - 149*V
	
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
	imul	eax,ebx,KOEF_V2R	; 359*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[edi+2],ah	; R1

	movzx	ebp,byte ptr [esi+2]	; U
	sub	ebp,128
	imul	eax,ebp,KOEF_U2B	; 454*U	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[edi],ah	; B1

					;EDX = 255*Y
	imul	eax,ebx,KOEF_V2G	; 183*V
	sub	edx,eax

	imul	eax,ebp,KOEF_U2G	; 88*U
	sub	edx,eax			; 255*Y - 101*U - 148*V
	
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



;void YUV_RGB(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public YUV_RGB
YUV_RGB proc \
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
	imul	eax,KOEF_V2R		; 292*V	
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormR1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR1:	mov	[edi],ah		; R1

	movzx	eax,byte ptr [ebx]	; U
	sub	eax,128
	imul	eax,KOEF_U2B		; 520*U
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB1
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax	
NormB1:	mov	[edi+2],ah		; B1

				;EDX = 255*Y
	movzx	eax,byte ptr [ebp]	; V
	sub	eax,128	
	imul	eax,149			; 149*V
	sub	edx,eax

	movzx	eax,byte ptr [ebx]	; U
	sub	eax,128
	imul	eax,KOEF_U2G			; 101*U
	sub	edx,eax			; 256*Y - 101*U - 149*V
	
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
	imul	eax,KOEF_V2R		; 292*V	
	add	eax,edx
	
	test	eax,0FFFF0000h
	jz	NormR2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR2:	mov	[edi+3],ah	; R2	

	movzx	eax,byte ptr[ebx]	; U
	sub	eax,128
	imul	eax,520			; 520*U
	add	eax,edx

	test	eax,0FFFF0000h
	jz	NormB2
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB2:	mov	[edi+5],ah	; B2

				;EDX = 255*Y
	movzx	eax,byte ptr [ebp]	; V
	sub	eax,128	
	imul	eax,149
	sub	edx,eax

	movzx	eax,byte ptr [ebx]	; U
	sub	eax,128
	imul	eax,KOEF_U2G			; 101*U
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
YUV_RGB endp


;void PelYUV_RGB(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB
PelYUV_RGB proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
	v: ptr BYTE, \
	PixelCount: DWORD
local	PelCount: DWORD
local	ValV_R: DWORD
local	ValU_B: DWORD
;local	ValUV_G: DWORD
local	PixelCount3: DWORD
local	PixelCount3m6: DWORD

	mov	edi,[OutData]
	mov	esi,[y]	
	mov	ecx,[PixelCount]
	mov	eax,ecx
	add	eax,ecx
	add	eax,ecx			; 3*PixelCount
	mov	[PixelCount3],eax
	sub	eax,6
	mov	[PixelCount3m6],eax	; 3*PixelCount - 6

	shr	ecx,1
	or	ecx,ecx
	jz	Exit
	mov	[PelCount],ecx

	xor	edx,edx
LoopPel:mov	ebx,[v]
	mov	al,[ebx]
	inc	ebx
	sub	al,128
	mov	[v],ebx
	mov	ch,al			; V
	movsx	eax,al
	imul	eax,KOEF_V2R		; 292*V	
	mov	[ValV_R],eax
	mov	ebx,eax	

	mov	dh,[esi]		; 255*y11
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormR11
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR11:mov	[edi],ah		; R11

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormR12
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR12:mov	[edi+3],ah		; R12

	mov	ebx,[u]
	mov	al,[ebx]
	inc	ebx
	sub	al,128
	mov	[u],ebx
	mov	cl,al			; U
	movsx	eax,al
	imul	eax,KOEF_U2B		; 520*U
	mov	[ValU_B],eax
	mov	ebx,eax

	mov	dh,[esi]		; 256*y11
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormB11
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB11:mov	[edi+2],ah		; B11

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormB12
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB12:mov	[edi+5],ah		; R12

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	mov	dh,[esi]		; Y11
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormG11
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormG11:mov	[edi+1],ah		; G11

	mov	dh,[esi+1]		; Y11
	mov	eax,ebx
	add	eax,edx	
	test	eax,0FFFF0000h
	jz	NormG12
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormG12:mov	[edi+4],ah		; G12

;---------------------------------------
	mov	eax,[PixelCount]
	add	edi,[PixelCount3]
	mov	cx, word ptr [esi+eax]

	mov	eax,ebx			; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21

	and	ecx,00000FF00h

	test	eax,0FFFF0000h
	jz	NormG21
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormG21:mov	[edi+1],ah		; G21

	add	ebx,ecx			; 256*Y22 + ValUV_G
	test	ebx,0FFFF0000h
	jz	NormG22
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
NormG22:mov	[edi+4],bh		; G22

	mov	eax,[ValV_R]
	mov	ebx,eax
	add	eax,edx			; 256*y21
	test	eax,0FFFF0000h
	jz	NormR21
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR21:mov	[edi],ah		; R21

	add	ebx,ecx			; + 255*y22
	test	ebx,0FFFF0000h
	jz	NormR22
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
NormR22:mov	[edi+3],bh		; R22

	mov	eax,[ValU_B]
	mov	ebx,eax
	add	eax,edx			; +256*y21

	test	eax,0FFFF0000h
	jz	NormB21
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB21:mov	[edi+2],ah		; B21

	add	ebx,ecx			; + 255*y22
;	mov	eax,ebx
;	sar	eax,24
;	not	eax
;	test	ebx,0FFFF0000h
;	cmovnz	bx,ax
;	mov	[edi+5],bh		; B22

	test	ebx,0FFFF0000h
	jz	NormB22
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
NormB22:mov	[edi+5],bh		; B22

	add	esi,2
	sub	edi,[PixelCount3m6]	

	dec	[PelCount]
	jnz	LoopPel		

Exit:
	ret
PelYUV_RGB endp


;void PelYUV_RGB_Flip(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB_Flip
PelYUV_RGB_Flip proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
	v: ptr BYTE, \
	PixelCount: DWORD
local	PelCount: DWORD
local	ValV_R: DWORD
local	ValU_B: DWORD
;local	ValUV_G: DWORD
local	PixelCount3: DWORD
local	PixelCount3p6: DWORD

	mov	edi,[OutData]
	mov	esi,[y]	
	mov	ecx,[PixelCount]
	mov	eax,ecx
	add	eax,ecx
	add	eax,ecx			; 3*PixelCount
	mov	[PixelCount3],eax
	add	eax,6
	mov	[PixelCount3p6],eax	; 3*PixelCount - 6

	shr	ecx,1
	or	ecx,ecx
	jz	Exit
	mov	[PelCount],ecx

	xor	edx,edx
LoopPel:mov	ebx,[v]
	mov	al,[ebx]
	inc	ebx
	sub	al,128
	mov	[v],ebx
	mov	ch,al			; V
	movsx	eax,al
	imul	eax,KOEF_V2R		; 292*V	
	mov	[ValV_R],eax
	mov	ebx,eax	

	mov	dh,[esi]		; 255*y11
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormR11
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR11:mov	[edi],ah		; R11

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormR12
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR12:mov	[edi+3],ah		; R12

	mov	ebx,[u]
	mov	al,[ebx]
	inc	ebx
	sub	al,128
	mov	[u],ebx
	mov	cl,al			; U
	movsx	eax,al
	imul	eax,KOEF_U2B		; 520*U
	mov	[ValU_B],eax
	mov	ebx,eax

	mov	dh,[esi]		; 256*y11
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormB11
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB11:mov	[edi+2],ah		; B11

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormB12
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB12:mov	[edi+5],ah		; R12

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	mov	dh,[esi]		; Y11
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,edx
	test	eax,0FFFF0000h
	jz	NormG11
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormG11:mov	[edi+1],ah		; G11

	mov	dh,[esi+1]		; Y11
	mov	eax,ebx
	add	eax,edx	
	test	eax,0FFFF0000h
	jz	NormG12
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormG12:mov	[edi+4],ah		; G12

;---------------------------------------
	mov	eax,[PixelCount]
	sub	edi,[PixelCount3]
	mov	cx, word ptr [esi+eax]

	mov	eax,ebx			; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21

	and	ecx,00000FF00h

	test	eax,0FFFF0000h
	jz	NormG21
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormG21:mov	[edi+1],ah		; G21

	add	ebx,ecx			; 256*Y22 + ValUV_G
	test	ebx,0FFFF0000h
	jz	NormG22
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
NormG22:mov	[edi+4],bh		; G22

	mov	eax,[ValV_R]
	mov	ebx,eax
	add	eax,edx			; 256*y21
	test	eax,0FFFF0000h
	jz	NormR21
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormR21:mov	[edi],ah		; R21

	add	ebx,ecx			; + 255*y22
	test	ebx,0FFFF0000h
	jz	NormR22
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
NormR22:mov	[edi+3],bh		; R22

	mov	eax,[ValU_B]
	mov	ebx,eax
	add	eax,edx			; +256*y21

	test	eax,0FFFF0000h
	jz	NormB21
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
NormB21:mov	[edi+2],ah		; B21

	add	ebx,ecx			; + 255*y22
;	mov	eax,ebx
;	sar	eax,24
;	not	eax
;	test	ebx,0FFFF0000h
;	cmovnz	bx,ax
;	mov	[edi+5],bh		; B22

	test	ebx,0FFFF0000h
	jz	NormB22
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
NormB22:mov	[edi+5],bh		; B22

	add	esi,2
	add	edi,[PixelCount3p6]

	dec	[PelCount]
	jnz	LoopPel		

Exit:
	ret
PelYUV_RGB_Flip endp


        end
