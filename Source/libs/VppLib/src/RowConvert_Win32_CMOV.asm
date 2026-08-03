;***************************************************************************************
;*                       (c)2025-2026 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.686              ;Target processor.  Use instructions for Pentium class machines
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.


KOEF_U2B	EQU	520
KOEF_U2G	EQU	101
KOEF_V2R	EQU	292
KOEF_V2G	EQU	149



;void PelYUV_RGB24_CMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount)
	public PelYUV_RGB24_CMOV
PelYUV_RGB24_CMOV proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
	v: ptr BYTE, \
	PixelCount: DWORD
local	PelCount: DWORD
local	ValV_R: DWORD
local	ValU_B: DWORD
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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah		; R11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ebx,0FFFF0000h
	cmovz	ax,bx	
	mov	[edi+3],ah		; R12

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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah		; B11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+5],ah		; R12
	xor	edx,edx

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	mov	dh,[esi]		; Y11
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx	
	mov	[edi+1],ah		; G11

	mov	dh,[esi+1]		; Y11
	mov	eax,ebx
	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+4],ah		; G12

;---------------------------------------
	mov	eax,[PixelCount]
	add	edi,[PixelCount3]
	mov	cx, word ptr [esi+eax]

	mov	eax,ebx			; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21

	mov	al,ch			; Y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 CH=1; >0 CH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax
	mov	[edi+1],ch		; G21
	
	xor	ecx,ecx
	mov	ch,al			; Y22

	add	ebx,ecx			; 256*Y22 + ValUV_G
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+4],bh		; G22

	mov	eax,[ValV_R]
	mov	ebx,eax
	add	eax,edx			; 256*y21
	mov	al,ch			; y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax	
	mov	[edi],ch		; R21
	xor	ecx,ecx
	mov	ch,al			; y22

	add	ebx,ecx			; + 256*y22
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+3],bh		; R22

	mov	eax,[ValU_B]
	add	ecx,eax			; ValU_B + 256*y22
	add	eax,edx			; ValU_B + 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[edi+2],ah		; B21

	mov	eax,ecx			; + 256*y22
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovnz	cx,ax
	mov	[edi+5],ch		; B22

	add	esi,2
	sub	edi,[PixelCount3m6]	

	dec	[PelCount]
	jnz	LoopPel		

Exit:
	ret
PelYUV_RGB24_CMOV endp



;void PelYUV_RGB24_FlipCMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount)
	public PelYUV_RGB24_FlipCMOV
PelYUV_RGB24_FlipCMOV proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
	v: ptr BYTE, \
	PixelCount: DWORD
local	PelCount: DWORD
local	ValV_R: DWORD
local	ValU_B: DWORD
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
	mov	[PixelCount3p6],eax	; 3*PixelCount + 6

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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah		; R11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ebx,0FFFF0000h
	cmovz	ax,bx	
	mov	[edi+3],ah		; R12

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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah		; B11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+5],ah		; R12
	xor	edx,edx

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	mov	dh,[esi]		; Y11
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx	
	mov	[edi+1],ah		; G11

	mov	dh,[esi+1]		; Y11
	mov	eax,ebx
	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+4],ah		; G12

;---------------------------------------
	mov	eax,[PixelCount]
	sub	edi,[PixelCount3]
	mov	cx, word ptr [esi+eax]

	mov	eax,ebx			; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21

	mov	al,ch			; Y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 CH=1; >0 CH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax
	mov	[edi+1],ch		; G21
	
	xor	ecx,ecx
	mov	ch,al			; Y22

	add	ebx,ecx			; 256*Y22 + ValUV_G
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+4],bh		; G22

	mov	eax,[ValV_R]
	mov	ebx,eax
	add	eax,edx			; 256*y21
	mov	al,ch			; y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax	
	mov	[edi],ch		; R21
	xor	ecx,ecx
	mov	ch,al			; y22

	add	ebx,ecx			; + 256*y22
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+3],bh		; R22

	mov	eax,[ValU_B]
	add	ecx,eax			; ValU_B + 256*y22
	add	eax,edx			; ValU_B + 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[edi+2],ah		; B21

	mov	eax,ecx			; + 256*y22
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovnz	cx,ax
	mov	[edi+5],ch		; B22

	add	esi,2
	add	edi,[PixelCount3p6]

	dec	[PelCount]
	jnz	LoopPel		

Exit:
	ret
PelYUV_RGB24_FlipCMOV endp



;*************************************************************************************


;void PelYUV_RGB32_CMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount)
	public PelYUV_RGB32_CMOV
PelYUV_RGB32_CMOV proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
	v: ptr BYTE, \
	PixelCount: DWORD
local	PelCount: DWORD
local	ValV_R: DWORD
local	ValU_B: DWORD
local	PixelCount4: DWORD
local	PixelCount4m8: DWORD

	mov	edi,[OutData]
	mov	esi,[y]	
	mov	ecx,[PixelCount]
	mov	eax,ecx
	shl	eax,2			; 4*PixelCount
	mov	[PixelCount4],eax
	sub	eax,8
	mov	[PixelCount4m8],eax	; 4*PixelCount - 8

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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah		; R11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ebx,0FFFF0000h
	cmovz	ax,bx	
	mov	[edi+4],ah		; R12

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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah		; B11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+6],ah		; R12
	xor	edx,edx

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	mov	dh,[esi]		; Y11
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx	
	mov	[edi+1],ah		; G11

	mov	dh,[esi+1]		; Y11
	mov	eax,ebx
	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+5],ah		; G12

;---------------------------------------
	mov	eax,[PixelCount]
	add	edi,[PixelCount4]
	mov	cx, word ptr [esi+eax]

	mov	eax,ebx			; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21

	mov	al,ch			; Y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 CH=1; >0 CH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax
	mov	[edi+1],ch		; G21
	
	xor	ecx,ecx
	mov	ch,al			; Y22

	add	ebx,ecx			; 256*Y22 + ValUV_G
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+5],bh		; G22

	mov	eax,[ValV_R]
	mov	ebx,eax
	add	eax,edx			; 256*y21
	mov	al,ch			; y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax	
	mov	[edi],ch		; R21
	xor	ecx,ecx
	mov	ch,al			; y22

	add	ebx,ecx			; + 256*y22
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+4],bh		; R22

	mov	eax,[ValU_B]
	add	ecx,eax			; ValU_B + 256*y22
	add	eax,edx			; ValU_B + 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[edi+2],ah		; B21

	mov	eax,ecx			; + 256*y22
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovnz	cx,ax
	mov	[edi+6],ch		; B22

	add	esi,2
	sub	edi,[PixelCount4m8]	

	dec	[PelCount]
	jnz	LoopPel		

Exit:
	ret
PelYUV_RGB32_CMOV endp



;void PelYUV_RGB32_FlipCMOV(unsigned char *OutData, const unsigned char *y, const signed char *u, const signed char *v, unsigned PixelCount)
	public PelYUV_RGB32_FlipCMOV
PelYUV_RGB32_FlipCMOV proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
	v: ptr BYTE, \
	PixelCount: DWORD
local	PelCount: DWORD
local	ValV_R: DWORD
local	ValU_B: DWORD
local	PixelCount4: DWORD
local	PixelCount4p8: DWORD

	mov	edi,[OutData]
	mov	esi,[y]	
	mov	ecx,[PixelCount]
	mov	eax,ecx
	shl	eax,2			; 4*PixelCount
	mov	[PixelCount4],eax
	add	eax,8
	mov	[PixelCount4p8],eax	; 4*PixelCount + 8

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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah		; R11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	ebx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ebx,0FFFF0000h
	cmovz	ax,bx	
	mov	[edi+4],ah		; R12

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
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah		; B11
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]		; 256*y12
	add	eax,edx
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+6],ah		; R12
	xor	edx,edx

	movsx	eax,ch			; V
	movsx	ecx,cl			; U
	imul	eax,-KOEF_V2G
	mov	dh,[esi]		; Y11
	imul	ecx,-KOEF_U2G
	add	eax,ecx
	mov	ebx,eax
;	mov	[ValUV_G],eax		; No need to store.

	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx	
	mov	[edi+1],ah		; G11

	mov	dh,[esi+1]		; Y11
	mov	eax,ebx
	add	eax,edx
	mov	ecx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+5],ah		; G12

;---------------------------------------
	mov	eax,[PixelCount]
	sub	edi,[PixelCount4]
	mov	cx, word ptr [esi+eax]

	mov	eax,ebx			; ValUV_G
	mov	dh,cl			; 256 * Y21
	add	eax,edx			; 256*Y21

	mov	al,ch			; Y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 CH=1; >0 CH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax
	mov	[edi+1],ch		; G21
	
	xor	ecx,ecx
	mov	ch,al			; Y22

	add	ebx,ecx			; 256*Y22 + ValUV_G
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+5],bh		; G22

	mov	eax,[ValV_R]
	mov	ebx,eax
	add	eax,edx			; 256*y21
	mov	al,ch			; y22
	mov	ecx,eax
	sar	ecx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ecx
	test	eax,0FFFF0000h
	cmovz	cx,ax	
	mov	[edi],ch		; R21
	xor	ecx,ecx
	mov	ch,al			; y22

	add	ebx,ecx			; + 256*y22
	mov	eax,ebx
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+4],bh		; R22

	mov	eax,[ValU_B]
	add	ecx,eax			; ValU_B + 256*y22
	add	eax,edx			; ValU_B + 256*y21
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx
	test	eax,0FFFF0000h
	cmovnz	ax,bx
	mov	[edi+2],ah		; B21

	mov	eax,ecx			; + 256*y22
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	ecx,0FFFF0000h
	cmovnz	cx,ax
	mov	[edi+6],ch		; B22

	add	esi,2
	add	edi,[PixelCount4p8]

	dec	[PelCount]
	jnz	LoopPel		

Exit:
	ret
PelYUV_RGB32_FlipCMOV endp


;*************************************************************************************


	public  CompareRowAsm
CompareRowAsm	proc \
	uses ebx edi esi \
	In1Ptr:ptr byte, \
	In2Ptr:ptr byte, \
	OutPtr:ptr byte, \	
        Count:DWORD

	mov	edi,[OutPtr]
	or	edi,edi		; rdi=destination ptr
	jz	ToEnd
	mov	esi,[In1Ptr]
	or	esi,esi		; rsi=source ptr 1
	jz	ToEnd
	mov	edx,[In2Ptr]
	or	edx,edx		; rdx=source ptr 2
	jz	ToEnd
	mov	ecx,[Count]
	jecxz	ToEnd

LoopPx: movzx	eax,byte ptr [esi]
	movzx	ebx,byte ptr [edx]
	sub	ax,bx
	inc	esi
	inc	edx
	add	ax,128
	mov	bl,0h		; bh=0 from movzx
	cmovs	ax,bx
	or	ah,ah
	mov	bl,0FFh
	cmovnz	ax,bx
	mov	byte ptr [edi],al
	inc	edi
	dec	ecx
	jnz	LoopPx

ToEnd:
        ret                     ; _cdecl return                
CompareRowAsm	endp


;void YUYV_BGR_CMOV(unsigned char *OutData, const unsigned char *yuyv, unsigned PixelCount);
	public YUYV_BGR_CMOV
YUYV_BGR_CMOV proc \
        uses edi esi ebx ebp, \
        OutData: ptr BYTE, \
        yuyv: ptr BYTE, \
	PixelCount: DWORD
	
	mov	edi,[OutData]
	mov	esi,[yuyv]
	mov	ecx,[PixelCount]	
	
	sub	ecx,2
	jb	Exit

LoopPx:	mov	ebx,[esi]	; YUYV
	
	xor	edx,edx
	mov	dh,bl		; 255*Y		

	mov	eax,ebx
	rol	eax,8
	and	eax,0FFh	; V

	sub	eax,128
	mov	ebp,eax		; store V to EBP
	imul	eax,KOEF_V2R	; 291*V	
	add	eax,edx
	mov	edx,eax	
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah	; <<<< R1 ****

	xor	edx,edx
	mov	dh,bl		; 255*Y1	
	movzx	eax,bh		; U

	sub	eax,128
	imul	eax,KOEF_U2B	; 520*U	
	add	eax,edx
	mov	edx,eax
	sar	eax,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah	; <<<< B1 ****

	xor	edx,edx
	mov	dh,bl		; 255*Y1
				; EDX = 255*Y	
	imul	eax,ebp,KOEF_V2G; V normalised *148
	sub	edx,eax

        movzx	eax,bh		; U
	sub	eax,128
	imul	eax,KOEF_U2G	; 102*U
	sub	edx,eax		; 255*Y - 102*U - 148*V
	mov	eax,edx
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx	
	test	eax,0FFFF0000h
	cmovz	dx,ax	
	mov	[edi+1],dh	; <<<< G1 ****

		; 2nd pixel
	mov	edx,ebx
	ror	edx,8
	and	edx,0FF00h	; 255*Y2
	
	imul	eax,ebp,KOEF_V2R; 291*V	
	add	eax,edx
	
	mov	al,bh		; store U
	mov	ebx,eax
	sar	ebx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	ebx

	test	eax,0FFFF0000h
	cmovz	bx,ax
	mov	[edi+5],bh	; <<<< R2 ****

	movzx	ebx,al		; U
	sub	ebx,128	
	imul	eax,ebx,KOEF_U2B; 521*U
	add	eax,edx

	mov	al,dh		; save Y2
	mov	edx,eax
	
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx
	test	eax,0FFFF0000h
	cmovz	dx,ax
	mov	[edi+3],dh	; <<<< B2 ****

	xor	edx,edx		; restore Y2
	mov	dh,al
				;EDX = 255*Y	
	imul	eax,ebp,KOEF_V2G; V normalised * 148
	sub	edx,eax

	imul	eax,ebx,KOEF_U2G; 102*U normalised
	sub	edx,eax		; 255*Y - 102*U - 148*V
	mov	eax,edx
	sar	edx,24			; signed DWORD <0 AH=1; >0 AH=0;
	not	edx
	test	eax,0FFFF0000h
	cmovz	dx,ax
	mov	[edi+4],dh	; <<<< G2 ****

	add	esi,4		; inc Y per 2 pixels
	add	edi,6		; Shift to new pixel R3
	sub	ecx,2
	jae	LoopPx

Exit:
	ret
YUYV_BGR_CMOV endp


        end
