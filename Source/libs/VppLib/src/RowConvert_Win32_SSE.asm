;***************************************************************************************
;*                       (c)2025-2026 Digiteq Automotive - Jaroslav Fojtik
;*  SSE4.1 Strategy B - 4-pixel main loop + scalar tail for remainder 2
;****************************************************************************************/

.686
.XMM
.MODEL FLAT, C

.CODE

KOEF_U2B	EQU	520
KOEF_U2G	EQU	101
KOEF_V2R	EQU	292
KOEF_V2G	EQU	149


;void PelYUV_RGB24_CMOV_SSE(unsigned char *OutData, const unsigned char *y,
;                       const signed char *u, const signed char *v, unsigned PixelCount)
	public PelYUV_RGB24_CMOV_SSE
PelYUV_RGB24_CMOV_SSE proc \
        uses edi esi ebx, \
        OutData: ptr BYTE, \
        y: ptr BYTE, \
        u: ptr BYTE, \
        v: ptr BYTE, \
        PixelCount: DWORD

local	PelCount: DWORD
local	PixelCount3: DWORD
local	LineWidth: DWORD
local	FirstRowPtr: DWORD
local	Remain: DWORD
local	sValV_R: DWORD
local	sValU_B: DWORD
local	sValUV_G: DWORD

	mov	edi,[OutData]
	mov	esi,[y]
	mov	eax,[PixelCount]
	mov	[LineWidth],eax

	mov	ecx,eax
	lea	eax,[eax+eax*2]
	mov	[PixelCount3],eax

	mov	eax,ecx
	and	eax,2
	mov	[Remain],eax			; 0 or 2 remainder pixels

	shr	ecx,2
	mov	[PelCount],ecx
	jz	DoTail

LoopPel:
	; reload constants every iteration (overwritten by temporaries)
	mov	eax,KOEF_V2R
	movd	xmm7,eax
	pshufd	xmm7,xmm7,0

	mov	eax,KOEF_U2B
	movd	xmm6,eax
	pshufd	xmm6,xmm6,0

	mov	eax,-KOEF_V2G
	movd	xmm5,eax
	pshufd	xmm5,xmm5,0

	mov	eax,-KOEF_U2G
	movd	xmm4,eax
	pshufd	xmm4,xmm4,0

	mov	[FirstRowPtr],edi

	; load 2 signed V (sub-128 done on AL before sign-extend)
	mov	ebx,[v]
	mov	al,byte ptr [ebx]
	sub	al,128
	movsx	eax,al
	movd	xmm0,eax
	mov	al,byte ptr [ebx+1]
	sub	al,128
	movsx	eax,al
	movd	xmm1,eax
	punpcklqdq xmm0,xmm1
	add	dword ptr [v],2

	; load 2 signed U
	mov	ebx,[u]
	mov	al,byte ptr [ebx]
	sub	al,128
	movsx	eax,al
	movd	xmm1,eax
	mov	al,byte ptr [ebx+1]
	sub	al,128
	movsx	eax,al
	movd	xmm2,eax
	punpcklqdq xmm1,xmm2
	add	dword ptr [u],2

	movdqa	xmm3,xmm0			; keep originals for green
	movdqa	xmm2,xmm1

	pmulld	xmm0,xmm7			; ValV_R
	pmulld	xmm1,xmm6			; ValU_B

	pmulld	xmm3,xmm5			; V * -KOEF_V2G
	pmulld	xmm2,xmm4			; U * -KOEF_U2G
	paddd	xmm3,xmm2			; ValUV_G

	; broadcast to [Val0 Val0 Val1 Val1]
	movd	eax,xmm0
	pshufd	xmm0,xmm0,2
	movd	ebx,xmm0
	movd	xmm0,eax
	pinsrd	xmm0,ebx,1
	pshufd	xmm0,xmm0,050h

	movd	eax,xmm1
	pshufd	xmm1,xmm1,2
	movd	ebx,xmm1
	movd	xmm1,eax
	pinsrd	xmm1,ebx,1
	pshufd	xmm1,xmm1,050h

	movd	eax,xmm3
	pshufd	xmm3,xmm3,2
	movd	ebx,xmm3
	movd	xmm3,eax
	pinsrd	xmm3,ebx,1
	pshufd	xmm3,xmm3,050h

	; first-row Y << 8
	movzx	eax,byte ptr [esi]
	shl	eax,8
	movd	xmm2,eax
	movzx	eax,byte ptr [esi+1]
	shl	eax,8
	pinsrd	xmm2,eax,1
	movzx	eax,byte ptr [esi+2]
	shl	eax,8
	pinsrd	xmm2,eax,2
	movzx	eax,byte ptr [esi+3]
	shl	eax,8
	pinsrd	xmm2,eax,3

	movdqa	xmm4,xmm2
	paddd	xmm4,xmm0			; R
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1			; B
	paddd	xmm2,xmm3			; G

	; store first row (4 pixels) with classic CMOV saturation
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+1],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah

	psrldq	xmm4,4
	psrldq	xmm2,4
	psrldq	xmm5,4
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+3],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+4],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+5],ah

	psrldq	xmm4,4
	psrldq	xmm2,4
	psrldq	xmm5,4
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+6],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+7],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+8],ah

	psrldq	xmm4,4
	psrldq	xmm2,4
	psrldq	xmm5,4
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+9],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+10],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+11],ah

	; second-row Y
	mov	eax,[LineWidth]
	add	esi,eax

	movzx	eax,byte ptr [esi]
	shl	eax,8
	movd	xmm2,eax
	movzx	eax,byte ptr [esi+1]
	shl	eax,8
	pinsrd	xmm2,eax,1
	movzx	eax,byte ptr [esi+2]
	shl	eax,8
	pinsrd	xmm2,eax,2
	movzx	eax,byte ptr [esi+3]
	shl	eax,8
	pinsrd	xmm2,eax,3

	movdqa	xmm4,xmm2
	paddd	xmm4,xmm0
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1
	paddd	xmm2,xmm3

	mov	edi,[FirstRowPtr]
	add	edi,[PixelCount3]

	; store second row (identical saturation)
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+1],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah

	psrldq	xmm4,4
	psrldq	xmm2,4
	psrldq	xmm5,4
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+3],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+4],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+5],ah

	psrldq	xmm4,4
	psrldq	xmm2,4
	psrldq	xmm5,4
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+6],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+7],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+8],ah

	psrldq	xmm4,4
	psrldq	xmm2,4
	psrldq	xmm5,4
	movd	eax,xmm4
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+9],ah
	movd	eax,xmm2
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+10],ah
	movd	eax,xmm5
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+11],ah

	; advance to next group of 4 (net +4 on first-row Y)
	mov	edi,[FirstRowPtr]
	add	edi,12
	sub	esi,[LineWidth]
	add	esi,4

	dec	[PelCount]
	jnz	LoopPel

DoTail:
	cmp	dword ptr [Remain],0
	je	Exit

	; ----- scalar tail for remaining 2 pixels (exact original CMOV path) -----
	xor	edx,edx

	; V -> sValV_R
	mov	ebx,[v]
	mov	al,[ebx]
	inc	ebx
	mov	[v],ebx
	sub	al,128
	mov	ch,al
	movsx	eax,al
	imul	eax,KOEF_V2R
	mov	[sValV_R],eax
	mov	ebx,eax

	; R11 / R12
	mov	dh,[esi]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+3],ah
	xor	edx,edx

	; U -> sValU_B
	mov	ebx,[u]
	mov	al,[ebx]
	inc	ebx
	mov	[u],ebx
	sub	al,128
	mov	cl,al
	movsx	eax,al
	imul	eax,KOEF_U2B
	mov	[sValU_B],eax
	mov	ebx,eax

	; B11 / B12
	mov	dh,[esi]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah
	xor	edx,edx

	mov	eax,ebx
	mov	dh,[esi+1]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+5],ah
	xor	edx,edx

	; Green -> sValUV_G
	movsx	eax,ch
	movsx	ecx,cl
	imul	ecx,-KOEF_U2G
	imul	eax,-KOEF_V2G
	add	eax,ecx
	mov	[sValUV_G],eax
	mov	ebx,eax

	; G11 / G12
	mov	dh,[esi]
	add	eax,edx
	mov	ecx,eax
	sar	eax,24
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+1],ah

	mov	dh,[esi+1]
	mov	eax,ebx
	add	eax,edx
	mov	ecx,eax
	sar	eax,24
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+4],ah

	; second row of tail
	mov	eax,[LineWidth]
	add	esi,eax
	mov	eax,[PixelCount3]
	add	edi,eax
	xor	edx,edx

	; R21 / R22
	mov	eax,[sValV_R]
	mov	dh,[esi]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi],ah
	xor	edx,edx

	mov	eax,[sValV_R]
	mov	dh,[esi+1]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+3],ah
	xor	edx,edx

	; B21 / B22
	mov	eax,[sValU_B]
	mov	dh,[esi]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+2],ah
	xor	edx,edx

	mov	eax,[sValU_B]
	mov	dh,[esi+1]
	add	eax,edx
	mov	edx,eax
	sar	eax,24
	not	eax
	test	edx,0FFFF0000h
	cmovz	ax,dx
	mov	[edi+5],ah
	xor	edx,edx

	; G21 / G22
	mov	eax,[sValUV_G]
	mov	dh,[esi]
	add	eax,edx
	mov	ecx,eax
	sar	eax,24
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+1],ah

	mov	eax,[sValUV_G]
	mov	dh,[esi+1]
	add	eax,edx
	mov	ecx,eax
	sar	eax,24
	not	eax
	test	ecx,0FFFF0000h
	cmovz	ax,cx
	mov	[edi+4],ah

Exit:
	ret
PelYUV_RGB24_CMOV_SSE endp


	end