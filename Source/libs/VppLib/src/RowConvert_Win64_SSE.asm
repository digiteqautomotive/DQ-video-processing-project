;***************************************************************************************
;*   (c)2025-2026 Digiteq Automotive - designed by Jaroslav Fojtik All rights reserved.
;*
;*  64bit, SSE4.1 Strategy B - 4-pixel main loop + remainder-2 tail
;*  LICENSE: Software License Agreement (BSD License)
;*  Improved with assistance from xAI's Grok.
;*
;*  Four kernels, same YUV math:
;*    PelYUV_RGB24_SSE      RGB24, second row BELOW (add pitch)
;*    PelYUV_RGB24_FlipSSE  RGB24, second row ABOVE (sub pitch)
;*    PelYUV_RGB32_SSE      RGBX,  second row BELOW, packed 16-byte store
;*    PelYUV_RGB32_FlipSSE  RGBX,  second row ABOVE, packed 16-byte store
;*
;*  Y/U/V always walk forward. Only RGB row-2 direction flips.
;*  One U,V sample is shared by a 2x2 Y block (YUV420).
;*  Inner loop: 4 Y on row 1 + 4 Y on row 2 = two 2x2 pels.
;*  Width remainder 2 is handled in DoTail (never 1 or 3; width is even).
;*
;*  Saturation: psrad 8 then packusdw+packuswb (unsigned 0..255).
;*  RGB32 alpha is written as 0 (CMOV left A untouched).
;****************************************************************************************/

.CODE             ;Indicates the start of a code segment.


KOEF_U2B	EQU	520
KOEF_U2G	EQU	101
KOEF_V2R	EQU	292
KOEF_V2G	EQU	149


;void PelYUV_RGB24_SSE(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
	public PelYUV_RGB24_SSE
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB24_SSE proc \
        uses rdi rsi rbx R12,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB24_SSE proc \
        uses rdi rsi rbx R12
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+4*8]	;28h stays for 5th arg; rdi,rsi,rbx are now on the stack, i.e. 4*8
ENDIF
	sub	rsp,32
	movdqu	[rsp],xmm6
	movdqu	[rsp+16],xmm7		; Visual studio ABI needs to preserve xmm6 and xmm7.

	mov	R10,R12			; [PixelCount]
	lea	rcx,[R12+R12*2]		; [PixelCount3]

			; xmm7 is constant in whole function	
	mov	eax,KOEF_V2R
	movd	xmm7,eax
	pshufd	xmm7,xmm7,0
	
	pxor	xmm6,xmm6		; Zero	

	sub	R12,4
	jb	DoTail

LoopPel:
			; reload constants every iteration (overwritten by temporaries)
	mov	eax,-KOEF_U2G
	movd	xmm4,eax
	pshufd	xmm4,xmm4,0

	mov	eax,-KOEF_V2G
	movd	xmm5,eax
	pshufd	xmm5,xmm5,0

			; load 2 signed V (sub-128 done on AL before sign-extend)
	mov	ax,word ptr [R9]		;[v]
	sub	al,128
	sub	ah,128
	movd	xmm0,eax
	punpcklbw xmm0, xmm0	
	add	R9,2				;[v]
	punpcklwd xmm0, xmm0
	punpckldq xmm0, xmm6
	psrad xmm0, 24 

			; load 2 signed U
	mov	ax,word ptr [R8]		;[u]
	sub	al,128
	sub	ah,128
	movd	xmm1,eax
	punpcklbw xmm1, xmm1	
	add	R8,2				; [u]
	punpcklwd xmm1, xmm1
	punpckldq xmm1, xmm6
	psrad	xmm1, 24

	mov	eax,KOEF_U2B			; xmm6 used only in this local block
	movd	xmm6,eax
	pshufd	xmm6,xmm6,0

	movdqa	xmm3,xmm0			; keep originals for green
	movdqa	xmm2,xmm1

	pmulld	xmm0,xmm7			; V*KOEF_V2R  ValV_R
	pmulld	xmm1,xmm6			; U*KOEF_U2B  ValU_B

	pmulld	xmm3,xmm5			; V * -KOEF_V2G
	pmulld	xmm2,xmm4			; U * -KOEF_U2G
	paddd	xmm3,xmm2			; (V*-KOEF_V2G)+(U*-KOEF_U2G)

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
	pxor	xmm6,xmm6		; Zero
	mov	eax,[RSI]
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm4,xmm2
	paddd	xmm4,xmm0		; R
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1		; B
	paddd	xmm2,xmm3		; G

			; store first row (4 pixels) with classic CMOV saturation
	psrad	xmm4, 8			; Divide all by 256 with signed preserved
	;pmaxsd	xmm4, xmm6		; Remove sign on 0 saturation clamp.
	packusdw xmm4, xmm4		; Pack with unsigned saturation; SSE4.1!
	packuswb xmm4, xmm4
	movd	eax,xmm4
	mov	[RDI],al		; R11	 
	mov	[RDI+3],ah		; R21
	shr	eax,16
	mov	[RDI+6],al		; R31	
	mov	[RDI+9],ah		; R41

	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2
	movd	eax,xmm2
	mov	[RDI+1],al		; G11
	mov	[RDI+4],ah		; G21
	shr	eax,16
	mov	[RDI+7],al		; G31
	mov	[RDI+10],ah		; G41

	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5
	movd	eax,xmm5
	mov	[RDI+2],al		; G11
	mov	[RDI+5],ah		; G21
	shr	eax,16
	mov	[RDI+8],al		; G31
	mov	[RDI+11],ah		; G41

			; second-row Y
	mov	rax,R10			; [PixelCount]
	pxor	xmm4,xmm4
	mov	eax,[RSI+rax]	

	movd	xmm2,eax
	punpcklbw xmm2,xmm4
	psllw	xmm2,8
	punpcklwd xmm2,xmm4

	movdqa	xmm4,xmm2	; Y
	paddd	xmm4,xmm0
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1
	paddd	xmm2,xmm3

	add	RDI,rcx			; [PixelCount3]

			; store second row (identical saturation)
	psrad	xmm4, 8			; Divide all by 256 with signed preserved
	;pmaxsd	xmm4, xmm6		; Remove sign on 0 saturation clamp.
	packusdw xmm4, xmm4		; Pack with unsigned saturation; SSE4.1!
	packuswb xmm4, xmm4
	movd	eax,xmm4
	mov	[RDI],al		; R12
	mov	[RDI+3],ah		; R22
	shr	eax,16
	mov	[RDI+6],al		; R32
	mov	[RDI+9],ah		; R42

	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2
	movd	eax,xmm2
	mov	[RDI+1],al		; G12
	mov	[RDI+4],ah		; G22
	shr	eax,16
	mov	[RDI+7],al		; G32
	mov	[RDI+10],ah		; G42

	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5
	movd	eax,xmm5
	mov	[RDI+2],al		; G12
	mov	[RDI+5],ah		; G22
	shr	eax,16
	mov	[RDI+8],al		; G32
	mov	[RDI+11],ah		; G42

			; advance to next group of 4 (net +4 on first-row Y)
	sub	RDI,rcx			; [PixelCount3]
	add	RSI,4
	add	RDI,12

	sub	R12,4
	jae	LoopPel

DoTail:
	add	R12,4
	jz	Exit

	; ----- scalar tail for remaining 2 pixels (exact original CMOV path) -----

	; xmm7 = KOEF_V2R broadcast, xmm6 = 0
	; V
	mov	al,byte ptr [R9]	; [v]
	sub	al,128
	movsx	eax,al
	movd	xmm0,eax
	movdqa	xmm5,xmm0		; signed V
	pmulld	xmm0,xmm7		; ValV_R
	pshufd	xmm0,xmm0,0

	; U
	mov	al,byte ptr [R8]	; [u]
	sub	al,128
	movsx	eax,al
	movd	xmm1,eax
	movdqa	xmm4,xmm1		; signed U
	mov	eax,KOEF_U2B
	movd	xmm3,eax
	pmulld	xmm1,xmm3		; ValU_B
	pshufd	xmm1,xmm1,0

	; ValUV_G = V*-KOEF_V2G + U*-KOEF_U2G
	mov	eax,-KOEF_V2G
	movd	xmm3,eax
	pmulld	xmm5,xmm3
	mov	eax,-KOEF_U2G
	movd	xmm3,eax
	pmulld	xmm4,xmm3
	paddd	xmm5,xmm4
	pshufd	xmm5,xmm5,0

	; Y11 Y12 Y21 Y22  ->  dwords << 8
	movzx	eax,word ptr [RSI]
	movzx	edx,word ptr [RSI+R10]	;[PixelCount]
	shl	edx,16
	or	eax,edx
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm3,xmm2
	paddd	xmm3,xmm0		; R
	movdqa	xmm4,xmm2
	paddd	xmm4,xmm1		; B
	paddd	xmm2,xmm5		; G

	psrad	xmm3,8
	packusdw xmm3,xmm3
	packuswb xmm3,xmm3
	movd	eax,xmm3
	mov	[RDI],al		; R11
	mov	[RDI+3],ah		; R12
	shr	eax,16
	mov	[RDI+rcx],al		; R21
	mov	[RDI+rcx+3],ah		; R22

	psrad	xmm2,8
	packusdw xmm2,xmm2
	packuswb xmm2,xmm2
	movd	eax,xmm2
	mov	[RDI+1],al		; G11
	mov	[RDI+4],ah		; G12
	shr	eax,16
	mov	[RDI+rcx+1],al		; G21
	mov	[RDI+rcx+4],ah		; G22

	psrad	xmm4,8
	packusdw xmm4,xmm4
	packuswb xmm4,xmm4
	movd	eax,xmm4
	mov	[RDI+2],al		; B11
	mov	[RDI+5],ah		; B12
	shr	eax,16
	mov	[RDI+rcx+2],al		; B21
	mov	[RDI+rcx+5],ah		; B22

Exit:
	movdqu	xmm6,[rsp]
	movdqu	xmm7,[rsp+16]
	add	rsp,32
	ret
PelYUV_RGB24_SSE endp


;void PelYUV_RGB32_SSE(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
; Same YUV math as PelYUV_RGB24_SSE. Output is RGBX (R,G,B,A=0), 4 bytes/pixel.
; 4 pixels = 16 bytes -> one movdqu instead of 12 RGB24 byte stores.
; Row pitch = PixelCount*4. Advance after group = +16.
	public PelYUV_RGB32_SSE
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB32_SSE proc \
        uses rdi rsi rbx R12,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB32_SSE proc \
        uses rdi rsi rbx R12
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+4*8]	;28h stays for 5th arg; rdi,rsi,rbx,R12 are now on the stack, i.e. 4*8
ENDIF
	sub	rsp,32
	movdqu	[rsp],xmm6
	movdqu	[rsp+16],xmm7		; Visual studio ABI needs to preserve xmm6 and xmm7.

	mov	R10,R12			; [PixelCount]
	lea	rcx,[R12*4]		; [PixelCount4] RGB32 row pitch

			; xmm7 is constant in whole function
	mov	eax,KOEF_V2R
	movd	xmm7,eax
	pshufd	xmm7,xmm7,0

	pxor	xmm6,xmm6		; Zero, also A=0

	sub	R12,4
	jb	DoTail

LoopPel:
			; reload constants every iteration (overwritten by temporaries)
	mov	eax,-KOEF_U2G
	movd	xmm4,eax
	pshufd	xmm4,xmm4,0

	mov	eax,-KOEF_V2G
	movd	xmm5,eax
	pshufd	xmm5,xmm5,0

			; load 2 signed V (sub-128 done on AL before sign-extend)
	mov	ax,word ptr [R9]		;[v]
	sub	al,128
	sub	ah,128
	movd	xmm0,eax
	punpcklbw xmm0, xmm0
	add	R9,2				;[v]
	punpcklwd xmm0, xmm0
	punpckldq xmm0, xmm6
	psrad xmm0, 24

			; load 2 signed U
	mov	ax,word ptr [R8]		;[u]
	sub	al,128
	sub	ah,128
	movd	xmm1,eax
	punpcklbw xmm1, xmm1
	add	R8,2				; [u]
	punpcklwd xmm1, xmm1
	punpckldq xmm1, xmm6
	psrad	xmm1, 24

	mov	eax,KOEF_U2B			; xmm6 used only in this local block
	movd	xmm6,eax
	pshufd	xmm6,xmm6,0

	movdqa	xmm3,xmm0			; keep originals for green
	movdqa	xmm2,xmm1

	pmulld	xmm0,xmm7			; V*KOEF_V2R  ValV_R
	pmulld	xmm1,xmm6			; U*KOEF_U2B  ValU_B

	pmulld	xmm3,xmm5			; V * -KOEF_V2G
	pmulld	xmm2,xmm4			; U * -KOEF_U2G
	paddd	xmm3,xmm2			; (V*-KOEF_V2G)+(U*-KOEF_U2G)

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
	pxor	xmm6,xmm6		; Zero
	mov	eax,[RSI]
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm4,xmm2
	paddd	xmm4,xmm0		; R
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1		; B
	paddd	xmm2,xmm3		; G

			; pack R,G,B then interleave RGBX (A=0) and store 16 bytes
	psrad	xmm4, 8
	packusdw xmm4, xmm4
	packuswb xmm4, xmm4		; R0 R1 R2 R3
	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2		; G0 G1 G2 G3
	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5		; B0 B1 B2 B3

	punpcklbw xmm4,xmm2		; R0 G0 R1 G1 ...
	punpcklbw xmm5,xmm6		; B0 A0 B1 A0 ...  A=0
	punpcklwd xmm4,xmm5		; RGBX x4
	movdqu	[RDI],xmm4

			; second-row Y
	mov	rax,R10			; [PixelCount]
	pxor	xmm4,xmm4
	mov	eax,[RSI+rax]

	movd	xmm2,eax
	punpcklbw xmm2,xmm4
	psllw	xmm2,8
	punpcklwd xmm2,xmm4

	movdqa	xmm4,xmm2		; Y
	paddd	xmm4,xmm0
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1
	paddd	xmm2,xmm3

	add	RDI,rcx			; [PixelCount4] second RGB row BELOW

	psrad	xmm4, 8
	packusdw xmm4, xmm4
	packuswb xmm4, xmm4
	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2
	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5

	punpcklbw xmm4,xmm2
	punpcklbw xmm5,xmm6
	punpcklwd xmm4,xmm5
	movdqu	[RDI],xmm4

			; restore first-row rdi, then +4 Y / +16 RGB
	sub	RDI,rcx			; [PixelCount4]
	add	RSI,4
	add	RDI,16			; 4 pixels * 4 bytes

	sub	R12,4
	jae	LoopPel

DoTail:
	add	R12,4
	jz	Exit

	; ----- remainder 2 pixels -----
	; lanes after pack: [Y11 Y12 Y21 Y22] -> low 8 bytes row1 RGBX, high row2
	; xmm7 = KOEF_V2R broadcast, xmm6 = 0
	; V
	mov	al,byte ptr [R9]	; [v]
	sub	al,128
	movsx	eax,al
	movd	xmm0,eax
	movdqa	xmm5,xmm0		; signed V
	pmulld	xmm0,xmm7		; ValV_R
	pshufd	xmm0,xmm0,0

	; U
	mov	al,byte ptr [R8]	; [u]
	sub	al,128
	movsx	eax,al
	movd	xmm1,eax
	movdqa	xmm4,xmm1		; signed U
	mov	eax,KOEF_U2B
	movd	xmm3,eax
	pmulld	xmm1,xmm3		; ValU_B
	pshufd	xmm1,xmm1,0

	; ValUV_G = V*-KOEF_V2G + U*-KOEF_U2G
	mov	eax,-KOEF_V2G
	movd	xmm3,eax
	pmulld	xmm5,xmm3
	mov	eax,-KOEF_U2G
	movd	xmm3,eax
	pmulld	xmm4,xmm3
	paddd	xmm5,xmm4
	pshufd	xmm5,xmm5,0

	; Y11 Y12 Y21 Y22  ->  dwords << 8
	movzx	eax,word ptr [RSI]
	movzx	edx,word ptr [RSI+R10]	;[PixelCount]
	shl	edx,16
	or	eax,edx
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm3,xmm2
	paddd	xmm3,xmm0		; R
	movdqa	xmm4,xmm2
	paddd	xmm4,xmm1		; B
	paddd	xmm2,xmm5		; G

	psrad	xmm3,8
	packusdw xmm3,xmm3
	packuswb xmm3,xmm3
	psrad	xmm2,8
	packusdw xmm2,xmm2
	packuswb xmm2,xmm2
	psrad	xmm4,8
	packusdw xmm4,xmm4
	packuswb xmm4,xmm4

	punpcklbw xmm3,xmm2		; RG
	punpcklbw xmm4,xmm6		; BA  A=0
	punpcklwd xmm3,xmm4		; row1 RGBX | row2 RGBX

	movq	qword ptr [RDI],xmm3		; 2 px first row
	psrldq	xmm3,8
	movq	qword ptr [RDI+rcx],xmm3	; 2 px second row BELOW [PixelCount4]

Exit:
	movdqu	xmm6,[rsp]
	movdqu	xmm7,[rsp+16]
	add	rsp,32
	ret
PelYUV_RGB32_SSE endp


;void PelYUV_RGB24_FlipSSE(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
; Same math as PelYUV_RGB24_SSE.
; Caller points rdi at the BOTTOM RGB row of the 2-line pair.
; Y row 1 (rsi) is stored there; Y row 2 is stored one RGB row UP
; (sub PixelCount3). Y/U/V still walk forward.
	public PelYUV_RGB24_FlipSSE
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB24_FlipSSE proc \
        uses rdi rsi rbx R12,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB24_FlipSSE proc \
        uses rdi rsi rbx R12
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+4*8]	;28h stays for 5th arg; rdi,rsi,rbx,R12 are now on the stack, i.e. 4*8
ENDIF
	sub	rsp,32
	movdqu	[rsp],xmm6
	movdqu	[rsp+16],xmm7		; Visual studio ABI needs to preserve xmm6 and xmm7.

	mov	R10,R12			; [PixelCount]
	lea	rcx,[R12+R12*2]		; [PixelCount3]

			; xmm7 is constant in whole function
	mov	eax,KOEF_V2R
	movd	xmm7,eax
	pshufd	xmm7,xmm7,0

	pxor	xmm6,xmm6		; Zero

	sub	R12,4
	jb	DoTail

LoopPel:
			; reload constants every iteration (overwritten by temporaries)
	mov	eax,-KOEF_U2G
	movd	xmm4,eax
	pshufd	xmm4,xmm4,0

	mov	eax,-KOEF_V2G
	movd	xmm5,eax
	pshufd	xmm5,xmm5,0

			; load 2 signed V (sub-128 done on AL before sign-extend)
	mov	ax,word ptr [R9]		;[v]
	sub	al,128
	sub	ah,128
	movd	xmm0,eax
	punpcklbw xmm0, xmm0
	add	R9,2				;[v]
	punpcklwd xmm0, xmm0
	punpckldq xmm0, xmm6
	psrad xmm0, 24

			; load 2 signed U
	mov	ax,word ptr [R8]		;[u]
	sub	al,128
	sub	ah,128
	movd	xmm1,eax
	punpcklbw xmm1, xmm1
	add	R8,2				; [u]
	punpcklwd xmm1, xmm1
	punpckldq xmm1, xmm6
	psrad	xmm1, 24

	mov	eax,KOEF_U2B			; xmm6 used only in this local block
	movd	xmm6,eax
	pshufd	xmm6,xmm6,0

	movdqa	xmm3,xmm0			; keep originals for green
	movdqa	xmm2,xmm1

	pmulld	xmm0,xmm7			; V*KOEF_V2R  ValV_R
	pmulld	xmm1,xmm6			; U*KOEF_U2B  ValU_B

	pmulld	xmm3,xmm5			; V * -KOEF_V2G
	pmulld	xmm2,xmm4			; U * -KOEF_U2G
	paddd	xmm3,xmm2			; (V*-KOEF_V2G)+(U*-KOEF_U2G)

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

			; first-row Y << 8 (BOTTOM RGB row of the pair)
	pxor	xmm6,xmm6		; Zero
	mov	eax,[RSI]
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm4,xmm2
	paddd	xmm4,xmm0		; R
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1		; B
	paddd	xmm2,xmm3		; G

			; store first row (4 pixels) packus saturation, RGB24 scatter
	psrad	xmm4, 8
	packusdw xmm4, xmm4
	packuswb xmm4, xmm4
	movd	eax,xmm4
	mov	[RDI],al		; R11
	mov	[RDI+3],ah		; R12
	shr	eax,16
	mov	[RDI+6],al		; R13
	mov	[RDI+9],ah		; R14

	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2
	movd	eax,xmm2
	mov	[RDI+1],al		; G11
	mov	[RDI+4],ah		; G12
	shr	eax,16
	mov	[RDI+7],al		; G13
	mov	[RDI+10],ah		; G14

	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5
	movd	eax,xmm5
	mov	[RDI+2],al		; B11
	mov	[RDI+5],ah		; B12
	shr	eax,16
	mov	[RDI+8],al		; B13
	mov	[RDI+11],ah		; B14

			; second-row Y, RGB one row UP
	mov	rax,R10			; [PixelCount]
	pxor	xmm4,xmm4
	mov	eax,[RSI+rax]

	movd	xmm2,eax
	punpcklbw xmm2,xmm4
	psllw	xmm2,8
	punpcklwd xmm2,xmm4

	movdqa	xmm4,xmm2		; Y
	paddd	xmm4,xmm0
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1
	paddd	xmm2,xmm3

	sub	RDI,rcx			; [PixelCount3] second RGB row UP

	psrad	xmm4, 8
	packusdw xmm4, xmm4
	packuswb xmm4, xmm4
	movd	eax,xmm4
	mov	[RDI],al		; R21
	mov	[RDI+3],ah		; R22
	shr	eax,16
	mov	[RDI+6],al		; R23
	mov	[RDI+9],ah		; R24

	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2
	movd	eax,xmm2
	mov	[RDI+1],al		; G21
	mov	[RDI+4],ah		; G22
	shr	eax,16
	mov	[RDI+7],al		; G23
	mov	[RDI+10],ah		; G24

	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5
	movd	eax,xmm5
	mov	[RDI+2],al		; B21
	mov	[RDI+5],ah		; B22
	shr	eax,16
	mov	[RDI+8],al		; B23
	mov	[RDI+11],ah		; B24

			; restore bottom-row rdi, then +4 Y / +12 RGB
	add	RDI,rcx			; [PixelCount3]
	add	RSI,4
	add	RDI,12

	sub	R12,4
	jae	LoopPel

DoTail:
	add	R12,4
	jz	Exit

	; ----- remainder 2 pixels -----
	; xmm7 = KOEF_V2R broadcast, xmm6 = 0
	; V
	mov	al,byte ptr [R9]	; [v]
	sub	al,128
	movsx	eax,al
	movd	xmm0,eax
	movdqa	xmm5,xmm0		; signed V
	pmulld	xmm0,xmm7		; ValV_R
	pshufd	xmm0,xmm0,0

	; U
	mov	al,byte ptr [R8]	; [u]
	sub	al,128
	movsx	eax,al
	movd	xmm1,eax
	movdqa	xmm4,xmm1		; signed U
	mov	eax,KOEF_U2B
	movd	xmm3,eax
	pmulld	xmm1,xmm3		; ValU_B
	pshufd	xmm1,xmm1,0

	; ValUV_G = V*-KOEF_V2G + U*-KOEF_U2G
	mov	eax,-KOEF_V2G
	movd	xmm3,eax
	pmulld	xmm5,xmm3
	mov	eax,-KOEF_U2G
	movd	xmm3,eax
	pmulld	xmm4,xmm3
	paddd	xmm5,xmm4
	pshufd	xmm5,xmm5,0

	; Y11 Y12 Y21 Y22  ->  dwords << 8
	movzx	eax,word ptr [RSI]
	movzx	edx,word ptr [RSI+R10]	;[PixelCount]
	shl	edx,16
	or	eax,edx
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm3,xmm2
	paddd	xmm3,xmm0		; R
	movdqa	xmm4,xmm2
	paddd	xmm4,xmm1		; B
	paddd	xmm2,xmm5		; G

	neg	rcx			; rdi - PixelCount3 (row UP)

	psrad	xmm3,8
	packusdw xmm3,xmm3
	packuswb xmm3,xmm3
	movd	eax,xmm3
	mov	[RDI],al		; R11
	mov	[RDI+3],ah		; R12
	shr	eax,16
	mov	[RDI+rcx],al		; R21 (row UP)
	mov	[RDI+rcx+3],ah		; R22

	psrad	xmm2,8
	packusdw xmm2,xmm2
	packuswb xmm2,xmm2
	movd	eax,xmm2
	mov	[RDI+1],al		; G11
	mov	[RDI+4],ah		; G12
	shr	eax,16
	mov	[RDI+rcx+1],al		; G21
	mov	[RDI+rcx+4],ah		; G22

	psrad	xmm4,8
	packusdw xmm4,xmm4
	packuswb xmm4,xmm4
	movd	eax,xmm4
	mov	[RDI+2],al		; B11
	mov	[RDI+5],ah		; B12
	shr	eax,16
	mov	[RDI+rcx+2],al		; B21
	mov	[RDI+rcx+5],ah		; B22

Exit:
	movdqu	xmm6,[rsp]
	movdqu	xmm7,[rsp+16]
	add	rsp,32
	ret
PelYUV_RGB24_FlipSSE endp


;void PelYUV_RGB32_FlipSSE(unsigned char *OutData, const unsigned char *y, const unsigned char *u, const unsigned char *v, unsigned PixelCount)
; RGB32 packed stores like PelYUV_RGB32_SSE, row direction like Flip RGB24:
; rdi is BOTTOM RGB row; second Y row is stored one pitch UP (sub PixelCount4).
	public PelYUV_RGB32_FlipSSE
IF (@Version LT 1200) OR (@Version GE 1300)
PelYUV_RGB32_FlipSSE proc \
        uses rdi rsi rbx R12,
        OutData: ptr BYTE,
        y: ptr BYTE,
        u: ptr BYTE,		; R8
	v: ptr BYTE,		; R9
	PixelCount: DWORD	; on stack
ELSE
; The ASM from MSVC2013 seems to be much more worse than previous one. Use blind arguments.
PelYUV_RGB32_FlipSSE proc \
        uses rdi rsi rbx R12
ENDIF
	mov	rdi,rcx		; [OutData]
	mov	rsi,rdx		; [y]
IF (@Version LT 1200) OR (@Version GE 1300)
	mov	R12d,PixelCount
ELSE
	mov	R12d,[rsp+28h+4*8]	;28h stays for 5th arg; rdi,rsi,rbx,R12 are now on the stack, i.e. 4*8
ENDIF
	sub	rsp,32
	movdqu	[rsp],xmm6
	movdqu	[rsp+16],xmm7		; Visual studio ABI needs to preserve xmm6 and xmm7.

	mov	R10,R12			; [PixelCount]
	lea	rcx,[R12*4]		; [PixelCount4] RGB32 row pitch

			; xmm7 is constant in whole function
	mov	eax,KOEF_V2R
	movd	xmm7,eax
	pshufd	xmm7,xmm7,0

	pxor	xmm6,xmm6		; Zero, also A=0

	sub	R12,4
	jb	DoTail

LoopPel:
			; reload constants every iteration (overwritten by temporaries)
	mov	eax,-KOEF_U2G
	movd	xmm4,eax
	pshufd	xmm4,xmm4,0

	mov	eax,-KOEF_V2G
	movd	xmm5,eax
	pshufd	xmm5,xmm5,0

			; load 2 signed V (sub-128 done on AL before sign-extend)
	mov	ax,word ptr [R9]		;[v]
	sub	al,128
	sub	ah,128
	movd	xmm0,eax
	punpcklbw xmm0, xmm0
	add	R9,2				;[v]
	punpcklwd xmm0, xmm0
	punpckldq xmm0, xmm6
	psrad xmm0, 24

			; load 2 signed U
	mov	ax,word ptr [R8]		;[u]
	sub	al,128
	sub	ah,128
	movd	xmm1,eax
	punpcklbw xmm1, xmm1
	add	R8,2				; [u]
	punpcklwd xmm1, xmm1
	punpckldq xmm1, xmm6
	psrad	xmm1, 24

	mov	eax,KOEF_U2B			; xmm6 used only in this local block
	movd	xmm6,eax
	pshufd	xmm6,xmm6,0

	movdqa	xmm3,xmm0			; keep originals for green
	movdqa	xmm2,xmm1

	pmulld	xmm0,xmm7			; V*KOEF_V2R  ValV_R
	pmulld	xmm1,xmm6			; U*KOEF_U2B  ValU_B

	pmulld	xmm3,xmm5			; V * -KOEF_V2G
	pmulld	xmm2,xmm4			; U * -KOEF_U2G
	paddd	xmm3,xmm2			; (V*-KOEF_V2G)+(U*-KOEF_U2G)

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

			; first-row Y << 8 (BOTTOM RGB row of the pair)
	pxor	xmm6,xmm6		; Zero
	mov	eax,[RSI]
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm4,xmm2
	paddd	xmm4,xmm0		; R
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1		; B
	paddd	xmm2,xmm3		; G

			; pack R,G,B then interleave RGBX (A=0) and store 16 bytes
	psrad	xmm4, 8
	packusdw xmm4, xmm4
	packuswb xmm4, xmm4		; R0 R1 R2 R3
	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2		; G0 G1 G2 G3
	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5		; B0 B1 B2 B3

	punpcklbw xmm4,xmm2		; R0 G0 R1 G1 ...
	punpcklbw xmm5,xmm6		; B0 A0 B1 A0 ...  A=0
	punpcklwd xmm4,xmm5		; RGBX x4
	movdqu	[RDI],xmm4

			; second-row Y, RGB one row UP
	mov	rax,R10			; [PixelCount]
	pxor	xmm4,xmm4
	mov	eax,[RSI+rax]

	movd	xmm2,eax
	punpcklbw xmm2,xmm4
	psllw	xmm2,8
	punpcklwd xmm2,xmm4

	movdqa	xmm4,xmm2		; Y
	paddd	xmm4,xmm0
	movdqa	xmm5,xmm2
	paddd	xmm5,xmm1
	paddd	xmm2,xmm3

	sub	RDI,rcx			; [PixelCount4] second RGB row UP

	psrad	xmm4, 8
	packusdw xmm4, xmm4
	packuswb xmm4, xmm4
	psrad	xmm2, 8
	packusdw xmm2, xmm2
	packuswb xmm2, xmm2
	psrad	xmm5, 8
	packusdw xmm5, xmm5
	packuswb xmm5, xmm5

	punpcklbw xmm4,xmm2
	punpcklbw xmm5,xmm6
	punpcklwd xmm4,xmm5
	movdqu	[RDI],xmm4

			; restore bottom-row rdi, then +4 Y / +16 RGB
	add	RDI,rcx			; [PixelCount4]
	add	RSI,4
	add	RDI,16			; 4 pixels * 4 bytes

	sub	R12,4
	jae	LoopPel

DoTail:
	add	R12,4
	jz	Exit

	; ----- remainder 2 pixels -----
	; lanes after pack: [Y11 Y12 Y21 Y22] -> low 8 bytes row1 RGBX, high row2
	; xmm7 = KOEF_V2R broadcast, xmm6 = 0
	; V
	mov	al,byte ptr [R9]	; [v]
	sub	al,128
	movsx	eax,al
	movd	xmm0,eax
	movdqa	xmm5,xmm0		; signed V
	pmulld	xmm0,xmm7		; ValV_R
	pshufd	xmm0,xmm0,0

	; U
	mov	al,byte ptr [R8]	; [u]
	sub	al,128
	movsx	eax,al
	movd	xmm1,eax
	movdqa	xmm4,xmm1		; signed U
	mov	eax,KOEF_U2B
	movd	xmm3,eax
	pmulld	xmm1,xmm3		; ValU_B
	pshufd	xmm1,xmm1,0

	; ValUV_G = V*-KOEF_V2G + U*-KOEF_U2G
	mov	eax,-KOEF_V2G
	movd	xmm3,eax
	pmulld	xmm5,xmm3
	mov	eax,-KOEF_U2G
	movd	xmm3,eax
	pmulld	xmm4,xmm3
	paddd	xmm5,xmm4
	pshufd	xmm5,xmm5,0

	; Y11 Y12 Y21 Y22  ->  dwords << 8
	movzx	eax,word ptr [RSI]
	movzx	edx,word ptr [RSI+R10]	;[PixelCount]
	shl	edx,16
	or	eax,edx
	movd	xmm2,eax
	punpcklbw xmm2,xmm6
	psllw	xmm2,8
	punpcklwd xmm2,xmm6

	movdqa	xmm3,xmm2
	paddd	xmm3,xmm0		; R
	movdqa	xmm4,xmm2
	paddd	xmm4,xmm1		; B
	paddd	xmm2,xmm5		; G

	psrad	xmm3,8
	packusdw xmm3,xmm3
	packuswb xmm3,xmm3
	psrad	xmm2,8
	packusdw xmm2,xmm2
	packuswb xmm2,xmm2
	psrad	xmm4,8
	packusdw xmm4,xmm4
	packuswb xmm4,xmm4

	punpcklbw xmm3,xmm2		; RG
	punpcklbw xmm4,xmm6		; BA  A=0
	punpcklwd xmm3,xmm4		; row1 RGBX | row2 RGBX

	neg	rcx			; rdi - PixelCount4 (row UP)
	movq	qword ptr [RDI],xmm3		; 2 px first row (BOTTOM)
	psrldq	xmm3,8
	movq	qword ptr [RDI+rcx],xmm3	; 2 px second row UP

Exit:
	movdqu	xmm6,[rsp]
	movdqu	xmm7,[rsp+16]
	add	rsp,32
	ret
PelYUV_RGB32_FlipSSE endp


        end
