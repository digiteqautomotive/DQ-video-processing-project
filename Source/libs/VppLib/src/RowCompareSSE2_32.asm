;*                      (c)2025-2026 Digiteq Automotive - Jaroslav Fojtik
;* This code could be redistributed under LGPL licency.
;****************************************************************************************/

.686              ;Target processor.  Use instructions for Pentium class machines
.XMM		  ; https://learn.microsoft.com/en-us/cpp/assembler/masm/dot-xmm?view=msvc-170
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.



	public  CompareRowAsmSSE2
CompareRowAsmSSE2 proc \
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
	sub	ecx,16
	jb	Pix1_7

	pxor	xmm4,xmm4	; = 0
	mov	eax,80808080h
	movd	xmm5,eax
	punpcklbw xmm5,xmm5	; 8080808080808080h
	punpcklbw xmm5,xmm5	; 80808080808080808080808080808080h

Pix8:   movdqu	xmm0,[esi]
	add	esi,16
	movdqu	xmm1,[edx]
	add	edx,16
	movhlps xmm2,xmm0
	movhlps xmm3,xmm1

	punpcklbw xmm0,xmm4
	punpcklbw xmm1,xmm4
	psubsw	xmm0,xmm1		; 8 numbers in range <255;-255>

	punpcklbw xmm2,xmm4
	punpcklbw xmm3,xmm4
	psubsw	xmm2,xmm3		; 8 numbers in range <255;-255>

	packsswb xmm0,xmm2		; merge all 16 numbers together	
	paddb	xmm0,xmm5		; add without saturation
	movdqu	[edi],xmm0

	add	edi,16
	sub	ecx,16
	jae	Pix8

Pix1_7:	add	ecx,16
	jz	ToEnd	

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

CompareRowAsmSSE2 endp


        end
