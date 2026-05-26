;*                      (c)2025-2026 Digiteq Automotive - Jaroslav Fojtik
;* This code could be redistributed under LGPL licency.
;****************************************************************************************/

.CODE _text             ;Indicates the start of a code segment.


	public  CompareRowAsmSSE2
CompareRowAsmSSE2 proc \
	uses rbx
;	_Width:DWORD,		; RCX	In1
;	Height:DWORD,		; RDX	In2
;	ptr_in:ptr byte,	; R8	Out
;       ptr_out:ptr byte	; R9	count

	or	R8,R8		; rdi=destination ptr
	jz	ToEnd
	or	rcx,rcx		; rcx=source ptr 1
	jz	ToEnd
	or	rdx,rdx		; rdx=source ptr 2
	jz	ToEnd
	sub	R9,16
	jb	Pix1_7

	pxor	xmm4,xmm4	; = 0
	mov	eax,80808080h
	movd	xmm5,eax
	punpcklbw xmm5,xmm5	; 8080808080808080h
	punpcklbw xmm5,xmm5	; 80808080808080808080808080808080h

Pix8:   movdqu	xmm0,[rcx]
	add	rcx,16
	movdqu	xmm1,[rdx]
	add	rdx,16
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
	movdqu	[R8],xmm0

	add	R8,16
	sub	R9,16
	jae	Pix8

Pix1_7:	add	R9,16
	jz	ToEnd	
	mov	R10w,00FFh
	mov	R11w,0000h
LoopPx: movzx	eax,byte ptr [rcx]
	movzx	ebx,byte ptr [rdx]
	sub	ax,bx
	inc	rcx
	inc	rdx
	add	ax,128
	cmovs	ax,R11w
	or	ah,ah
	cmovnz	ax,R10w
	mov	byte ptr [R8],al
	inc	R8
	dec	R9
	jnz	LoopPx

ToEnd:
        ret                     ; _cdecl return                
CompareRowAsmSSE2 endp



        end


