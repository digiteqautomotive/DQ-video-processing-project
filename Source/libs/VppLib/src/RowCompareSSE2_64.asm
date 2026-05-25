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
	sub	R9,8
	jb	Pix1_7

	pxor	xmm3,xmm3	; = 0
	mov	eax,7F7F7F7Fh
	movd	xmm4,eax
	punpcklbw xmm4,xmm4	; 7F7F7F7F7F7F7F7Fh

Pix8:   movq	xmm0,qword ptr[rcx]
	add	rcx,8
	movq	xmm1,qword ptr[rdx]
	add	rdx,8

	punpcklbw xmm0,xmm3
	punpcklbw xmm1,xmm3
	psubsw	xmm0,xmm1
	packsswb xmm0,xmm3
	paddsb	xmm0,xmm4
	movq	qword ptr[R8],xmm0
	add	R8,8
	sub	R9,8
	jae	Pix8

Pix1_7:	add	R9,8
	jz	ToEnd	
	mov	R10w,00FFh
	mov	R11w,0000h
LoopPx: movzx	eax,byte ptr [rcx]
	movzx	ebx,byte ptr [rdx]
	sub	ax,bx
	inc	rcx
	inc	rdx
	add	ax,127
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


