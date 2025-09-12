;***************************************************************************
; unit:    raster      release 0.36                                        *
; purpose: general manipulation n dimensional matrices n = 1, 2 and 3.     *
;          Use this file or rasterc.c. You cannot link both files together *
; licency:     GPL or LGPL                                                 *
; Copyright: (c) 2021-2025 Jaroslav Fojtik                                 *
;***************************************************************************

.CODE             ;Indicates the start of a code segment.


;*************************************************************************************


public  Flip24_2
Flip24_2 proc \
	uses rdi rsi
;       cols:DWORD	RCX
;       rows:DWORD	RDX
;       Data:ptr byte	R8
;       Data2:ptr byte	R9

	or	rdx,rdx
	jz	ToEnd
	jrcxz	ToEnd		; ignore value 0

	mov	rsi,R8
	add	rsi,rcx
	add	rsi,rcx
	add	rsi,rcx
	sub	rsi,3
	mov	rdi,R9

	mov	R9,rcx
	mov	R8,rdx

PIXEL:	mov	al,[rsi]	
	mov	[rdi],al
	mov	dl,[rsi+1]	
	mov	[rdi+1],dl
	mov	al,[rsi+2]	; byte 3
	mov	[rdi+2],al
	add	rdi,3
	sub	rsi,3
	dec	rcx
	jnz	PIXEL

	mov	rcx,R9
	imul	rax,R9,6
	add	rsi,rax	

	dec	R8
	jnz	PIXEL

ToEnd:
        ret                     ; _cdecl return
                
Flip24_2 endp


;*************************************************************************************


	public  Flip32_2
Flip32_2	proc \
	uses rdi rsi
;       count:DWORD	RCX
;       Data:ptr byte	RDX
;       Data2:ptr byte	R8

	or	rdx,rdx
	jz	ToEnd
	jrcxz	ToEnd		; ignore value 0

	mov	rsi,rcx
	shl	rsi,2
	add	rsi,R8
	sub	rsi,4
	mov	rdi,R9

	mov	R9,rcx
	mov	R8,rdx

PIXEL:	mov	eax,[rsi]
	sub	rsi,4
	mov	[rdi],eax
	add	rdi,4
	dec	rcx
	jnz	PIXEL

	mov	rcx,R9
	mov	rax,R9
	shl	rax,3		; *8
	add	rsi,rax	

	dec	R8
	jnz	PIXEL

ToEnd:
        ret                     ; _cdecl return                
Flip32_2	endp


;*************************************************************************************


	public  Rotate24_180
Rotate24_180	proc \
	uses rdi rsi
;	_Width:DWORD,		; RCX
;	Height:DWORD,		; RDX
;	ptr_in:ptr byte,	; R8
;       ptr_out:ptr byte	; R9

	or	rdx,rdx
	jz	ToEnd		; ignore 0 Height
	jrcxz	ToEnd		; ignore 0 Width

	mov	rax,rcx
	mul	rdx
	or	rdx,rdx
	jnz	ToEnd		; Overflow
	mov	rcx,rax		; Width*Heigh
	add	rax,rcx		; (Width*Heigh)*2
	jc	ToEnd
	add	rax,rcx		; (Width*Heigh)*3
	jc	ToEnd
	sub	rax,3		; (Width*Heigh)*3 - 3

	add	R8,rax
	mov	rsi,R8
	mov	rdi,R9

PIXEL:	mov	al,[rsi]	
	mov	dl,[rsi+1]
	mov	ah,[rsi+2]
	sub	rsi,3
	mov	[rdi],al
	mov	[rdi+1],dl
	mov	[rdi+2],ah
	add	rdi,3	
	dec	rcx
	jnz	PIXEL

ToEnd:
        ret                     ; _cdecl return                
Rotate24_180	endp

;*************************************************************************************


	public  Rotate32_90
Rotate32_90	proc \
	uses rdi rsi
;	_Width:DWORD,		; RCX
;	Height:DWORD,		; RDX
;	ptr_in:ptr byte,	; R8
;       ptr_out:ptr byte	; R9

	or	R8,R8
	jz	ToEnd
	or	R9,R9
	jz	ToEnd

	or	rdx,rdx	
	jz	ToEnd		; ignore 0 Height	
	mov	R10,rcx
	mov	R11,rcx
	mov	rax,rcx		; Width
	jrcxz	ToEnd		; ignore 0 Width

	mov	rcx,rdx

	dec	rax		; Width-1
	mul	rdx		; (Width-1)*Height
	or	rdx,rdx
	jnz	ToEnd		; Overflow
	shl	rax,1
	jc	ToEnd
	shl	rax,1		; 4*(Width-1)*Height
	jc	ToEnd
	add	rax,R9
	mov	rdi,rax		; pOutImg + 4*(Width-1)*Height;

	mov	rdx,rcx
	mov	R9,R10
	shl	R9,2		; 4*Width

LoopX:	mov	rsi,R8		; pSrc
	mov	rcx,rdx		; Height
LoopY:	mov	eax,[rsi]
	add	rsi,R9		; 4*Width
	mov	[rdi],eax
	add	rdi,4	
	dec	rcx
	jnz	LoopY
	mov	rax,rdx
	shl	rax,3		; 8*Height
	sub	rdi,rax
	add	R8,4
	dec	R11
	jnz	LoopX	

ToEnd:
        ret                     ; _cdecl return                
Rotate32_90	endp


;*************************************************************************************


	public  Rotate32_180
Rotate32_180	proc \
	uses rdi rsi
;	_Width:DWORD,		; RCX
;	Height:DWORD,		; RDX
;	ptr_in:ptr byte,	; R8
;       ptr_out:ptr byte	; R9

	or	rdx,rdx
	jz	ToEnd		; ignore 0 Height
	jrcxz	ToEnd		; ignore 0 Width

	mov	rax,rcx
	mul	rdx
	or	rdx,rdx
	jnz	ToEnd		; Overflow
	mov	rcx,rax		; Width*Heigh
	dec	rax
	shl	rax,1		; (Width*Heigh-1)*2
	jc	ToEnd
	shl	rax,1		; (Width*Heigh-1)*4
	jc	ToEnd

	add	R8,rax

PIXEL:	mov	eax,[R8]
	sub	R8,4
	mov	[R9],eax
	add	R9,4	
	dec	rcx
	jnz	PIXEL

ToEnd:
        ret                     ; _cdecl return                
Rotate32_180	endp


;*************************************************************************************


        end
