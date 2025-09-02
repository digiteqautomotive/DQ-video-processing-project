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


        end
