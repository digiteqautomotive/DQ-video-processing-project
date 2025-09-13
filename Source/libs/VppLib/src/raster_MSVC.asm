;***************************************************************************
; unit:    raster      release 0.36                                        *
; purpose: general manipulation n dimensional matrices n = 1, 2 and 3.     *
;          Use this file or rasterc.c. You cannot link both files together *
; licency:     GPL or LGPL                                                 *
; Copyright: (c) 1998-2025 Jaroslav Fojtik                                 *
;***************************************************************************

.486              ;Target processor.  Use instructions for Pentium class machines
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.



;*************************************************************************************

;void Flip24_2(unsigned _Width, unsigned _Height, void *ptr_in, void *ptr_out);
	public  Flip24_2
Flip24_2	proc \
        uses edi esi \
        _Width:DWORD, \
	_Height:DWORD, \
	ptr_in:ptr byte, \
        ptr_out:ptr byte

        mov     esi,[ptr_in]     ; esi=source ptr
	or	esi,esi
	jz	ToEnd	
	mov     edi,[ptr_out]     ; edi=destination ptr
	or	edi,edi
	jz	ToEnd	

	mov	ecx,[_Width]
	or	ecx,ecx
	jz	ToEnd
	mov	edx,[_Height]
	or	edx,edx
	jz	ToEnd

	add	esi,ecx
	add	esi,ecx
	add	esi,ecx
	sub	esi,3
	
PIXEL:	mov	al,[esi]	
	mov	[edi],al
	mov	ah,[esi+1]	
	mov	[edi+1],ah
	mov	al,[esi+2]	; byte 3
	mov	[edi+2],al
	add	edi,3
	sub	esi,3
	dec	ecx
	jnz	PIXEL

	mov	ecx,[_Width]
	imul	eax,ecx,6
	add	esi,eax	

	dec	edx
	jnz	PIXEL		

ToEnd:
        ret                     ; _cdecl return
                
Flip24_2 endp


;*************************************************************************************

;void Flip32_2(unsigned Width, unsigned Height, void *ptr_in, void *ptr_out);
	public  Flip32_2
Flip32_2	proc \
        uses edi esi \
	_Width:DWORD, \
	Height:DWORD, \
	ptr_in:ptr byte, \
        ptr_out:ptr byte

        mov     esi,[ptr_in]     ; esi=source ptr
	or	esi,esi
	jz	ToEnd	
	mov     edi,[ptr_out]     ; edi=destination ptr
	or	edi,edi
	jz	ToEnd	

	mov	ecx,_Width
	mov	eax,ecx
	or	ecx,ecx
	jz	ToEnd
	mov	edx,Height
	or	edx,edx
	jz	ToEnd

	sub	esi,4
	shl	eax,2		; Width*4
	add	esi,eax

PIXEL:	mov	eax,[esi]	
	mov	[edi],eax
	add	edi,4
	sub	esi,4
	dec	ecx
	jnz	PIXEL

	mov	ecx,_Width
	mov	eax,ecx
	shl	eax,3		; *8
	add	esi,eax	

	dec	edx
	jnz	PIXEL

ToEnd:
        ret                     ; _cdecl return
                
Flip32_2 endp


;*************************************************************************************

;void Rotate24_180(unsigned Width, unsigned Height, void *ptr_in, void *ptr_out);
	public  Rotate24_180
Rotate24_180	proc \
	uses edi esi \
	_Width:DWORD, \
	Height:DWORD, \
	ptr_in:ptr byte, \
        ptr_out:ptr byte

        mov     esi,[ptr_in]     ; esi=source ptr
	or	esi,esi
	jz	ToEnd	
	mov     edi,[ptr_out]     ; edi=destination ptr
	or	edi,edi
	jz	ToEnd	

	mov	ecx,_Width
	mov	eax,ecx
	or	ecx,ecx
	jz	ToEnd
	mov	eax,Height
	or	eax,eax
	jz	ToEnd

	mul	ecx
	mov	ecx,eax		; Width*Heigh
	dec	eax
	add	esi,eax
	add	esi,eax
	add	esi,eax		; + (Width*Heigh-1)*3

PIXEL:	mov	al,[esi]
	mov	dl,[esi+1]
	mov	ah,[esi+2]
	sub	esi,3
	mov	[edi],al	
	mov	[edi+1],dl	
	mov	[edi+2],ah
	add	edi,3
	dec	ecx
	jnz	PIXEL

ToEnd:
        ret                     ; _cdecl return
                
Rotate24_180 endp


;*************************************************************************************


;void Rotate32_90(unsigned Width, unsigned Height, void *ptr_in, void *ptr_out);
	public  Rotate32_90
Rotate32_90	proc \
	uses ebx edi esi \
	_Width:DWORD, \
	Height:DWORD, \
	ptr_in:ptr byte, \
        ptr_out:ptr byte

        mov     esi,[ptr_in]     ; esi=source ptr
	or	esi,esi
	jz	ToEnd	
	mov     edi,[ptr_out]     ; edi=destination ptr
	or	edi,edi
	jz	ToEnd

	mov	ecx,_Width
	mov	ebx,ecx
	mov	eax,ecx
	;or	ecx,ecx
	jecxz	ToEnd
	mov	eax,Height
	or	eax,eax
	jz	ToEnd

	dec	ecx		; Heigh-1
	mul	ecx
	mov	ecx,eax		; Width*(Heigh-1)
	or	edx,edx
	jnz	ToEnd		; Overflow
	shl	eax,1		; Width*(Heigh-1)*2
	jc	ToEnd
	shl	eax,1		; Width*(Heigh-1)*4
	jc	ToEnd
	add	edi,eax

	mov	edx,_Width
	shl	edx,2		; 4*Width

LoopX:	mov	esi,[ptr_in]	; pSrc
	mov	ecx,Height
LoopY:	mov	eax,[esi]
	add	esi,edx		; 4*Width
	mov	[edi],eax
	add	edi,4
	dec	ecx
	jnz	LoopY
	mov	eax,Height
	shl	eax,3		; 8*Height
	sub	edi,eax
	add	[ptr_in],4
	dec	ebx
	jnz	LoopX

ToEnd:
        ret                     ; _cdecl return
                
Rotate32_90 endp


;*************************************************************************************

;void Rotate32_180(unsigned Width, unsigned Height, void *ptr_in, void *ptr_out);
	public  Rotate32_180
Rotate32_180	proc \
	uses edi esi \
	_Width:DWORD, \
	Height:DWORD, \
	ptr_in:ptr byte, \
        ptr_out:ptr byte

        mov     esi,[ptr_in]     ; esi=source ptr
	or	esi,esi
	jz	ToEnd	
	mov     edi,[ptr_out]     ; edi=destination ptr
	or	edi,edi
	jz	ToEnd	

	mov	ecx,_Width
	mov	eax,ecx
	or	ecx,ecx
	jz	ToEnd
	mov	eax,Height
	or	eax,eax
	jz	ToEnd

	mul	ecx
	mov	ecx,eax		; Width*Heigh
	or	edx,edx
	jnz	ToEnd		; Overflow
	dec	eax
	shl	eax,1		; (Width*Heigh-1)*2
	jc	ToEnd
	shl	eax,1		; (Width*Heigh-1)*4
	jc	ToEnd
	add	esi,eax

PIXEL:	mov	eax,[esi]
	sub	esi,4
	mov	[edi],eax
	add	edi,4	
	dec	ecx
	jnz	PIXEL

ToEnd:
        ret                     ; _cdecl return
                
Rotate32_180 endp

;*************************************************************************************


;void Rotate32_270(unsigned Width, unsigned Height, void *ptr_in, void *ptr_out);
	public  Rotate32_270
Rotate32_270	proc \
	uses ebx edi esi \
	_Width:DWORD, \
	Height:DWORD, \
	ptr_in:ptr byte, \
        ptr_out:ptr byte

        mov     esi,[ptr_in]     ; esi=source ptr
	or	esi,esi
	jz	ToEnd	
	mov     edi,[ptr_out]     ; edi=destination ptr
	or	edi,edi
	jz	ToEnd

	mov	ecx,Height
	or	ecx,ecx
	jz	ToEnd
	mov	ebx,_Width
	or	ebx,ebx
	jz	ToEnd

	mov	eax,ecx
	dec	eax		; Height-1
	mul	ebx		; Width*(Height-1)
	or	edx,edx
	jnz	ToEnd
	shl	eax,1
	jc	ToEnd
	shl	eax,1		; 4*Width*(Height-1)
	jc	ToEnd
	add	esi,eax

	mov	edx,ebx		; Width
	shl	ebx,2		; 4*Width

LoopY:	mov	[ptr_in],esi
	mov	ecx,Height
LoopX:	mov	eax,[esi]
	sub	esi,ebx		; - 4*Width
	mov	[edi],eax
	add	edi,4
	dec	ecx
	jnz	LoopX
	mov	esi,[ptr_in]
	add	esi,4	
	dec	edx
	jnz	LoopY

ToEnd:
        ret                     ; _cdecl return
                
Rotate32_270 endp


;*************************************************************************************


        end
