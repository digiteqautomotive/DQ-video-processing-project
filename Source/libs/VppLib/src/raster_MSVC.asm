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
	shl	eax,1		; (Width*Heigh-1)*4
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


        end
