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


        end
