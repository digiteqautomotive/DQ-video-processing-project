;***************************************************************************************
;*                       (c)2025 Digiteq Automotive - Jaroslav Fojtik
;****************************************************************************************/

.486              ;Target processor.  Use instructions for Pentium class machines
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.


;void ScaleRowAsmARGB32(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmARGB32
ScaleRowAsmARGB32 proc \
        uses edi esi ebx \
        pDst:ptr byte, \
        _widthOut:DWORD, \
        pSrcRows:DWORD, \
        _widthIn:ptr byte
local	OutCouter: DWORD
local	StopPtr: DWORD
local	RestDDA: DWORD

	clc
	mov	edi,pDst	; pDst edi=destination pointer
	or	edi,edi
	jz	toend		; NULL pointer test

	mov	eax,_widthOut
	or	eax,eax
	jz	toend
	mov	OutCouter,eax

	mov	ebx,pSrcRows
	or	ebx,ebx
	jz	toend
	mov	ecx,[ebx]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+4]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+8]
	or	ecx,ecx
	jz	toend

	mov	eax,_widthIn
	or	eax,eax
	jz	toend
	dec	eax
	shl	eax,2		; 4*with_in
	add	eax,[ecx]	; END PTR
	mov	StopPtr,EAX

	xor	edx,edx
LoopPix0:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]		; Middle line
	mov	edx,[ebx+8]		; Bottom line
	mov	ebx,[ebx]		; Top line

	mov	esi,0
	sub	OutCouter,1
	jc	toend			; <0
	cmp	StopPtr,edx
	jge	LastColumn0
	mov	esi,4
LastColumn0:
			; R compound
	movzx	ax, byte ptr[ecx]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx]	; left top point
	adc	ah,0
	add	al, byte ptr[edx]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0
	shr	ax,4
	stosb

		; G compound
	inc	esi
	movzx	ax, byte ptr[ecx+1]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx+1]	; left top point
	adc	ah,0
	add	al, byte ptr[edx+1]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx+1]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0
	shr	ax,4
	stosb

		; B compound
	inc	esi
	movzx	ax, byte ptr[ecx+2]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx+2]	; left top point
	adc	ah,0
	add	al, byte ptr[edx+2]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx+2]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0	
	shr	ax,4
	stosb

		; A compound
	inc	esi
	movzx	ax, byte ptr[ecx+3]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx+3]	; left top point
	adc	ah,0
	add	al, byte ptr[edx+3]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx+3]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0
	shr	ax,4
	stosb

		; DDA integer only algorithm
	mov	eax,RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
		;edx already set	accuX = accuX % _widthOut;
	or	eax,eax			; zet Z flag
	jz	LoopPix0
	dec	eax
	jmp	Col1andUp


;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]
	add	ecx,eax
	mov	[ebx+4],ecx
	mov	edx,[ebx+8]
	add	edx,eax
	mov	[ebx+8],edx
	add	eax,[ebx]
	mov	[ebx],eax
	mov	ebx,eax

	mov	esi,4
	cmp	StopPtr,edx
	jge	LastColumn
	mov	esi,8
LastColumn:
		; R compound
	movzx	ax, byte ptr[ecx+4]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+4]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+4]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0
	shr	ax,4
	stosb

		; G compound
	inc	esi
	movzx	ax, byte ptr[ecx+5]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx+1]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx+1]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx+1]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+5]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+5]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0
	shr	ax,4
	stosb

		; B compound
	inc	esi
	movzx	ax, byte ptr[ecx+6]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx+2]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx+2]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx+2]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+6]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+6]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0	
	shr	ax,4
	stosb

		; A compound
	inc	esi
	movzx	ax, byte ptr[ecx+7]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx+3]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx+3]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx+3]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+7]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+7]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0
	shr	ax,4
	stosb

			; DDA integer only algorithm
	mov	eax,RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
	;edx already set	accuX = accuX % _widthOut;
Col1andUp:
	shl	eax,2		; *4

NoIncSrc:sub	OutCouter,1
	jnc	LoopCol1


toend:
        ret                     ; _cdecl return

ScaleRowAsmARGB32 endp


;void ScaleRowAsmRGB32(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmRGB32
ScaleRowAsmRGB32 proc \
        uses edi esi ebx \
        pDst:ptr byte, \
        _widthOut:DWORD, \
        pSrcRows:DWORD, \
        _widthIn:ptr byte
local	OutCouter: DWORD
local	StopPtr: DWORD
local	RestDDA: DWORD

	clc
	mov	edi,pDst	; pDst edi=destination pointer
	or	edi,edi
	jz	toend		; NULL pointer test

	mov	eax,_widthOut
	or	eax,eax
	jz	toend
	mov	OutCouter,eax

	mov	ebx,pSrcRows
	or	ebx,ebx
	jz	toend
	mov	ecx,[ebx]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+4]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+8]
	or	ecx,ecx
	jz	toend

	mov	eax,_widthIn
	or	eax,eax
	jz	toend
	dec	eax
	shl	eax,2		; 4*with_in
	add	eax,[ecx]	; END PTR
	mov	StopPtr,EAX

	xor	edx,edx
LoopPix0:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]		; Middle line
	mov	edx,[ebx+8]		; Bottom line
	mov	ebx,[ebx]		; Top line

	mov	esi,0
	sub	OutCouter,1
	jc	toend			; <0
	cmp	StopPtr,edx
	jge	LastColumn0
	mov	esi,4
LastColumn0:
			; R compound
	movzx	ax, byte ptr[ecx]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx]	; left top point
	adc	ah,0
	add	al, byte ptr[edx]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0
	shr	ax,4
	stosb

		; G compound
	inc	esi
	movzx	ax, byte ptr[ecx+1]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx+1]	; left top point
	adc	ah,0
	add	al, byte ptr[edx+1]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx+1]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0
	shr	ax,4
	stosb

		; B compound
	inc	esi
	movzx	ax, byte ptr[ecx+2]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx+2]	; left top point
	adc	ah,0
	add	al, byte ptr[edx+2]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx+2]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0	
	shr	ax,4
	stosb

		; A compound
	mov	al, byte ptr[ecx+3]
	stosb

		; DDA integer only algorithm
	mov	eax,RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
		;edx already set	accuX = accuX % _widthOut;
	or	eax,eax			; zet Z flag
	jz	LoopPix0
	dec	eax
	jmp	Col1andUp


;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]
	add	ecx,eax
	mov	[ebx+4],ecx
	mov	edx,[ebx+8]
	add	edx,eax
	mov	[ebx+8],edx
	add	eax,[ebx]
	mov	[ebx],eax
	mov	ebx,eax

	mov	esi,4
	cmp	StopPtr,edx
	jge	LastColumn
	mov	esi,8
LastColumn:
		; R compound
	movzx	ax, byte ptr[ecx+4]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+4]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+4]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0
	shr	ax,4
	stosb

		; G compound
	inc	esi
	movzx	ax, byte ptr[ecx+5]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx+1]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx+1]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx+1]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+5]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+5]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0
	shr	ax,4
	stosb

		; B compound
	inc	esi
	movzx	ax, byte ptr[ecx+6]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx+2]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx+2]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx+2]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+6]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+6]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0	
	shr	ax,4
	stosb

		; A compound
	mov	al, byte ptr[ecx+7]
	stosb

			; DDA integer only algorithm
	mov	eax,RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
	;edx already set	accuX = accuX % _widthOut;
Col1andUp:
	shl	eax,2		; *4

NoIncSrc:sub	OutCouter,1
	jnc	LoopCol1


toend:
        ret                     ; _cdecl return

ScaleRowAsmRGB32 endp


;==========================================================================================


;void ScaleRowAsmRGB24(void *pDst, unsigned _widthOut, void **pSrcRows, unsigned _widthIn);
        public  ScaleRowAsmRGB24
ScaleRowAsmRGB24 proc \
        uses edi esi ebx \
        pDst:ptr byte, \
        _widthOut:DWORD, \
        pSrcRows:DWORD, \
        _widthIn:ptr byte
local	OutCouter: DWORD
local	StopPtr: DWORD
local	RestDDA: DWORD

	clc
	mov	edi,pDst	; pDst edi=destination pointer
	or	edi,edi
	jz	toend		; NULL pointer test

	mov	eax,_widthOut
	or	eax,eax
	jz	toend
	mov	OutCouter,eax

	mov	ebx,pSrcRows
	or	ebx,ebx
	jz	toend
	mov	ecx,[ebx]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+4]
	or	ecx,ecx
	jz	toend
	mov	ecx,[ebx+8]
	or	ecx,ecx
	jz	toend

	mov	eax,_widthIn
	or	eax,eax
	jz	toend
	dec	eax
	shl	eax,2		; 4*with_in
	add	eax,[ecx]	; END PTR
	mov	StopPtr,EAX

	xor	edx,edx
LoopPix0:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]		; Middle line
	mov	edx,[ebx+8]		; Bottom line
	mov	ebx,[ebx]		; Top line

	mov	esi,0
	sub	OutCouter,1
	jc	toend			; <0
	cmp	StopPtr,edx
	jge	LastColumn0
	mov	esi,3
LastColumn0:
			; R compound
	movzx	ax, byte ptr[ecx]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx]	; left top point
	adc	ah,0
	add	al, byte ptr[edx]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0
	shr	ax,4
	stosb

		; G compound
	inc	esi
	movzx	ax, byte ptr[ecx+1]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx+1]	; left top point
	adc	ah,0
	add	al, byte ptr[edx+1]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx+1]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0
	shr	ax,4
	stosb

		; B compound
	inc	esi
	movzx	ax, byte ptr[ecx+2]
	shl	ax,2			; 2*center point
	add	al, byte ptr[ebx+2]	; left top point
	adc	ah,0
	add	al, byte ptr[edx+2]	; left bottom point
	adc	ah,0
	shl	ax,1			; 4*center point + 2*(.....)
	add	al, byte ptr[ecx+2]	; left middle point
	adc	ah,0
	add	al, byte ptr[ebx+esi]
	adc	ah,0
	add	al, byte ptr[ecx+esi]
	adc	ah,0
	add	al, byte ptr[edx+esi]
	adc	ah,0	
	shr	ax,4
	stosb

		; DDA integer only algorithm
	mov	eax,RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
		;edx already set	accuX = accuX % _widthOut;
	or	eax,eax			; zet Z flag
	jz	LoopPix0
	dec	eax
	jmp	Col1andUp


;-----------------------------------------------------------------
	; Pixel No 0 processed
LoopCol1:
	mov	RestDDA,edx

	mov	ebx,pSrcRows
	mov	ecx,[ebx+4]
	add	ecx,eax
	mov	[ebx+4],ecx
	mov	edx,[ebx+8]
	add	edx,eax
	mov	[ebx+8],edx
	add	eax,[ebx]
	mov	[ebx],eax
	mov	ebx,eax

	mov	esi,3
	cmp	StopPtr,edx
	jge	LastColumn
	mov	esi,6
LastColumn:
		; R compound
	movzx	ax, byte ptr[ecx+3]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+3]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+3]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0
	shr	ax,4
	stosb

		; G compound
	inc	esi
	movzx	ax, byte ptr[ecx+4]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx+1]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx+1]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx+1]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+4]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+4]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0
	shr	ax,4
	stosb

		; B compound
	inc	esi
	movzx	ax, byte ptr[ecx+5]
	shl	ax,3			; 4*center point
	add	al, byte ptr[ebx+2]	; left top point
	adc	ah,0
	add	al, byte ptr[ecx+2]	; left middle point
	adc	ah,0
	add	al, byte ptr[edx+2]	; left bottom point
	adc	ah,0
	add	al, byte ptr[ebx+5]	; middle top point
	adc	ah,0
	add	al, byte ptr[edx+5]	; middle bottom point
	adc	ah,0
	add	al, byte ptr[ebx+esi]	; right top point
	adc	ah,0
	add	al, byte ptr[ecx+esi]	; right middle point
	adc	ah,0
	add	al, byte ptr[edx+esi]	; right bottom point
	adc	ah,0	
	shr	ax,4
	stosb

			; DDA integer only algorithm
	mov	eax,RestDDA
	add	eax,_widthIn		;accuX += _widthIn;	
	xor	edx,edx
	div	_widthOut		;posx += accuX / _widthOut;
	;edx already set	accuX = accuX % _widthOut;
Col1andUp:
	imul	eax,eax,3		; *3

NoIncSrc:sub	OutCouter,1
	jnc	LoopCol1

toend:
        ret                     ; _cdecl return

ScaleRowAsmRGB24 endp


        end
