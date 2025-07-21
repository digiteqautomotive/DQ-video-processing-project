;***************************************************************************
; Copyright: (c) 2021-2025 Jaroslav Fojtik                                 *
;***************************************************************************

.586              ;Target processor.  Use instructions for Pentium class machines
.MMX
.MODEL FLAT, C    ;Use the flat memory model. Use C calling conventions

.CODE             ;Indicates the start of a code segment.


; https://docs.oracle.com/cd/E19120-01/open.solaris/817-5477/eojdc/index.html
; https://www.officedaytime.com/simd512e/

	public	GetFeaturesCPU
GetFeaturesCPU PROC \
        uses ebx esi
        
        mov	esi,0
        
	pushfd				; Save EFLAGS
	pushfd				; Store EFLAGS
	xor	dword ptr [esp],00200000h ; Invert the ID bit in stored EFLAGS
	popfd				; Load stored EFLAGS (with ID bit inverted)
	pushfd				; Store EFLAGS again (ID bit may or may not be inverted)
	pop	eax			; eax = modified EFLAGS (ID bit may or may not be inverted)
	xor	eax,[esp]		; eax = whichever bits were changed
	popfd				; Restore original EFLAGS
	test	eax,00200000h		; eax = zero if ID bit can't be changed, else non-zero
	jz	exit	

	mov	eax,esi
	mov	ecx,esi
	cpuid
	cmp	eax,1
	jl	exit
	
	mov	eax,1
	cpuid
	
	test	edx, 100000000000000000000000b
	jz	NoMMX
	or	esi,1
NoMMX:  test	edx, 10000000000000000000000000b
	jz	NoSSE
	or	esi,2
NoSSE:  test	edx, 100000000000000000000000000b
	jz	NoSSE2
	or	esi,4
NoSSE2: test	ecx, 1000000000b
	jz	NoSSSE3
	or	esi,8
NoSSSE3:test	ecx, 10000000000000000000b
	jz	NoSSE41
	or	esi,16
NoSSE41:test	ecx, 100000000000000000000b
	jz	NoSSE42
	or	esi,32
NoSSE42:	
exit:	mov	eax,esi
	ret
GetFeaturesCPU ENDP


	public	GetFeatures2
GetFeatures2 PROC \
        uses ebx esi
        jmp	GetFeaturesCPU
        ret
GetFeatures2 ENDP


;*************************************************************************************

EmitEMMS PROC
	emms
	ret
EmitEMMS ENDP


;*************************************************************************************

EmitFSAVE PROC \
	pBlock:ptr byte
	mov	ecx,pBlock
	fsave	[ecx]
	ret
EmitFSAVE ENDP

EmitFRSTOR PROC \
	pBlock:ptr byte
	mov	ecx,pBlock
	fsave	[ecx]
	ret
EmitFRSTOR ENDP



        end
