;***************************************************************************************
;*                       (c)2025 Digiteq Automotive - Jaroslav Fojtik
;* This code could be redistributed under LGPL licency.
;****************************************************************************************/

.CODE _text             ;Indicates the start of a code segment.


; https://docs.oracle.com/cd/E19120-01/open.solaris/817-5477/eojdc/index.html
; https://www.officedaytime.com/simd512e/

	public	GetFeaturesCPU
GetFeaturesCPU PROC \
        uses rbx rsi
        
        mov	esi,0

	pushfq				; Save EFLAGS
	pushfq				; Store EFLAGS
	xor	qword ptr [rsp],00200000h ; Invert the ID bit in stored EFLAGS
	popfq				; Load stored EFLAGS (with ID bit inverted)
	pushfq				; Store EFLAGS again (ID bit may or may not be inverted)
	pop	rax			; eax = modified EFLAGS (ID bit may or may not be inverted)
	xor	rax,[rsp]		; eax = whichever bits were changed
	popfq				; Restore original EFLAGS
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


;*************************************************************************************

EmitEMMS PROC
	emms
	ret
EmitEMMS ENDP


;*************************************************************************************


EmitFSAVE PROC
	fsave	[RCX]
	ret
EmitFSAVE ENDP

EmitFRSTOR PROC
	fsave	[RCX]
	ret
EmitFRSTOR ENDP



        end
