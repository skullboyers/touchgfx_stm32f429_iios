; Listing generated by Microsoft (R) Optimizing Compiler Version 19.25.28614.0 

	TITLE	E:\skull\TouchGFX\Projects\iioss\stm32f429_touchgfx_iios\Src\generated\texts\src\TypedTextDatabase.cpp
	.686P
	.XMM
	include listing.inc
	.model	flat

INCLUDELIB MSVCRTD
INCLUDELIB OLDNAMES

PUBLIC	?typedText_database_DEFAULT@@3QBUTypedTextData@TypedText@touchgfx@@B ; typedText_database_DEFAULT
PUBLIC	?typedTextDatabaseArray@@3QBQBUTypedTextData@TypedText@touchgfx@@B ; typedTextDatabaseArray
msvcjmc	SEGMENT
__47A90C35_vcruntime_new@h DB 01H
__A74F5D0F_Types@hpp DB 01H
__AB47D07E_Unicode@hpp DB 01H
__8B4C2B69_Font@hpp DB 01H
__1FD1C573_Texts@hpp DB 01H
__745EA44E_TypedText@hpp DB 01H
__6B9E985D_ConstFont@hpp DB 01H
__D24C4EAF_GeneratedFont@hpp DB 01H
__9DC57DED_TypedTextDatabase@cpp DB 01H
msvcjmc	ENDS
CONST	SEGMENT
?typedText_database_DEFAULT@@3QBUTypedTextData@TypedText@touchgfx@@B DB 01H ; typedText_database_DEFAULT
	DB	01H
	DB	00H
	DB	01H
	DB	00H
	DB	00H
	ORG $+2
?typedTextDatabaseArray@@3QBQBUTypedTextData@TypedText@touchgfx@@B DD FLAT:?typedText_database_DEFAULT@@3QBUTypedTextData@TypedText@touchgfx@@B ; typedTextDatabaseArray
CONST	ENDS
PUBLIC	?getInstance@TypedTextDatabase@@YAPBUTypedTextData@TypedText@touchgfx@@G@Z ; TypedTextDatabase::getInstance
PUBLIC	?getFonts@TypedTextDatabase@@YAPAPBVFont@touchgfx@@XZ ; TypedTextDatabase::getFonts
PUBLIC	?setFont@TypedTextDatabase@@YAPBVFont@touchgfx@@GPBV23@@Z ; TypedTextDatabase::setFont
PUBLIC	?resetFont@TypedTextDatabase@@YAXG@Z		; TypedTextDatabase::resetFont
PUBLIC	?getInstanceSize@TypedTextDatabase@@YAGXZ	; TypedTextDatabase::getInstanceSize
PUBLIC	__JustMyCode_Default
PUBLIC	?_fonts@@3PAPBVFont@touchgfx@@A			; _fonts
EXTRN	?getFont_verdana_20_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ:PROC ; getFont_verdana_20_4bpp
EXTRN	?getFont_verdana_40_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ:PROC ; getFont_verdana_40_4bpp
EXTRN	?getFont_verdana_10_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ:PROC ; getFont_verdana_10_4bpp
EXTRN	@__CheckForDebuggerJustMyCode@4:PROC
EXTRN	__RTC_CheckEsp:PROC
EXTRN	__RTC_InitBase:PROC
EXTRN	__RTC_Shutdown:PROC
_BSS	SEGMENT
?_fonts@@3PAPBVFont@touchgfx@@A DD 03H DUP (?)		; _fonts
_BSS	ENDS
CRT$XCU	SEGMENT
?_fonts$initializer$@@3P6AXXZA DD FLAT:??__E_fonts@@YAXXZ ; _fonts$initializer$
CRT$XCU	ENDS
;	COMDAT rtc$TMZ
rtc$TMZ	SEGMENT
__RTC_Shutdown.rtc$TMZ DD FLAT:__RTC_Shutdown
rtc$TMZ	ENDS
;	COMDAT rtc$IMZ
rtc$IMZ	SEGMENT
__RTC_InitBase.rtc$IMZ DD FLAT:__RTC_InitBase
rtc$IMZ	ENDS
; Function compile flags: /Odt
;	COMDAT __JustMyCode_Default
_TEXT	SEGMENT
__JustMyCode_Default PROC				; COMDAT
	push	ebp
	mov	ebp, esp
	pop	ebp
	ret	0
__JustMyCode_Default ENDP
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu /ZI
;	COMDAT ??__E_fonts@@YAXXZ
text$di	SEGMENT
??__E_fonts@@YAXXZ PROC					; `dynamic initializer for '_fonts'', COMDAT
; File E:\skull\TouchGFX\Projects\iioss\stm32f429_touchgfx_iios\Src\generated\texts\src\TypedTextDatabase.cpp
; Line 17
	push	ebp
	mov	ebp, esp
	sub	esp, 192				; 000000c0H
	push	ebx
	push	esi
	push	edi
	lea	edi, DWORD PTR [ebp-192]
	mov	ecx, 48					; 00000030H
	mov	eax, -858993460				; ccccccccH
	rep stosd
	mov	ecx, OFFSET __9DC57DED_TypedTextDatabase@cpp
	call	@__CheckForDebuggerJustMyCode@4
; Line 14
	call	?getFont_verdana_20_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ ; getFont_verdana_20_4bpp
	mov	DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A, eax
; Line 15
	call	?getFont_verdana_40_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ ; getFont_verdana_40_4bpp
	mov	DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A+4, eax
; Line 16
	call	?getFont_verdana_10_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ ; getFont_verdana_10_4bpp
	mov	DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A+8, eax
	pop	edi
	pop	esi
	pop	ebx
	add	esp, 192				; 000000c0H
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
??__E_fonts@@YAXXZ ENDP					; `dynamic initializer for '_fonts''
text$di	ENDS
; Function compile flags: /Odtp /RTCsu /ZI
;	COMDAT ?getInstanceSize@TypedTextDatabase@@YAGXZ
_TEXT	SEGMENT
?getInstanceSize@TypedTextDatabase@@YAGXZ PROC		; TypedTextDatabase::getInstanceSize, COMDAT
; File E:\skull\TouchGFX\Projects\iioss\stm32f429_touchgfx_iios\Src\generated\texts\src\TypedTextDatabase.cpp
; Line 44
	push	ebp
	mov	ebp, esp
	sub	esp, 192				; 000000c0H
	push	ebx
	push	esi
	push	edi
	lea	edi, DWORD PTR [ebp-192]
	mov	ecx, 48					; 00000030H
	mov	eax, -858993460				; ccccccccH
	rep stosd
	mov	ecx, OFFSET __9DC57DED_TypedTextDatabase@cpp
	call	@__CheckForDebuggerJustMyCode@4
; Line 45
	mov	eax, 3
; Line 46
	pop	edi
	pop	esi
	pop	ebx
	add	esp, 192				; 000000c0H
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
?getInstanceSize@TypedTextDatabase@@YAGXZ ENDP		; TypedTextDatabase::getInstanceSize
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu /ZI
;	COMDAT ?resetFont@TypedTextDatabase@@YAXG@Z
_TEXT	SEGMENT
tv65 = -196						; size = 4
_fontId$ = 8						; size = 2
?resetFont@TypedTextDatabase@@YAXG@Z PROC		; TypedTextDatabase::resetFont, COMDAT
; File E:\skull\TouchGFX\Projects\iioss\stm32f429_touchgfx_iios\Src\generated\texts\src\TypedTextDatabase.cpp
; Line 61
	push	ebp
	mov	ebp, esp
	sub	esp, 196				; 000000c4H
	push	ebx
	push	esi
	push	edi
	lea	edi, DWORD PTR [ebp-196]
	mov	ecx, 49					; 00000031H
	mov	eax, -858993460				; ccccccccH
	rep stosd
	mov	ecx, OFFSET __9DC57DED_TypedTextDatabase@cpp
	call	@__CheckForDebuggerJustMyCode@4
; Line 62
	movzx	eax, WORD PTR _fontId$[ebp]
	mov	DWORD PTR tv65[ebp], eax
	cmp	DWORD PTR tv65[ebp], 0
	je	SHORT $LN4@resetFont
	cmp	DWORD PTR tv65[ebp], 1
	je	SHORT $LN5@resetFont
	cmp	DWORD PTR tv65[ebp], 2
	je	SHORT $LN6@resetFont
	jmp	SHORT $LN1@resetFont
$LN4@resetFont:
; Line 65
	call	?getFont_verdana_20_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ ; getFont_verdana_20_4bpp
	mov	ecx, 4
	imul	edx, ecx, 0
	mov	DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A[edx], eax
; Line 66
	jmp	SHORT $LN1@resetFont
$LN5@resetFont:
; Line 68
	call	?getFont_verdana_40_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ ; getFont_verdana_40_4bpp
	mov	ecx, 4
	shl	ecx, 0
	mov	DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A[ecx], eax
; Line 69
	jmp	SHORT $LN1@resetFont
$LN6@resetFont:
; Line 71
	call	?getFont_verdana_10_4bpp@@YAAAVGeneratedFont@touchgfx@@XZ ; getFont_verdana_10_4bpp
	mov	ecx, 4
	shl	ecx, 1
	mov	DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A[ecx], eax
$LN1@resetFont:
; Line 74
	pop	edi
	pop	esi
	pop	ebx
	add	esp, 196				; 000000c4H
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
?resetFont@TypedTextDatabase@@YAXG@Z ENDP		; TypedTextDatabase::resetFont
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu /ZI
;	COMDAT ?setFont@TypedTextDatabase@@YAPBVFont@touchgfx@@GPBV23@@Z
_TEXT	SEGMENT
_old$ = -8						; size = 4
_fontId$ = 8						; size = 2
_font$ = 12						; size = 4
?setFont@TypedTextDatabase@@YAPBVFont@touchgfx@@GPBV23@@Z PROC ; TypedTextDatabase::setFont, COMDAT
; File E:\skull\TouchGFX\Projects\iioss\stm32f429_touchgfx_iios\Src\generated\texts\src\TypedTextDatabase.cpp
; Line 54
	push	ebp
	mov	ebp, esp
	sub	esp, 204				; 000000ccH
	push	ebx
	push	esi
	push	edi
	lea	edi, DWORD PTR [ebp-204]
	mov	ecx, 51					; 00000033H
	mov	eax, -858993460				; ccccccccH
	rep stosd
	mov	ecx, OFFSET __9DC57DED_TypedTextDatabase@cpp
	call	@__CheckForDebuggerJustMyCode@4
; Line 55
	movzx	eax, WORD PTR _fontId$[ebp]
	mov	ecx, DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A[eax*4]
	mov	DWORD PTR _old$[ebp], ecx
; Line 56
	movzx	eax, WORD PTR _fontId$[ebp]
	mov	ecx, DWORD PTR _font$[ebp]
	mov	DWORD PTR ?_fonts@@3PAPBVFont@touchgfx@@A[eax*4], ecx
; Line 57
	mov	eax, DWORD PTR _old$[ebp]
; Line 58
	pop	edi
	pop	esi
	pop	ebx
	add	esp, 204				; 000000ccH
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
?setFont@TypedTextDatabase@@YAPBVFont@touchgfx@@GPBV23@@Z ENDP ; TypedTextDatabase::setFont
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu /ZI
;	COMDAT ?getFonts@TypedTextDatabase@@YAPAPBVFont@touchgfx@@XZ
_TEXT	SEGMENT
?getFonts@TypedTextDatabase@@YAPAPBVFont@touchgfx@@XZ PROC ; TypedTextDatabase::getFonts, COMDAT
; File E:\skull\TouchGFX\Projects\iioss\stm32f429_touchgfx_iios\Src\generated\texts\src\TypedTextDatabase.cpp
; Line 49
	push	ebp
	mov	ebp, esp
	sub	esp, 192				; 000000c0H
	push	ebx
	push	esi
	push	edi
	lea	edi, DWORD PTR [ebp-192]
	mov	ecx, 48					; 00000030H
	mov	eax, -858993460				; ccccccccH
	rep stosd
	mov	ecx, OFFSET __9DC57DED_TypedTextDatabase@cpp
	call	@__CheckForDebuggerJustMyCode@4
; Line 50
	mov	eax, OFFSET ?_fonts@@3PAPBVFont@touchgfx@@A ; _fonts
; Line 51
	pop	edi
	pop	esi
	pop	ebx
	add	esp, 192				; 000000c0H
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
?getFonts@TypedTextDatabase@@YAPAPBVFont@touchgfx@@XZ ENDP ; TypedTextDatabase::getFonts
_TEXT	ENDS
; Function compile flags: /Odtp /RTCsu /ZI
;	COMDAT ?getInstance@TypedTextDatabase@@YAPBUTypedTextData@TypedText@touchgfx@@G@Z
_TEXT	SEGMENT
_id$ = 8						; size = 2
?getInstance@TypedTextDatabase@@YAPBUTypedTextData@TypedText@touchgfx@@G@Z PROC ; TypedTextDatabase::getInstance, COMDAT
; File E:\skull\TouchGFX\Projects\iioss\stm32f429_touchgfx_iios\Src\generated\texts\src\TypedTextDatabase.cpp
; Line 39
	push	ebp
	mov	ebp, esp
	sub	esp, 192				; 000000c0H
	push	ebx
	push	esi
	push	edi
	lea	edi, DWORD PTR [ebp-192]
	mov	ecx, 48					; 00000030H
	mov	eax, -858993460				; ccccccccH
	rep stosd
	mov	ecx, OFFSET __9DC57DED_TypedTextDatabase@cpp
	call	@__CheckForDebuggerJustMyCode@4
; Line 40
	movzx	eax, WORD PTR _id$[ebp]
	mov	eax, DWORD PTR ?typedTextDatabaseArray@@3QBQBUTypedTextData@TypedText@touchgfx@@B[eax*4]
; Line 41
	pop	edi
	pop	esi
	pop	ebx
	add	esp, 192				; 000000c0H
	cmp	ebp, esp
	call	__RTC_CheckEsp
	mov	esp, ebp
	pop	ebp
	ret	0
?getInstance@TypedTextDatabase@@YAPBUTypedTextData@TypedText@touchgfx@@G@Z ENDP ; TypedTextDatabase::getInstance
_TEXT	ENDS
END
