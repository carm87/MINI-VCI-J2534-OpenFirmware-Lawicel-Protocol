/* 
 crt0.S for LPC2xxx
 - based on examples from R O Software
 - based on examples from newlib-lpc
 - based on an example from Anglia Designs

 collected and modified by Martin Thomas
*/

		.global _etext						// -> .data initial values in ROM
		.global _data						// -> .data area in RAM
		.global _edata						// end of .data area
		.global __bss_start				// -> .bss area in RAM
		.global __bss_end__				// end of .bss area


// Stack Sizes and Stack Top Addresses defined in Linkerscript !!
		.global _UNDStackTop
		.global _ABTStackTop
		.global _FIQStackTop
		.global _IRQStackTop
		.global _SVCStackTop
		.global _USRStackTop


// Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs
		.set	MODE_USR, 0x10						// User Mode
		.set	MODE_FIQ, 0x11						// FIQ Mode
		.set	MODE_IRQ, 0x12						// IRQ Mode
		.set	MODE_SVC, 0x13						// Supervisor Mode
		.set	MODE_ABT, 0x17						// Abort Mode
		.set	MODE_UND, 0x1B						// Undefined Mode
		.set	MODE_SYS, 0x1F						// System Mode

		.equ	I_BIT,	0x80						// when I bit is set, IRQ is disabled
		.equ	F_BIT,	0x40						// when F bit is set, FIQ is disabled

		.equ	VPBDIV,				0xE01FC100	// VPBDIV Address
		
//		.equ	VPBDIV_Val,			0x00000000	// pclk = cclk / 4
		.equ	VPBDIV_Val,			0x00000001	// pclk = cclk
//		.equ	VPBDIV_Val,			0x00000002	// pclk = cclk / 2


		.equ	PLL_BASE,			0xE01FC080	// PLL Base Address
		.equ	PLLCON_OFS,			0x00			// PLL Control Offset
		.equ	PLLCFG_OFS,			0x04			// PLL Configuration Offset
		.equ	PLLSTAT_OFS,		0x08			// PLL Status Offset
		.equ	PLLFEED_OFS,		0x0C			// PLL Feed Offset
		.equ	PLLCON_PLLE,		(1<<0)		// PLL Enable
		.equ	PLLCON_PLLC,		(1<<1)		// PLL Connect
		.equ	PLLCFG_MSEL,		(0x1F<<0)	// PLL Multiplier
		.equ	PLLCFG_PSEL,		(0x03<<5)	// PLL Divider
		.equ	PLLSTAT_PLOCK,		(1<<10)		// PLL Lock Status


		.equ	PLL_MSEL,			5				// PLL_M = PLL_MSEL + 1
		.equ	PLL_PSEL,			1				// PLL_P = 2 ^ PLL_PSEL

		//	PLL_PSEL	PLL_P
		//	00			1
		//	01			2
		//	10			4
		//	11			8

		// Fosc = 12 MHz

		// Fcco = Fosc * PLL_M * 2 * PLL_P	( 156 - 320 MHz)
		// Fcpu = Fcco / ( 2 * PLL_P)

		.equ	PLLCFG_Val,     ( PLL_PSEL << 5 | PLL_MSEL)

		.equ	MAM_BASE,			0xE01FC000  // MAM Base Address
		.equ	MAMCR_OFS,			0x00        // MAM Control Offset
		.equ	MAMTIM_OFS,			0x04        // MAM Timing Offset

		.equ    MAMCR_Val,		0x00000002
		.equ    MAMTIM_Val,		0x00000003

		.text
		.arm
		.section .init, "ax"

		.code 32
		.align 2

		.global _boot
		.func   _boot
_boot:

// Runtime Interrupt Vectors
// -------------------------
Vectors:
		b     _start                    // reset - _start
		ldr   pc,_undf                  // undefined - _undf
		ldr   pc,_swi                   // SWI - _swi
		ldr   pc,_pabt                  // program abort - _pabt
		ldr   pc,_dabt                  // data abort - _dabt
		nop                             // reserved
		ldr   pc,[pc,#-0xFF0]           // IRQ - read the VIC
		ldr   pc,_fiq                   // FIQ - _fiq


// Use this group for development
_undf:  .word __undf                    // undefined
_swi:   .word __swi                     // SWI
_pabt:  .word __pabt                    // program abort
_dabt:  .word __dabt                    // data abort
_irq:   .word __irq                     // IRQ
_fiq:   .word __fiq                     // FIQ

__undf:	b	.							// undefined
__swi:	b	.							// SWI
__pabt:	b	.							// program abort
__dabt:	b	.							// data abort
__irq:	b	.							// IRQ
__fiq:	b	.							// FIQ

		.size _boot, . - _boot
		.endfunc


// Setup the operating mode & stack.
// ---------------------------------
		.global _start, start, _mainCRTStartup
		.func   _start

_start:
start:
_mainCRTStartup:

// VPB_DIV Setup
		LDR     R0, =VPBDIV
		LDR     R1, =VPBDIV_Val
		STR     R1, [R0]


// PLL Setup
		LDR     R0, =PLL_BASE
		MOV     R1, #0xAA
		MOV     R2, #0x55

// Configure and Enable PLL
		MOV     R3, #PLLCFG_Val
		STR     R3, [R0, #PLLCFG_OFS] 
		MOV     R3, #PLLCON_PLLE
		STR     R3, [R0, #PLLCON_OFS]
		STR     R1, [R0, #PLLFEED_OFS]
		STR     R2, [R0, #PLLFEED_OFS]

// Wait until PLL Locked
PLL_Loop:
		LDR     R3, [R0, #PLLSTAT_OFS]
		ANDS    R3, R3, #PLLSTAT_PLOCK
		BEQ     PLL_Loop

// Switch to PLL Clock
		MOV     R3, #(PLLCON_PLLE | PLLCON_PLLC)
		STR     R3, [R0, #PLLCON_OFS]
		STR     R1, [R0, #PLLFEED_OFS]
		STR     R2, [R0, #PLLFEED_OFS]

// MAM Setup
		LDR     R0, =MAM_BASE
		MOV     R1, #MAMTIM_Val
		STR     R1, [R0, #MAMTIM_OFS] 
		MOV     R1, #MAMCR_Val
		STR     R1, [R0, #MAMCR_OFS] 


// Initialize Interrupt System
// - Set stack location for each mode
// -----------------------------------------------        
		MSR   CPSR_c,#MODE_UND|I_BIT|F_BIT	// Undefined Instruction Mode
		LDR   sp,=_UNDStackTop

		MSR   CPSR_c,#MODE_ABT|I_BIT|F_BIT	// Abort Mode
		LDR   sp,=_ABTStackTop

		MSR   CPSR_c,#MODE_FIQ|I_BIT|F_BIT	// FIQ Mode
		LDR   sp,=_FIQStackTop

		MSR   CPSR_c,#MODE_IRQ|I_BIT|F_BIT	// IRQ Mode
		LDR   sp,=_IRQStackTop

		MSR   CPSR_c,#MODE_SVC|I_BIT|F_BIT	// SVC Mode
		LDR   sp,=_SVCStackTop

		MSR   CPSR_c,#MODE_USR				// User Mode
		LDR   sp,=_USRStackTop



// Copy initialized data to its execution address in RAM
// -----------------------------------------------------
		LDR   r1,=_etext                // -> ROM data start
		LDR   r2,=_data                 // -> data start
		LDR   r3,=_edata                // -> end of data
1:		CMP   r2,r3                     // check if data to move
		LDRLO r0,[r1],#4                // copy it
		STRLO r0,[r2],#4
		BLO   1b                        // loop until done


// Clear .bss
// ----------
		mov   r0,#0                     // get a zero
		ldr   r1,=__bss_start           // -> bss start
		ldr   r2,=__bss_end__           // -> bss end
2:		cmp   r1,r2                     // check if data to clear
		strlo r0,[r1],#4                // clear 4 bytes
		blo   2b                        // loop until done


/*
	Call C++ constructors (for objects in "global scope")
	ctor loop added by Martin Thomas 4/2005 
	based on a Anglia Design example-application for ST ARM
*/

		LDR 	r0, =__ctors_start__
		LDR 	r1, =__ctors_end__
ctor_loop:
		CMP 	r0, r1
		BEQ 	ctor_end
		LDR 	r2, [r0], #4
		STMFD 	sp!, {r0-r1}
		MOV 	lr, pc
		MOV 	pc, r2
		LDMFD 	sp!, {r0-r1}
		B 		ctor_loop
ctor_end:


// Call main program: main(0)
// --------------------------
		mov   r0,#0                     // no arguments (argc = 0)
		mov   r1,r0
		mov   r2,r0
		mov   fp,r0                     // null frame pointer
		mov   r7,r0                     // null frame pointer for thumb
		ldr   r10,=main
		mov   lr,pc

/* Enter the C code, use BX instruction so as to never return */
/* use BLX (?) main if you want to use c++ destructors below */

		bx    r10                       // enter main()



/* "global object"-dtors are never called and it should not be 
   needed since there is no OS to exit to. */
/* Call destructors */

#		LDR		r0, =__dtors_start__
#		LDR		r1, =__dtors_end__
dtor_loop:
#		CMP		r0, r1
#		BEQ		dtor_end
#		LDR		r2, [r0], #4
#		STMFD	sp!, {r0-r1}
#		MOV		lr, pc
#		MOV		pc, r2
#		LDMFD	sp!, {r0-r1}
#		B		dtor_loop
dtor_end:


        .size   _start, . - _start
        .endfunc

        .global _reset, reset, exit, abort
        .func   _reset
_reset:
reset:
exit:
abort:


		b     .                         // loop until reset

		.size _reset, . - _reset
		.endfunc

		.end

