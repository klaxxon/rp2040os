.syntax unified
.cpu cortex-m0
.fpu softvfp
.extern contextSwitch

.equ SIO_BASE, 0xd0000000
.equ GPIO_OUT_SET, 0x014
.equ GPIO_OUT_CLR, 0x018
.equ GPIO_SET, (SIO_BASE + GPIO_OUT_SET)
.equ GPIO_CLR, (SIO_BASE + GPIO_OUT_CLR)

// Context switch output pin for external monitoring.
// The CPU number is added to this so make sure you
// have this pin and next pin sequentially initialized
// as outputs.
//.equ CONTEXTSW_PIN, 14

.thumb

// setPSPControl(uint32_t stackPtr, uint32_t ctrl)
.global setPSPControl
.type setPSPControl, %function
setPSPControl:
  msr control, r1
  msr psp, r0
  mov sp, r0  // Put stackPtr arg into sp
  isb
  bx lr


// __setMSP(uint32_t sp)
.type __setMSP, %function
__setMSP:
  msr msp, r0
  bx lr

.type __setPSP, %function
__setPSP:
  msr psp, r0
  bx lr

.type __setGPIO, %function 
__setGPIO:
  push {r0-r3}
  ldr r1, =#(GPIO_SET)
  movs r2, #1
  lsls r2, r0
  str r2, [r1]
  pop {r0-r3}
  bx lr

.type __setGPIO, %function 
__clrGPIO:
  push {r0-r3}
  ldr r1, =#(GPIO_CLR)
  movs r2, #1
  lsls r2, r0
  str r2, [r1]
  pop {r0-r3}
  bx lr


.global isr_pendsv
.type isr_pendsv, %function
isr_pendsv:
  cpsid i // Disable interrupts
  .ifdef CONTEXTSW_PIN
  ldr r0, =#0xd0000000
  ldr r0, [r0] // CPU number
  adds r0, r0, #(CONTEXTSW_PIN)
  bl __setGPIO
  .endif
  mov r3, sp   // Save MSP stackPtr
  mrs r2, psp // Get current stackPtr
  mov sp, r2  // Use the current threads stackPtr
  // Save the other registers
  push {r4-r7}
  mov r4, r8
  mov r5, r9
  mov r6, r10
  mov r7, r11
  push {r4-r7}

  mov r2, sp  // Save the current threads stackPtr
  // cpu offset
  ldr r1, =#0xd0000000
  ldr r1, [r1] // CPU number
  lsls r1, r1, #2  // times 4
  ldr r0, =currentThread
  ldr r0, [r0, r1]
  str r2, [r0]

  msr msp, r3  // Back to kernel stack
  //ldr r0, =rnext
  //mov lr, r0
  bl contextSwitch
rnext:
  mov r3, sp   // Save MSP stackPtr
  // cpu offset
  ldr r1, =#0xd0000000
  ldr r1, [r1] // CPU number
  lsls r1, r1, #2  // times 4
  // Get nextStackPtr
  ldr r0, =currentThread
  ldr r0, [r0, r1]
  ldr r2, [r0] 
  mov sp, r2

  // Restore the higher registers
  pop {r4-r7}
  mov r8, r4
  mov r9, r5
  mov r10, r6
  mov r11, r7
  pop {r4-r7}
  // Restore the stack
  mov r1, sp
  msr psp, r1
  mov sp, r3

  .ifdef CONTEXTSW_PIN
  ldr r0, =#0xd0000000
  ldr r0, [r0] // CPU number
  adds r0, r0, #(CONTEXTSW_PIN)
  bl __clrGPIO
  .endif

  ldr r0, =0xfffffffd
  cpsie i // Enable interrupts
  bx r0

  


// startStackThread(uint32_t stackPtr)
.global startStackThread
.type startStackThread, %function
startStackThread:
  cpsid i // Disable interrupts
  mov sp, r0
  // The stack has the usual expanded stacking of 16 registers so we need to remove them
  pop {r4-r7}  // Actually r8-r11
  pop {r4-r7}
  pop {r0-r3}
  pop {r4-r5} // 2 more registers 
  pop {r3}
  mov lr, r3
  pop {r3}
  cpsie i // Enable interrupts
  bx  lr

