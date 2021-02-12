.syntax unified
.cpu cortex-m0
.fpu softvfp
.extern contextSwitch

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


.global isr_pendsv
.type isr_pendsv, %function
isr_pendsv:
  //cpsid i // Disable interrupts
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
  ldr r0, =currentThread
  ldr r0, [r0]
  str r2, [r0]

  msr msp, r3  // Back to kernel stack
  //ldr r0, =rnext
  //mov lr, r0
  bl contextSwitch
rnext:
  mov r3, sp   // Save MSP stackPtr
  // Get nextStackPtr
  ldr r0, =currentThread
  ldr r0,[r0]
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

  ldr r0, =0xfffffffd
  //cpsie i // Enable interrupts
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

