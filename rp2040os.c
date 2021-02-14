#include "rp2040os.h"

CpuStats cpuStats[MAX_CORES];
struct Thread threads[MAX_TASKS];
uint8_t threadCount = MAX_CORES;  // 0-MAX_CORES is idleThread
volatile uint8_t currentIdx = 0;
// These are so the assembly does not need to use index
Thread *currentThread[MAX_CORES];
//Thread *nextThread;

void setPSPControl(uint32_t sp, uint32_t ctrl);


uint64_t getTimeUs() {
  uint64_t tm = *(volatile uint32_t*)(TIMER_BASE + 0x0c);
  tm |= (*(uint32_t*)(TIMER_BASE + 0x08))<<16;
  return tm;
}

static inline void contextLock() {
  uint8_t cpu = *(uint32_t*)(SIO_BASE);
  while (!*CONTEXTSWITCH_SPINLOCK);
}

static inline void contextUnlock() {
  *CONTEXTSWITCH_SPINLOCK = 0;
}

void isr_systick(void) {
  // Kick off PendSV
  *(volatile uint32_t *)(0xe0000000|M0PLUS_ICSR_OFFSET) = (1L<<28);
}

void contextSwitch() {
  uint64_t now = getTimeUs();
  uint8_t cpu = *(uint32_t*)(SIO_BASE);
  uint32_t elapsed = (now - cpuStats[cpu].lastContextTime);
  cpuStats[cpu].lastContextTime = now;
  cpuStats[cpu].csCount++;

  currentThread[cpu]->execTime += elapsed;
  currentThread[cpu]->lastcpu = cpu;
  currentThread[cpu]->cpu = 0xff;

  // Mutex section
  #ifdef CONTEXTSW_PIN
  setGPIO(SQ_PIN+cpu);
  #endif
  contextLock();

  uint8_t cpriority = 255;
  // Unblock anything waiting if time is up
  // Find highest priority
  for(uint8_t a=MAX_CORES;a<threadCount;a++) {
    Thread *t = &threads[a];
    // Ignore if active on another core

    if (t->cpu != 0xff && t->cpu != cpu) continue;
    if (t->status == THREAD_STATUS_WAIT) {
      if (t->waitExpires <= now) {
        t->status = THREAD_STATUS_RUNNING;
      }
    }
    // Ignore if not running
    if (t->status != THREAD_STATUS_RUNNING) continue;
    if (t->priority < cpriority) cpriority = t->priority;
  }

  // Now find processes matching highest priority
  if (currentIdx < MAX_CORES) currentIdx = MAX_CORES-1; // Skip idle threads
  uint8_t a;
  for(a=MAX_CORES;a<threadCount;a++) {
    currentIdx++;
    if (currentIdx >= threadCount) currentIdx = MAX_CORES;
    Thread *t = &threads[currentIdx];
    // Ignpore if active on another cpu
    if (t->cpu != 0xff && t->cpu != cpu) continue;
    // Ignore if not running
    if (t->status != THREAD_STATUS_RUNNING) continue;

    // Match priority
    if (t->priority != cpriority) continue;
    break;
  }
  if (a >= threadCount) {
    currentIdx = cpu; // Idle on appropriate core
  }
  currentThread[cpu] = &threads[currentIdx];
  currentThread[cpu]->cpu = cpu;
  contextUnlock();
  #ifdef CONTEXTSW_PIN
  clrGPIO(SQ_PIN+cpu);
  #endif
  uint32_t ctime = getTimeUs()-now; // Context time
  cpuStats[cpu].contextTime +=  ctime; // SysTick counter decrements
}

static uint32_t idleStack[MAX_CORES][IDLE_STACKSIZE];
void idle() {
  while (true) asm("wfi"); 
}


void setupIdle(uint8_t cpu) {
  Thread *t = &threads[cpu];
  t->execTime = 0;
  t->status = THREAD_STATUS_RUNNING;
  t->priority = 255;
  t->cpu = 0xff; // Not active
  t->pid = cpu;
  #ifdef USE_THREAD_NAMES
  t->name = "IdleThread";
  #endif
  // Stack builds top down
  uint8_t len = IDLE_STACKSIZE;
  idleStack[cpu][--len] = 0x01000000; // Thumb bit of xPSR
  idleStack[cpu][--len] = (uint32_t)idle;  // start of thread routine
  t->stackPtr = (uint32_t)(idleStack[cpu] + IDLE_STACKSIZE - 16);
  currentThread[cpu] = t;
  setPSPControl(t->stackPtr, 2); // Privileged
}


void core1_main() {
  setupIdle(1);
  // Systick
  *(volatile unsigned int *)(0xe0000000|M0PLUS_SYST_RVR_OFFSET) = (SYSTEM_CLOCK_HZ / 1000000)*SCHED_INTERVAL_US; // reload count
  *(volatile unsigned int *)(0xe0000000|M0PLUS_SYST_CSR_OFFSET) = 7; // enable, SYSTICK at core clock, exception
  *(volatile unsigned int *)(0xe0000000|M0PLUS_SHPR3_OFFSET) = (0<<30) | (3<<22); // SysTick=0(high), PendSV=3(low) 
  startStackThread(currentThread[1]->stackPtr); // Does not return
}


// Must supply pointer to first thread added
void setupSched() {
  for(uint8_t a=0;a<MAX_CORES;a++) cpuStats[a].lastContextTime = getTimeUs();
  // Idle thread
  setupIdle(0);

  // Starting thread
  currentIdx = MAX_CORES; // First user process

  // Systick
  *(volatile unsigned int *)(0xe0000000|M0PLUS_SYST_RVR_OFFSET) = (SYSTEM_CLOCK_HZ / 1000000)*SCHED_INTERVAL_US; // reload count
  *(volatile unsigned int *)(0xe0000000|M0PLUS_SYST_CSR_OFFSET) = 7; // enable, SYSTICK at core clock, exception
  *(volatile unsigned int *)(0xe0000000|M0PLUS_SHPR3_OFFSET) = (0<<30) | (3<<22); // SysTick=0(high), PendSV=3(low) 
  multicore_launch_core1(core1_main);
  startStackThread(currentThread[0]->stackPtr); // Does not return
  while(1);
}

static inline void yield() {
  uint8_t cpu = *(uint32_t*)(SIO_BASE);
  uint8_t now = cpuStats[cpu].csCount;  // Keep this so we can watch for it to change
  *(volatile uint32_t *)(0xe0000000|M0PLUS_ICSR_OFFSET) = (1L<<26); // SysTick pending
  // Wait for context switch
  while (now == (uint8_t)cpuStats[cpu].csCount);
}


// Adds a thread to OS.  Stack points to the beginning of thread stack.  Len is the size of the thread stack.
#ifdef USE_THREAD_NAMES
void addThread(char *name, void(*hndl)(void*), uint32_t *stack, uint16_t len, uint8_t priority) {
#else 
void addThread(void(*hndl)(void*), uint32_t *stack, uint16_t len) {
#endif
  if (threadCount >= MAX_TASKS) return;
  Thread *t = &threads[threadCount];
  t->execTime = 0;
  t->status = THREAD_STATUS_RUNNING;
  t->priority = priority;
  t->cpu = 0xff; // Not active
  t->pid = threadCount;
#ifdef USE_THREAD_NAMES
  t->name = name;
#endif
  // The first thread is started from main so it needs no stack setup
  // Stack builds top down
  stack[len-1] = 0x01000000; // Thumb bit of xPSR
  stack[len-2] = (uint32_t)hndl;  // start of thread routine
  t->stackPtr = (uint32_t)(stack + len - 16);
  threadCount++;
}


void delayus(uint32_t us) {
  uint8_t threadID = currentIdx;
  threads[threadID].waitExpires = getTimeUs() + us;
  threads[threadID].status = THREAD_STATUS_WAIT;
  yield();
}


