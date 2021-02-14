#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#define Semaphore int32_t
#define SYSTEM_CLOCK_HZ 125000000
#define MAX_CORES 2
#define USER_TASKS 5
// Each core has its own idle thread 
#define MAX_TASKS (MAX_CORES+USER_TASKS)
#define SCHED_INTERVAL_US 10000
#define SCHED_TIMER_NUM 0
#define SCHED_IRQ TIMER_IRQ_0
#define IDLE_STACKSIZE 100
#define CONTEXTSWITCH_SPINLOCK ((uint32_t*)(SIO_BASE+SIO_SPINLOCK0_OFFSET))

#define USE_THREAD_NAMES 1

#define LED_PIN 25 
#define LED_PIN2 16
#define OSC_PIN 15
#define SQ_PIN 14

#define SYSTCOUNTER ((SYSTEM_CLOCK_HZ / 1000000)*SCHED_INTERVAL_US)

#define GPIO_OUT_SET 0x014
#define GPIO_OUT_CLR 0x018
#define setGPIO(x) (*(uint32_t*)(SIO_BASE+GPIO_OUT_SET) = (1<<x))
#define clrGPIO(x) (*(uint32_t*)(SIO_BASE+GPIO_OUT_CLR) = (1<<x))

typedef struct CpuStats {
  uint64_t contextTime;
  uint64_t idleTime;
  uint64_t lastContextTime;
} CpuStats;

CpuStats cpuStats[MAX_CORES];


enum threadStatus {
  THREAD_STATUS_IDLE = 0,
  THREAD_STATUS_RUNNING = 1,// Can be run
  THREAD_STATUS_WAIT = 2,
  THREAD_STATUS_DONE = 3,
};

typedef struct Thread {
  uint32_t stackPtr;  // Top of stack
  uint8_t pid;
  volatile enum threadStatus status;
  uint64_t execTime;
  volatile uint64_t waitExpires;
  #ifdef USE_THREAD_NAMES
  char *name;
  #endif
  uint8_t priority;
  volatile uint8_t cpu; // Active on CPU, 255 = not active
  volatile uint8_t lastcpu; // Last cpu used
} Thread;


struct Thread threads[MAX_TASKS];
volatile uint8_t threadCount = MAX_CORES;  // 0-MAX_CORES is idleThread
volatile uint8_t currentIdx = 0;
// These are so the assembly does not need to use index
Thread *currentThread[MAX_CORES];
//Thread *nextThread;

static volatile uint32_t sysCount, idleCount;


#define SYSTICKVAL  ((*(volatile uint32_t *)(0xe0000000|M0PLUS_SYST_CVR_OFFSET))&0xffffff)
void startStackThread(uint32_t stackPtr);
void setPSPControl(uint32_t sp, uint32_t ctrl);


uint64_t getTimeUs() {
  uint64_t tm = *(uint32_t*)(TIMER_BASE + 0x0c);
  tm |= (*(uint32_t*)(TIMER_BASE + 0x08))<<16;
  return tm;
}

static inline void contextLock() {
  uint8_t cpu = *(uint32_t*)(SIO_BASE);
  while (!*CONTEXTSWITCH_SPINLOCK) {
    uint32_t z = *(uint32_t*)(SIO_BASE+0x5c);
    z++;
  }
}

static inline void contextUnlock() {
  *CONTEXTSWITCH_SPINLOCK = 0;
}
/*
uint16_t logs[32]= {0};
uint8_t logpos = 0;

void addLog(uint16_t cpu, uint16_t thrd,uint16_t log) {
  while (!*((uint32_t*)(SIO_BASE+SIO_SPINLOCK1_OFFSET)));
  logs[logpos++] = (cpu*1000)+(thrd*100)+log;
  logpos &=31;
  *((uint32_t*)(SIO_BASE+SIO_SPINLOCK1_OFFSET)) = 1;
}
*/

void isr_systick(void) {
  // Kick off PendSV
  *(volatile uint32_t *)(0xe0000000|M0PLUS_ICSR_OFFSET) = (1L<<28);
}

void contextSwitch() {
  uint32_t now = getTimeUs();
  uint8_t cpu = *(uint32_t*)(SIO_BASE);
  uint32_t elapsed = (now - cpuStats[cpu].lastContextTime);
  cpuStats[cpu].lastContextTime = now;

  setGPIO(SQ_PIN+cpu);
  contextLock();
  currentThread[cpu]->execTime += elapsed;
  currentThread[cpu]->lastcpu = cpu;
  currentThread[cpu]->cpu = 0xff;
  sysCount++;

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
  clrGPIO(SQ_PIN+cpu);
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
  uint8_t now = sysCount;  // Keep this so we can watch for it to change
  *(volatile uint32_t *)(0xe0000000|M0PLUS_ICSR_OFFSET) = (1L<<26); // SysTick pending
  // Wait for context switch
  while (now == (uint8_t)sysCount);
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


static inline void delayus(uint32_t us) {
  uint8_t threadID = currentIdx;
  threads[threadID].waitExpires = getTimeUs() + us;
  threads[threadID].status = THREAD_STATUS_WAIT;
  yield();
}

#define delayms(x) delayus(x*1000)




static uint32_t blinkStack[128];
void blink() {
  uint32_t a;
  while (true) {
    setGPIO(LED_PIN2);
    delayms(300);
    clrGPIO(LED_PIN2);
    delayms(300);
  }
}


static uint32_t blink2Stack[128];
void blink2() {
  uint32_t a = 0;
  while (true) {
    setGPIO(LED_PIN);
    delayms(37);
    clrGPIO(LED_PIN);
    delayms(37);
  }
}

static uint32_t repStack[256];
void report() {
  uint64_t lastTime = getTimeUs();
  uint64_t lasts[MAX_TASKS]={0};
  while (true) {
    delayms(1000);
    uint64_t now = getTimeUs();
    uint64_t elapsed = now - lastTime;
    lastTime = now;
    printf("\nWall time %llu\n", now/1000000);
    for(uint8_t a=0;a<MAX_CORES;a++) {
      printf("CPU%d Ctx=%0.3f%%, Idle=%0.3f%%\n", a, 100.0*cpuStats[a].contextTime/now, 100.0*threads[a].execTime/now);
    }
    #ifdef USE_THREAD_NAMES
    printf("Thrd   Name     S PRI  CPU     LastCPU\n");
    #else
    printf("Thrd   Total   S PRI CPU\n");
    #endif
    for(uint8_t a=MAX_CORES;a<threadCount;a++) {
      Thread *t = &threads[a];
    #ifdef USE_THREAD_NAMES
      printf("%4d %-10s ", a-MAX_CORES, t->name);
    #else
      printf("%4d ", a);
    #endif
      if (a) {
        switch (t->status) {
          case THREAD_STATUS_IDLE: printf("I "); break;
          case THREAD_STATUS_DONE: printf("X "); break;
          case THREAD_STATUS_RUNNING: printf("R "); break;
          case THREAD_STATUS_WAIT: printf("W "); break;
        }
      } else printf("  ");
      uint64_t telapsed = t->execTime - lasts[a];
      lasts[a] = t->execTime;
      char cpu = '0' + t->lastcpu;
      printf("%3d %6.3f%%  %c\n", t->priority, 100.0*telapsed/elapsed, cpu);
    }
    /*
    printf("Logs ");
    for(uint8_t a=0;a<32;a++) {
      printf("%04d ", logs[(logpos+a)&31]);
    }
    printf("\n");
    */
  }
}



static uint32_t hogStack[128];
void hog() {
  while (true) {
    setGPIO(SQ_PIN);
    delayms(10);
    clrGPIO(SQ_PIN);
    delayms(10);
  }
}


static uint32_t spinnerStack[128];
void spinner() {
  while (true);
}


int main() {
  stdio_init_all(); 
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_init(LED_PIN2);
  gpio_set_dir(LED_PIN2, GPIO_OUT);
  gpio_init(OSC_PIN);
  gpio_set_dir(OSC_PIN, GPIO_OUT);
  gpio_init(SQ_PIN);
  gpio_set_dir(SQ_PIN, GPIO_OUT);

  addThread("Red LED", blink, blinkStack, sizeof(blinkStack)>>2, 100);
  addThread("Green LED", blink2, blink2Stack, sizeof(blink2Stack)>>2, 100);
  addThread("Report", report, repStack, sizeof(repStack)>>2, 255);
  addThread("SquareHog", hog, hogStack, sizeof(hogStack)>>2, 0);
  addThread("Spinner", spinner, spinnerStack, sizeof(spinnerStack)>>2, 150);
  setupSched(); // No return
}
