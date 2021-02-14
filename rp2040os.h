#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "osConfig.h"


// Each core has its own idle thread 
#define MAX_TASKS (MAX_CORES + USER_TASKS)
#define SCHED_TIMER_NUM 0
#define SCHED_IRQ TIMER_IRQ_0
#define IDLE_STACKSIZE 100
#define CONTEXTSWITCH_SPINLOCK ((uint32_t*)(SIO_BASE+SIO_SPINLOCK0_OFFSET))
#define SYSTCOUNTER ((SYSTEM_CLOCK_HZ / 1000000)*SCHED_INTERVAL_US)

#define GPIO_OUT_SET 0x014
#define GPIO_OUT_CLR 0x018
#define setGPIO(x) (*(uint32_t*)(SIO_BASE+GPIO_OUT_SET) = (1<<x))
#define clrGPIO(x) (*(uint32_t*)(SIO_BASE+GPIO_OUT_CLR) = (1<<x))
#define SYSTICKVAL  ((*(volatile uint32_t *)(0xe0000000|M0PLUS_SYST_CVR_OFFSET))&0xffffff)


typedef struct CpuStats {
  uint64_t lastContextTime;
  #ifdef COLLECT_STATS
  uint64_t contextTime;
  #endif
} CpuStats;


enum threadStatus {
  THREAD_STATUS_IDLE = 0,
  THREAD_STATUS_RUNNING = 1,// Can be run
  THREAD_STATUS_WAIT = 2,
  THREAD_STATUS_DONE = 3,
  THREAD_STATUS_ZOMBIE = 4, // Thread exited
};

typedef struct Thread Thread;
struct Thread {
  uint32_t stackPtr;  // Top of stack
  uint8_t pid;
  volatile enum threadStatus status;
  #ifdef COLLECT_STATS
  uint64_t execTime;
  volatile uint8_t lastcpu; // Last cpu used
  #endif
  volatile uint64_t waitExpires;
  #ifdef USE_THREAD_NAMES
  char *name;
  #endif
  uint8_t priority;
  volatile uint8_t cpu; // Active on CPU, 255 = not active
  volatile bool yielded;
  Thread *next; // So we do not have to index
  #ifdef STACK_WATCH
  uint32_t *stack;
  uint16_t stackSize;
  #endif
};

extern CpuStats cpuStats[MAX_CORES];
extern struct Thread threads[MAX_TASKS];
extern uint8_t threadCount;


void delayus(uint32_t us);
#define delayms(x) delayus(x*1000)
void startStackThread(uint32_t stackPtr);
uint64_t getTimeUs();
uint8_t getPID();
typedef uint32_t Mutex;
void initMutex(Mutex*);
void mutexLock(Mutex*);
void mutexUnlock(Mutex*);


// 
void setupSched();
void yield();
// Adds a thread to OS.  Stack points to the beginning of thread stack.  Len is the size of the thread stack.
#ifdef USE_THREAD_NAMES
void addThread(char *name, void(*hndl)(void*), uint32_t *stack, uint16_t len, uint8_t priority);
#else 
void addThread(void(*hndl)(void*), uint32_t *stack, uint16_t len);
#endif


