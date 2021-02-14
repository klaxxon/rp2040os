#include <stdio.h>
#include <string.h>
#include "rp2040os.h"

#define LED_PIN 25 
#define LED_PIN2 16
#define CONTEXTSW_PIN 14




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
      uint64_t exec = threads[a].execTime-lasts[a];
      printf("CPU%d Ctx=%0.3f%%, ", a, 100.0*cpuStats[a].contextTime/now);
      float cpu = (100.0*exec)/elapsed;
      printf("Util=%7.3f%%  Idle=%7.3f%%\n", 100-cpu, cpu);
      lasts[a] = threads[a].execTime;
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
  }
}


static uint32_t spinnerStack[128];
void spinner() {
  while (true) {
    for(uint16_t a=0;a<25000;a++);
    yield();
  }
}


int main() {
  stdio_init_all(); 
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  gpio_init(LED_PIN2);
  gpio_set_dir(LED_PIN2, GPIO_OUT);
  gpio_init(CONTEXTSW_PIN);
  gpio_set_dir(CONTEXTSW_PIN, GPIO_OUT);
  gpio_init(CONTEXTSW_PIN+1);
  gpio_set_dir(CONTEXTSW_PIN+1, GPIO_OUT);

  addThread("Red LED", blink, blinkStack, sizeof(blinkStack)>>2, 100);
  addThread("Green LED", blink2, blink2Stack, sizeof(blink2Stack)>>2, 100);
  addThread("Report", report, repStack, sizeof(repStack)>>2, 255);
  addThread("Spinner", spinner, spinnerStack, sizeof(spinnerStack)>>2, 150);
  setupSched(); // No return
}
