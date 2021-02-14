
// Set to the cips system clock speed in Hz
#define SYSTEM_CLOCK_HZ 125000000
// Number of cores to use
#define MAX_CORES 2
// Max number of user tasks allowed
#define USER_TASKS 5
// Microseconds between context switches
#define SCHED_INTERVAL_US 10000
// Assign names to threads (uses a little more memory)
#define USE_THREAD_NAMES 1
// Use a little extra memory to collect thread/cpu stats
#define COLLECT_STATS 1
// Uses 12 bytes to mark beginning and end of stack and checks
//#define STACK_WATCH 1

// Option hardware pin to use as output when a context switch occurs
// Make sure the pin is initialized as an output before initializing
// scheduler.
#define CONTEXTSW_PIN 14
