### Raspberry Pi RP2040 Realtime OS <br/>
<br/>
Learning project to create a pre-emptive, real-time OS support
for the RP2040. Scheduler code is written in C to allow for easy
experimentation.  Current capabilities include: <br/>
<br/>
Pre-emptive<br/>
Thread priority<br/>
Waits/delays<br/>
Cooperative yield<br/>
Mutexes<br/>
Stack checks (issues breakpoint)<br/>
<br/>
<br/>
The context switch times on the 125MHz Pico with four threads are:<br/>
No optimizations <br/>
<table>
   <tr><th></th><th>Context Time</th></tr>
   <tr><td>No stats collection:</td><td>5.8 uS</td></tr>
   <tr><td>With stats collecton:</td><td>9.3 uS</td></tr>
   <tr><td>With stats/stack checks:</td><td> 9.9 uS</td></tr>
</table>
<br/>
Example code has a simple "top" like output every two seconds to the serial port. The Ctx= values are the 
percentage of time spent in the context switch. Util= is the percentage of time spent in a thread and Idle=
percent spent in one of the cores idle threads.<br/>
The S or status indicator is simply: W-waiting, R-running, Z-zombied or thread returned.<br/>
<pre>
Wall time 0:56:38
CPU0 Ctx=0.095%, Util=  1.229%  Idle= 98.771%
CPU1 Ctx=0.074%, Util=  0.081%  Idle= 99.919%
Thrd   Name     S PRI  CPU     LastCPU
   0 Red LED    W 100  0.001%  1
   1 Green LED  W 100  0.011%  0
   2 Report     R 255  1.115%  0
   3 Spinner    Z 150  0.000%  0
</pre>


The current implementation consists of three files and a main example:<br/>
<br/>
<pre>
rp2040os.h    OS header
rp2040os.c    Implementation in C
func.s        Implementation function in assembly
main.c        Example
<br/>
</pre>
A simple use case to kick off two threads:<br/>
<pre>
#include "rp2040.h"


static uint32_t blink1Stack[128];
void blink1() {
  while (true) {
    setGPIO(LED_PIN1);
    delayms(300);
    clrGPIO(LED_PIN1);
    delayms(300);
  }
}

static uint32_t blink2Stack[128];
void blink2() {
  while (true) {
    setGPIO(LED_PIN2);
    delayms(300);
    clrGPIO(LED_PIN2);
    delayms(300);
  }
}

int main() {
  stdio_init_all(); 
  gpio_init(LED_PIN1);
  gpio_set_dir(LED_PIN1, GPIO_OUT);
  gpio_init(LED_PIN2);
  gpio_set_dir(LED_PIN2, GPIO_OUT);
  
  addThread("Red LED", blink1, blink1Stack, sizeof(blink1Stack), 100);
  addThread("Green LED", blink2, blink2Stack, sizeof(blink2Stack), 100);
  setupSched(); // No return
}
</pre>
<br/>
### Stack sizes<br/>
Be aware that the more library functions you call, the more stack space you may require.  For example, using the printf function will require more than 256 32bit values in a stack to handle the functions memory requirements.  If the unit under test is attached to a debugger, the STACK_WATCH can be turned on and will trigger a BKPT (breakpoint) in the debugger if any of the stack guard values are modified, indicating a possible overflow condition.<br/>

<br/>
### osConfig.h<br/>
This file lets you customize how the scheduler works.  By default, it is set to the RP2040 specifications or two cores.  Setting the USER_TASKS to 
the correct amount for your application will minimize the amount of memory used.  Additional thread information such as naming, stats and stack monitoring also consume a little more memory, but can be very helpful with debugging.<br/>
The CONTEXTSW_PIN can be used with an oscilloscope to see the context switches in real-time.<br/>




### Building<br/>
export PICO_SDK_PATH=path to your pico-sdk<br/>
Copy the $PICO_SDK_PATH/external/pico_sdk_import.cmake into main directory.<br/>
<pre>
$> mkdir build
$> cd build
$> cmake .. -DCMAKE_BUILD_TYPE=Debug
$> make
</pre>

