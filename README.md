### Raspberry Pi RP2040 Realtime OS <br/>
<br/>
Hobby project to create a simple, pre-emptive, real-time OS support
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
   <tr><td>With stats collecton:</td><td>9.5 uS</td></tr>
   <tr><td>With stats/stack checks:</td><td> 9.9 uS</td></tr>
</table>
<br/>
Example code has a simple "top" like output every two seconds to the serial port.<br/>
<pre>
Wall time 0:36
CPU0 Ctx=0.144%, Util=  1.215%  Idle= 98.785%
CPU1 Ctx=0.079%, Util=  0.082%  Idle= 99.918%
Thrd   Name     S PRI  CPU     LastCPU
   0 Red LED    W 100  0.001%  0
   1 Green LED  W 100  0.011%  0
   2 Report     R 255  1.100%  0
   3 Spinner    Z 150  0.000%  1

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

### Building<br/>
export PICO_SDK_PATH=path to your pico-sdk<br/>
Copy the $PICO_SDK_PATH/external/pico_sdk_import.cmake into main directory.<br/>
<pre>
$> mkdir build
$> cd build
$> cmake .. -DCMAKE_BUILD_TYPE=Debug
$> make
</pre>

