### Raspberry Pi RP2040 Realtime OS <br/>
<br/>
Hobby project to create a simple, pre-emptive, real-time OS support
for the RP2040. Current capabilities include: <br/>
<br/>
Pre-emptive<br/>
Thread priority<br/>
Waits/delays<br/>
Yield<br/>
Mutexes<br/>
<br/>
<br/>
As of 917b17b7, the context switch times on the 125MHz Pico are:<br/>
No optimizations <br/>
No stats collection:  5.8 uS<br/>
With stats collecton: 9.5 uS<br/>
<br/>
Example code has a simple "top" like output every second to the serial port.<br/>
<pre>
Wall time 15
CPU0 Ctx=0.102%, Util=100.000%  Idle=  0.000%
CPU1 Ctx=0.097%, Util=  2.251%  Idle= 97.749%
Thrd   Name     S PRI  CPU     LastCPU
   0 Red LED    W 100  0.002%  0
   1 Green LED  W 100  0.011%  0
   2 Report     R 255  2.141%  1
   3 Spinner    R 150 99.874%  0
</pre>


The current implementation consists of three files:<br/>
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

