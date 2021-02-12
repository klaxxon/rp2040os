### Raspberry Pi RP2040 Realtime OS <br/>
<br/>
Hobby project to create a simple, pre-emptive, real-time OS support
for the RP2040. Current capabilities include: <br/>
<br/>
Thread priority<br/>
Efficient sleep/delay<br/>
Cooperative yield<br/>
<br/>
<br/>


The current implementation consists of three files:<br/>
<br/>
<code>
os.c    OS implementation in C<br/>
func.s  OS implementation function in assembly<br/>
os.h    OS header file<br/>
<br/>
</code>

###Builing###<br/>
export PICO_SDK_PATH=path to your pico-sdk<br/>
Copy the $PICO_SDK_PATH/external/pico_sdk_import.cmake into main directory.<br/>
<code>
$> mkdir build<br/>
$> cd build<br/>
$> cmake .. -DCMAKE_BUILD_TYPE=Debug<br/>
$> make<br/>
</code>

