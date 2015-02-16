#include "mbed.h"
#include "MPS2_RESOURCE_PACK.h"
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);
DigitalOut myled5(LED5);
DigitalOut myled6(LED6);
DigitalOut myled7(LED7);
DigitalOut myled8(LED8);

Serial pc(USBTX,USBRX);

int main() {
    while(1) {
            myled1 = 1;
            wait(0.2);
            myled2 = 1;
            wait(0.2);
            myled1 = 0;
            wait(0.2);
            myled3 = 1;
            wait(0.2);
            myled2 = 0;
            wait(0.2);
            myled4 = 1;
            wait(0.2);
            myled3 = 0;
            wait(0.2);
            myled5 = 1;
            wait(0.2);
            myled4 = 0;
            wait(0.2);
            myled6 = 1;
            wait(0.2);
            myled5 = 0;
            wait(0.2);
            myled7 = 1;
            wait(0.2);
            myled6 = 0;
            wait(0.2);
            myled8 = 1;
            wait(0.2);
            myled7 = 0;
            wait(0.2);
            myled8 = 0;
            wait(0.2);
            pc.printf("hello world");
            
    }
}
