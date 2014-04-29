#include "mbed.h"

#if defined(TARGET_K64F)
<<<<<<< HEAD
#define TEST_LED D5
=======
#define TEST_LED D9
>>>>>>> 5bf985ebc651a2c31cefabd9d62c51dc465ef60a

#elif defined(TARGET_NUCLEO_F103RB)
#define TEST_LED D3

#else
#error This test is not supported on this target.
#endif

PwmOut led(TEST_LED);

int main() {
    float crt = 1.0, delta = 0.05;

    led.period_ms(2); // 500Hz
    while (true) {
        led.write(crt);
        wait_ms(50);
        crt = crt + delta;
        if (crt > 1.0) {
            crt = 1.0;
            delta = -delta;
        }
        else if (crt < 0) {
            crt = 0;
            delta = -delta;
        }
    }
}
