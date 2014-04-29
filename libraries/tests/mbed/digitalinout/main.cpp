#include "test_env.h"

<<<<<<< HEAD
#if defined(TARGET_K64F)
DigitalInOut d1(D0);
DigitalInOut d2(D7);

#elif defined(TARGET_KL25Z)
DigitalInOut d1(PTA5);
DigitalInOut d2(PTC6);

#elif defined(TARGET_KL05Z)
DigitalInOut d1(PTB11);
DigitalInOut d2(PTB1);
=======
#if defined(TARGET_FF_ARDUINO)
DigitalInOut d1(D0);
DigitalInOut d2(D7);
>>>>>>> 5bf985ebc651a2c31cefabd9d62c51dc465ef60a

#elif defined(TARGET_LPC1114)
DigitalInOut d1(dp1);
DigitalInOut d2(dp2);

#elif defined(TARGET_NUCLEO_F103RB)
DigitalInOut d1(PC_6);
DigitalInOut d2(PB_8);

#else
DigitalInOut d1(p5);
DigitalInOut d2(p25);

#endif


int main() {
    bool check = true;


    d1.output();
    d2.input();
    d1 = 1; wait(0.1); if (d2 != 1) check = false;
    d1 = 0; wait(0.1);  if (d2 != 0) check = false;

    d1.input();
    d2.output();
    d2 = 1; wait(0.1); if (d1 != 1) check = false;
    d2 = 0; wait(0.1); if (d1 != 0) check = false;

    notify_completion(check);
}
