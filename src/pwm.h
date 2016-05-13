// pwm.h
// adapted from https://github.com/basilfx/LPC810-FanController/blob/13396add42e1e850be6507b6471ecfede6403d4a/firmware/src/main.c

#include "LPC8xx.h"

void init_pwm();
void set_pwm(uint8_t percentage);

