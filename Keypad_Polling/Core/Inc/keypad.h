#ifndef __KEYPAD_H
#define __KEYPAD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    extern const char keymap[4][4];
    extern uint16_t output_pins[];
    extern uint16_t input_pins[];

    void Keypad_Reset(void);

    char Keypad_Scan(uint16_t *input_key);

#ifdef __cplusplus
}
#endif

#endif /* __KEYPAD_H */