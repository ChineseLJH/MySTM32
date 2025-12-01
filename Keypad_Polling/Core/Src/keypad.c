#include "keypad.h"

const char keymap[4][4] =
    {
        {'1', '2', '3', '+'},
        {'4', '5', '6', '-'},
        {'7', '8', '9', '*'},
        {'.', '0', '#', '/'}};
uint16_t output_pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};
uint16_t input_pins[] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

void Keypad_Reset(void)
{
    // Implementation for resetting the keypad state
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
}

char Keypad_Scan(uint16_t *input_key)
{
    for (int i = 0; i <= 3; i++)
    {
        HAL_GPIO_WritePin(GPIOA, output_pins[i], GPIO_PIN_SET);
        if (HAL_GPIO_ReadPin(GPIOA, *input_key) == GPIO_PIN_SET)
            return keymap[i][(input_key - input_pins)];
    }

    return '.';
}

