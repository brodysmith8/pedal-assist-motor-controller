#ifndef SEVEN_SEGMENT_DISPLAY_H
#define SEVEN_SEGMENT_DISPLAY_H
#include "math_functions.h"

#define HEX3_HEX0_BASE 0xFF200020
#define HEX5_HEX4_BASE 0xFF200030

typedef unsigned int hex_digit_t;

hex_digit_t SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[12] = {
    0b00111111 << 24, /* 0 */
    0b00000110 << 24, /* 1 */
    0b01011011 << 24, /* 2 */
    0b01001111 << 24, /* 3 */
    0b01100110 << 24, /* 4 */
    0b01101101 << 24, /* 5 */
    0b01111101 << 24, /* 6 */
    0b00000111 << 24, /* 7 */
    0b01111111 << 24, /* 8 */
    0b01101111 << 24, /* 9 */
    0b00000000 << 24, /*   */
    0b01000000 << 24  /* - */
};

unsigned int ERR_PATTERN[] = {
    0b01010000 << 24, /* r */
    0b01011100 << 24, /* o */
    0b01010000 << 24, /* r */
    0b01010000 << 24, /* r */
    0b01111001 << 24, /* E */
    0b00000000 << 24  /*   */
};

unsigned int SELECT_PATTERN[] = {
    0b01111000 << 24, /* t */
    0b00111001 << 24, /* C */
    0b01111001 << 24, /* E */
    0b00111000 << 24, /* L */
    0b01111001 << 24, /* E */
    0b01101101 << 24  /* S */
};

unsigned int BOOST_PATTERN[] = {
    0b00000000 << 24, /*   */
    0b01111000 << 24, /* t */
    0b01101101 << 24, /* S */
    0b00111111 << 24, /* O */
    0b00111111 << 24, /* O */
    0b01111111 << 24  /* B */
};

// volatile because hardware
volatile unsigned int *const hex_register_one_ptr = (unsigned int *)(HEX3_HEX0_BASE);
volatile unsigned int *const hex_register_two_ptr = (unsigned int *)(HEX5_HEX4_BASE);

void write_to_hex(unsigned int *to_write) {
    *hex_register_one_ptr = 0;
    *hex_register_two_ptr = 0;
    int idx;
    // register length - 1. Don't bitshift after last entry
    for (idx = 0; idx < 3; idx++) {
        *hex_register_one_ptr |= *(to_write + idx);
        *hex_register_one_ptr >>= 8;
    }

    *hex_register_one_ptr |= *(to_write + 3);

    *hex_register_two_ptr |= (*(to_write + 4) >> 24);
    *hex_register_two_ptr |= (*(to_write + 5) >> 16);
}

void display_select() { write_to_hex(SELECT_PATTERN); }

void display_boost() { write_to_hex(BOOST_PATTERN); }

void display_floating_point(double *num, int decimal_places, int can_be_negative) {
    int integral_places = 6 - decimal_places - can_be_negative;
    if (integral_places < 1) {
        write_to_hex(ERR_PATTERN);
        return;
    }

    int is_negative = *num < 0;
    double integral_portion_f = 0;
    double fractional_portion_f = modf(*num, &integral_portion_f);

    int integral_portion = (int)fabs(integral_portion_f);
    int fractional_portion = abs(round_f(fractional_portion_f * pow(10.0, (double)decimal_places)));
    //printf("integral portion: %d\n", integral_portion);
    unsigned int digits[6] = {0};
    int idx = 0;
    // zero-fill first in case fractional portion is les sthan max
    for (int i = 0; i < 6; i++) {
        digits[i] = SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[0];
    }

    do {
        digits[idx] = SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[fractional_portion % 10];

        fractional_portion /= 10; // floor of the number div 10 to advance to next place
        idx += 1;
    } while (fractional_portion > 0);

    do {
        digits[idx] = SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[integral_portion % 10];

        integral_portion /= 10; // floor of the number div 10 to advance to next place
        idx += 1;
    } while (integral_portion > 0);

    if (!can_be_negative) {
        write_to_hex(digits);
        return;
    }

    if (idx == 6 && is_negative) {
        write_to_hex(ERR_PATTERN);
        return;
    }
    
    digits[5] = is_negative ? SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[11] : SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[10]; // - or nothing
    
    write_to_hex(digits);
}

void display_boost_setting(int *boost_setting) {
    unsigned int digits[6] = {0};
    for (int i = 0; i < 6; i++) {
        digits[i] = BOOST_PATTERN[i];
    }
    digits[0] = SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[*boost_setting % 10];
    write_to_hex(digits);
}

#endif