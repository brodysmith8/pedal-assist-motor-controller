#ifndef SEVEN_SEGMENT_DISPLAY_H
#define SEVEN_SEGMENT_DISPLAY_H
#include "math.h"
#include "stdlib.h"

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
    int fractional_portion = (int)fabs(round(fractional_portion_f * pow(10.0, (double)decimal_places)));
    //printf("integral portion: %d\n", integral_portion);
    unsigned int digits[6] = {0};
    int idx = 0;
    // zero-fill first in case fractional portion is les sthan max
    for (int i = 0; i < 6; i++) {
        digits[i] = 0;
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

/*
    This FOC implementation is described formally in https://doi.org/10.1016/B978-0-323-91162-7.00010-2
*/

#ifndef FOC_H
#define FOC_H
#include "math.h"

// motor-specific parameters
#define MOTOR_INDUCTANCE_HENRY 0.0002
#define MOTOR_RESISTANCE_OHMS 0.12
#define MOTOR_MAX_TORQUE_NM 45.5
#define MOTOR_POLES 20.0

// precalculated motor-specific constants
#define MOTOR_POLE_PAIRS (MOTOR_POLES / 2.0)

// precalculated constants
#define INVERSE_ROOT_TWO 1.0 / sqrt(2.0)
#define INVERSE_ROOT_THREE 1.0 / sqrt(3.0)
#define INVERSE_ROOT_SIX 1.0 / sqrt(6.0)
#define ROOT_TWO_OVER_THREE sqrt(2.0 / 3.0)
#define TWO_PI 6.283
#define RADS_TO_DEG_MULTIPLIER 57.2957795

// easier to do this than passing around array pointers everywhere
typedef struct _DoubleVector3D {
    double vec[3]; // use memcpy(sizeof(&arr_to_copy)) to assign to this
} DoubleVector3D;

// Output a vector representing the instantaneous stator current components in (a, b, c) axes.
// This vector takes form \vec{i_s} = i_a + i_b * e^{j * 2pi/3} + i_c * e^{j * 4pi/3}
// void get_hall_effect_phase_current_vector(FloatVector3D *);

// Power-invariant Clarke transform
// Input is a stator current vector, output is space vector in (alpha, beta, 0) axes
void clarke_transform(DoubleVector3D *);

void inverse_clarke_transform(DoubleVector3D*);

void park_transform(DoubleVector3D *, double *, double *, double *);

void inverse_park_transform(DoubleVector3D*, double*, double*);

void calculate_normalized_stator_current_phase_angle(double *, double *);

void calculate_normalized_space_vector_phase_angle(DoubleVector3D*, double*);

void calculate_output_voltages(DoubleVector3D *, double *);

void normalize_voltages(DoubleVector3D*);

// if we are given rotor position, since BLDC is synchronous (i.e. magn field generated in stator and that in rotor
// rotate at the same freq.) we don't need to account for slip difference, so all we need is rotor position in rads
// given by rotary encoder in motor (thank God)
void foc(double *, double *, double *, double *, DoubleVector3D *, double *);

#endif

// Output a vector representing the instantaneous stator current components in (a, b, c) axes.
// This vector takes form \vec{i_s} = i_a + i_b * e^{j * 2pi/3} + i_c * e^{j * 4pi/3}
// void get_hall_effect_phase_current_vector(FloatVector3D *rotation_vector_buffer) {
//     rotation_vector_buffer->vec[0] = 0.0f;
//     rotation_vector_buffer->vec[1] = 0.0f;
//     rotation_vector_buffer->vec[2] = 0.0f;
// }

// Power-invariant Clarke transform
// Input is a stator current vector, output is time-variant space vector in (alpha, beta, 0) axes
void clarke_transform(DoubleVector3D *rotation_vector_buffer) {
    double i_a = rotation_vector_buffer->vec[0];
    double i_b = rotation_vector_buffer->vec[1];
    double i_c = rotation_vector_buffer->vec[2];

    double i_alpha = (2 * i_a - i_b - i_c) * INVERSE_ROOT_SIX;
    double i_beta = (i_b - i_c) * INVERSE_ROOT_TWO;
    rotation_vector_buffer->vec[0] = i_alpha;
    rotation_vector_buffer->vec[1] = i_beta;
    rotation_vector_buffer->vec[2] = 0.0;
}

// Power-invariant inverse Clarke transform
// Input is a time-variant two-phase rotation vector in (alpha, beta, 0) axes, output is time-variant
// three-phase space vector in (a, b, c) axes representing necessary control voltage vector for inverter
// input is I_pg 252 fig 4.27 in [1]
void inverse_clarke_transform(DoubleVector3D *rotation_vector_buffer) {
    double i_alpha = rotation_vector_buffer->vec[0];
    double i_beta = rotation_vector_buffer->vec[1];
    double i_gamma = rotation_vector_buffer->vec[2];

    double i_a = i_gamma * INVERSE_ROOT_THREE;
    double i_b = i_a - i_alpha * INVERSE_ROOT_SIX;
    double i_c = i_b - i_beta * INVERSE_ROOT_TWO;
    i_b += i_beta * INVERSE_ROOT_TWO;
    i_a += i_alpha * ROOT_TWO_OVER_THREE;

    rotation_vector_buffer->vec[0] = i_a;
    rotation_vector_buffer->vec[1] = i_b;
    rotation_vector_buffer->vec[2] = i_c;
}

// Input is time-variant two-phase rotation vector in (alpha, beta, 0) with rotor position in rads,
// outputs time-invariant rotation vector in (d, q) axes
void park_transform(DoubleVector3D *rotation_vector_buffer, double *rotor_position_rads, double *cosine, double *sine) {
    double i_alpha = rotation_vector_buffer->vec[0];
    double i_beta = rotation_vector_buffer->vec[1];

    // calculate these once per FOC iteration for computational efficiency
    *cosine = cos(*rotor_position_rads);
    *sine = sin(*rotor_position_rads);

    double i_d = *cosine * i_alpha + *sine * i_beta;
    double i_q = *cosine * i_beta - *sine * i_alpha;
    rotation_vector_buffer->vec[0] = i_d;
    rotation_vector_buffer->vec[1] = i_q;
}

// Input is (d, q) axis rotation vector and output is (alpha, beta, 0) time-variant rotation vector
// Uses cosine and sine calculated from the forward park transform and thus the associated rotor position
void inverse_park_transform(DoubleVector3D *rotation_vector_buffer, double *cosine, double *sine) {
    double i_d = rotation_vector_buffer->vec[0];
    double i_q = rotation_vector_buffer->vec[1];

    double i_alpha = i_d * *cosine - i_q * *sine;
    double i_beta = i_d * *sine + i_q * *cosine;

    rotation_vector_buffer->vec[0] = i_alpha;
    rotation_vector_buffer->vec[1] = i_beta;
}

// normalized in that 0<=phase angle<=2pi 
void calculate_normalized_stator_current_phase_angle(double* rotor_position_rads, double* phase_angle_rads) {
    double angle = fmod(MOTOR_POLE_PAIRS * (*rotor_position_rads), TWO_PI);
    if (angle >= 0) {
        *phase_angle_rads = angle;
    } else {
        *phase_angle_rads = angle + TWO_PI;
    }
}

// calculate phi in SVPWM hex axis
void calculate_normalized_space_vector_phase_angle(DoubleVector3D* rotation_vector_buffer, double* phase_angle_rads) {
    *phase_angle_rads = atan(rotation_vector_buffer->vec[1] / rotation_vector_buffer->vec[0]); // arctan(beta/alpha)
}

// Input (d, q) current rotation vector, output reference voltage vector in (d, q) axis for the inverse
// Clarke/Park transform for inverter control voltage vector in (a, b, c)
void calculate_output_voltages(DoubleVector3D *rotation_vector_buffer, double *target_torque_nm) {
    // calculate "error signal" E(s) as defined in https://doi.org/10.1016/B978-012471370-3/50010-3 pg 619 eq. 4.7
    double id_error = 0 - rotation_vector_buffer->vec[0];                 // i_sdref - i_sd
    double iq_error = *target_torque_nm - rotation_vector_buffer->vec[1]; // i_sqref - i_sq

    /*
     proportional integration controller step to convert currents to reference voltages
     Using equations described in https://doi.org/10.1016/B978-012471370-3/50010-3 pg 617 eq. 3.12 and also
     V(s) = K + K_I / s = alpha L + alpha R / s
    [https://odr.chalmers.se/server/api/core/bitstreams/ee9d8007-36b9-4ef5-84e8-99bfd337ccf7/content, eq 2.26] alpha = 1 / T_e, where
    T_e is the "closed-loop electrical time constant", defined as T_e = t_er / ln(9) (t_er = rise time) motor specific? yes! T_e is
    related to i_q: i_q = T_e / K_m (this is the goal of the PI step), where K_m is the torque constant of the rotor (Newton metres per
    Amp). For the Bafang SWX02, I can't find K_m anywhere, but it is in the range of 0.92 Nm / A as per this simulation
    https://ebikes.ca/tools/simulator.html?motor=cust_10.38_0.12_0.2_20_0.77_0.0185_0&batt=B4816_GA&cont=FR_H

    Additionally, motor inductance (L) and resistance (R) are needed here. They are also not available anywhere, so per the above
    simulation, they are estimated at L = 0.2 mH and R = 0.12
    */

    // // this is actually two PI controllers acting in the same code
    // float v_sd = 0.0;
    // float v_sq = 0.0;
    // float T_s = 0.000001; // say the time between calculations is about 10 ms (doesn't really matter here)

    // // u(s) = (P + I/s + Ds) * e(s), but this is a discrete implementation
    // float d_proportional = (-rotation_vector_buffer->vec[0]); // u_p = P * e(k)
    // float d_integral = pid_d_integral_prev +

    //     v_sd =

    // going to forgo PI controller stuff right now and just make something a bit simpler. It's too much
    // control theory for me to learn right now and I need to balance other responsibilities :(

    // instead of PI control, we will make it so that we have a max output voltage of 5 V, and the output voltages
    // will be proportional to this in conj. with the max operational torque of the SWX02 (~45 N*m). Vsd should always
    // be zero anyways because BLDC is a synchronous machine and requires no axial torque (i.e. torque in the d-axis),
    // so we should always just aim for 0.
    double v_sd = 0 + id_error;
    double v_sq = *target_torque_nm + iq_error;
    rotation_vector_buffer->vec[0] = v_sd;
    rotation_vector_buffer->vec[1] = v_sq;
}

//
void normalize_voltages(DoubleVector3D *rotation_vector_buffer) {
    double sum = fabs(rotation_vector_buffer->vec[0]) + fabs(rotation_vector_buffer->vec[1]) + fabs(rotation_vector_buffer->vec[2]);
    rotation_vector_buffer->vec[0] = (rotation_vector_buffer->vec[0] / sum) * 5.0;
    rotation_vector_buffer->vec[1] = (rotation_vector_buffer->vec[1] / sum) * 5.0;
    rotation_vector_buffer->vec[2] = (rotation_vector_buffer->vec[2] / sum) * 5.0;
}

// if we are given rotor position, since BLDC is synchronous (i.e. magn field generated in stator and that in rotor
// rotate at the same freq.) we don't need to account for slip difference, so all we need is rotor position in rads
// given by rotary encoder in motor (thank God)
void foc(double *id, double *iq, double *target_torque_nm, double *rotor_position_rads, DoubleVector3D *rotation_vector_buffer, double *phase_angle_rads) {
    DoubleVector3D flux_linkage_buffer;
    DoubleVector3D output_voltage_buffer;
    double cosine = 0.0;
    double sine = 0.0;

    //get_hall_effect_phase_current_vector(rotation_vector_buffer);
    clarke_transform(rotation_vector_buffer);                                    // (a, b, c) -> (alpha, beta, 0)
    park_transform(rotation_vector_buffer, rotor_position_rads, &cosine, &sine); // (alpha, beta, 0) -> (direct, quadrature)
    calculate_output_voltages(rotation_vector_buffer, target_torque_nm);         // output is still in d,q
    inverse_park_transform(rotation_vector_buffer, &cosine, &sine);              // (d,q) -> (alpha, beta, 0)
    normalize_voltages(rotation_vector_buffer);
    calculate_normalized_space_vector_phase_angle(rotation_vector_buffer, phase_angle_rads);
    //inverse_clarke_transform(rotation_vector_buffer);                            // (alpha, beta, 0) -> (a, b, c) new phase voltages are ready to be sent to inverter now
}

#include "stdio.h"

#define SW_BASE 0xFF200040
#define GPIO_BASE 0xFF200060
#define ADC_BASE 0xFF204000
#define GPIO_CFG 0b111111111
#define BUTTON_BASE 0xFF200050

#define ADC_TO_RADS_MULTIPLIER_X1000 1.5339807
#define ADC_TO_NM_MULTIPLER (MOTOR_MAX_TORQUE_NM / 4096.0)

double BOOST_COEFFICIENT_LOOKUP_T[4] = {
    0.0,
    1.0,
    1.25,
    1.5
};

#define yes 1
#define no 0

volatile int DELAY_LENGTH = 2000000;
volatile int delay_count = 0;
int display_mode = 0;
int potentiometer_1_voltage = 0; // 2 bytes
int potentiometer_2_voltage = 0;
volatile int *gpio_ptr = (int *)(GPIO_BASE);
volatile int *adc_channel0_ptr = (int *)(ADC_BASE);
volatile int *adc_channel1_ptr = (int *)(ADC_BASE + 0x4);
volatile char *switch_bank_ptr = (char *)(SW_BASE);

volatile int *const button_bank_ptr = (int *)(BUTTON_BASE);
void write_buttons_to_buffer(char *buffer) {
    *buffer = *button_bank_ptr & 0b1000;
    *(buffer + 0x1) = *button_bank_ptr & 0b0100;
    *(buffer + 0x2) = *button_bank_ptr & 0b0010;
    *(buffer + 0x3) = *button_bank_ptr & 0b0001;
}

inline void start_adc() {
    *adc_channel1_ptr = 1; // auto update
}

int voltage_capture = 0;
void read_potentiometers(int *voltage_1, int *voltage_2) {
    voltage_capture = *adc_channel0_ptr;
    // printf("vc: %d\n", voltage_capture);

    if (voltage_capture & 0x10000) { // set to 0x8000 for lab
        *voltage_1 = voltage_capture & 0b0000111111111111;
        //printf("v_1: %d\n", *voltage_1);
        start_adc();
    }

    voltage_capture = *adc_channel1_ptr;
    if (voltage_capture & 0x10000) {
        *voltage_2 = voltage_capture & 0b0000111111111111;
        start_adc();
    }

    // printf("voltage 1 %d\n", *voltage_1);
    // printf("voltage 2 %d\n", *voltage_2);
}

inline void setup_gpio() {
    *(gpio_ptr + 0x4) = GPIO_CFG; // set direction control register
}

// display the requested torque
int to_write = 0;
void display_reading_on_led_bank(short *value) {
    // clear display before writing new
    // printf("value %d\n", *value);
    *gpio_ptr = 0;
    to_write = 0;
    to_write += ((*value & 0b100000000000) != 0) * 5;
    to_write += ((*value & 0b10000000000) != 0) * 2;
    to_write += ((*value & 0b1000000000) != 0) * 1;
    to_write += ((*value & 0b100000000) != 0) * 1;
    to_write += ((*value & 0b11111000) != 0) * 1; // last 3 are so insignificant that it doesn't really matter
    *gpio_ptr = to_write;
}

void get_boost_mode(int *boost_mode) {
    char buttons[4];
    write_buttons_to_buffer(buttons);
    if (buttons[3]) {
        *boost_mode = 1;
        return;
    } else if (buttons[2]) {
        *boost_mode = 2;
        return;
    } else if (buttons[1]) {
        *boost_mode = 3;
        return;
    }
    // otherwise, leave boost_mode unaffected
}

void get_display_mode(int *display_mode_buffer) { *display_mode_buffer = *(int *)(SW_BASE); }

// potentiometer 1
void get_input_torque(double *input_torque) {
    // round up in case of inaccurate ADC
    if (potentiometer_1_voltage > 4080.0) {
        *input_torque = MOTOR_MAX_TORQUE_NM;
        return;
    }

    *input_torque = potentiometer_1_voltage * ADC_TO_NM_MULTIPLER;
    //printf("torque in adc voltage: %d\nmultiplier: %f\ntorque in: %f\n", potentiometer_1_voltage, ADC_TO_NM_MULTIPLER, *input_torque);
}

// potentiometer 2
void get_rotor_position(double *rotor_position_rads) {
    // round up in case of inaccurate ADC
    if (potentiometer_2_voltage > 4080.0) {
        *rotor_position_rads = TWO_PI;
        return;
    }

    // the faster way is just multiplying it by a float multiplier with less decimal places, but that's 
    // really inaccurate because the float but still works for about the first two decimal places 
    *rotor_position_rads = ((double)potentiometer_2_voltage) * ADC_TO_RADS_MULTIPLIER_X1000;
    //printf("rotor position adc voltage: %d\nmultiplier: %f\nrotor position: %f\n", potentiometer_2_voltage, ADC_TO_RADS_MULTIPLIER_X1000, *rotor_position_rads);
}

double rads_to_degrees(double phase_angle_rads) { return RADS_TO_DEG_MULTIPLIER * phase_angle_rads; }

void normalize_boost_torque(double *requested_torque) {
    if (*requested_torque > MOTOR_MAX_TORQUE_NM) {
        *requested_torque = MOTOR_MAX_TORQUE_NM;
    }
}

int main(void) {
    /* software-only values */
    int is_running = 1;

    /* control values */
    int boost_mode = 0;
    double input_torque_newton_metres = 0.0;

    double id = 0.0;
    double iq = 0.0;
    double calculated_torque_newton_metres = 0.0;
    double rotor_position_rads = 0.0;

    double test_alpha = -3.4444;
    double test_beta = 4.5555;
    double test_phase_angle_rads = 6.12432;
    double test_rotor_angle_rads = 3.14159;
    double test_torque = 43.432;

    DoubleVector3D rotation_vector_buffer;
    rotation_vector_buffer.vec[0] = 0.0;
    rotation_vector_buffer.vec[1] = 0.0;
    rotation_vector_buffer.vec[2] = 0.0;
    DoubleVector3D inverter_voltage_control_vector;
    double phase_angle_rads = 0.0;

    start_adc();
    setup_gpio();

    // here would be a complex algorithm to determine the initial position of the rotor, or,
    // if an absolute rotary encoder is used, it would just read the position. Since this is
    // a potentiometer, we get actual shaft position (which is analogous to rotor position)
    // by just reading it (thankfully)

    while (boost_mode == 0) {
        // SELECt ... BOOSt ... SELECt ... BOOSt ...
        get_boost_mode(&boost_mode); // from button
        //printf("boost mode: %d\n", boost_mode);
        display_select();
        for (delay_count = DELAY_LENGTH; delay_count != 0; --delay_count)
            ; // delay loop
        display_boost();
        for (delay_count = DELAY_LENGTH; delay_count != 0; --delay_count)
            ; // delay loop
        get_boost_mode(&boost_mode); // from button
        //printf("boost mode: %d\n", boost_mode);
    }

    while (is_running) {
    // for (int i = 0; i < 100; i++) {
        // meta (demo-only): read potentiometers
        read_potentiometers(&potentiometer_1_voltage, &potentiometer_2_voltage);

        get_input_torque(&input_torque_newton_metres); // with GPIO (potentiometer 1)
        get_rotor_position(&rotor_position_rads);
        calculated_torque_newton_metres = BOOST_COEFFICIENT_LOOKUP_T[boost_mode] * input_torque_newton_metres;
        //printf("calculated torque: %f\n", calculated_torque_newton_metres);
        normalize_boost_torque(&calculated_torque_newton_metres);
        //printf("normalized calculated torque (torque to motor): %f\n", calculated_torque_newton_metres);

        foc(&id, &iq, &calculated_torque_newton_metres, &rotor_position_rads, &inverter_voltage_control_vector, &phase_angle_rads);

        // rotor_position_rads += 0.00001; // get_rotary_encoder_position(&rotor_position_rads); // with GPIO (potentiometer 2)

        printf("v_alpha: %f\nv_beta: %f\nphase angle rads: %f\nphase angle degs: %f\nrotor position: %f\ntorque force: \n\n",
         inverter_voltage_control_vector.vec[0],
                inverter_voltage_control_vector.vec[1], phase_angle_rads, rads_to_degrees(phase_angle_rads), rotor_position_rads);//,
        //        calculate_actual_torque(inverter_voltage_control_vector.vec[0], inverter_voltage_control_vector.vec[1]));

        get_display_mode(&display_mode); // from switch bank
        switch (display_mode) {
        case 0:
            // display v_alpha
            display_floating_point(&inverter_voltage_control_vector.vec[0], 4, yes); //&inverter_voltage_control_vector.vec[0]);
            break;
        case 1:
            // display v_beta
            display_floating_point(&inverter_voltage_control_vector.vec[1], 4, yes);//&inverter_voltage_control_vector.vec[1]);
            break;
        case 2:
            // display phase angle rads
            display_floating_point(&phase_angle_rads, 5, no);
            break;
        case 3:
            // display rotor position rads
            display_floating_point(&rotor_position_rads, 2, no); // 2 dp because was having trouble with multiplier accuracy (fixed now)
            break;
        case 4:
            // display boost setting
            display_boost_setting(&boost_mode);
            break;
        case 5:
            // display input torque N*m
            display_floating_point(&input_torque_newton_metres, 4, no);
            break;
        case 6:
            // display multiplied torque N*m
            display_floating_point(&calculated_torque_newton_metres, 4, no);
            break;
        }
        get_boost_mode(&boost_mode); // from button
        for (delay_count = DELAY_LENGTH; delay_count != 0; --delay_count)
            ; // delay loop
    }

    return 0;
}