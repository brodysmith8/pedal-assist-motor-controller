#include "seven_segment_display.h"
#include "foc.h"
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

    if (voltage_capture & 0x8000) { // set to 0x8000 for lab, 0x10000 for sim
        *voltage_1 = voltage_capture & 0b0000111111111111;
        //printf("v_1: %d\n", *voltage_1);
        start_adc();
    }

    voltage_capture = *adc_channel1_ptr;
    if (voltage_capture & 0x8000) {
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