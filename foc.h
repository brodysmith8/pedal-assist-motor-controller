/*
    This FOC implementation is described formally in https://doi.org/10.1016/B978-0-323-91162-7.00010-2
*/

#ifndef FOC_H
#define FOC_H
#include "math_functions.h"

// motor-specific parameters
#define MOTOR_INDUCTANCE_HENRY 0.0002
#define MOTOR_RESISTANCE_OHMS 0.12
#define MOTOR_MAX_TORQUE_NM 45.5
#define MOTOR_POLES 20.0

// precalculated motor-specific constants
#define MOTOR_POLE_PAIRS (MOTOR_POLES / 2.0)

// precalculated constants
#define INVERSE_ROOT_TWO (1.0 / sqrt(2.0))
#define INVERSE_ROOT_THREE (1.0 / sqrt(3.0))
#define INVERSE_ROOT_SIX (1.0 / sqrt(6.0))
#define ROOT_TWO_OVER_THREE sqrt(2.0 / 3.0)
#define TWO_PI 6.28318530718
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