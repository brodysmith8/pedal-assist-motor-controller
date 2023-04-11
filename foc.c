/*
    This implementation is described formally in https://doi.org/10.1016/B978-0-323-91162-7.00010-2 ([1])
*/

#include "foc.h"

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