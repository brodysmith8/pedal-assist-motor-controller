/*
    AUTHOR:         Brody Smith <bsmit272@uwo.ca>
    DATE:           April 14, 2023
    DESCRIPTION:    Motor controller for pedal-assisted bicycle, as part of 
                    final design project for ECE3375B - Microprocessors and 
                    Microcomputers
*/

/************************************************************
    IMPORTANT NOTE: Project-specific code starts at line 274
************************************************************/ 

/********************************************
Math Function Declaration and Implementation
********************************************/
#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

// Precalculated constants
#define PI 3.14159265359
#define PI_OVER_TWO 1.57079632679
#define PI_TIMES_TWO 6.28318530718

// Report reference [15]
double sqrt(double n){
  // Max and min are used to take into account numbers less than 1
  double lo = (1 > n ? n : 1), hi = (1 > n ? 1 : n), mid;

  // Update the bounds to be off the target by a factor of 10
  while(100 * lo * lo < n) lo *= 10;
  while(0.01 * hi * hi > n) hi *= 0.1;

  for(int i = 0 ; i < 100 ; i++){
      mid = (lo+hi)/2;
      if(mid*mid == n) return mid;
      if(mid*mid > n) hi = mid;
      else lo = mid;
  }
  return mid;
}

// Report reference [16]
double modf(double x, double* y) {
    union {double f; unsigned long long int i;} u = {x};
	unsigned long long int mask;
	int e = (int)(u.i>>52 & 0x7ff) - 0x3ff;

	/* no fractional part */
	if (e >= 52) {
		*y = x;
		if (e == 0x400 && u.i<<12 != 0) /* nan */
			return x;
		u.i &= 1ULL<<63;
		return u.f;
	}

	if (e < 0) {
		u.i &= 1ULL<<63;
		*y = u.f;
		return x;
	}

	mask = -1ULL>>12>>e;
	if ((u.i & mask) == 0) {
		*y = x;
		u.i &= 1ULL<<63;
		return u.f;
	}
	u.i &= ~mask;
	*y = u.f;
	return x - u.f;
}

// Self-made
int abs(int x) {
    return (x >= 0 ? x : -x);
}

// Report reference [17]
double fabs(double x) {
    union {double f; unsigned long long int i;} u = {x};
	u.i &= -1ULL/2;
	return u.f;
}

// Self-made
int round_f(double number) {
    return (number >= 0.0) ? (int)(number + 0.5) : (int)(number - 0.5);
}

// Self-made
int round_i(int number) {
    return (number >= 0) ? (int)((double)number + 0.5) : (int)((double)number - 0.5); 
}

// Self-made
double pow(double num, double exponent) {
    double temp = 1.0;
    for(int i = 0; i < exponent; i++) {
        temp*=num;
    }
    return temp;
}

// Report reference [18]  
int compare_float(double f1, double f2)
{
 double precision = 0.00000000000000000001;
 if ((f1 - precision) < f2)
  {
 return -1;
  }
 else if ((f1 + precision) > f2)
 {
  return 1;
 }
 else
  {
 return 0;
  }
}

// Report reference [18]  
double cos(double x){
 if( x < 0.0f ) 
  x = -x;

  if (0 <= compare_float(x,PI_TIMES_TWO)) 
 {
 do {
  x -= PI_TIMES_TWO;
  }while(0 <= compare_float(x,PI_TIMES_TWO));

  }

  if ((0 <= compare_float(x, PI)) && (-1 == compare_float(x, PI_TIMES_TWO)))
  {
   x -= PI;
   return ((-1)*(1.0f - (x*x/2.0f)*( 1.0f - (x*x/12.0f) * ( 1.0f - (x*x/30.0f) * (1.0f - (x*x/56.0f )*(1.0f - (x*x/90.0f)*(1.0f - (x*x/132.0f)*(1.0f - (x*x/182.0f)))))))));
  } 
 return 1.0f - (x*x/2.0f)*( 1.0f - (x*x/12.0f) * ( 1.0f - (x*x/30.0f) * (1.0f - (x*x/56.0f )*(1.0f - (x*x/90.0f)*(1.0f - (x*x/132.0f)*(1.0f - (x*x/182.0f)))))));
}

// Self-made
double sin(double x){return cos(x - PI_OVER_TWO);}

// Report reference [19]  
int isnan(double x)
{
    union { unsigned long long int u; double f; } ieee754;
    ieee754.f = x;
    return ( (unsigned)(ieee754.u >> 32) & 0x7fffffff ) +
           ( (unsigned)ieee754.u != 0 ) > 0x7ff00000;
}


// Report reference [20]  
double fmod(double x, double y) {
    union {double f; unsigned long long int i;} ux = {x}, uy = {y};
	int ex = ux.i>>52 & 0x7ff;
	int ey = uy.i>>52 & 0x7ff;
	int sx = ux.i>>63;
	unsigned long long int i;

	/* in the followings uxi should be ux.i, but then gcc wrongly adds */
	/* float load/store to inner loops ruining performance and code size */
	unsigned long long int uxi = ux.i;

	if (uy.i<<1 == 0 || isnan(y) || ex == 0x7ff)
		return (x*y)/(x*y);
	if (uxi<<1 <= uy.i<<1) {
		if (uxi<<1 == uy.i<<1)
			return 0*x;
		return x;
	}

	/* normalize x and y */
	if (!ex) {
		for (i = uxi<<12; i>>63 == 0; ex--, i <<= 1);
		uxi <<= -ex + 1;
	} else {
		uxi &= -1ULL >> 12;
		uxi |= 1ULL << 52;
	}
	if (!ey) {
		for (i = uy.i<<12; i>>63 == 0; ey--, i <<= 1);
		uy.i <<= -ey + 1;
	} else {
		uy.i &= -1ULL >> 12;
		uy.i |= 1ULL << 52;
	}

	/* x mod y */
	for (; ex > ey; ex--) {
		i = uxi - uy.i;
		if (i >> 63 == 0) {
			if (i == 0)
				return 0*x;
			uxi = i;
		}
		uxi <<= 1;
	}
	i = uxi - uy.i;
	if (i >> 63 == 0) {
		if (i == 0)
			return 0*x;
		uxi = i;
	}
	for (; uxi>>52 == 0; uxi <<= 1, ex--);

	/* scale result */
	if (ex > 0) {
		uxi -= 1ULL << 52;
		uxi |= (unsigned long long int)ex << 52;
	} else {
		uxi >>= -ex + 1;
	}
	uxi |= (unsigned long long int)sx << 63;
	ux.i = uxi;
	return ux.f;
} 

// Report reference [21]  
double ATAN_LUT_d[102] = {
 0,                   0.00999966668666524, 0.0199973339731505,  0.0299910048568779,  0.0399786871232900,
 0.0499583957219428,  0.0599281551212079,  0.0698860016346425,  0.0798299857122373,  0.0897581741899505,
 0.0996686524911620,  0.109559526773944,   0.119428926018338,   0.129275004048143,   0.139095941482071,
 0.148889947609497,   0.158655262186401,   0.168390157147530,   0.178092938231198,   0.187761946513593,
 0.197395559849881,   0.206992194219821,   0.216550304976089,   0.226068387993884,   0.235544980720863,
 0.244978663126864,   0.254368058553266,   0.263711834462266,   0.273008703086711,   0.282257421981491,
 0.291456794477867,   0.300605670042395,   0.309702944542456,   0.318747560420644,   0.327738506780556,
 0.336674819386727,   0.345555580581712,   0.354379919123438,   0.363147009946176,   0.371856073848581,
 0.380506377112365,   0.389097231055278,   0.397627991522129,   0.406098058317616,   0.414506874584786,
 0.422853926132941,   0.431138740718782,   0.439360887284591,   0.447519975157170,   0.455615653211225,
 0.463647609000806,   0.471615567862328,   0.479519291992596,   0.487358579505190,   0.495133263468404,
 0.502843210927861,   0.510488321916776,   0.518068528456721,   0.525583793551610,   0.533034110177490,
 0.540419500270584,   0.547740013715902,   0.554995727338587,   0.562186743900029,   0.569313191100662,
 0.576375220591184,   0.583373006993856,   0.590306746935372,   0.597176658092678,   0.603982978252998,
 0.610725964389209,   0.617405891751573,   0.624023052976757,   0.630577757214935,   0.637070329275684,
 0.643501108793284,   0.649870449411948,   0.656178717991395,   0.662426293833151,   0.668613567927821,
 0.674740942223553,   0.680808828915828,   0.686817649758645,   0.692767835397122,   0.698659824721463,
 0.704494064242218,   0.710271007486686,   0.715991114416300,   0.721654850864761,   0.727262687996690,
 0.732815101786507,   0.738312572517228,   0.743755584298860,   0.749144624606017,   0.754480183834406,
 0.759762754875771,   0.764992832710910,   0.770170914020331,   0.775297496812126,   0.780373080066636,
 0.785398163397448,   0.790373246728302
};

// Report reference [21]  
double atan(double x) {
  if (x >= 0) {
    if (x <= 1) {
      int index = round_i(x * 100);
      return (ATAN_LUT_d[index] + (x * 100 - index) * (ATAN_LUT_d[index + 1] - ATAN_LUT_d[index]));
    } else {
      double re_x = 1 / x;
      int index = round_i(re_x * 100);
      return (PI_OVER_TWO - (ATAN_LUT_d[index] + (re_x * 100 - index) * (ATAN_LUT_d[index + 1] - ATAN_LUT_d[index])));
    }
  } else {
    if (x >= -1) {
      double abs_x = -x;
      int index = round_i(abs_x * 100);
      return -(ATAN_LUT_d[index] + (abs_x * 100 - index) * (ATAN_LUT_d[index + 1] - ATAN_LUT_d[index]));
    } else {
      double re_x = 1 / (-x);
      int index = round_i(re_x * 100);
      return (ATAN_LUT_d[index] + (re_x * 100 - index) * (ATAN_LUT_d[index+1] - ATAN_LUT_d[index])) - PI_OVER_TWO;
    }
  }
}

#endif // end math_functions.h

/***********************************************************
Seven-Segment Display Driver Declaration and Implementation
***********************************************************/
#ifndef SEVEN_SEGMENT_DISPLAY_H
#define SEVEN_SEGMENT_DISPLAY_H
#include "math_functions.h"

// Hardware addresses
#define SEVENSD3_SEVENSD0_BASE 0xFF200020
#define SEVENSD5_SEVENSD4_BASE 0xFF200030

typedef unsigned int sevensd_digit_t;

sevensd_digit_t SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[12] = {
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

// pointers declared volatile because values may change due to extraneous causes, and 
// compiler-optimization of these values ought to be forgone
volatile unsigned int *const sevensd_register_one_ptr = (unsigned int *)(SEVENSD3_SEVENSD0_BASE);
volatile unsigned int *const sevensd_register_two_ptr = (unsigned int *)(SEVENSD5_SEVENSD4_BASE);

/*
    Function:       WRITE TO SEVEN-SEGMENT DISPLAY
    Description:    Write passed buffer to 7SD
    Input:          Buffer to write
    Output:         None
*/
void write_to_7sd(unsigned int *to_write) {
    *sevensd_register_one_ptr = 0;
    *sevensd_register_two_ptr = 0;
    int idx;

    // register length - 1. Don't bitshift after last entry
    for (idx = 0; idx < 3; idx++) {
        *sevensd_register_one_ptr |= *(to_write + idx);
        *sevensd_register_one_ptr >>= 8;
    }

    *sevensd_register_one_ptr |= *(to_write + 3);

    *sevensd_register_two_ptr |= (*(to_write + 4) >> 24);
    *sevensd_register_two_ptr |= (*(to_write + 5) >> 16);
}

/*
    Function:       DISPLAY SELECT
    Description:    Display "SELECT" pattern on 7SD
    Input:          None
    Output:         None
*/
void display_select() { write_to_7sd(SELECT_PATTERN); }

/*
    Function:       DISPLAY BOOST
    Description:    Display "BOOST" pattern on 7SD
    Input:          None
    Output:         None
*/
void display_boost() { write_to_7sd(BOOST_PATTERN); }

/*
    Function:       DISPLAY FLOATING POINT
    Description:    Display floating-point number on 7SD 
    Input:          Number to display,
                    Number of decimal places,
                    Can passed number be less than 0? 
    Output:         None
*/
void display_floating_point(double *num, int decimal_places, int can_be_negative) {
    int integral_places = 6 - decimal_places - can_be_negative;
    if (integral_places < 1) {
        write_to_7sd(ERR_PATTERN);
        return;
    }

    int is_negative = *num < 0;
    double integral_portion_f = 0;
    double fractional_portion_f = modf(*num, &integral_portion_f);

    int integral_portion = (int)fabs(integral_portion_f);
    int fractional_portion = abs(round_f(fractional_portion_f * pow(10.0, (double)decimal_places)));

    unsigned int digits[6] = {0};
    int idx = 0;
    // zero-fill first in case fractional portion is less than max
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
        write_to_7sd(digits);
        return;
    }

    if (idx == 6 && is_negative) {
        write_to_7sd(ERR_PATTERN);
        return;
    }
    
    // display negative sign at the front, or nothing if the number is positive
    digits[5] = is_negative ? SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[11] : SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[10];

    write_to_7sd(digits);
}

/*
    Function:       DISPLAY BOOST SETTING
    Description:    Display current boost mode on 7SD
    Input:          Current boost mode
    Output:         None
*/
void display_boost_setting(int *boost_setting) {
    unsigned int digits[6] = {0};
    for (int i = 0; i < 6; i++) {
        digits[i] = BOOST_PATTERN[i];
    }
    digits[0] = SEVEN_SEG_DISPLAY_PATTERN_LOOKUP[*boost_setting % 10];
    write_to_7sd(digits);
}

#endif // end seven_segment_display.h

/******************************************************
Field-Oriented Control Declaration [7], [8], [9], [29]
******************************************************/
#ifndef FOC_H
#define FOC_H
#include "math_functions.h"

// Motor-specific parameters
#define MOTOR_INDUCTANCE_HENRY 0.0002
#define MOTOR_RESISTANCE_OHMS 0.12
#define MOTOR_MAX_TORQUE_NM 45.5
#define MOTOR_POLES 20.0

// Precalculated motor-specific constants
#define MOTOR_POLE_PAIRS (MOTOR_POLES / 2.0)

// Precalculated constants
#define INVERSE_ROOT_TWO (1.0 / sqrt(2.0))
#define INVERSE_ROOT_THREE (1.0 / sqrt(3.0))
#define INVERSE_ROOT_SIX (1.0 / sqrt(6.0))
#define ROOT_TWO_OVER_THREE sqrt(2.0 / 3.0)
#define TWO_PI 6.28318530718
#define RADS_TO_DEG_MULTIPLIER 57.2957795

// Instead of passing around pointers, the structure containing the 
// array representing a 3D vector is used
typedef struct _DoubleVector3D {
    double vec[3]; 
} DoubleVector3D;

/*
    Function:       POWER-INVARIANT CLARKE TRANSFORM
    Description:    Transforms (a,b,c)-axis vector to (alpha,beta,0)-axis vector
    Input:          (a,b,c)-axis vector
    Output:         (alpha,beta,0)-axis vector
*/
void clarke_transform(DoubleVector3D *);

/*
    Function:       INVERSE CLARKE TRANSFORM
    Description:    Transforms (alpha,beta,0)-axis vector to (a,b,c)-axis vector
    Input:          (alpha,beta,0)-axis vector
    Output:         (a,b,c)-axis vector
*/
void inverse_clarke_transform(DoubleVector3D*);

/*
    Function:       PARK TRANSFORM
    Description:    Transforms (alpha,beta,0)-axis vector to (d,q)-axis vector
    Input:          (alpha,beta,0)-axis vector
    Output:         (d,q)-axis vector
*/
void park_transform(DoubleVector3D *, double *, double *, double *);

/*
    Function:       INVERSE PARK TRANSFORM
    Description:    Transforms (d,q)-axis vector to (alpha,beta,0)-axis vector
    Input:          (d,q)-axis vector
    Output:         (alpha,beta,0)-axis vector
*/
void inverse_park_transform(DoubleVector3D*, double*, double*);

/*
    Function:       CALCULATE AND NORMALIZE STATOR CURRENT PHASE ANGLE
    Description:    Calculates stator current phase angle from polled values and limits it to 0 <= angle <= 2PI
    Input:          Rotor angle buffer, phase angle buffer
    Output:         Phase angle buffer written to
*/
void calculate_normalized_stator_current_phase_angle(double *, double *);

/*
    Function:       CALCULATE SPACE VECTOR PHASE ANGLE
    Description:    Calculates phase angle from calculated stator current values
    Input:          (alpha,beta,0)-axis vector, phase angle buffer
    Output:         Phase angle buffer written to
*/
void calculate_normalized_space_vector_phase_angle(DoubleVector3D*, double*);

/*
    Function:       CALCULATE OUTPUT VOLTAGES
    Description:    Sets control voltages in (d,q)-axis vector to torque setpoint
    Input:          (d,q)-axis vector, torque setpoint
    Output:         (d,q)-axis vector written to
*/
void calculate_output_voltages(DoubleVector3D *, double *);

/*
    Function:       NORMALIZE VOLTAGES
    Description:    Scales vector components to fractions of a 5-Volt control signal
    Input:          (d,q)-axis vector
    Output:         (d,q)-axis vector written to
*/
void normalize_voltages(DoubleVector3D*);

/*
    Function:       FOC
    Description:    Sequentially execute all subroutines of the Field-Oriented Control algorithm
    Input:          Direct-axis current buffer,
                    Quadrature-axis current buffer,
                    Torque setpoint in Newton-metres, 
                    Rotor position in radians,
                    (a,b,c)-axis vector representing sensed stator currents, 
                    Phase angle in radians
    Output:         (a,b,c)-axis vector written to as buffer; replaced with (alpha,beta,0) vector
*/
void foc(double *, double *, double *, double *, DoubleVector3D *, double *);

#endif // end foc.h

/**********************************
Field-Oriented Control Implementation
**********************************/
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

void inverse_park_transform(DoubleVector3D *rotation_vector_buffer, double *cosine, double *sine) {
    double i_d = rotation_vector_buffer->vec[0];
    double i_q = rotation_vector_buffer->vec[1];

    double i_alpha = i_d * *cosine - i_q * *sine;
    double i_beta = i_d * *sine + i_q * *cosine;

    rotation_vector_buffer->vec[0] = i_alpha;
    rotation_vector_buffer->vec[1] = i_beta;
}

void calculate_normalized_stator_current_phase_angle(double* rotor_position_rads, double* phase_angle_rads) {
    double angle = fmod(MOTOR_POLE_PAIRS * (*rotor_position_rads), TWO_PI);
    if (angle >= 0) {
        *phase_angle_rads = angle;
    } else {
        *phase_angle_rads = angle + TWO_PI;
    }
}

void calculate_normalized_space_vector_phase_angle(DoubleVector3D* rotation_vector_buffer, double* phase_angle_rads) {
    *phase_angle_rads = atan(rotation_vector_buffer->vec[1] / rotation_vector_buffer->vec[0]); // arctan(beta/alpha)
}

void calculate_output_voltages(DoubleVector3D *rotation_vector_buffer, double *target_torque_nm) {
    double id_error = 0 - rotation_vector_buffer->vec[0];                 // i_sdref - i_sd
    double iq_error = *target_torque_nm - rotation_vector_buffer->vec[1]; // i_sqref - i_sq

    // instead of PI control, we will make it so that we have a max output voltage of 5 V, and the output voltages
    // will be proportional to this in conj. with the max operational torque of the SWX02 (~45 N*m). Vsd should always
    // be zero anyways because BLDC is a synchronous machine and requires no axial torque (i.e. torque in the d-axis),
    // so we should always just aim for 0.
    double v_sd = 0 + id_error;
    double v_sq = *target_torque_nm + iq_error;
    rotation_vector_buffer->vec[0] = v_sd;
    rotation_vector_buffer->vec[1] = v_sq;
}

void normalize_voltages(DoubleVector3D *rotation_vector_buffer) {
    double sum = fabs(rotation_vector_buffer->vec[0]) + fabs(rotation_vector_buffer->vec[1]) + fabs(rotation_vector_buffer->vec[2]);
    rotation_vector_buffer->vec[0] = (rotation_vector_buffer->vec[0] / sum) * 5.0;
    rotation_vector_buffer->vec[1] = (rotation_vector_buffer->vec[1] / sum) * 5.0;
    rotation_vector_buffer->vec[2] = (rotation_vector_buffer->vec[2] / sum) * 5.0;
}

void foc(double *id, double *iq, double *target_torque_nm, double *rotor_position_rads, DoubleVector3D *rotation_vector_buffer, double *phase_angle_rads) {
    double cosine = 0.0;
    double sine = 0.0;

    clarke_transform(rotation_vector_buffer);                                                   // (a,b,c) -> (alpha,beta,0)
    park_transform(rotation_vector_buffer, rotor_position_rads, &cosine, &sine);                // (alpha,beta,0) -> (d,q)
    calculate_output_voltages(rotation_vector_buffer, target_torque_nm);                        // (d,q) -> (d,q)
    inverse_park_transform(rotation_vector_buffer, &cosine, &sine);                             // (d,q) -> (alpha,beta,0)
    normalize_voltages(rotation_vector_buffer);                                                 // (alpha,beta,0) -> (alpha,beta,0)
    calculate_normalized_space_vector_phase_angle(rotation_vector_buffer, phase_angle_rads);    // (alpha,beta,0) -> (alpha,beta,0)
}
// end Field-Oriented Control implementation

/****************
Main Declaration
*****************/
#include <stdio.h>

// Hardware address definitions
#define SW_BASE 0xFF200040
#define GPIO_BASE 0xFF200060
#define ADC_BASE 0xFF204000
#define GPIO_CFG 0b111111111
#define BUTTON_BASE 0xFF200050

// Hardware pointers
volatile int *gpio_ptr = (int *)(GPIO_BASE);
volatile int *adc_channel0_ptr = (int *)(ADC_BASE);
volatile int *adc_channel1_ptr = (int *)(ADC_BASE + 0x4);
volatile int *const button_bank_ptr = (int *)(BUTTON_BASE);
volatile char *switch_bank_ptr = (char *)(SW_BASE);

// Precalculated constant definitions
#define ADC_TO_RADS_MULTIPLIER_X1000 1.5339807
#define ADC_TO_NM_MULTIPLER (MOTOR_MAX_TORQUE_NM / 4096.0)

// Lookup table for boost multipliers given a specific boost mode 
double BOOST_COEFFICIENT_LOOKUP_T[4] = {
    0.0,
    1.0,
    1.25,
    1.5
};

#define yes 1
#define no 0

// Software-only global variables
volatile int DELAY_LENGTH = 2000000;
volatile int delay_count = 0;
int display_mode = 0;
int voltage_capture = 0;
int potentiometer_1_voltage = 0; 
int potentiometer_2_voltage = 0;

// Writes current button bank values to the passed buffer
void write_buttons_to_buffer(char *buffer) {
    *buffer = *button_bank_ptr & 0b1000;
    *(buffer + 0x1) = *button_bank_ptr & 0b0100;
    *(buffer + 0x2) = *button_bank_ptr & 0b0010;
    *(buffer + 0x3) = *button_bank_ptr & 0b0001;
}

// Starts analog-to-digital converter sampling
inline void start_adc() {
    *adc_channel1_ptr = 1; 
}

// Polls ADC data register and, if updated, writes to passed buffers
void read_potentiometers(int *voltage_1, int *voltage_2) {
    voltage_capture = *adc_channel0_ptr;

    if (voltage_capture & 0x10000) { // set to 0x8000 for lab, 0x10000 for sim
        *voltage_1 = voltage_capture & 0b0000111111111111;
        start_adc();
    }

    voltage_capture = *adc_channel1_ptr;
    if (voltage_capture & 0x10000) {
        *voltage_2 = voltage_capture & 0b0000111111111111;
        start_adc();
    }
}

// Loads GPIO_CFG into control register, setting pin directions
inline void setup_gpio() {
    *(gpio_ptr + 0x4) = GPIO_CFG; // set direction control register
}

// Get current boost mode from button bank, write it to passed buffer
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

// Get current display mode from switch bank, write it to passed buffer
void get_display_mode(int *display_mode_buffer) { *display_mode_buffer = *(int *)(SW_BASE); }

// Get input torque from potentiometer 1 
void get_input_torque(double *input_torque) {
    *input_torque = potentiometer_1_voltage * ADC_TO_NM_MULTIPLER;
}

// Get rotor position from potentiometer 2
void get_rotor_position(double *rotor_position_rads) {
    *rotor_position_rads = ((double)potentiometer_2_voltage) * ADC_TO_RADS_MULTIPLIER_X1000;
}

// Convert passed angle from radians to degrees
double rads_to_degrees(double angle_rads) { return RADS_TO_DEG_MULTIPLIER * angle_rads; }

// Limit boost torque to the maximum motor torque
void normalize_boost_torque(double *requested_torque) {
    if (*requested_torque > MOTOR_MAX_TORQUE_NM) {
        *requested_torque = MOTOR_MAX_TORQUE_NM;
    }
}

int main(void) {
    // software-only values
    int is_running = 1;

    // control values 
    int boost_mode = 0;
    double input_torque_newton_metres = 0.0;

    double id = 0.0;
    double iq = 0.0;
    double calculated_torque_newton_metres = 0.0;
    double rotor_position_rads = 0.0;

    DoubleVector3D rotation_vector_buffer;
    rotation_vector_buffer.vec[0] = 0.0;
    rotation_vector_buffer.vec[1] = 0.0;
    rotation_vector_buffer.vec[2] = 0.0;
    DoubleVector3D inverter_voltage_control_vector;
    double phase_angle_rads = 0.0;

    // Initialize ADC and GPIO 
    start_adc();
    setup_gpio();

    // Get initial boost mode 
    while (boost_mode == 0) {
        // SELECt ... BOOSt ... SELECt ... BOOSt ...
        get_boost_mode(&boost_mode); // from button bank

        display_select();
        for (delay_count = DELAY_LENGTH; delay_count != 0; --delay_count)
            ; // delay loop
        display_boost();
        for (delay_count = DELAY_LENGTH; delay_count != 0; --delay_count)
            ; // delay loop

        get_boost_mode(&boost_mode); // from button bank
    }

    // Main loop
    while (is_running) {
        // meta (demo-only): read potentiometers
        read_potentiometers(&potentiometer_1_voltage, &potentiometer_2_voltage);

        get_input_torque(&input_torque_newton_metres); // with GPIO (potentiometer 1)
        get_rotor_position(&rotor_position_rads);
        calculated_torque_newton_metres = BOOST_COEFFICIENT_LOOKUP_T[boost_mode] * input_torque_newton_metres;
        normalize_boost_torque(&calculated_torque_newton_metres);

        foc(&id, &iq, &calculated_torque_newton_metres, &rotor_position_rads, &inverter_voltage_control_vector, &phase_angle_rads);

        get_display_mode(&display_mode); // from switch bank
        switch (display_mode) {
        case 0:
            // display v_alpha
            display_floating_point(&inverter_voltage_control_vector.vec[0], 4, yes); 
            break;
        case 1:
            // display v_beta
            display_floating_point(&inverter_voltage_control_vector.vec[1], 4, yes);
            break;
        case 2:
            // display phase angle rads
            display_floating_point(&phase_angle_rads, 5, no);
            break;
        case 3:
            // display rotor position rads
            display_floating_point(&rotor_position_rads, 2, no); 
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