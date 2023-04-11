/*
    I don't want to do this because I know it's frowned-upon to reinvent the wheel 
    but math.h is not linking properly on DE10-Standard
*/

#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

#define PI 3.14159265359
#define PI_OVER_TWO 1.57079632679
#define PI_TIMES_TWO 6.28318530718

/// https://stackoverflow.com/a/39712957 
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

// https://git.musl-libc.org/cgit/musl/tree/src/math/modf.c
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

int abs(int x) {
    return (x >= 0 ? x : -x);
}

// https://git.musl-libc.org/cgit/musl/tree/src/math/fabs.c
double fabs(double x) {
    union {double f; unsigned long long int i;} u = {x};
	u.i &= -1ULL/2;
	return u.f;
}

int round_f(double number) {
    return (number >= 0.0) ? (int)(number + 0.5) : (int)(number - 0.5);
}

int round_i(int number) {
    return (number >= 0) ? (int)((double)number + 0.5) : (int)((double)number - 0.5); // lol
}

double pow(double num, double exponent) {
    double temp = 1.0;
    for(int i = 0; i < exponent; i++) {
        temp*=num;
    }
    return temp;
}

// this, cosine, and sine from https://gist.github.com/giangnguyen2412/bcab883b5a53b437b980d7be9745beaf  
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

double sin(double x){return cos(x-PI_OVER_TWO);}

// https://stackoverflow.com/a/20723890/
int isnan(double x)
{
    union { unsigned long long int u; double f; } ieee754;
    ieee754.f = x;
    return ( (unsigned)(ieee754.u >> 32) & 0x7fffffff ) +
           ( (unsigned)ieee754.u != 0 ) > 0x7ff00000;
}


// https://git.musl-libc.org/cgit/musl/tree/src/math/fmod.c
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

// https://github.com/xiezhq-hermann/atan_lookup/blob/master/atan.cpp
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

#endif