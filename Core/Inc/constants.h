#pragma once

#include <math.h>

#define ROOT_THREE_F            1.7320508f
#define INVERSE_ROOT_THREE_F    0.57735026f
#define ONE_THIRD_F             0.33333333f
#define TWO_THIRDS_F            0.66666667f
#define PI_F                    3.14159265f
#define TWO_PI_F				6.28318531f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define mod(a, b) ((((a) % (b)) + (b)) % (b))

#define signf(x) (((x) > 0.0f) ? 1.0f : (((x) < 0.0f) ? -1.0f : 0.0f))

#define absf(x) ((x >= 0.0f) ? x : (-x))

inline float wrap_pi(float x)
{
    x = fmodf(x + PI_F, TWO_PI_F);
    if(x < 0.0f) {
        x += TWO_PI_F;
	}
    return x - PI_F;
}