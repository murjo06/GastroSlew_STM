#pragma once

#define ROOT_THREE_F            1.7320508f
#define INVERSE_ROOT_THREE_F    0.57735026f
#define ONE_THIRD_F             0.33333333f
#define TWO_THIRDS_F            0.66666667f
#define PI_F                    3.14159265f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define mod(a, b) ((((a) % (b)) + (b)) % (b))

#define signf(x) (((x) > 0.0f) ? 1 : (((x) < 0.0f) ? -1 : 0))