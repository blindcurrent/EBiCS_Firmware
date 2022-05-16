#include "svpwm.h"

// Square Root of 3
#define _SQRT3	28  //1.73205081*16

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta);
//void svpwm2(q31_t q31_u_alpha, q31_t q31_u_beta, q31_t q31_angle);


/**
   * @brief Compute SVPWM switchtimes given the d/q coordinates of the voltage vector.
   * @param[in] MS->u_d
   * @param[in] MS->u_q
   * @param[in] q31_theta
   * @param[out] switchtime (global)
   */
void compute_svpwm_switchtime(MotorState_t* MS, q31_t q31_theta)
{
    static q31_t q31_sinevalue, q31_cosinevalue;
	static q31_t q31_u_alpha, q31_u_beta;
    
    q31_t q31_theta2 = q31_theta;

    q31_t q31_u_d = -MS->u_d;
    q31_t q31_u_q = MS->u_q;


    arm_sin_cos_q31(q31_theta2, &q31_sinevalue, &q31_cosinevalue);

	//inverse Park transformation
	arm_inv_park_q31(q31_u_d, q31_u_q, &q31_u_alpha, &q31_u_beta, -q31_sinevalue, q31_cosinevalue);
	//arm_inv_park_q31(MS_FOC->u_d, MS_FOC->u_q, &q31_u_alpha, &q31_u_beta, sinevalue, cosinevalue);
	
    //call SVPWM calculation
	svpwm(q31_u_alpha, q31_u_beta);
    
    // svpwm2 only works with negative theta & negative u_beta -> todo: check why this is the case
    // svpwm2(q31_u_alpha, -q31_u_beta, q31_theta2);
    // svpwm2(q31_u_alpha, q31_u_beta, q31_theta2);
}

void svpwm(q31_t q31_u_alpha, q31_t q31_u_beta)	
{

//SVPWM according to chapter 4.9 of UM1052


	q31_t q31_U_alpha = (_SQRT3 *_T * q31_u_alpha)>>4;
	q31_t q31_U_beta = -_T * q31_u_beta;
	q31_t X = q31_U_beta;
	q31_t Y = (q31_U_alpha+q31_U_beta)>>1;
	q31_t Z = (q31_U_beta-q31_U_alpha)>>1;

	//Sector 1 & 4
	if ((Y>=0 && Z<0 && X>0)||(Y < 0 && Z>=0 && X<=0)){
		switchtime[0] = ((_T+X-Z)>>12) + (_T>>1); //right shift 11 for dividing by peroid (=2^11), right shift 1 for dividing by 2
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[1] - (X>>11);
		//temp4=1;
	}

	//Sector 2 & 5
	if ((Y>=0 && Z>=0) || (Y<0 && Z<0) ){
		switchtime[0] = ((_T+Y-Z)>>12) + (_T>>1);
		switchtime[1] = switchtime[0] + (Z>>11);
		switchtime[2] = switchtime[0] - (Y>>11);
		//temp4=2;
	}

	//Sector 3 & 6
	if ((Y<0 && Z>=0 && X>0)||(Y >= 0 && Z<0 && X<=0)){
		switchtime[0] = ((_T+Y-X)>>12) + (_T>>1);
		switchtime[2] = switchtime[0] - (Y>>11);
		switchtime[1] = switchtime[2] + (X>>11);
		//temp4=3;
	}


}


// svpwm2 defines
// _T = 1 << 11
#define SQRT_SHIFT 6
#define SHIFT 17                // (SQRT_SHIFT + 11)
#define TWO_SHIFTED 536870912   // 2 << (SQRT_SHIFT + 22)
#define ONE_SHIFTED 268435456   // 1 << (SQRT_SHIFT + 22)
#define SQRT3_SHIFTED 111       // sqrt(3) << SQRT_SHIFT

/*void svpwm2(q31_t q31_u_alpha, q31_t q31_u_beta, q31_t q31_angle)
{
    uint8_t sector;
    q31_t Ualpha = q31_u_alpha * _T * SQRT3_SHIFTED;
    q31_t Ubeta = (q31_u_beta * _T) << SQRT_SHIFT;  

    if(q31_angle > 0)
    {
        if(q31_angle < DEG_plus60)
            sector = 1;
        else if(q31_angle < DEG_plus120)
            sector = 2;
        else
            sector = 3;
    }
    else
    {
        if(q31_angle < DEG_minus120)
            sector = 4;
        else if(q31_angle < DEG_minus60)
            sector = 5;
        else
            sector = 6;
    }

    if( sector == 1 || sector == 4 )
    {
        switchtime[0] = (Ualpha + Ubeta + TWO_SHIFTED)  >> (SHIFT + 2);
        switchtime[1] = (-Ualpha + 3 * Ubeta + TWO_SHIFTED) >> (SHIFT + 2);
        switchtime[2] = (-Ualpha - Ubeta + TWO_SHIFTED) >> (SHIFT + 2);
    }
    else if( sector == 2 || sector == 5 )
    {
        switchtime[0] = (Ualpha + ONE_SHIFTED) >> (SHIFT + 1);
        switchtime[1] = (Ubeta + ONE_SHIFTED) >> (SHIFT + 1);
        switchtime[2] = (ONE_SHIFTED - Ubeta) >> (SHIFT + 1);
    }
    else //if( sector == 3 || sector == 6 )
    {
        switchtime[0] = (Ualpha - Ubeta + TWO_SHIFTED) >> (SHIFT + 2);
        switchtime[1] = (-Ualpha + Ubeta + TWO_SHIFTED) >> (SHIFT + 2);
        switchtime[2] = (-Ualpha - 3 * Ubeta + TWO_SHIFTED) >> (SHIFT + 2);
    }
}*/