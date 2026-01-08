#include "stdafx.h"
#include "Control.h"

#define ConstMATH_PI        3.1415926535897932384626433832795
#define ConstMATH_SQRT3     1.7320508075688772935274463415059
#define ConstMATH_SQRT2     1.4142135623730950488016887242097

const float MATH_PI = ConstMATH_PI;
const float MATH_2PI = ConstMATH_PI * 2.0;
const float MATH_2PI_3 = ConstMATH_PI * 2.0 / 3.0;
const float MATH_PI_3 = ConstMATH_PI / 3.0;
const float MATH_1_2PI = 1.0 / (ConstMATH_PI * 2.0);
const float MATH_1_PI = 1.0 / ConstMATH_PI;
const float MATH_1_3 = 1.0 / 3.0;
const float MATH_1_SQRT3 = 1.0 / ConstMATH_SQRT3;
const float MATH_1_SQRT2 = 1.0 / ConstMATH_SQRT2;
const float MATH_SQRT2_3 = ConstMATH_SQRT2 / 3.0;
const float MATH_SQRT3_2 = ConstMATH_SQRT3 / 2.0;
const float MATH_SQRT2 = ConstMATH_SQRT2;
const float MATH_SQRT3 = ConstMATH_SQRT3;
const float MATH_1_2 = 1.0 / 2.0;
const float MATH_2_3 = 2.0 / 3.0;

/* Definition of the LQR controller void lqrControl(const float x[n. states], float u[n. inputs]) */
void lqrControl(const float x[4], const float xd[2], const float xr[4], float u[2])
{
    /* u = -K * x */
    u[0] = 0.0;
    u[1] = 0.0;
    
    /* For each state, assign each input as the multiplication of the column of that state times the state*/
    for (int nx = 0; nx < 4; ++nx) {
        u[0] -= Kx[0][nx] * x[nx];
        u[1] -= Kx[1][nx] * x[nx];
    }

    for (int nd = 0; nd < 2; ++nd) {
        u[0] -= Ku[0][nd] * xd[nd];
        u[1] -= Ku[1][nd] * xd[nd];
    }

    for (int nr = 0; nr < 4; ++nr) {
        u[0] -= Kr[0][nr] * xr[nr];
        u[1] -= Kr[1][nr] * xr[nr];
    }
}

void resonantSystem(const float e[2],
                    float rho_k[4],
                    float rho_k_1[4])
{
    rho_k_1[0] = Ard[0][0] * rho_k[0] + Ard[0][1] * rho_k[1] + Ard[0][2] * rho_k[2] + Ard[0][3] * rho_k[3] + Brd[0][0] * e[0] + Brd[0][1] * e[1];
    rho_k_1[1] = Ard[1][0] * rho_k[0] + Ard[1][1] * rho_k[1] + Ard[1][2] * rho_k[2] + Ard[1][3] * rho_k[3] + Brd[1][0] * e[0] + Brd[1][1] * e[1];
    rho_k_1[2] = Ard[2][0] * rho_k[0] + Ard[2][1] * rho_k[1] + Ard[2][2] * rho_k[2] + Ard[2][3] * rho_k[3] + Brd[2][0] * e[0] + Brd[2][1] * e[1];
    rho_k_1[3] = Ard[3][0] * rho_k[0] + Ard[3][1] * rho_k[1] + Ard[3][2] * rho_k[2] + Ard[3][3] * rho_k[3] + Brd[3][0] * e[0] + Brd[3][1] * e[1];
}

/* At the end of the Output loop, save the actual value as the last value*/
void variablesUpdate()
{
    var_Control_struct.States.u_k[0] = var_Control_struct.Output.m_k_alphabeta.alpha;
    var_Control_struct.States.u_k[1] = var_Control_struct.Output.m_k_alphabeta.beta;
    
    var_Control_struct.States.rho_k[0] = var_Control_struct.States.rho_k_1[0];
    var_Control_struct.States.rho_k[1] = var_Control_struct.States.rho_k_1[1];
    var_Control_struct.States.rho_k[2] = var_Control_struct.States.rho_k_1[2];
    var_Control_struct.States.rho_k[3] = var_Control_struct.States.rho_k_1[3];
}