#include "stdafx.h"
#include "Converter.h"

float Kx[2][4] = Kx_val;
float Ku[2][2] = Ku_val;
float Kr[2][4] = Kr_val;
float Ard[4][4] = Ard_val;
float Brd[4][2] = Brd_val;

struct Control_struct var_Control_struct;
struct Converter_struct conv;
struct Measurements Meas;

struct SimulationState *aState_global;