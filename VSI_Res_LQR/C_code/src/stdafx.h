#pragma once

#include "DllHeader.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "Matrices.h"
#include "Control.h"
#include "Converter.h"
#include "Coordinates.h"

typedef uint16_t Uint16;
typedef uint32_t Uint32;
typedef uint64_t Uint64;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

struct Measurements
{
	struct abc_struct is; /* In reality, this is if*/
	struct abc_struct vc;
	float Vdc;
};

extern float Kx[2][4];
extern float Ku[2][2];
extern float Kr[2][4];
extern float Ard[4][4];
extern float Brd[4][2];

extern struct Measurements Meas;
extern struct SimulationState *aState_global;