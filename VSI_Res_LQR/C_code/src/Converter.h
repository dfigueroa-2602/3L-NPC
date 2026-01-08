#include "stdafx.h"
#include "Control.h"

#pragma once
#ifndef Converter_H_
#define Converter_H_

struct Converter_struct
{
    struct abc_struct mod_abc;
};

extern struct Converter_struct conv;
void Converter_NPC_minmax(struct abc_struct *u_abc, float Vdc, struct Converter_struct *conv);

#endif /* Converter_H_ */