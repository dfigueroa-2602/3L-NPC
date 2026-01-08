#include "stdafx.h"
#include "Control.h"
#include "Converter.h"

/* Function to create the minmax injection for the NPC inverter */
void Converter_NPC_minmax(struct abc_struct *u_abc, float Vdc, struct Converter_struct *conv)
{
    if (Vdc <= 0.0f) {
        conv->mod_abc.a = 0.5f;
        conv->mod_abc.b = 0.5f;
        conv->mod_abc.c = 0.5f;
        return;
    }

    float invVdc = 2.0f / fmaxf(Vdc, 0.0001f);

    /* Normalization to obtain the index vector according to the DC voltage */
    float m_a = u_abc->a * invVdc;
    float m_b = u_abc->b * invVdc;
    float m_c = u_abc->c * invVdc;

    /* Calculation of the common mode */
    float m_max = fmaxf(m_a, fmaxf(m_b, m_c));
    float m_min = fminf(m_a, fminf(m_b, m_c));
    float ucm   = -0.5f * (m_min + m_max);

    /* To the modulation vector, the common mode is added */
    m_a += ucm; m_b += ucm; m_c += ucm;

    /* Then, the module of each phase is obtained*/
    float mod_a = (m_a + 1.0f) - floorf(m_a + 1.0f) - 0.5f;
    float mod_b = (m_b + 1.0f) - floorf(m_b + 1.0f) - 0.5f;
    float mod_c = (m_c + 1.0f) - floorf(m_c + 1.0f) - 0.5f;

    /* Then, those modes are min-maxed obtaining second minmax offset after wrapping */
    float mod_max = fmaxf(mod_a, fmaxf(mod_b, mod_c));
    float mod_min = fminf(mod_a, fminf(mod_b, mod_c));
    float ucm3    = -0.5f * (mod_min + mod_max);

    /* Finally, the modulation vector with the common mode is summed with this new common mode */
    m_a += ucm3; m_b += ucm3; m_c += ucm3;

    conv->mod_abc.a = fminf(fmaxf(m_a, -1.15f), 1.15f);
    conv->mod_abc.b = fminf(fmaxf(m_b, -1.15f), 1.15f);
    conv->mod_abc.c = fminf(fmaxf(m_c, -1.15f), 1.15f);
}