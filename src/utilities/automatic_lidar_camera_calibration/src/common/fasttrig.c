/* ================================================================
** fasttrig.[ch]
**
** LUT trig utilities.
**
** 02 JUL 2009  Ryan Eustice  Created from mitdgc-log-viewer/src/common/fasttrig.[ch]
** ================================================================

MIT 2007 DARPA Urban Challenge Log File Viewer
Copyright (C) 2007-2008  Massachusetts Institute of Technology

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <error.h>
//#include <gsl/gsl_math.h>
//#include <gsl/gsl_rng.h>

//#include "gsl_util.h"
#include "fasttrig.h"

#define SINCOS_MSB_BITS 14
#define SINCOS_LSB_BITS 14
#define SINCOS_MSB_TABLE_SIZE (1 << SINCOS_MSB_BITS)
#define SINCOS_LSB_TABLE_SIZE (1 << SINCOS_LSB_BITS)
#define SINCOS_BITS  (SINCOS_MSB_BITS + SINCOS_LSB_BITS)
struct sincos
{
    float m_sin, m_cos;
};
static struct sincos msb_table[SINCOS_MSB_TABLE_SIZE];
static struct sincos lsb_table[SINCOS_LSB_TABLE_SIZE];

#define ASIN_HIGH 0.9999
#define ARC_TABLE_BITS 16
#define ARC_TABLE_SIZE (1 << ARC_TABLE_BITS)
struct afcns
{
    float m_asin, m_asinH, m_atan;
};
static struct afcns arc_table[ARC_TABLE_SIZE+1];

static int initialized = 0;

void
fasttrig_init (void)
{
    if (initialized)
        return;

    for (size_t i = 0; i < SINCOS_MSB_TABLE_SIZE; i++) {
        double theta = (2. * M_PI * i / SINCOS_MSB_TABLE_SIZE);
        msb_table[i].m_sin = sin (theta);
        msb_table[i].m_cos = cos (theta);
    }

    for (size_t i = 0; i < SINCOS_LSB_TABLE_SIZE; i++) {
        double theta = (2. * M_PI * i / (SINCOS_MSB_TABLE_SIZE * SINCOS_LSB_TABLE_SIZE));
        lsb_table[i].m_sin = sin (theta);
        lsb_table[i].m_cos = cos (theta);
    }

    for (size_t i = 0; i < ARC_TABLE_SIZE; i++) {
        double v = ((double) i) / ARC_TABLE_SIZE;
        arc_table[i].m_asin = asin (v);
        arc_table[i].m_atan = atan (v);
        assert (arc_table[i].m_atan >= 0);

        v = (((double) i) / ARC_TABLE_SIZE) * (1. - ASIN_HIGH) + ASIN_HIGH;
        arc_table[i].m_asinH = asin (v);
    }
    arc_table[ARC_TABLE_SIZE].m_asin = M_PI_2;
    arc_table[ARC_TABLE_SIZE].m_asinH = M_PI_2;
    arc_table[ARC_TABLE_SIZE].m_atan = M_PI_4;

    initialized = 1;
}

static void
tabtrig_idx (const double theta, size_t *lsb_idx, size_t *msb_idx)
{
    static const double idx_res = SINCOS_MSB_TABLE_SIZE * SINCOS_LSB_TABLE_SIZE / (2. * M_PI); // idx resolution 
    uint32_t idx = idx_res * theta;

    // rewrite theta = M + L, where L is very small.
    *lsb_idx = idx & (SINCOS_LSB_TABLE_SIZE - 1);
    *msb_idx = (idx >> SINCOS_LSB_BITS) & (SINCOS_MSB_TABLE_SIZE - 1);
}

/** compute sincos, accurate to one in 1 << (SINCOS_BITS  - 3) **/
void
_fsincos (const double theta, double *s, double *c, FASTTRIG_ARGS_DEF)
{
    if (!initialized)
        FASTTRIG_INIT ();

    size_t lsb_idx, msb_idx;
    tabtrig_idx (theta, &lsb_idx, &msb_idx);

    // rewrite theta = M + L, where L is very small.
    float sinM = msb_table[msb_idx].m_sin;
    float cosM = msb_table[msb_idx].m_cos;
    float sinL = lsb_table[lsb_idx].m_sin;
    float cosL = lsb_table[lsb_idx].m_cos;

    // angle sum formulas
    // we lose a few bits of precision here... about 3
    *s = sinM*cosL + cosM*sinL;
    *c = cosM*cosL - sinM*sinL;
}

/** compute sin, accurate to one in 1 << (SINCOS_BITS  - 3) **/
double
_fsin (const double theta, FASTTRIG_ARGS_DEF)
{
    if (!initialized)
        FASTTRIG_INIT ();

    size_t lsb_idx, msb_idx;
    tabtrig_idx (theta, &lsb_idx, &msb_idx);

    // rewrite theta = M + L, where L is very small.
    float sinM = msb_table[msb_idx].m_sin;
    float cosM = msb_table[msb_idx].m_cos;
    float sinL = lsb_table[lsb_idx].m_sin;
    float cosL = lsb_table[lsb_idx].m_cos;
    
    // angle sum formula
    return sinM*cosL + cosM*sinL;
}

/** compute cos, accurate to one in 1 << (SINCOS_BITS  - 3) **/
double
_fcos (const double theta, FASTTRIG_ARGS_DEF)
{
    if (!initialized)
        FASTTRIG_INIT ();

    size_t lsb_idx, msb_idx;
    tabtrig_idx (theta, &lsb_idx, &msb_idx);

    // rewrite theta = M + L, where L is very small.
    float sinM = msb_table[msb_idx].m_sin;
    float cosM = msb_table[msb_idx].m_cos;
    float sinL = lsb_table[lsb_idx].m_sin;
    float cosL = lsb_table[lsb_idx].m_cos;
    
    // angle sum formula
    return cosM*cosL - sinM*sinL;
}


double
_fasin (const double y, FASTTRIG_ARGS_DEF)
{
    if (!initialized)
        FASTTRIG_INIT ();

    /* basic idea: asin is well-behaved over [-.98, .98]
     * We've precomputed a lookup table over this range, and a second
     * LUT over [.98, 1], and we bilinearly interpolate the
     * answer depending upon which range we fall in.
     */

    double yabs = fabs (y);

    if (y > 1)
        return M_PI_2;
    if (y < -1)
        return -M_PI_2;

    double a, b, drem;
    if (yabs > ASIN_HIGH) {
        double didx = ARC_TABLE_SIZE * (yabs - ASIN_HIGH) / (1. - ASIN_HIGH);
        size_t idx = (size_t) didx;
        drem = didx - idx;
        a = arc_table[idx].m_asinH;
        b = arc_table[idx+1].m_asinH;
    }
    else {
        double didx = ARC_TABLE_SIZE * yabs;
        size_t idx = (size_t) didx;
        drem = didx - idx;
        a = arc_table[idx].m_asin;
        b = arc_table[idx+1].m_asin;
    }
    double rho = (1-drem)*a + drem*b;

    if (y < 0)
        return -rho;
    else
        return rho;
}


double
_fatan2 (const double y, const double x,  FASTTRIG_ARGS_DEF)
{
    if (!initialized)
        FASTTRIG_INIT ();

    /* basic idea: atan is well-behaved over first 45 degrees, so we
     * do a reduction of all atan2 operations to the first 45 degrees
     * by possibly swapping the x and y axes.  We've precomputed a
     * lookup table over this range, and we bilinearly interpolate the
     * answer, then do the necessary twiddling to get back the right
     * answer.
     */
    double yabs = fabs (y);
    double xabs = fabs (x);

    double S1, A, S2, didx;
    if (xabs < yabs) {
        S1 = (y < 0) ? -1 : 1;
        A = M_PI_2;
        S2 = (x*y < 0) ? 1 : -1;

        didx = ARC_TABLE_SIZE * xabs / yabs;
    }
    else if (xabs > yabs) {
        S1 = (y < 0) ? -1 : 1;
        A = (x < 0) ? M_PI : 0;
        S2 = (x*y < 0) ? -1 : 1;

        didx = ARC_TABLE_SIZE * yabs / xabs;
    }
    else {
        if (x > 0) {
            if (y > 0)
                return  M_PI_4; // Q-I
            else
                return -M_PI_4; // Q-IV
        }
        else {
            if (y > 0)
                return  M_PI_4 + M_PI_2; // Q-II
            else
                return -M_PI_4 - M_PI_2; // Q-III
        }

    }

    size_t idx = (size_t) didx;
    double drem = didx - idx;
    double a = arc_table[idx].m_atan;
    double b = arc_table[idx+1].m_atan;
    double rho = (1-drem)*a + drem*b;

    return (S1*A + S2*rho);
}

#define RTOD (180. / M_PI)
#define DTOR (M_PI / 180.)


void
fasttrig_test_trig (void)
{
#if 0		
    if (!initialized)
        fasttrig_init ();

    gsl_rng *r = gsl_rng_alloc (gsl_rng_taus);
    gsl_rng_set (r, gslu_rng_seed ());

    double eps = 1. / (1<<17);
    int limit = 200000000;

    printf ("     %20s : %15s %15s %15s %10s\n", "angle", "math", "fasttrig", "diff", "err/eps");
    double max_s_err = 0, max_c_err = 0, max_t_err = 0;
    for (size_t i = 0; i < limit; i++) {
        double theta = gsl_rng_uniform (r) * 2. * M_PI;
        double s, c, s2, c2;
        sincos (theta, &s, &c);
        fsincos (theta, &s2, &c2);

        double t, t2;
        t = tan (theta);
        t2 = ftan (theta);

        double s_err = fabs (s-s2), c_err = fabs (c-c2), t_err = fabs (t-t2);
        if (s_err > max_s_err) {
            printf("fsin %20.5f : %15.12f %15.12f %15.12f %10.5f\n", theta * RTOD, s, s2, s_err, s_err/eps);
            max_s_err = s_err;
        }
        if (c_err > max_c_err) {
            printf("fcos %20.5f : %15.12f %15.12f %15.12f %10.5f\n", theta * RTOD, c, c2, c_err, c_err/eps);
            max_c_err = c_err;
        }
        if (t_err > max_t_err) {
            printf("ftan %20.5f : %15.12f %15.12f %15.12f %10.5f\n", theta * RTOD, t, t2, t_err, t_err/eps);
            max_t_err = t_err;
        }
    }
#endif
}

void
fasttrig_test_arctrig (void)
{
#if 0
    if (!initialized)
        fasttrig_init ();

    gsl_rng *r = gsl_rng_alloc (gsl_rng_taus);
    gsl_rng_set (r, gslu_rng_seed ());

    printf ("       %18s : %15s %15s %15s\n", "angle err", "math", "fasttrig", "arg(s)");
    double max_s_err = 0, max_c_err = 0, max_t_err = 0;
    for (size_t i = 0; i < 200000000; i++) {
        double x = gsl_rng_uniform (r) * 2. - 1.;
        double y = gsl_rng_uniform (r) * 2. - 1.;

        double s = asin (y);
        double s2 = fasin (y);

        double c = acos (y);
        double c2 = facos (y);

        double t = atan2 (y, x);
        double t2 = fatan2 (y, x);

        double s_err = fabs (s-s2), c_err = fabs (c-c2), t_err = fabs (t-t2);
        if (s_err > max_s_err) {
            printf ("fasin: %18.5f : %15f %15f %15f\n", s_err * RTOD, s * RTOD, s2 * RTOD, y);
            max_s_err = s_err;
        }
        if (c_err > max_c_err) {
            printf ("facos: %18.5f : %15f %15f %15f\n", c_err * RTOD, c * RTOD, c2 * RTOD, y);
            max_c_err = c_err;
        }
        if (t_err > max_t_err) {
            printf ("fatan: %18.5f : %15f %15f %15f %15f\n", t_err * RTOD, t * RTOD, t2 * RTOD, x, y);
            max_t_err = t_err;
        }
    }
#endif
}

/*
int main(int argc, char *argv[])
{
    fasttrig_init ();
    fasttrig_trig_test ();
    fasttrig_arctrig_test ();
}
*/
