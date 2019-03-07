#include <stdio.h>
#include <sys/ioctl.h>
#include <stdbool.h>

#include <gsl/gsl_statistics_double.h>

#include "gsl_util.h"

/*============================LINEAR ALGEBRA===============================*/

/*==============================VECTOR====================================*/
void
_gslu_vector_printf (const gsl_vector *a, const char *name, const char *fmt, const bool tranpose)
{
    struct winsize w;
    ioctl (0, TIOCGWINSZ, &w);
    int ncols = round (w.ws_col / 12.);
    /*
    printf ("lines %d\n", w.ws_row);
    printf ("columns %d\n", w.ws_col);
    */

    if (name != NULL)
        printf ("%s =\n", name);

    int __default__ = 1;
    if (fmt == NULL)
        fmt = "%10.4f";
    else
        __default__ = 0;

    const double ZERO = 1e-9;
    for (size_t n=0; n < (tranpose ? a->size : 1); n += ncols) {
        if (tranpose && ncols < a->size) {
            if (n == (a->size - 1))
                printf ("    Column %u\n", (int) n);
            else
                printf ("    Columns %u through %u\n", (int) n, (int) GSL_MIN(n+ncols, a->size)-1);
        }
        for (size_t i=GSL_MIN(n, a->size); i<GSL_MIN(n+ncols, a->size); i++) {
            double v = gsl_vector_get (a, i);
            if (__default__ && fabs (v) < ZERO)
                printf ("%10.4g", 0.);
            else
                printf (fmt, v);
            if (tranpose)
                printf (" ");
            else
                printf ("\n");
        }
        printf ("\n");
    }
}

void
gslu_vector_printf (const gsl_vector *a, const char *name, const char *fmt)
{
    _gslu_vector_printf (a, name, fmt, 0);
}

void
gslu_vector_printf_row (const gsl_vector *a, const char *name, const char *fmt)
{
    _gslu_vector_printf (a, name, fmt, 1);
}

/*==============================MATRIX====================================*/
void
gslu_matrix_printf (const gsl_matrix *A, const char *name, const char *fmt)
{

    struct winsize w;
    ioctl (0, TIOCGWINSZ, &w);
    int ncols = round (w.ws_col / 12.);
    /*
    printf ("lines %d\n", w.ws_row);
    printf ("columns %d\n", w.ws_col);
    */

    if (name != NULL)
        printf ("%s =\n", name);

    int __default__ = 1;
    if (fmt == NULL)
        fmt = "%10.4f";
    else
        __default__ = 0;


    const double ZERO = 1e-9;
    for (size_t n=0; n < A->size2; n += ncols) {
        if (ncols < A->size2) {
            if (n == (A->size2 - 1))
                printf ("    Column %u\n", (int) n);
            else
                printf ("    Columns %u through %u\n", (int) n, (int) GSL_MIN(n+ncols, A->size2)-1);
        }
        for (size_t i=0; i<A->size1; i++) {
            for (size_t j=GSL_MIN(n, A->size2); j<GSL_MIN(n+ncols, A->size2); j++) {
                double v = gsl_matrix_get (A, i, j);
                if (__default__ && fabs (v) < ZERO)
                    printf ("%10.4g", 0.);
                else
                    printf (fmt, v);
                printf (" ");
            }
            printf ("\n");
        }
    }
}


/*===================SIMPLE MATRIX VECTOR OPS=================================*/

/*===============================BLAS======================================*/

/*===========================RANDOM NUMBER GENERATOR=========================*/
