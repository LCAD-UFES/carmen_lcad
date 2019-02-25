#ifndef __GSL_UTIL_H__
#define __GSL_UTIL_H__

#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permutation.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Macros for creating a gsl_vector_view and gsl_matrix_view object
 * and data on the stack
 */
#define GSLU_VECTOR_VIEW(var,i)                                         \
    struct {                                                            \
        double data[i];                                                 \
        gsl_vector vector;                                              \
    } var;                                                              \
    {   /* _view has local scope */                                     \
        gsl_vector_view _view = gsl_vector_view_array (var.data, i);    \
        var.vector = _view.vector;                                      \
    }
        

#define GSLU_MATRIX_VIEW(var,i,j)                                       \
    struct {                                                            \
        double data[i*j];                                               \
        gsl_matrix matrix;                                              \
    } var;                                                              \
    {   /* _view has local scope */                                     \
        gsl_matrix_view _view = gsl_matrix_view_array (var.data, i, j); \
        var.matrix = _view.matrix;                                      \
    }



/*-------Early function declarations to satisfy compiler dependencies-------*/
static inline gsl_matrix * 
gslu_matrix_dup (const gsl_matrix *A);


/*============================LINEAR ALGEBRA===============================*/
typedef struct _gslu_linalg_LU gslu_linalg_LU;
struct _gslu_linalg_LU {
    gsl_matrix *LU;
    gsl_permutation *p;
    int signum;
};

typedef struct _gslu_linalg_QR gslu_linalg_QR;
struct _gslu_linalg_QR {
    gsl_matrix *QR;
    gsl_vector *tau;
};

typedef struct _gslu_linalg_SV gslu_linalg_SV;
struct _gslu_linalg_SV {
    gsl_matrix *U;
    gsl_vector *S;
    gsl_matrix *V;
};

// free gslu_LU object
static inline void
gslu_linalg_LU_free (gslu_linalg_LU *lu)
{
    if (lu != NULL) {
        gsl_matrix_free (lu->LU);
        gsl_permutation_free (lu->p);
        free (lu);
    }
}

// compute LU decomposition, returns NULL on error
static inline gslu_linalg_LU *
gslu_linalg_LU_decomp_alloc (const gsl_matrix *A)
{
    gslu_linalg_LU *lu = malloc (sizeof (gslu_linalg_LU));
    lu->LU = gslu_matrix_dup (A);
    lu->p = gsl_permutation_alloc (A->size2);
    if (gsl_linalg_LU_decomp (lu->LU, lu->p, &lu->signum) != GSL_SUCCESS) {
        gslu_linalg_LU_free (lu);
        return NULL;
    }
    return lu;
}

// free gslu_QR object
static inline void
gslu_linalg_QR_free (gslu_linalg_QR *qr)
{
    if (qr != NULL) {
        gsl_matrix_free (qr->QR);
        gsl_vector_free (qr->tau);
        free (qr);
    }
}

// compute QR decomposition, returns NULL on error
static inline gslu_linalg_QR *
gslu_linalg_QR_decomp_alloc (const gsl_matrix *A)
{
    gslu_linalg_QR *qr = malloc (sizeof (gslu_linalg_QR));
    qr->QR = gslu_matrix_dup (A);
    qr->tau = gsl_vector_alloc (GSL_MIN (A->size1, A->size2));
    if (gsl_linalg_QR_decomp (qr->QR, qr->tau) != GSL_SUCCESS) {
        gslu_linalg_QR_free (qr);
        return NULL;
    }
    return qr;
}

// free gslu_SV object
static inline void
gslu_linalg_SV_free (gslu_linalg_SV *sv)
{
    gsl_matrix_free (sv->U);
    gsl_vector_free (sv->S);
    gsl_matrix_free (sv->V);
    free (sv);
}

// compute SV decomposition, returns NULL on error
static inline gslu_linalg_SV *
gslu_linalg_SV_decomp_alloc (const gsl_matrix *A)
{
    gslu_linalg_SV *sv = malloc (sizeof (gslu_linalg_SV));
    sv->U = gslu_matrix_dup (A);
    sv->S = gsl_vector_alloc (A->size2);
    sv->V = gsl_matrix_alloc (A->size2, A->size2);
    gsl_vector *work = gsl_vector_alloc (A->size2);
    if (gsl_linalg_SV_decomp (sv->U, sv->V, sv->S, work) != GSL_SUCCESS) {
        gslu_linalg_SV_free (sv);
        gsl_vector_free (work);
        return NULL;
    }
    gsl_vector_free (work);
    return sv;
}

/*==============================VECTOR====================================*/

/* prints the contents of a vector to stdout.  each element is formatted
 * using the printf-style format specifier fmt.  If fmt is NULL, then it
 * defaults to "%f"
 */
void
gslu_vector_printf (const gsl_vector *v, const char *name, const char *fmt);

void
gslu_vector_printf_row (const gsl_vector *a, const char *name, const char *fmt);

// duplicate a vector
static inline gsl_vector *
gslu_vector_dup (const gsl_vector *a)
{
    gsl_vector *b = gsl_vector_alloc (a->size);
    gsl_vector_memcpy (b, a);
    return b;
}

// returns 1 if vectors a and b have the same size
static inline int
gslu_vector_is_same_size (const gsl_vector *a, const gsl_vector *b)
{
    return a->size == b->size;
}

// returns 1 if all elements of a and b are exactly the same
static inline int
gslu_vector_is_equal (const gsl_vector *A, const gsl_vector *B)
{
    for (size_t i=0; i<A->size; i++) {
        double a = gsl_vector_get (A, i);
        double b = gsl_vector_get (B, i);
        if (fabs (a-b) > 1e-14)
            return 0;
    }
    return 1;
}


// copies vector b into a subvector of vector a starting at a(i)
static inline int
gslu_vector_set_subvector (gsl_vector *a, const size_t i, const gsl_vector *b)
{
    gsl_vector_view asub = gsl_vector_subvector (a, i, b->size);
    return gsl_vector_memcpy (&asub.vector, b);
}

// sum of the individual elements of a vector
static inline double
gslu_vector_sum (const gsl_vector *v)
{
    double sum = 0;
    for (size_t i=0; i<v->size; i++)
        sum += gsl_vector_get (v, i);
    return sum;
}

// absolute sum of the individual elements of a vector
static inline double
gslu_vector_abs_sum (const gsl_vector *v)
{
    return gsl_blas_dasum (v);
}

// magnitude of a vector
static inline double
gslu_vector_norm (const gsl_vector *v)
{
    return gsl_blas_dnrm2 (v);
}

// computes the euclidean distance between two vectors.
static inline double
gslu_vector_dist (const gsl_vector *a, const gsl_vector *b)
{
    gsl_vector *c = gslu_vector_dup (a);
    gsl_vector_sub (c, b);
    double dist = gslu_vector_norm (c);
    gsl_vector_free (c);
    return dist;
}

// computes vector dot product a'*b
static inline double
gslu_vector_dot (const gsl_vector *a, const gsl_vector *b)
{
    double result;
    gsl_blas_ddot (a, b, &result);
    return result;
}

/*==============================MATRIX====================================*/

/* prints the contents of a matrix to stdout.  each element is formatted
 * using the printf-style format specifier fmt.  If fmt is NULL, then it
 * defaults to "%f"
 */
void
gslu_matrix_printf (const gsl_matrix *m, const char *name, const char *fmt);


// prints a double array as a size1 by size2 gsl_matrix
static inline void
gslu_array_printf (const double data[], size_t size1, size_t size2, const char *name, const char *fmt)
{
    gsl_matrix_const_view A = gsl_matrix_const_view_array (data, size1, size2);
    gslu_matrix_printf (&A.matrix, name, fmt);
}

// duplicate a matrix
static inline gsl_matrix *
gslu_matrix_dup (const gsl_matrix *A)
{
    gsl_matrix *B = gsl_matrix_alloc (A->size1, A->size2);
    gsl_matrix_memcpy (B, A);
    return B;
}

// returns 1 if matrices A and B have the same size
static inline int 
gslu_matrix_is_same_size (const gsl_matrix *A, const gsl_matrix *B) 
{
    return A->size1 == B->size1 && A->size2 == B->size2;
}

// returns 1 if matrix A is square in size
static inline int
gslu_matrix_is_square (const gsl_matrix *A)
{
    return A->size1 == A->size2;
}

// returns 1 if all elements of A and B are exactly the same
static inline int
gslu_matrix_is_equal (const gsl_matrix *A, const gsl_matrix *B)
{
    for (size_t i=0; i<A->size1; i++) {
        for (size_t j=0; j<A->size2; j++) {
            double a = gsl_matrix_get (A, i, j);
            double b = gsl_matrix_get (B, i, j);
            if (fabs (a-b) > 1e-14)
                return 0;
        }
    }
    return 1;
}

// copies matrix B into a submatrix of matrix A starting at A(i,j)
static inline int
gslu_matrix_set_submatrix (gsl_matrix *A, const size_t i, const size_t j, const gsl_matrix *B)
{
    gsl_matrix_view Asub = gsl_matrix_submatrix (A, i, j, B->size1, B->size2);
    return gsl_matrix_memcpy (&Asub.matrix, B);
}

// in-place inverse of matrix
static inline int
gslu_matrix_inv (gsl_matrix *A)
{
    gslu_linalg_LU *lu = gslu_linalg_LU_decomp_alloc (A);
    int ret = gsl_linalg_LU_invert (lu->LU, lu->p, A);
    gslu_linalg_LU_free (lu);
    return ret;
}

static inline gsl_matrix *
gslu_matrix_inv_alloc (const gsl_matrix *A)
{
    gsl_matrix *B = gslu_matrix_dup (A);
    gslu_matrix_inv (B);
    return B;
}

// transpose of matrix
static inline gsl_matrix *
gslu_matrix_transpose_alloc (const gsl_matrix *A)
{
    gsl_matrix *B = gsl_matrix_alloc (A->size2, A->size1);
    gsl_matrix_transpose_memcpy (B, A);
    return B;
}

// determinant of matrix
static inline double
gslu_matrix_det (const gsl_matrix *A)
{
    gslu_linalg_LU *lu = gslu_linalg_LU_decomp_alloc (A);
    double det = gsl_linalg_LU_det (lu->LU, lu->signum);
    gslu_linalg_LU_free (lu);
    return det;
}

// trace of matrix
static inline double
gslu_matrix_trace (const gsl_matrix *A)
{
    gsl_vector_const_view d = gsl_matrix_const_diagonal (A);
    return gslu_vector_sum (&d.vector);
}


/*===================SIMPLE MATRIX VECTOR OPS=================================*/

// computes b = A*x
static inline int
gslu_mv (gsl_vector *b, const gsl_matrix *A, const gsl_vector *x)
{
    return gsl_blas_dgemv (CblasNoTrans, 1.0, A, x, 0.0, b);
}

static inline gsl_vector *
gslu_mv_alloc (const gsl_matrix *A, const gsl_vector *x)
{
    gsl_vector *b = gsl_vector_alloc (A->size1);
    gslu_mv (b, A, x);
    return b;
}


// computes C = A*B
static inline int
gslu_mm (gsl_matrix *C, const gsl_matrix *A, const gsl_matrix *B)
{
    return gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, A, B, 0.0, C);
}

static inline gsl_matrix *
gslu_mm_alloc (const gsl_matrix *A, const gsl_matrix *B)
{
    gsl_matrix *C = gsl_matrix_alloc (A->size1, B->size2);
    gslu_mm (C, A, B);
    return C;
}


// computes D = A*B*C
static inline int
gslu_mmm (gsl_matrix *D, const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C)
{
    gsl_matrix *tmp = gslu_matrix_dup (D);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, A, B, 0.0, tmp);
    int ret = gsl_blas_dgemm (CblasNoTrans, CblasNoTrans, 1.0, tmp, C, 0.0, D);
    gsl_matrix_free (tmp);
    return ret;
}

static inline gsl_matrix *
gslu_mmm_alloc (const gsl_matrix *A, const gsl_matrix *B, const gsl_matrix *C)
{
    gsl_matrix *D = gsl_matrix_alloc (A->size1, C->size2);
    gslu_mmm (D, A, B, C);
    return D;
}


/*===============================BLAS======================================*/

// result = alpha * op(A)*x + beta*y
static inline gsl_vector *
gslu_blas_dgemv_alloc (CBLAS_TRANSPOSE_t TransA, double alpha, 
                       const gsl_matrix *A, const gsl_vector *x, double beta, const gsl_vector *y)
{
    gsl_vector *result = gslu_vector_dup (y);
    gsl_blas_dgemv (TransA, alpha, A, x, beta, result);
    return result;
}

// result = alpha * op(A)*op(B) + beta*C
static inline gsl_matrix *
gslu_blas_dgemm_alloc (CBLAS_TRANSPOSE_t TransA, CBLAS_TRANSPOSE_t TransB, double alpha,
                       const gsl_matrix *A, const gsl_matrix *B, double beta, const gsl_matrix *C)
{
    gsl_matrix *result = gslu_matrix_dup (C);
    gsl_blas_dgemm (TransA, TransB, alpha, A, B, beta, result);
    return result;
}

/*===========================RANDOM NUMBER GENERATOR=========================*/

static inline int
gslu_rng_seed (void)
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int) tv.tv_usec;
}


#ifdef __cplusplus
}
#endif

#endif // __GSL_UTIL_H__
