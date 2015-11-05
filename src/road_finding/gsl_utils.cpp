#include "gsl_utils.h"

double gsl_mahalanobis_distance(gsl_vector *x, gsl_vector *u, gsl_matrix *C)
{
  int size = C->size1;

  gsl_matrix *C_cpy = gsl_matrix_alloc(size, size);
  gsl_matrix *C_inv = gsl_matrix_alloc(size, size);
  gsl_permutation *p = gsl_permutation_alloc(size);
  int sign;

  gsl_matrix_memcpy(C_cpy, C);

  gsl_matrix *X = gsl_matrix_alloc(size, 1);
  gsl_matrix *U = gsl_matrix_alloc(size, 1);
  gsl_matrix_set_col(X, 0, x);
  gsl_matrix_set_col(U, 0, u);

  gsl_matrix_sub(X, U); // x - u

  gsl_linalg_LU_decomp(C_cpy, p, &sign);
  gsl_linalg_LU_invert(C_cpy, p, C_inv);

  gsl_matrix *CX = gsl_matrix_alloc(size, 1);
  gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, C_inv, X, 0.0, CX);
  gsl_matrix *XCX = gsl_matrix_alloc(1, 1);
  gsl_blas_dgemm(CblasTrans, CblasNoTrans, 1.0, X, CX, 0.0, XCX);

  double distance = gsl_matrix_get(XCX, 0, 0);

  gsl_matrix_free(C_cpy);
  gsl_matrix_free(C_inv);
  gsl_matrix_free(X);
  gsl_matrix_free(U);
  gsl_matrix_free(CX);
  gsl_matrix_free(XCX);
  gsl_permutation_free(p);

  return distance;
}

double gsl_get_determinant(gsl_matrix *A)
{
  gsl_permutation *p = gsl_permutation_calloc(A->size1);
  gsl_matrix *tmp_ptr = gsl_matrix_calloc(A->size1, A->size1);
  gsl_matrix_memcpy(tmp_ptr, A);
  int sign = 0;

  gsl_linalg_LU_decomp(tmp_ptr, p, &sign);
  double determinant = gsl_linalg_LU_det(tmp_ptr, sign);

  gsl_permutation_free(p);
  gsl_matrix_free(tmp_ptr);

  return determinant;
}

double gsl_get_normal(gsl_vector *x, gsl_vector *u, gsl_matrix *cov)
{
	int size = cov->size1;
  gsl_matrix *tmp_ptr = gsl_matrix_alloc(cov->size1, cov->size2);
  gsl_matrix_memcpy(tmp_ptr, cov);

  double det = gsl_get_determinant(tmp_ptr);

  double exp;

  if (fabs(det) < 0.0001) //singular matrix
    exp = 0.0;
  else
    exp = -0.5 * gsl_mahalanobis_distance(x, u, cov);

  double f1 = 1.0 / sqrt( pow(2 * M_PI, size) * fabs(det));
  double f2 = pow(M_E, exp);

  double normal = f1 * f2;

  gsl_matrix_free(tmp_ptr);

  return normal;
}
