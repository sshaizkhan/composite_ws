#include "symmetricMatrixSPDProjector.h"
#include "matrix.h"

void SymmetricMatrixSPDProjector::Project(int n, double * mtx)
{
  Matrix<double> A(n, n, mtx, false, false);
  Matrix<double> Q(n, n);
  Matrix<double> Lambda(n, 1);
  A.SymmetricEigenDecomposition(Q, Lambda); // A = Q * Lambda * Q^T
  for (int i = 0; i < n; i++)
  {
    if (Lambda(i, 0) < 0.0)
      Lambda(i, 0) = 0;
  }
  A = Q * Matrix<double>(n, Lambda.GetData(), false) * Transpose(Q);
  memcpy(mtx, A.GetData(), sizeof(double) * n * n);
}

