/*************************************************************************
 *                                                                       *
 * Copyright (C) 2020 University of Southern California                  *
 * All rights reserved.                                                  *
 *                                                                       *
 * If you would like to use this code for any purpose                    *
 * (commercial, academic, non-profit, etc.),                             *
 * please contact jernej.barbic@gmail.com, or write to:                  *
 *                                                                       *
 * USC Stevens Center for Innovation                                     *
 * 1150 South Olive Street, Suite 2300                                   *
 * Los Angeles, California 90015                                         *
 * United States                                                         *
 *                                                                       *
 * Constraints between n objects                                         *
 * Code authors: Bohan Wang, Jernej Barbic                               *
 *                                                                       *
 *************************************************************************/

#include "symmetricMatrixSPDProjector.h"
#include "clothFEMStencilForceModel.h"
#include "clothFEM.h"

#include <mkl_lapacke.h>
#include <mkl_cblas.h>

#include <iostream>
#include <cstdlib>

ClothFEMStencilForceModel::ClothFEMStencilForceModel(const ClothFEM * clothFEM_):
  clothFEM(clothFEM_)
{
  numStencilsInDifferentTypes.push_back(clothFEM->GetNumTriangles());
  numStencilsInDifferentTypes.push_back(clothFEM->GetNumQuads());

  numStencilVerticesInDifferentTypes.push_back(3);
  numStencilVerticesInDifferentTypes.push_back(4);

  n3 = clothFEM->GetNumVertices() * 3;
}

ClothFEMStencilForceModel::~ClothFEMStencilForceModel()
{
}

template<int dim>
void projectSPD(double *A)
{
  double eigenValues[dim];
  double eigenVectors[dim * dim], eigenVectors2[dim * dim];
  memcpy(eigenVectors, A, sizeof(double) * dim * dim);

  // compute eigenvalues
  lapack_int info = LAPACKE_dsyev(LAPACK_COL_MAJOR, 'V', 'U', dim, eigenVectors, dim, eigenValues);
  if (info > 0) 
  {
    std::cerr << "The algorithm failed to compute eigenvalues." << std::endl;
    exit(1);
  }

  for (int i = 0; i < dim; i++) 
  {
    if (eigenValues[i] < 0) 
    {
      eigenValues[i] = 0;
    }
  }

  memcpy(eigenVectors2, eigenVectors, sizeof(double) * dim * dim);
  for (int i = 0; i < dim; i++)
    cblas_dscal(dim, eigenValues[i], eigenVectors2 + i * dim, 1);

  cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, dim, dim, dim, 1.0, eigenVectors2, dim, eigenVectors, dim, 0.0, A, dim);
}

void ClothFEMStencilForceModel::GetStencilLocalEnergyAndForceAndMatrix(int stencilType, int stencilId, const double *u, double *energy, double *internalForces, double *tangentStiffnessMatrix)
{
  if (stencilType == 0) 
  {
    clothFEM->ComputeShearAndStretchTerms(stencilId, u, energy, internalForces, tangentStiffnessMatrix);

    if (filterK && tangentStiffnessMatrix) 
    {
      projectSPD<9>(tangentStiffnessMatrix);
    }
  } 
  else if (stencilType == 1) 
  {
    clothFEM->ComputeBendTerms(stencilId, u, energy, internalForces, tangentStiffnessMatrix);

    if (filterK && tangentStiffnessMatrix) 
    {
      projectSPD<12>(tangentStiffnessMatrix);
    }
  } 
  else 
  {
    std::cerr << "Incorrect element type." << std::endl;
    abort();
  }
}

const int * ClothFEMStencilForceModel::GetStencilVertexIndices(int stencilType, int stencilId) const
{
  if (stencilType == 0)
    return clothFEM->GetTriangleVertexIndices(stencilId);
  else if (stencilType == 1)
    return clothFEM->GetQuadVertexIndices(stencilId);
  else 
  {
    std::cerr << "Incorrect element type." << std::endl;
    abort();
  }

  return nullptr;
}

