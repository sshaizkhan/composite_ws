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
 * Code authors: Hongyi Xu, Jernej Barbic                                *
 *                                                                       *
 *************************************************************************/

#pragma once

#ifndef _CLOTHFEM_STENCIL_FORCEMODEL_H_
#define _CLOTHFEM_STENCIL_FORCEMODEL_H_

#include "stencilForceModel.h"

class ClothFEM;

// Stencils for cloth simulation. There are two different stencils here.
// One is a triangle (# vertices = 3) (for in-place stretch and shear);
// and the other is an edge joining two triangles (# vertices = 4) (for bending).
// See comments in the parent class.
class ClothFEMStencilForceModel : public StencilForceModel
{
public:
  ClothFEMStencilForceModel(const ClothFEM * clothBW);
  virtual ~ClothFEMStencilForceModel();

  virtual void GetStencilLocalEnergyAndForceAndMatrix(int stencilType, int stencilId, const double *u, double *energy, double *internalForces, double *tangentStiffnessMatrix) override;
  virtual const int *GetStencilVertexIndices(int stencilType, int stencilId) const override;

  void EnableKFilter(int enable) { filterK = enable; }
  const ClothFEM * GetForceModelHandle() { return clothFEM; }

protected:
  const ClothFEM * clothFEM;
  int filterK = 0;
};

#endif

