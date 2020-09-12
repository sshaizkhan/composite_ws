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

#pragma once

#ifndef _CLOTH_FEM_H_
#define _CLOTH_FEM_H_

#include "vec3d.h"
#include "vec3i.h"

class ClothFEMImpl;
class SparseMatrix;

/*

The stretch and shear terms use:

  Pascal Volino, Nadia Magnenat-Thalmann, Francois Faure:
  A simple approach to nonlinear tensile stiffness for accurate cloth simulation
  ACM Transactions on Graphics, 2009, Article No. 105

The bend term uses:

  Rasmus Tamstorf, Eitan Grinspun:
  Discrete bending forces and their Jacobians
  Graphical Models 75(6), 362-370, 2013

*/

class ClothFEM
{
public:
  // cloth material parameters for stretching and shearing
  struct StretchAndShearMaterial
  {
    double E; // shell Young's modulus; unit is [N / m]
    double nu; // Poisson's ratio; nu < 0.5; dimensionless
    StretchAndShearMaterial() : E(1e5), nu(0.45) {}
    StretchAndShearMaterial(double E_, double nu_) :
      E(E_), nu(nu_) {}
  };
  // cloth material parameters for bending
  struct BendMaterial
  {
    double E; // Young's modulus; unit is [N / m^2]
    double nu; // Poisson's ratio; nu < 0.5; dimensionless
    double h; // thickness of the shell; unit is [m]
    BendMaterial() : E(1e5), nu(0.45), h(1e-3) {}
    BendMaterial(double E_, double nu_, double h_) :
      E(E_), nu(nu_), h(h_) {}
  };
  
  ClothFEM(int numVertices, const double * restPositions, const double * masses,
    int numTriangles, const int * triangles, 
    const int * triangleStretchAndShearMaterialIDs,
    const int * triangleBendMaterialIDs,
    int numStretchAndShearMaterials,
    const StretchAndShearMaterial * stretchAndShearMaterials,
    int numBendMaterials, 
    const BendMaterial * bendMaterials);

  ClothFEM(int numVertices, const double * restPositions, const double * masses,
    int numTriangles, const int * triangles, const double * triangleUVs,
    const int * triangleStretchAndShearMaterialIDs,
    const int * triangleBendMaterialIDs,
    int numStretchAndShearMaterials, 
    const StretchAndShearMaterial * stretchAndShearMaterials,
    int numBendMaterials, 
    const BendMaterial * bendMaterials);

  ~ClothFEM();

  int GetNumVertices() const;
  int GetNumTriangles() const;
  int GetNumQuads() const;
  int GetNumStretchAndShearMaterials() const;
  int GetNumBendMaterials() const;

  const Vec3i * GetTriangles() const;
  const Vec3d * GetRestPositions() const;

  const int * GetTriangleVertexIndices(int tri) const;
  const int * GetQuadVertexIndices(int quad) const;
  const int * GetQuadTriangleIndices(int quad) const;

  // Allows user to toggle on/off the use of rest angles in bend force/stiffness matrix
  // calculations. If set to 1, bend force/matrix will be computed in relation to the quad's rest
  // angle. If set to 0, bend force/matrix will be computed in relation to a flat angle of 0.0.
  // Default is 1.
  void UseRestAnglesForBendForces(bool useRestAnglesForBendForces);

  // Creates the mass matrix (which is diagonal); each diagonal entry is expanded into 
  // a diagonal submatrix of size 'expanded' (typically, for 3D simulations, expanded should be 3).
  void GenerateMassMatrix(SparseMatrix ** M, int expanded = 3) const;

  // Set material to the entire object.
  void SetStretchAndShearMaterial(double E, double nu); 
  void SetStretchAndShearMaterial(const StretchAndShearMaterial * stretchAndShearMaterial);
  void SetBendMaterial(double E, double nu, double h);
  void SetBendMaterial(const BendMaterial * bendMaterial); 

  // compute elasticity
  void ComputeShearAndStretchTerms(int tri, const double *u, double *E, double f[9], double K[81]) const;
  void ComputeBendTerms(int quad, const double *u, double *E, double f[12], double K[144]) const;

protected:
  ClothFEMImpl *impl;
};

#endif

