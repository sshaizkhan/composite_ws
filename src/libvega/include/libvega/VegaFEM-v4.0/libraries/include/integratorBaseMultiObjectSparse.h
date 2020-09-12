/*************************************************************************
 *                                                                       *
 * A class to timestep large sparse dynamics of many objects,            *
 * using implicit Euler, with constraints.                               *
 *                                                                       *
 * Copyright (C) 2015 University of Southern California                  *
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
 * Code authors: Hongyi Xu, Jernej Barbic                                *
 *                                                                       *
 *************************************************************************/

/*
  A base class to timestep large sparse dynamics of multiple objects.
  See also integratorBase.h .
*/

#ifndef _INTEGRATORBASEMULTIOBJECTSPARSE_H_
#define _INTEGRATORBASEMULTIOBJECTSPARSE_H_

#include "sparseMatrix.h"
#include "forceModel.h"
#include "integratorBase.h"
#include <vector>

class IntegratorBaseMultiObjectSparse : public IntegratorBase
{
public:

  // constrainedDOFs is an integer array of degrees of freedom that are to be fixed to zero (e.g., to permanently fix a vertex in a deformable simulation)
  // constrainedDOFs are 0-indexed (separate DOFs for x,y,z), and must be pre-sorted (ascending)
  // Entries in massMatrix and forceModel are referenced by IntegratorBaseMultiObjectSparse, don't delete them until this object is no longer used
  IntegratorBaseMultiObjectSparse(int numObjects, const int * r, double timestep, SparseMatrix ** massMatrix, ForceModel ** forceModel, const int * numConstrainedDOFs, int ** constrainedDOFs, const double * dampingMassCoef, const double * dampingStiffnessCoef);

  virtual ~IntegratorBaseMultiObjectSparse();

  inline virtual void SetForceModel(int objectID, ForceModel * forceModel_) { forceModel[objectID] = forceModel_; }

  // damping matrix provides damping in addition to mass and stiffness damping (it does not replace it)
  virtual void SetDampingMatrix(int objectID, const SparseMatrix * dampingMatrix, int reuseTopology=1);

  // add an offset to the tangent stiffness matrix (default: no offset)
  // you must use reuseTopology=0 if the offset has been previously set, and the topology of the new tangentStiffnessMatrixOffset matrix is different from the old one
  virtual void SetTangentStiffnessMatrixOffset(int objectID, const SparseMatrix * tangentStiffnessMatrixOffset, int reuseTopology=1); 
  // the operand must have same topology as the previously set tangent stiffness matrix
  virtual void AddTangentStiffnessMatrixOffset(int objectID, const SparseMatrix * tangentStiffnessMatrixOffset); 
  virtual void ClearTangentStiffnessMatrixOffset(int objectID);

  // performs one step of simulation (returns 0 on sucess, and 1 on failure)
  // failure can occur, for example, if you are using the positive definite solver and the system matrix has negative eigenvalues
  virtual int DoTimestep() = 0;

  // returns the execution time of the last r x r linear system solve
  inline virtual double GetSystemSolveTime() { return systemSolveTime; }
  inline virtual double GetForceAssemblyTime() { return forceAssemblyTime; }

  virtual double GetKineticEnergy(int objectID);
  virtual double GetTotalMass(int objectID);

  virtual double GetKineticEnergy();
  virtual double GetTotalMass();

  inline int GetNumObjects() { return numObjects; }
  using IntegratorBase::Getr;
  inline int Getr(int objectID) { return rArray[objectID]; }

protected:
  int numObjects;
  std::vector<int> rArray, rStart;
  
  std::vector<SparseMatrix *> massMatrix; 
  std::vector<ForceModel *> forceModel;
  std::vector<double> dampingMassCoef, dampingStiffnessCoef;

  std::vector<SparseMatrix *> dampingMatrix;

  std::vector<SparseMatrix *> tangentStiffnessMatrixOffset;

  std::vector<int> numConstrainedDOFs;
  std::vector<std::vector<int> > constrainedDOFs;

  double systemSolveTime;
  double forceAssemblyTime;

  static int ComputeTotalDOFs(int numObjects, const int * r);
};

#endif

