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
  A class to timestep large sparse dynamics of many objects, 
  using implicit Euler, with constraints.

  Constraints are solved using a KKT system, similar to:

  M. Bergou, S. Mathur, M. Wardetzky, E. Grinspun,
  TRACKS: Toward Directable Thin Shells
  SIGGRAPH(ACM Transactions on Graphics), pp.50, 2007

  This class uses PARDISO as the solver.
*/

#ifndef _IMPLICITEULERMULTIOBJECTWITHCONSTRAINTSSPARSE_H_
#define _IMPLICITEULERMULTIOBJECTWITHCONSTRAINTSSPARSE_H_

#ifdef __APPLE__
  #include "TargetConditionals.h"
#endif

// This code supports three different solvers for sparse linear systems of equations:
// SPOOLES, PARDISO, Jacobi-preconditioned Conjugate Gradients
// You must define exactly one of the macros SPOOLES, PARDISO, PCG,
// by changing the file integratorSolverSelection.h .
// PCG is available with our code; look for it in the "sparseMatrix" library (CGSolver.h)
// SPOOLES is available at: http://www.netlib.org/linalg/spooles/spooles.2.2.html
// For PARDISO, the class was tested with the PARDISO implementation from the Intel Math Kernel Library

#include "integratorSolverSelection.h"
#include "sparseMatrix.h"
#include "integratorBaseMultiObjectSparse.h"
#include "sparseSolvers.h"
#include <vector>

class ImplicitEulerMultiObjectWithConstraintsSparse : public IntegratorBaseMultiObjectSparse
{
public:

  // constrainedDOFs is an integer array of degrees of freedom that are to be fixed to zero (e.g., to permanently fix a vertex in a deformable simulation)
  // constrainedDOFs are 0-indexed (separate DOFs for x,y,z), and must be pre-sorted (ascending)
  // numThreads applies only to the PARDISO solver; if numThreads > 0, the sparse linear solves are multi-threaded; default: 0 (use single-threading)
  ImplicitEulerMultiObjectWithConstraintsSparse(int numObjects, const int * r, double timestep, 
    SparseMatrix ** massMatrix, ForceModel ** forceModel, const int * numConstrainedDOFs, int ** constrainedDOFs, 
    const double * dampingMassCoef, const double * dampingStiffnessCoef, int maxIterations = 1,
     double epsilon = 1E-6, int numSolverThreads=0); 

  ImplicitEulerMultiObjectWithConstraintsSparse(int numObjects, const int * r, double timestep, 
    SparseMatrix ** massMatrix, ForceModel ** forceModel, const int * numConstrainedDOFs, int ** constrainedDOFs, 
    double dampingMassCoef, double dampingStiffnessCoef, int maxIterations = 1,
     double epsilon = 1E-6, int numSolverThreads=0); 

  virtual ~ImplicitEulerMultiObjectWithConstraintsSparse();

  // damping matrix provides damping in addition to mass and stiffness damping (it does not replace it)
  virtual void SetDampingMatrix(int objectID, const SparseMatrix * dampingMatrix, int reuseTopology=1);
  inline virtual void SetTimestep(double timestep) { this->timestep = timestep; }
  
  // initialGuessFactor -> [0, 1] controls at which displacement to compute the nonlinear elastic force and stiffness matrix
  //  0: compute at last disp. q  
  //  1: compute at q + h * qvel
  void setInitialGuessFactor(double guess) { initialGuessFactor = guess; }
  double getInitialGuessFactor() const { return initialGuessFactor; }

  // sets q, qvel 
  // automatically computes acceleration assuming zero external force
  // returns 0 on succes, 1 if solver fails to converge
  // note: there are also other state setting routines in the base class
  virtual int SetState(double * q, double * qvel=NULL);

  virtual int SetState(int objectID, const double * q, const double * qvel=NULL);

  // add a local tangent stiffness matrix offset to each object
  // see parent class for usage
  virtual void SetTangentStiffnessMatrixOffset(int objectID, const SparseMatrix * tangentStiffnessMatrixOffset, int reuseTopology=1);

  // add an offset to the **global** multi-object tangent stiffness matrix (default: no offset)
  // adjusts the global stiffness matrix topology, by initializing the non-zero locations specified by "tangentStiffnessMatrixOffsetTopology"
  // note: the actual values in "tangentStiffnessMatrixOffsetTopology" matter and will be used as the initial offset; if you don't want this, you can first clear the "tangentStiffnessMatrixOffsetTopology" to 0
  // if you want to clear the offset, set tangentStiffnessMatrixOffsetTopology to NULL
  // note: this function builds indices for fast matrix assignment of individual object's tangent stiffness matrices, and the offset assignment via "SetTangentStiffnessMatrixOffset", which takes time, so it should not be called often
  void InitTangentStiffnessMatrixOffset(const SparseMatrix * tangentStiffnessMatrixOffsetTopology); 
  // set the offset to a new value (must be previously initialized via "InitTangentStiffnessMatrixOffset")
  void SetTangentStiffnessMatrixOffset(const SparseMatrix * tangentStiffnessMatrixOffset); 

  // performs one step of simulation (returns 0 on sucess, and 1 on failure)
  // failure can occur, for example, if you are using the positive definite solver and the system matrix has negative eigenvalues
  virtual int DoTimestep(); 

  // dynamic solver is default (i.e. useStaticSolver=false)
  virtual void UseStaticSolver(bool useStaticSolver);

  void SetConstraintMatrix(const SparseMatrix * C, const SparseMatrix * B = NULL); // B is the lower-right block; B=NULL will set it to zero; note: do not use an empty sparse matrix for B to achieve a zero B, must use B=NULL
  void SetConstraintRhs(const double * constraintRhs);
  // this function clears any previously set C and B, i.e., returns to non-constrained state
  void ClearConstraints();

  void SetNumSolverThreads(int numSolverThreads);

  // qBoundaryDOFs is an array of length numConstrainedDOFsTotal / numConstrainedDOFs[objectID]
  // that tells the fixedDOFs of q at next timestep
  void SetDirichletBoundaryCondition(const double * qBoundaryTotal);
  void SetDirichletBoundaryCondition(int objectID, const double * qBoundary);

protected:
  std::vector<SparseMatrix *> rayleighDampingMatrix;
  std::vector<SparseMatrix *> tangentStiffnessMatrix;
 
  int maxIterations;
  double epsilon;
  int numSolverThreads;
  
  SparseMatrix * systemMatrix;

  SparseMatrix * globalTangentStiffnessMatrixOffset;

  //int * positiveDefiniteSolver;
  LagrangeMultiplierSolver * lagrangeMultiplierSolver;

  SparseMatrix * C;
  SparseMatrix * Ceff; // "effective" (properly scaled) matrix that is passed to the solver
  SparseMatrix * B; // lower-right block
  SparseMatrix * Beff; // "effective" (properly scaled) matrix that is passed to the solver
  std::vector<double> constraintRhs;

  std::vector<double> rhs;
  bool useStaticSolver;

  int numConstrainedDOFsTotal;
  std::vector<int> constrainedDOFsTotal;

  double initialGuessFactor;

  std::vector<double> dampingForceBuffer;
  
  std::vector<double> qBoundaryTotal;

  void BuildSystemMatrixTopology();
  void RebuildSolver();
  int BuildUnconstrainedSystemMatrixAndRhs();
  int AddConstraintMatrixAndRhs();
};

#endif

