/*************************************************************************
 *                                                                       *
 * Implicit Bathe integration                                            *
 *                                                                       *
 * Copyright (C) 2019 University of Southern California                  *
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
 *************************************************************************/

/*
  A class to timestep large sparse dynamics using implicit Bathe.
  E.g., unreduced nonlinear FEM deformable dynamics.

  See also integratorBase.h .

  This class either uses SPOOLES, PARDISO, or our own Jacobi-preconitioned 
  CG to solve the large sparse linear systems.

  You can switch between these solvers at compile time,
  by modifying the file integratorSolverSelection.h (see below)
  (run-time solver switching would be possible too with more coding).
*/

#ifndef _IMPLICITBATHESPARSE_H_
#define _IMPLICITBATHESPARSE_H_

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
#include "integratorBaseSparse.h"

#ifdef PARDISO
  #include "sparseSolvers.h"
#endif
#ifdef SPOOLES
  #include "sparseSolvers.h"
#endif
#ifdef PCG
  #include "CGSolver.h"
#endif

class ImplicitBatheSparse : public IntegratorBaseSparse
{
public:

  // constrainedDOFs is an integer array of degrees of freedom that are to be fixed to zero (e.g., to permanently fix a vertex in a deformable simulation)
  // constrainedDOFs are 0-indexed (separate DOFs for x,y,z), and must be pre-sorted (ascending)
  // numThreads applies only to the PARDISO solver; if numThreads > 0, the sparse linear solves are multi-threaded; default: 0 (use single-threading)
  ImplicitBatheSparse(int r, double timestep, SparseMatrix * massMatrix, ForceModel * forceModel, int positiveDefiniteSolver=0, int numConstrainedDOFs=0, int * constrainedDOFs=NULL, double dampingMassCoef=0.0, double dampingStiffnessCoef=0.0, int maxIterations = 1, double epsilon = 1E-6, int numSolverThreads=0); 

  virtual ~ImplicitBatheSparse();

  // damping matrix provides damping in addition to mass and stiffness damping (it does not replace it)
  virtual void SetDampingMatrix(SparseMatrix * dampingMatrix);

  // sets q, qvel 
  // automatically computes acceleration assuming zero external force
  // returns 0 on succes, 1 if solver fails to converge
  // note: there are also other state setting routines in the base class
  virtual int SetState(double * q, double * qvel=NULL);

  // performs one step of simulation (returns 0 on sucess, and 1 on failure)
  // failure can occur, for example, if you are using the positive definite solver and the system matrix has negative eigenvalues
  virtual int DoTimestep(); 

  inline void SetForceModel(ForceModel * forceModel) {this->forceModel = forceModel; }
  // dynamic solver is default (i.e. useStaticSolver=false)
  //virtual void UseStaticSolver(bool useStaticSolver);

protected:
  SparseMatrix * rayleighDampingMatrix;
  SparseMatrix * tangentStiffnessMatrix;
  SparseMatrix * systemMatrix;

  double * bufferConstrained;
  double * q_2;
  double * qvel_2;
  double * qaccel_2;

  // parameters for implicit Bathe
  double epsilon; 
  int maxIterations;

  int positiveDefiniteSolver;
  int numSolverThreads;
  #ifdef PARDISO
    PardisoSolver * pardisoSolver;
  #endif

  #ifdef PCG
    CGSolver * jacobiPreconditionedCGSolver;
  #endif
};

#endif

