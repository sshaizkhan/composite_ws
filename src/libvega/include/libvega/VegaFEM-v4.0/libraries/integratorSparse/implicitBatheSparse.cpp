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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "matrixIO.h"
#include "performanceCounter.h"
#include "constrainedDOFs.h"
#include "implicitBatheSparse.h"

ImplicitBatheSparse::ImplicitBatheSparse(int r, double timestep, SparseMatrix * massMatrix_, ForceModel * forceModel_, int positiveDefiniteSolver_, int numConstrainedDOFs_, int * constrainedDOFs_, double dampingMassCoef, double dampingStiffnessCoef, int maxIterations, double epsilon, int numSolverThreads_): IntegratorBaseSparse(r, timestep, massMatrix_, forceModel_, numConstrainedDOFs_, constrainedDOFs_, dampingMassCoef, dampingStiffnessCoef), positiveDefiniteSolver(positiveDefiniteSolver_), numSolverThreads(numSolverThreads_)
{
  this->maxIterations = maxIterations; // maxIterations = 1 for semi-implicit
  this->epsilon = epsilon; 

  forceModel->GetTangentStiffnessMatrixTopology(&tangentStiffnessMatrix);

  if (tangentStiffnessMatrix->Getn() != massMatrix->Getn())
  {
    printf("Error: the provided mass matrix does not have correct size. Mass matrix: %d x %d. Stiffness matrix: %d x %d.\n", massMatrix->Getn(), massMatrix->Getn(), tangentStiffnessMatrix->Getn(), tangentStiffnessMatrix->Getn());
    exit(1);
  }

  rayleighDampingMatrix = new SparseMatrix(*tangentStiffnessMatrix);
  rayleighDampingMatrix->BuildSubMatrixIndices(*massMatrix);
  rayleighDampingMatrix->BuildSubMatrixIndices(*dampingMatrix, 1);
  tangentStiffnessMatrix->BuildSubMatrixIndices(*massMatrix);
  tangentStiffnessMatrix->BuildSubMatrixIndices(*dampingMatrix, 1);

  if (tangentStiffnessMatrix->GetNumRows() != massMatrix->GetNumRows())
  {
    printf("Error: mass matrix and stiffness matrix don't have same dimensions.\n");
    exit(1);
  }

  bufferConstrained = (double*) malloc (sizeof(double) * (r - numConstrainedDOFs));

  systemMatrix = new SparseMatrix(*tangentStiffnessMatrix);
  systemMatrix->RemoveRowsColumns(numConstrainedDOFs, constrainedDOFs);
  systemMatrix->BuildSuperMatrixIndices(numConstrainedDOFs, constrainedDOFs, tangentStiffnessMatrix);

  #ifdef PARDISO
    printf("Creating Pardiso solver. Positive-definite solver: %d. Num threads: %d\n", positiveDefiniteSolver, numSolverThreads);
    if (positiveDefiniteSolver)
      pardisoSolver = new PardisoSolver(systemMatrix, numSolverThreads, PardisoSolver::REAL_SPD, PardisoSolver::PARALLEL_NESTED_DISSECTION);
    else pardisoSolver = new PardisoSolver(systemMatrix, numSolverThreads, PardisoSolver::REAL_SYM_INDEFINITE, PardisoSolver::PARALLEL_NESTED_DISSECTION);
  #endif

  #ifdef PCG
    jacobiPreconditionedCGSolver = new CGSolver(systemMatrix);
  #endif

  q_2 = (double*)malloc(sizeof(double) * r);
  qvel_2 = (double*)malloc(sizeof(double) * r);
  qaccel_2 = (double*)malloc(sizeof(double) * r);

}

ImplicitBatheSparse::~ImplicitBatheSparse()
{
  delete(tangentStiffnessMatrix);
  delete(rayleighDampingMatrix);
  free(bufferConstrained);
}

void ImplicitBatheSparse::SetDampingMatrix(SparseMatrix * dampingMatrix)
{
  IntegratorBaseSparse::SetDampingMatrix(dampingMatrix);
  tangentStiffnessMatrix->BuildSubMatrixIndices(*dampingMatrix, 1);
}

// sets the state based on given q, qvel
// automatically computes acceleration assuming zero external force
int ImplicitBatheSparse::SetState(double * q_, double * qvel_)
{
  memcpy(q, q_, sizeof(double)*r);

  if (qvel_ != NULL)
    memcpy(qvel, qvel_, sizeof(double)*r);
  else
    memset(qvel, 0, sizeof(double)*r);

  for(int i=0; i<numConstrainedDOFs; i++)
    q[constrainedDOFs[i]] = qvel[constrainedDOFs[i]] = qaccel[constrainedDOFs[i]] = 0.0;

  // M * qaccel + C * qvel + R(q) = P_0 
  // R(q) = P_0 = 0
  // i.e. M * qaccel = - C * qvel - R(q)

  forceModel->GetForceAndMatrix(q, internalForces, tangentStiffnessMatrix);

  *rayleighDampingMatrix = dampingStiffnessCoef * (*tangentStiffnessMatrix);
  rayleighDampingMatrix->AddSubMatrix(dampingMassCoef, *massMatrix);

  // buffer = C * qvel
  rayleighDampingMatrix->MultiplyVector(qvel, buffer);
  dampingMatrix->MultiplyVectorAdd(qvel, buffer);

  for(int i=0; i<r; i++)
    buffer[i] = -buffer[i] - internalForces[i];

  // solve M * qaccel = buffer
  ConstrainedDOFs::RemoveDOFs(r, bufferConstrained, buffer, numConstrainedDOFs, constrainedDOFs);

  // use tangentStiffnessMatrix as the buffer place
  tangentStiffnessMatrix->ResetToZero();
  tangentStiffnessMatrix->AddSubMatrix(1.0, *massMatrix);
  tangentStiffnessMatrix->AddSubMatrix(1.0, *dampingMatrix, 1);
  systemMatrix->AssignSuperMatrix(*tangentStiffnessMatrix); // must go via a matrix with tangentStiffnessMatrix's topology, because the AssignSuperMatrix indices were computed with respect to such topology

  memset(buffer, 0, sizeof(double) * r);

  #ifdef SPOOLES
    SPOOLESSolver solver(systemMatrix);
    int info = solver.SolveLinearSystem(buffer, bufferConstrained);
    char solverString[16] = "SPOOLES";
  #endif

  //massMatrix->Save("M");
  //systemMatrix->Save("A");

  #ifdef PARDISO
    pardisoSolver->FactorMatrix(systemMatrix);
    int info = pardisoSolver->SolveLinearSystem(buffer, bufferConstrained);
    char solverString[16] = "PARDISO";
  #endif

  #ifdef PCG
    int info = jacobiPreconditionedCGSolver->SolveLinearSystemWithJacobiPreconditioner(buffer, bufferConstrained, 1e-6, 10000);
    if (info > 0)
      info = 0;
    char solverString[16] = "PCG";
  #endif

  if (info != 0)
  {
    printf("Error: %s sparse solver returned non-zero exit status %d.\n", solverString, (int)info);
    return 1;
  }
  
  ConstrainedDOFs::InsertDOFs(r, buffer, qaccel, numConstrainedDOFs, constrainedDOFs);

  return 0;
}
 
int ImplicitBatheSparse::DoTimestep()
{
  int numIter = 0;

  double error0 = 0; // error after the first step
  double errorQuotient;

  // store current amplitudes and set initial guesses for qaccel, qvel
  for(int i=0; i<r; i++)
  {
    q_1[i] = q[i]; 
    qvel_1[i] = qvel[i];
    qaccel_1[i] = qaccel[i];
  }

  double h2 = timestep * timestep;
  double invh = 1.0/timestep;
  //first half step: 
  //(16M + 4hC + h^2K)dq = h^2(f_ext - f_int) + M (16(q_1 - q) + 8h qdot_1 + h^2 qddot_1)
  //                       +C(4h(q_1-q) + h^2 qdot_1)
  do
  {
    PerformanceCounter counterForceAssemblyTime;
    forceModel->GetForceAndMatrix(q, internalForces, tangentStiffnessMatrix);
    counterForceAssemblyTime.StopCounter();
    forceAssemblyTime = counterForceAssemblyTime.GetElapsedTime();

    // scale internal forces
    for(int i=0; i<r; i++)
      internalForces[i] *= internalForceScalingFactor;

    *tangentStiffnessMatrix *= internalForceScalingFactor;

    tangentStiffnessMatrix->ScalarMultiply(dampingStiffnessCoef, rayleighDampingMatrix);
    rayleighDampingMatrix->AddSubMatrix(dampingMassCoef, *massMatrix);
    rayleighDampingMatrix->AddSubMatrix(1.0, *dampingMatrix);

    for(int i = 0; i < r; i++)
    {
      qdelta[i] = q_1[i] - q[i];
      qresidual[i] = h2 * (externalForces[i] - internalForces[i]);
      buffer[i] = timestep * (8 * qvel_1[i] + qaccel_1[i] * timestep); 
    }

    massMatrix->MultiplyVectorAdd(buffer, qresidual);
    for(int i = 0; i < r; i++)
      buffer[i] = qvel_1[i] * h2;
    rayleighDampingMatrix->MultiplyVectorAdd(buffer, qresidual);

    // 16 M + 4hC
    *rayleighDampingMatrix *= 4 * timestep;
    rayleighDampingMatrix->AddSubMatrix(16.0, *massMatrix);
    rayleighDampingMatrix->MultiplyVectorAdd(qdelta, qresidual);

    double error = 0;
    for(int i=0; i<r; i++)
      error += qresidual[i] * qresidual[i];

    // on the first iteration, compute initial error
    if (numIter == 0) 
    {
      error0 = error;
      errorQuotient = 1.0;
    }
    else
    {
      // error divided by the initial error, before performing this iteration
      errorQuotient = error / error0; 
    }

    if (errorQuotient < epsilon * epsilon)
    {
      break;
    }

    *tangentStiffnessMatrix *= h2;
    rayleighDampingMatrix->ScalarMultiplyAdd(1.0, tangentStiffnessMatrix);

    ConstrainedDOFs::RemoveDOFs(r, bufferConstrained, qresidual, numConstrainedDOFs, constrainedDOFs);
    systemMatrix->AssignSuperMatrix(*tangentStiffnessMatrix);

    // solve: systemMatrix * buffer = bufferConstrained

    PerformanceCounter counterSystemSolveTime;
    memset(buffer, 0, sizeof(double) * r);

    #ifdef SPOOLES
      SPOOLESSolver solver(systemMatrix);
      int info = solver.SolveLinearSystem(buffer, bufferConstrained);
      char solverString[16] = "SPOOLES";
    #endif

    #ifdef PARDISO
      int info = pardisoSolver->FactorMatrix(systemMatrix);
      if (info == 0)
        info = pardisoSolver->SolveLinearSystem(buffer, bufferConstrained);
      char solverString[16] = "PARDISO";
    #endif

    #ifdef PCG
      int info = jacobiPreconditionedCGSolver->SolveLinearSystemWithJacobiPreconditioner(buffer, bufferConstrained, 1e-6, 10000);
      if (info > 0)
        info = 0;
      char solverString[16] = "PCG";
    #endif

    if (info != 0)
    {
      printf("Error: %s sparse solver returned non-zero exit status %d.\n", solverString, (int)info);
      return 1;
    }

    counterSystemSolveTime.StopCounter();
    systemSolveTime = counterSystemSolveTime.GetElapsedTime();

    ConstrainedDOFs::InsertDOFs(r, buffer, qdelta, numConstrainedDOFs, constrainedDOFs);

    // update state
    for(int i=0; i<r; i++)
    {
      q[i] += qdelta[i];
      qvel[i] = 4 * (q[i] - q_1[i]) * invh -qvel_1[i];
      qaccel[i] = 4 * (qvel[i] - qvel_1[i]) * invh - qaccel_1[i];
    }

    for(int i=0; i<numConstrainedDOFs; i++)
      q[constrainedDOFs[i]] = qvel[constrainedDOFs[i]] = qaccel[constrainedDOFs[i]] = 0.0;

    numIter++;
  }
  while (numIter < maxIterations);

  // store current amplitudes at the middle timestep;
  for(int i=0; i<r; i++)
  {
    q_2[i] = q[i]; 
    qvel_2[i] = qvel[i];
    qaccel_2[i] = qaccel[i];
  }

  //second half step: 
  //(9M + 3hC + h^2K)dq = h^2(f_ext - f_int) + M (-9q + 12q_2 - 3q_1 + 4hqdot_2 - hqdot_1)
  //                       +Ch(-3q + 4q_2 -q_1)
  do
  {
    PerformanceCounter counterForceAssemblyTime;
    forceModel->GetForceAndMatrix(q, internalForces, tangentStiffnessMatrix);
    counterForceAssemblyTime.StopCounter();
    forceAssemblyTime += counterForceAssemblyTime.GetElapsedTime();

    // scale internal forces
    for(int i=0; i<r; i++)
      internalForces[i] *= internalForceScalingFactor;

    *tangentStiffnessMatrix *= internalForceScalingFactor;

    tangentStiffnessMatrix->ScalarMultiply(dampingStiffnessCoef, rayleighDampingMatrix);
    rayleighDampingMatrix->AddSubMatrix(dampingMassCoef, *massMatrix);
    rayleighDampingMatrix->AddSubMatrix(1.0, *dampingMatrix);

    *rayleighDampingMatrix *= timestep;
    for(int i = 0; i < r; i++)
    {
      qdelta[i] = 4 * q_2[i] - 3 * q[i] - q_1[i];
      buffer[i] = 3 * qdelta[i] + 4 * timestep * qvel_2[i] - timestep * qvel_1[i];
      qresidual[i] = h2 * (externalForces[i] - internalForces[i]);
    }

    massMatrix->MultiplyVectorAdd(buffer, qresidual);
    rayleighDampingMatrix->MultiplyVectorAdd(qdelta, qresidual);

    double error = 0;
    for(int i=0; i<r; i++)
      error += qresidual[i] * qresidual[i];

    // on the first iteration, compute initial error
    if (numIter == 0) 
    {
      error0 = error;
      errorQuotient = 1.0;
    }
    else
    {
      // error divided by the initial error, before performing this iteration
      errorQuotient = error / error0; 
    }

    if (errorQuotient < epsilon * epsilon)
    {
      break;
    }

    // 9 M + 3hC
    *rayleighDampingMatrix *= 3;
    rayleighDampingMatrix->AddSubMatrix(9.0, *massMatrix);

    *tangentStiffnessMatrix *= h2;
    rayleighDampingMatrix->ScalarMultiplyAdd(1.0, tangentStiffnessMatrix);

    ConstrainedDOFs::RemoveDOFs(r, bufferConstrained, qresidual, numConstrainedDOFs, constrainedDOFs);
    systemMatrix->AssignSuperMatrix(*tangentStiffnessMatrix);

    // solve: systemMatrix * buffer = bufferConstrained

    PerformanceCounter counterSystemSolveTime;
    memset(buffer, 0, sizeof(double) * r);

    #ifdef SPOOLES
      SPOOLESSolver solver(systemMatrix);
      int info = solver.SolveLinearSystem(buffer, bufferConstrained);
      char solverString[16] = "SPOOLES";
    #endif

    #ifdef PARDISO
      int info = pardisoSolver->FactorMatrix(systemMatrix);
      if (info == 0)
        info = pardisoSolver->SolveLinearSystem(buffer, bufferConstrained);
      char solverString[16] = "PARDISO";
    #endif

    #ifdef PCG
      int info = jacobiPreconditionedCGSolver->SolveLinearSystemWithJacobiPreconditioner(buffer, bufferConstrained, 1e-6, 10000);
      if (info > 0)
        info = 0;
      char solverString[16] = "PCG";
    #endif

    if (info != 0)
    {
      printf("Error: %s sparse solver returned non-zero exit status %d.\n", solverString, (int)info);
      return 1;
    }

    counterSystemSolveTime.StopCounter();
    systemSolveTime += counterSystemSolveTime.GetElapsedTime();

    ConstrainedDOFs::InsertDOFs(r, buffer, qdelta, numConstrainedDOFs, constrainedDOFs);

    // update state
    for(int i=0; i<r; i++)
    {
      q[i] += qdelta[i];
      qvel[i] = invh * (q_1[i] - 4 * q_2[i] + 3 * q[i]);
      qaccel[i] = invh * (qvel_1[i] - 4 * qvel_2[i] + 3 * qvel[i]);
    }

    for(int i=0; i<numConstrainedDOFs; i++)
      q[constrainedDOFs[i]] = qvel[constrainedDOFs[i]] = qaccel[constrainedDOFs[i]] = 0.0;

    numIter++;
  }
  while (numIter < maxIterations);

  return 0;
}


