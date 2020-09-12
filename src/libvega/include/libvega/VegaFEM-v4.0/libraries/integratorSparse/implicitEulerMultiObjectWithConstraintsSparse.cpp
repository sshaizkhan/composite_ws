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

  Dynamics:
  M u'' + D(u) u' + f_int(u) = f_ext
  subject to:
  C u = b
 
  systemMatrix =
  [ M + h D(u_k) + h^2 K(u_k)       h C^T ]
  [                       h C       B     ]

  x =
  [ delta v ]
  [ lambda  ]

  rhs =
  [ h (f_ext - f_int(u_k) - (h K(u_k) + D(u_k)) v_k) ]
  [ b - C (u_k + h v_k)                                ]

  Solve:
  systemMatrix * x = rhs

  v_{k+1} = v_k + delta v
  u_{k+1} = u_k + h v_{k+1}

  This class uses PARDISO as the solver.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "matrixIO.h"
#include "performanceCounter.h"
#include "constrainedDOFs.h"
#include "implicitEulerMultiObjectWithConstraintsSparse.h"
#include "sparseMatrix.h"
#include <cassert>
#include <iostream>
using namespace std;

ImplicitEulerMultiObjectWithConstraintsSparse::ImplicitEulerMultiObjectWithConstraintsSparse(int numObjects, const int *r, double timestep, 
    SparseMatrix ** massMatrix_,  ForceModel ** forceModel_, const int* numConstrainedDOFs_, int ** constrainedDOFs_, 
    const double * dampingMassCoef, const double * dampingStiffnessCoef, int maxIterations_, double epsilon_, int numSolverThreads_): 
    IntegratorBaseMultiObjectSparse(numObjects, r, timestep, massMatrix_, forceModel_, numConstrainedDOFs_, constrainedDOFs_, dampingMassCoef, 
    dampingStiffnessCoef), maxIterations(maxIterations_), epsilon(epsilon_), numSolverThreads(numSolverThreads_), systemMatrix(NULL), 
    globalTangentStiffnessMatrixOffset(NULL), lagrangeMultiplierSolver(NULL), C(NULL), Ceff(NULL), B(NULL), Beff(NULL)
{
  useStaticSolver = false;

  tangentStiffnessMatrix.resize(numObjects, NULL);
  rayleighDampingMatrix.resize(numObjects, NULL);

  // build matrix topologies
  bool error = false;

  for(int objectID=0; objectID<numObjects; objectID++)
  {
    forceModel[objectID]->GetTangentStiffnessMatrixTopology(&tangentStiffnessMatrix[objectID]);

    if (tangentStiffnessMatrix[objectID]->Getn() != massMatrix[objectID]->Getn())
    {
      printf("Error: the provided mass matrix of object %d does not have correct size. Mass matrix: %d x %d. Stiffness matrix: %d x %d.\n", objectID, massMatrix[objectID]->Getn(), massMatrix[objectID]->Getn(), tangentStiffnessMatrix[objectID]->Getn(), tangentStiffnessMatrix[objectID]->Getn());
      error = true;
      break;
    }

    rayleighDampingMatrix[objectID] = new SparseMatrix(*tangentStiffnessMatrix[objectID]);
    rayleighDampingMatrix[objectID]->BuildSubMatrixIndices(*massMatrix[objectID]);
    tangentStiffnessMatrix[objectID]->BuildSubMatrixIndices(*massMatrix[objectID]);
    if (dampingMatrix[objectID])
      tangentStiffnessMatrix[objectID]->BuildSubMatrixIndices(*dampingMatrix[objectID], 1);

    if (tangentStiffnessMatrix[objectID]->GetNumRows() != massMatrix[objectID]->GetNumRows())
    {
      printf("Error: mass matrix and stiffness matrix of object %d don't have the same dimensions.\n", objectID);
      error = true;
      break;
    }
  }

  if (error)
  {
    for(int objectID=0; objectID<numObjects; objectID++)
      delete(tangentStiffnessMatrix[objectID]);

    for(int objectID=0; objectID<numObjects; objectID++)
      delete(rayleighDampingMatrix[objectID]);
    throw 1;
  }

  BuildSystemMatrixTopology();

  // allocate the memory (for the case C=NULL)
  int rTotal = Getr();
  rhs.resize(rTotal);

  numConstrainedDOFsTotal = 0;
  for(int objectID=0; objectID<numObjects; objectID++)
    numConstrainedDOFsTotal += numConstrainedDOFs[objectID];

  constrainedDOFsTotal.resize(numConstrainedDOFsTotal);
  numConstrainedDOFsTotal = 0;
  for(int objectID=0; objectID<numObjects; objectID++)
  {
    for (int i=0; i<numConstrainedDOFs[objectID]; i++)
      constrainedDOFsTotal[numConstrainedDOFsTotal+i] = constrainedDOFs[objectID][i] + rStart[objectID];
    numConstrainedDOFsTotal += numConstrainedDOFs[objectID];
  }
  qBoundaryTotal.resize(numConstrainedDOFsTotal, 0.0);

  // note: C = Ceff = B = Beff = NULL at this point
  RebuildSolver();
  initialGuessFactor = 0;
}

ImplicitEulerMultiObjectWithConstraintsSparse::ImplicitEulerMultiObjectWithConstraintsSparse(int numObjects, const int *r, double timestep, 
    SparseMatrix ** massMatrix_,  ForceModel ** forceModel_, const int* numConstrainedDOFs_, int ** constrainedDOFs_, 
    double dampingMassCoef, double dampingStiffnessCoef, int maxIterations_, double epsilon_, int numSolverThreads_): 
ImplicitEulerMultiObjectWithConstraintsSparse(numObjects, r, timestep, massMatrix_, forceModel_, numConstrainedDOFs_, constrainedDOFs_,
  std::vector<double>(numObjects, dampingMassCoef).data(), std::vector<double>(numObjects, dampingStiffnessCoef).data(),
  maxIterations_, epsilon_, numSolverThreads_) {}

ImplicitEulerMultiObjectWithConstraintsSparse::~ImplicitEulerMultiObjectWithConstraintsSparse()
{
  for(int objectID=0; objectID<numObjects; objectID++)
    delete(tangentStiffnessMatrix[objectID]);

  for(int objectID=0; objectID<numObjects; objectID++)
    delete(rayleighDampingMatrix[objectID]);

  delete(systemMatrix);
  delete(globalTangentStiffnessMatrixOffset);
  delete(lagrangeMultiplierSolver);
  delete(C);
  delete(Ceff);
  delete(B);
  delete(Beff);
}

void ImplicitEulerMultiObjectWithConstraintsSparse::BuildSystemMatrixTopology()
{
  // build the topology of the system matrix
  int rTotal = Getr();
  SparseMatrixOutline systemMatrixOutline(rTotal);

  for(int objectID=0; objectID<numObjects; objectID++)
    systemMatrixOutline.AddBlockMatrix(rStart[objectID], rStart[objectID], tangentStiffnessMatrix[objectID]);

  if (globalTangentStiffnessMatrixOffset != NULL)
  {
    // add globalTangentStiffnessMatrixOffset to the topology
    systemMatrixOutline.AddBlockMatrix(0, 0, globalTangentStiffnessMatrixOffset);
  }

  delete(systemMatrix);
  systemMatrix = new SparseMatrix(&systemMatrixOutline);

  // build indices for rapid adding of local stiffness matrices to the global system matrix
  for(int objectID=0; objectID<numObjects; objectID++)
    systemMatrix->BuildSubMatrixIndices(*(tangentStiffnessMatrix[objectID]), objectID, rStart[objectID], rStart[objectID]);

  // build indices for rapid adding of a particular globalTangentStiffnessMatrixOffset to the systemMatrix
  if (globalTangentStiffnessMatrixOffset != NULL)
    systemMatrix->BuildSubMatrixIndices(*globalTangentStiffnessMatrixOffset, numObjects);
}

void ImplicitEulerMultiObjectWithConstraintsSparse::SetDampingMatrix(int objectID, const SparseMatrix * dampingMatrix_, int reuseTopology)
{
  if (dampingMatrix[objectID] == NULL)
    reuseTopology = false;
  IntegratorBaseMultiObjectSparse::SetDampingMatrix(objectID, dampingMatrix_, reuseTopology);
  if (reuseTopology == false)
    tangentStiffnessMatrix[objectID]->BuildSubMatrixIndices(*dampingMatrix[objectID], 1);
}

// sets the state based on given q, qvel
int ImplicitEulerMultiObjectWithConstraintsSparse::SetState(double * q_, double * qvel_)
{
  int rTotal = Getr();
  memcpy(q, q_, sizeof(double) * rTotal);

  if (qvel_ != NULL)
    memcpy(qvel, qvel_, sizeof(double) * rTotal);
  else
    memset(qvel, 0, sizeof(double) * rTotal);

  // We don't do this (set fixed DOF to zero) now because we now support Dirichlet boundary conditions
//  for(int i=0; i<numConstrainedDOFsTotal; i++)
//    q[constrainedDOFsTotal[i]] = qvel[constrainedDOFsTotal[i]] = 0.0;
  return 0;
}

// sets the state for objectID based on given q, qvel
int ImplicitEulerMultiObjectWithConstraintsSparse::SetState(int objectID, const double * q_, const double * qvel_)
{
  memcpy(&q[rStart[objectID]], q_, sizeof(double) * rArray[objectID]);

  if (qvel_ != NULL)
    memcpy(&qvel[rStart[objectID]], qvel_, sizeof(double) * rArray[objectID]);
  else
    memset(&qvel[rStart[objectID]], 0, sizeof(double) * rArray[objectID]);

  // We don't do this (set fixed DOF to zeri) now because we now support Dirichlet boundary conditions
//  for(int i=0; i<numConstrainedDOFs[objectID]; i++)
//    q[rStart[objectID] + constrainedDOFs[objectID][i]] = qvel[rStart[objectID] + constrainedDOFs[objectID][i]] = 0.0;
 
  return 0;
}

int ImplicitEulerMultiObjectWithConstraintsSparse::BuildUnconstrainedSystemMatrixAndRhs()
{
  int rTotal = Getr();
  // overall equation:
  // Implicit Euler update rule: v_{n+1} = v_n + h a_{n+1}, u_{n+1} = u_n + h v_{n+1}
  // M a = f_all
  // M dv_{n+1} = h f_all(u_{n+1}, v_{n+1})
  // M dv_{n+1} = h (f_ext(u_{n+1}) - f_damp(u*, v*) - f_int(u*) - D(u*,v*)(v_{n+1} - v*) - K(u*)(u_{n+1} - u*) )
  // M dv_{n+1} = h (f_ext(u_{n+1}) - f_damp(u*, v*) - f_int(u*) - D(u*,v*)(v_n + dv_{n+1} - v*) - K(u*)[u_n + h (v_n + dv_{n+1}) - u*] )
  // [M + h^2 K(u*) + h D(u*,v*)] dv_{n+1} = -h[ K(u*) (u_n + h v_n - u*) + D(u*,v*) (v_n - v*) + f_damp(u*,v*) + f_int(u*) - f_ext(u_{n+1})]
  // define dv* = v_{n+1} - v* = v_n + dv_{n+1} - v*,
  // [M + h^2 K(u*) + h D(u*,v*)] dv* = -h[K(u*) (u_n + h v_n - u*) + D(u*,v*) (v_n - v*) + f_damp(u*,v*) + f_int(u*) - f_ext(u_{n+1})] - [M + h^2 K(u*) + h D(u*,v*)](v*-v_n)
  // [M + h^2 K(u*) + h D(u*,v*)] dv* = -h[K(u*)(u_n - u* + hv*) + f_damp(u*,v*) + f_int(u*) - f_ext(u_{n+1}) ] + M(v_n - v*)

  // IF f_damp has stiffness on u:
  // [M + h^2 KD(u*) + h D(u*,v*)] dv* = -h[KD(u*)(u_n - u* + hv*) + f_damp(u*,v*) + f_int(u*) - f_ext(u_{n+1}) ] + M(v_n - v*)


  // assmue f_damp(u,v) = D(u) * v,
  // [M + h (h K(u*) + D(u*))] dv* = -h [(h K(u*) + D(u*))v* + K(u*)(u_n - u*) + f_int(u*) - f_ext(u_{n+1})] + M(v_n - v*)    ... (1)

  // v_{n+1} = v* + dv*
  // u_{n+1} = u_n + h * v_{n+1}
  // In the code, q stores u*, q_1 stores u_n, qvel stores v*, qvel_1 stores v_n

  // Note that here if we use dv* = v_{n+1} - v* into equation (1):
  // [M + h (h K(u*) + D(u*))] v_{n+1} = -h [K(u*)(u_n - u*) + f_int(u*) - f_ext(u_{n+1})] + M v_n
  // So here actually v* has no effect on final result
  memset(&rhs[0], 0, sizeof(double) * rTotal);
  systemMatrix->ResetToZero();

  double forceAssemblyTime_t = 0.0;
  for (int objectID=0; objectID<numObjects; objectID++)
  {
    int rS = rStart[objectID];
    PerformanceCounter counterForceAssemblyTime;
    forceModel[objectID]->GetForceAndMatrix(&q[rS], &internalForces[rS], tangentStiffnessMatrix[objectID]);

    counterForceAssemblyTime.StopCounter();
    forceAssemblyTime_t += counterForceAssemblyTime.GetElapsedTime();

    // scale internalForces and tangentStiffnessMatrix
    for(int i=rS; i<rArray[objectID] + rS; i++)
      internalForces[i] *= internalForceScalingFactor;
    *tangentStiffnessMatrix[objectID] *= internalForceScalingFactor;

    if (useStaticSolver)
    {
      // fint + K * qdelta = fext
      // unimplemented
    }

    // build effective stiffness: 
    // Keff = M + h D + h^2 * K
    // compute force residual, store it into aux variable qresidual
    // qresidual = h * (-D qdot - fint + fext - h * K * qdot)) // this is semi-implicit Euler
    // qresidual = M (qvel_1 - qvel) + h * (-D qdot - fint + fext - K * (q_1 - q + h qdot) )) // for fully implicit Euler

    // build Rayleigh damping matrix = dampingMassCoef * massMatrix + dampingStiffnessCoef * tangentStiffnessMatrix
    tangentStiffnessMatrix[objectID]->ScalarMultiply(dampingStiffnessCoef[objectID], rayleighDampingMatrix[objectID]);
    rayleighDampingMatrix[objectID]->AddSubMatrix(dampingMassCoef[objectID], *massMatrix[objectID]);

    if (tangentStiffnessMatrixOffset[objectID] != NULL)
      tangentStiffnessMatrix[objectID]->AddSubMatrix(1.0, *(tangentStiffnessMatrixOffset[objectID]), 2);

    // set rhs = K(u*) (u_n - u*)
    // if (initalGuessFactor > 0 || numIter != 0)
    if (equal(q_1+rS, q_1 + rS + rArray[objectID], q+rS) == false)
    {
      for(int i = rS; i < rS+rArray[objectID]; i++)
        buffer[i] = q_1[i] - q[i];
      tangentStiffnessMatrix[objectID]->MultiplyVector(&buffer[rS], &rhs[rS]);
    }

    // build effective stiffness: add mass matrix and damping matrix to tangentStiffnessMatrix

    *tangentStiffnessMatrix[objectID] *= timestep;

    *tangentStiffnessMatrix[objectID] += *rayleighDampingMatrix[objectID];
    if (dampingMatrix[objectID])
      tangentStiffnessMatrix[objectID]->AddSubMatrix(1.0, *dampingMatrix[objectID], 1);
    // now tangentStiffnessMatrix = h K(u*) + D(u*) = h K(u*) + D_rayleigh(u*) + D_additional

    tangentStiffnessMatrix[objectID]->MultiplyVectorAdd(&qvel[rS], &rhs[rS]);
    // now rhs = (h K(u*) + D(u*)) v* + K(u*) (u_n - u*)

    *tangentStiffnessMatrix[objectID] *= timestep;
    tangentStiffnessMatrix[objectID]->AddSubMatrix(1.0, *massMatrix[objectID]);
    // now tangentStiffnessMatrix = M + h^2 K(u*) + h D(u*)

    // assign it to the systemMatrix
    systemMatrix->AssignSubMatrix(*(tangentStiffnessMatrix[objectID]), objectID);
  }

  // assign any global tangent stiffness matrix offset
  if (globalTangentStiffnessMatrixOffset != NULL)
  {
    // if (initalGuessFactor > 0 || numIter != 0)
    if (equal(q_1, q_1 + rTotal, q) == false)
    {
      for(int i = 0; i < rTotal; i++)
        buffer[i] = q_1[i] - q[i];
      globalTangentStiffnessMatrixOffset->MultiplyVectorAdd(buffer, &rhs[0]);
    }

    globalTangentStiffnessMatrixOffset->MultiplyVector(qvel, buffer);
    for(int i = 0; i < rTotal; i++)
      rhs[i] += timestep * buffer[i];

    systemMatrix->AddSubMatrix(timestep * timestep, *globalTangentStiffnessMatrixOffset, numObjects);
  }

  forceAssemblyTime += forceAssemblyTime_t;

  for(int i=0; i<rTotal; i++)
  {
    rhs[i] += internalForces[i] - externalForces[i];
    rhs[i] *= -timestep;
  }
  // now rhs =  -h [(h K(u*) + D(u*))v* + K(u*)(u_n - u*) + f_int(u*) - f_ext]

  //  if (numIter != 0)
  if(equal(qvel_1, qvel_1 + rTotal, qvel) == false) 
  {
    for(int i=0; i<rTotal; i++)
      buffer[i] = qvel_1[i] - qvel[i];
    for(int objectID = 0; objectID < numObjects; objectID++)
      massMatrix[objectID]->MultiplyVectorAdd(&buffer[rStart[objectID]], &rhs[rStart[objectID]]);
  }
  // finally rhs = -h [(h K(u*) + D(u*))v* + K(u*)(u_n - u*) + f_int(u*) - f_ext(u*)] + M(v_n - v*)
  return 0;
}

// constraints satisfy g(u) = 0,
// To ensure g_{n+1}(u_{n+1}) = 0, we add a fictitious force (dgdu)^T lambda
// because v_{n+1} = v* + dv*,  u_{n+1} = u_n + h * v_{n+1}
// we have g_{n+1}(u_n + h v* + h dv*) = 0
// approximately, g_{n+1}(u_n + h v*) + dgdu_{n+1}(u_n + h v*) h dv* = 0
// dgdu_{n+1}(u_n + h v*) h dv* = - g_{n+1}(u_n + h v*)

// Here, g(u) = C u - constraintedRhs = 0, dgdu = C
// So C is defined to be: C_{n+1} u_{n+1} = constrainedRhs_{n+1}
// Now the unknowns in the system is dv* = v_{n+1} - v*
// So the constrained equations in the system is: C (h dv* + h v* + u_n) = constrainedRhs
//                                                hC dv* = constrainedRhs - (h v* + u_n)
int ImplicitEulerMultiObjectWithConstraintsSparse::AddConstraintMatrixAndRhs()
{
  int rTotal = Getr();
  memcpy(&rhs[rTotal], &constraintRhs[0], sizeof(double) * C->Getn());

  for(int i=0; i<rTotal; i++)
    buffer[i] = -1.0 * (timestep * qvel[i] + q_1[i]);

  C->MultiplyVectorAdd(buffer, &rhs[rTotal]);

  assert(Ceff);
  *Ceff = *C;
  *Ceff *= timestep; // not stricly needed, but may be useful in case the matrix re-ordering is computed based on values, as opposed to just topology

  if (B != NULL)
  {
    *Beff = *B;
    *Beff *= timestep;
  }

  // improve robustness by scaling the constraint rows to match the size of entries in the main block
  double maxEntry = systemMatrix->GetMaxAbsEntry();
  double CeffMaxEntry = Ceff->GetMaxAbsEntry();
  double scale = maxEntry / CeffMaxEntry;
  *Ceff *= scale;

  if (Beff != NULL)
    *Beff *= scale;

  for(int i=0; i<Ceff->Getn(); i++)
    rhs[rTotal+i] *= scale;
  return 0;
}

int ImplicitEulerMultiObjectWithConstraintsSparse::DoTimestep()
{
  int rTotal = Getr();

  // store current amplitudes and set initial guesses for qaccel, qvel
  for(int i=0; i<rTotal; i++)
  {
    q_1[i] = q[i]; 
    qvel_1[i] = qvel[i];
    qaccel_1[i] = qaccel[i] = 0;
   
    //q[i] = q_1[i] + qvel_1[i] * timestep;
  }

  // initialGuessFactor: 0: u* = u_n, 1: u* = u_n + h v_n
  if (initialGuessFactor > 0)
  {
    for(int i = 0; i < rTotal; i++)
      q[i] += initialGuessFactor * timestep * qvel_1[i];

//    int rLocal = 0;
//    for(int objectID=0; objectID<numObjects; objectID++)
//    {
//      for(int i=0; i<numConstrainedDOFs[objectID]; i++)
//      {
//        int ind = rLocal+constrainedDOFs[objectID][i];
//        q[ind] = q_1[ind];
//        qvel[ind] = qvel_1[ind];
//      }
//      rLocal += rArray[objectID];
//    }
  }

  systemSolveTime = 0.0;
  forceAssemblyTime = 0.0;

  int numIter = 0;
  double error0 = 0.0;
  do
  {
    // build system matrix and rhs
    BuildUnconstrainedSystemMatrixAndRhs();
    if (C != NULL)
      AddConstraintMatrixAndRhs();
    
    if (maxIterations > 1)
    {
      double errorQuotient = 0.0;
      ConstrainedDOFs::RemoveDOFs(rTotal, buffer, rhs.data(), numConstrainedDOFsTotal, constrainedDOFsTotal.data());
      double error = 0.0;
      for(int i=0; i<rTotal - numConstrainedDOFsTotal; i++)
        error += buffer[i] * buffer[i];

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
        break;
    }

    lagrangeMultiplierSolver->UpdateAJB(systemMatrix, Ceff, Beff); // this also re-factors the matrix
    
    // In case of Dirichlet boundary conditions, boundary values are added in buffer
    for(int i = 0; i < numConstrainedDOFsTotal; i++)
    {
      int cind = constrainedDOFsTotal[i];
      buffer[cind] = (qBoundaryTotal[i] - q_1[cind]) / timestep - qvel[cind];
    }
//    memset(buffer, 0, sizeof(double) * r);
    PerformanceCounter counterSystemSolveTime;
    // solve dv* and stores in buffer
    int info = lagrangeMultiplierSolver->SolveLinearSystem(buffer, &rhs[0]);
    counterSystemSolveTime.StopCounter();
    systemSolveTime += counterSystemSolveTime.GetElapsedTime();

    if (info != 0)
    {
      printf("Error: lagrangeMultiplier solver returned non-zero exit status %d.\n", (int)info);
      return 1;
    }
  
    // update state
    for(int i=0; i<rTotal; i++)
    {
      qvel[i] += buffer[i];
      q[i] = q_1[i] + timestep * qvel[i];
    }

    // now we support Dirichlet boundary condition, we no longer set constrainedDOFs to be 0.0
//    for(int i=0; i<numConstrainedDOFsTotal; i++)
//      q[constrainedDOFsTotal[i]] = qvel[constrainedDOFsTotal[i]] = qaccel[constrainedDOFsTotal[i]] = 0.0;

    numIter++;
  }
  while(numIter < maxIterations);

  return 0;
}

void ImplicitEulerMultiObjectWithConstraintsSparse::UseStaticSolver(bool useStaticSolver_)
{ 
  useStaticSolver = useStaticSolver_;

  if (!useStaticSolver) 
  {
    memset(qvel, 0, sizeof(double) * r);
    memset(qaccel, 0, sizeof(double) * r);
    memset(qvel_1, 0, sizeof(double) * r); 
    memset(qaccel_1, 0, sizeof(double) * r);
    memcpy(q_1, q, sizeof(double) * r);
  }
} 

void ImplicitEulerMultiObjectWithConstraintsSparse::SetConstraintMatrix(const SparseMatrix * C_, const SparseMatrix * B_)
{
  if (((C_ == NULL) && (C == NULL)) || ((C_ != NULL) && (C != NULL) && (*C_ == *C)))
    return;

  delete(C);
  delete(Ceff);

  if (C_ == NULL)
  {
    C = NULL;
    Ceff = NULL;
  }
  else
  {
    C = new SparseMatrix(*C_);
    Ceff = new SparseMatrix(*C);

    int rTotal = Getr();
    int numConstraints = C->Getn();

    free(buffer);
    buffer = (double*) malloc (sizeof(double) * (numConstraints + rTotal));
    rhs.resize(numConstraints + rTotal);
    constraintRhs.resize(numConstraints);
  }

  // set lower-right block
  if ((B == NULL) && (B_ == NULL))
  {
    // do nothing
  }
  else
  {
    delete(B);
    delete(Beff);

    if (B_ == NULL)
      B = NULL;
    else
      B = new SparseMatrix(*B_);

    if (B == NULL)
      Beff = NULL;
    else
      Beff = new SparseMatrix(*B);
  }

  RebuildSolver(); // note: this will form the system matrix and permute it, but it will not factor it
}

void ImplicitEulerMultiObjectWithConstraintsSparse::SetConstraintRhs(const double * constraintRhs_)
{
  if (C == NULL)
    return;

  memcpy(constraintRhs.data(), constraintRhs_, sizeof(double) * C->Getn());
}

void ImplicitEulerMultiObjectWithConstraintsSparse::ClearConstraints() 
{
  SetConstraintMatrix(NULL);
  SetConstraintRhs(NULL);
  //we don't call RebuildSolver() here because SetConstraintMatrix() will do it
}

void ImplicitEulerMultiObjectWithConstraintsSparse::SetTangentStiffnessMatrixOffset(int objectID, const SparseMatrix * tangentStiffnessMatrixOffset_, int reuseTopology)
{
  if (tangentStiffnessMatrixOffset[objectID] == NULL)
    reuseTopology = 0;

  IntegratorBaseMultiObjectSparse::SetTangentStiffnessMatrixOffset(objectID, tangentStiffnessMatrixOffset_, reuseTopology);

  if (!reuseTopology)
    tangentStiffnessMatrix[objectID]->BuildSubMatrixIndices(*(tangentStiffnessMatrixOffset[objectID]), 2);
}

void ImplicitEulerMultiObjectWithConstraintsSparse::RebuildSolver()
{
  int updatable = 1;
  int addDirichlet = numConstrainedDOFsTotal ? 1: 0;
  delete(lagrangeMultiplierSolver);
  lagrangeMultiplierSolver = new LagrangeMultiplierSolver(systemMatrix, Ceff, Beff, numConstrainedDOFsTotal, constrainedDOFsTotal.data(), numSolverThreads, updatable, addDirichlet);
}

// add an offset to the **global** multi-object tangent stiffness matrix (default: no offset)
void ImplicitEulerMultiObjectWithConstraintsSparse::InitTangentStiffnessMatrixOffset(const SparseMatrix * tangentStiffnessMatrixOffsetTopology) // adjusts the global stiffness matrix topology, by initializing the non-zero locations specified by "tangentStiffnessMatrixOffsetTopology"
{
  delete(globalTangentStiffnessMatrixOffset);
  globalTangentStiffnessMatrixOffset = NULL;
  if (tangentStiffnessMatrixOffsetTopology != NULL)
    globalTangentStiffnessMatrixOffset = new SparseMatrix(*tangentStiffnessMatrixOffsetTopology);

  BuildSystemMatrixTopology();
  RebuildSolver();
}

void ImplicitEulerMultiObjectWithConstraintsSparse::SetTangentStiffnessMatrixOffset(const SparseMatrix * tangentStiffnessMatrixOffset) // set the offset (must be previously initialized)
{
  *globalTangentStiffnessMatrixOffset = *tangentStiffnessMatrixOffset;
}

void ImplicitEulerMultiObjectWithConstraintsSparse::SetDirichletBoundaryCondition(const double * qBTotal)
{
  memcpy(qBoundaryTotal.data(), qBTotal, sizeof(double) * numConstrainedDOFsTotal);
}

void ImplicitEulerMultiObjectWithConstraintsSparse::SetDirichletBoundaryCondition(int objectID, const double * qBoundary)
{
  int qBoundaryStart = 0;
  for(int i = 0; i < objectID; i++)
    qBoundaryStart += numConstrainedDOFs[i];
  memcpy(&qBoundaryTotal[qBoundaryStart], qBoundary, sizeof(double) * numConstrainedDOFs[objectID]);
}

void ImplicitEulerMultiObjectWithConstraintsSparse::SetNumSolverThreads(int numSolverThreads_)
{
  numSolverThreads = numSolverThreads_;
  if (lagrangeMultiplierSolver != NULL)
    lagrangeMultiplierSolver->SetNumThreads(numSolverThreads);
}

