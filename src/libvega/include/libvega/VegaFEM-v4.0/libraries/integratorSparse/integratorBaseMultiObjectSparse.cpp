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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "integratorBaseMultiObjectSparse.h"

IntegratorBaseMultiObjectSparse::IntegratorBaseMultiObjectSparse(int numObjects_, const int * r, double timestep, SparseMatrix ** massMatrix_, ForceModel ** forceModel_, const int * numConstrainedDOFs_, int ** constrainedDOFs_, const double * dampingMassCoef_, const double * dampingStiffnessCoef_): IntegratorBase(ComputeTotalDOFs(numObjects_, r), timestep, 0.0, 0.0), numObjects(numObjects_)
{
  systemSolveTime = 0.0;
  forceAssemblyTime = 0.0;
  rArray.assign(r, r + numObjects);
  rStart.resize(numObjects);
  if (numObjects > 0)
  {
    rStart[0] = 0;
    for(int i = 1; i < numObjects; i++)
      rStart[i] = rStart[i-1] + rArray[i-1];
  }
  
  if (numConstrainedDOFs_)
  {
    numConstrainedDOFs.assign(numConstrainedDOFs_, numConstrainedDOFs_ + numObjects);
  }
  else 
  {
    numConstrainedDOFs.assign(numObjects, 0);
  }

  constrainedDOFs.assign(numObjects, std::vector<int>());
  for (int objectID = 0; objectID < numObjects; objectID++)
    if (constrainedDOFs_ && constrainedDOFs_[objectID])
      constrainedDOFs[objectID].assign(constrainedDOFs_[objectID], constrainedDOFs_[objectID] + numConstrainedDOFs[objectID]);

  massMatrix.assign(massMatrix_, massMatrix_ + numObjects);
  forceModel.assign(forceModel_, forceModel_ + numObjects);

  dampingMassCoef.assign(dampingMassCoef_, dampingMassCoef_ + numObjects);
  dampingStiffnessCoef.assign(dampingStiffnessCoef_, dampingStiffnessCoef_ + numObjects);

  dampingMatrix.resize(numObjects, NULL);
  tangentStiffnessMatrixOffset.resize(numObjects, NULL);
}

int IntegratorBaseMultiObjectSparse::ComputeTotalDOFs(int numObjects, const int * r)
{
  int rTotal = 0;
  for(int i=0; i<numObjects; i++)
    rTotal += r[i];
  return rTotal;
}

IntegratorBaseMultiObjectSparse::~IntegratorBaseMultiObjectSparse()
{
  for(int objectID=0; objectID<numObjects; objectID++)
  {
    delete dampingMatrix[objectID];
    delete(tangentStiffnessMatrixOffset[objectID]);
  }
}

void IntegratorBaseMultiObjectSparse::SetDampingMatrix(int objectID, const SparseMatrix * dampingMatrix_, int reuseTopology)
{
  if (reuseTopology && dampingMatrix[objectID] != NULL)
    *(dampingMatrix[objectID]) = *dampingMatrix_;
  else
  {
    delete(dampingMatrix[objectID]);
    dampingMatrix[objectID] = new SparseMatrix(*dampingMatrix_);
  }
}

double IntegratorBaseMultiObjectSparse::GetKineticEnergy(int objectID)
{
  return 0.5 * massMatrix[objectID]->QuadraticForm(&qvel[rStart[objectID]]);
}

double IntegratorBaseMultiObjectSparse::GetKineticEnergy()
{
  double totalEnergy = 0;
  
  for(int objectID=0; objectID<numObjects; objectID++)
    totalEnergy += GetKineticEnergy(objectID);
  return totalEnergy;
}

double IntegratorBaseMultiObjectSparse::GetTotalMass(int objectID)
{
  return massMatrix[objectID]->SumEntries();
}

double IntegratorBaseMultiObjectSparse::GetTotalMass()
{
  double totalMass = 0;
  
  for(int objectID=0; objectID<numObjects; objectID++)
  {
    totalMass += massMatrix[objectID]->SumEntries();
  }
  
  return totalMass;
}

void IntegratorBaseMultiObjectSparse::SetTangentStiffnessMatrixOffset(int objectID, const SparseMatrix * tangentStiffnessMatrixOffset_, int reuseTopology)
{
  if (reuseTopology && (tangentStiffnessMatrixOffset[objectID] != NULL))
    *(tangentStiffnessMatrixOffset[objectID]) = *tangentStiffnessMatrixOffset_;
  else
  {
    delete(tangentStiffnessMatrixOffset[objectID]);
    tangentStiffnessMatrixOffset[objectID] = new SparseMatrix(*tangentStiffnessMatrixOffset_);
  }
}

void IntegratorBaseMultiObjectSparse::AddTangentStiffnessMatrixOffset(int objectID, const SparseMatrix * tangentStiffnessMatrixOffset_)
{
  *(tangentStiffnessMatrixOffset[objectID]) += *tangentStiffnessMatrixOffset_;
}

void IntegratorBaseMultiObjectSparse::ClearTangentStiffnessMatrixOffset(int objectID)
{
  delete(tangentStiffnessMatrixOffset[objectID]);
  tangentStiffnessMatrixOffset[objectID] = NULL;
}


