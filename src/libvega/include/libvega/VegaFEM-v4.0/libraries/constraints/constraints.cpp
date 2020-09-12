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

#include "constraints.h"
#include "triangle.h"
#include "sparseMatrix.h"

using namespace std;

Constraints::Constraints(int numObjects_, int const * r_, double const * restPositions_) : numObjects(numObjects_), r(r_), restPositions(restPositions_), JOutline(NULL), C(NULL), b(NULL)
{
  skinnings = new Skinning*[numObjects];
  numSkinnedVertices = (int*)malloc(sizeof(int) * numObjects);
  skinnedVertices = (int**)malloc(sizeof(int*) * numObjects);
  skinnedVertexPositions = (double**)malloc(sizeof(double*) * numObjects);

  // build column starts (locations where object i begins in the constraint matrix)
  columnStarts = (int*) malloc (sizeof(int) * numObjects);
  int columnStart = 0;
  for(int objectID=0; objectID < numObjects; objectID++)
  {
    columnStarts[objectID] = columnStart;
    columnStart += r[objectID];
  }

  for(int i=0; i<numObjects; i++)
  {
    skinnings[i] = NULL;
    skinnedVertices[i] = NULL;
    skinnedVertexPositions[i] = NULL;
  }
}

Constraints::~Constraints()
{
  delete(JOutline);
  delete(C);
  free(b);
  free(columnStarts);

  delete [] skinnings;
  free(numSkinnedVertices);
  free(skinnedVertices);
  free(skinnedVertexPositions);
}

void Constraints::Reset()
{
  delete(JOutline);
  delete(C);
  free(b);

  JOutline = NULL;
  C = NULL;
  b = NULL;
}

void Constraints::AddFixedConstraint(int object, int vertex, double pos[3])
{
  if (JOutline == NULL) 
  {
    JOutline = new SparseMatrixOutline(3);
    b = (double*)malloc(sizeof(double) * 3);
  }
  else
  {
    JOutline->IncreaseNumRows(3);
    b = (double*)realloc(b, sizeof(double) * (JOutline->Getn()));
  }

  int columnStart = columnStarts[object] + 3 * vertex;
  int rowStart = JOutline->Getn() - 3;
  for(int i=0; i<3; i++)
    JOutline->AddEntry(rowStart+i, columnStart+i, 1.0);

  for(int i=0; i<3; i++)
    b[rowStart+i] = pos[i] - restPositions[columnStart+i];
}

void Constraints::AddLinearConstraint(int numInvolvedDOFs, int * objects, int * dofs, double * weights, double b_)
{
  if (JOutline == NULL) 
  {
    JOutline = new SparseMatrixOutline(1);
    b = (double*) malloc (sizeof(double) * 1);
  }
  else
  {
    JOutline->IncreaseNumRows(1);
    b = (double*) realloc (b, sizeof(double) * (JOutline->Getn()));
  }

  int row = JOutline->Getn() - 1;
  for(int i=0; i<numInvolvedDOFs; i++)
  {
    int object = objects[i];
    int dof = dofs[i];
    double weight = weights[i];
    int column = columnStarts[object] + dof;
    JOutline->AddEntry(row, column, weight);
  }

  b[row] = b_; 
}

void Constraints::AddFixedBarycentricConstraint(int object, int vertex[4], double barycentricWeights[4], double pos[3])
{
  int numInvolvedDOFs = 4;
  int objects[4] = { object, object, object, object };
  int dofs[4] = { 3 * vertex[0], 3 * vertex[1], 3 * vertex[2], 3 * vertex[3] };

  // compute the rest position of this point
  double restPosition[3];
  int columnStart = columnStarts[object];
  for(int i=0; i<3; i++)
  {
    restPosition[i] = 0.0;
    for(int j=0; j<4; j++)
      restPosition[i] += barycentricWeights[j] * restPositions[columnStart + 3 * vertex[j] + i];
  }

  // constrain x,y,z DOFs
  for(int i=0; i<3; i++)
  {
    double b = pos[i] - restPosition[i];
    AddLinearConstraint(numInvolvedDOFs, objects, dofs, barycentricWeights, b);
    for(int j=0; j<4; j++)
      dofs[j] += 1;
  }
}

void Constraints::AddPointBarycentricConstraint(int objectA, int vertexA[4], double barycentricWeightsA[4], int objectB, int vertexB[4], double barycentricWeightsB[4])
{
  int numInvolvedDOFs = 8;
  int objects[8] = { objectA, objectA, objectA, objectA, objectB, objectB, objectB, objectB };
  int dofs[8] = { 3 * vertexA[0], 3 * vertexA[1], 3 * vertexA[2], 3 * vertexA[3],
                  3 * vertexB[0], 3 * vertexB[1], 3 * vertexB[2], 3 * vertexB[3] };
  double weights[8] = { barycentricWeightsA[0], barycentricWeightsA[1], barycentricWeightsA[2], barycentricWeightsA[3],
                        -barycentricWeightsB[0], -barycentricWeightsB[1], -barycentricWeightsB[2], -barycentricWeightsB[3] };

  // compute the rest positions of the two points
  double restPositionA[3];
  double restPositionB[3];
  int columnStartA = columnStarts[objectA];
  int columnStartB = columnStarts[objectB];
  for(int i=0; i<3; i++)
  {
    restPositionA[i] = 0.0;
    restPositionB[i] = 0.0;
    for(int j=0; j<4; j++)
    {
      restPositionA[i] += barycentricWeightsA[j] * restPositions[columnStartA + 3 * vertexA[j] + i];
      restPositionB[i] += barycentricWeightsB[j] * restPositions[columnStartB + 3 * vertexB[j] + i];
    }
  }

  // wA_0 * (restA0 + uA0) + wA_1 * (restA1 + uA1) + wA_2 * (restA2 + uA2) + wA_3 * (restA3 + uA3) =
  // = wB_0 * (restB0 + uB0) + wB_1 * (restB1 + uB1) + wB_2 * (restB2 + uB2) + wB_3 * (restB3 + uB3)

  // constrain x,y,z DOFs
  for(int i=0; i<3; i++)
  {
    double b = restPositionB[i] - restPositionA[i];
    AddLinearConstraint(numInvolvedDOFs, objects, dofs, weights, b);
    for(int j=0; j<8; j++)
      dofs[j] += 1;
  }
}

void Constraints::AddFixedBarycentricTangentialConstraint(int object, const int vertex[4], const double barycentricWeights[4], const double planePoint[3], const double planeNormal[3])
{
  // (alpha * nx) u_0^x + (alpha * ny) u_0^y + (alpha * nz) u_0^z +
  // (beta * nx) u_1^x + (beta * ny) u_1^y + (beta * nz) u_1^z +
  // (gamma * nx) u_2^x + (gamma * ny) u_2^y + (gamma * nz) u_2^z +
  // (delta * nx) u_3^x + (delta * ny) u_3^y + (delta * nz) u_3^z = 
 //      (p0 - (alpha * x0 + beta * x1 + gamma * x2 + delta * x3)) n

  int numInvolvedDOFs = 12;
  int objects[12] = { object, object, object, 
                      object, object, object,
                      object, object, object,
                      object, object, object };
  int dofs[12] = { 3 * vertex[0] + 0, 3 * vertex[0] + 1, 3 * vertex[0] + 2, 
                   3 * vertex[1] + 0, 3 * vertex[1] + 1, 3 * vertex[1] + 2, 
                   3 * vertex[2] + 0, 3 * vertex[2] + 1, 3 * vertex[2] + 2, 
                   3 * vertex[3] + 0, 3 * vertex[3] + 1, 3 * vertex[3] + 2 };

  double weights[12] = { 
    barycentricWeights[0] * planeNormal[0], barycentricWeights[0] * planeNormal[1], barycentricWeights[0] * planeNormal[2], 
    barycentricWeights[1] * planeNormal[0], barycentricWeights[1] * planeNormal[1], barycentricWeights[1] * planeNormal[2], 
    barycentricWeights[2] * planeNormal[0], barycentricWeights[2] * planeNormal[1], barycentricWeights[2] * planeNormal[2], 
    barycentricWeights[3] * planeNormal[0], barycentricWeights[3] * planeNormal[1], barycentricWeights[3] * planeNormal[2] };

  // compute the rest position of this point
  double restPosition[3];
  int columnStart = columnStarts[object];
  for(int i=0; i<3; i++)
  {
    restPosition[i] = 0.0;
    for(int j=0; j<4; j++)
      restPosition[i] += barycentricWeights[j] * restPositions[columnStart + 3 * vertex[j] + i];
  }

  // compute rhs
  Vec3d planePointv(planePoint);
  Vec3d planeNormalv(planeNormal);
  Vec3d restPositionv(restPosition);
  double b = dot(planePointv - restPositionv, planeNormalv);

  AddLinearConstraint(numInvolvedDOFs, objects, dofs, weights, b);
}

void Constraints::AddPointBarycentricTangentialConstraint(int objectA, const int vertexA[4], const double barycentricWeightsA[4], 
  int objectB, const int vertexB[4], const double barycentricWeightsB[4], 
  const double normal[3], double offset) 
  // constrain point to point, but allow points to slide each in a plane that is normal to "normal", 
  // and with the two planes at a normal distance of "offset" (measured from plane A to plane B, in the direction of normal), 
  // with the two planes sliding along the normal direction without resistance: (xB - xA) dot normal = offset
{
  // (alphaA * nx) uA_0^x + (alphaA * ny) uA_0^y + (alphaA * nz) uA_0^z +
  // (betaA * nx) uA_1^x + (betaA * ny) uA_1^y + (betaA * nz) uA_1^z +
  // (gammaA * nx) uA_2^x + (gammaA * ny) uA_2^y + (gammaA * nz) uA_2^z +
  // (deltaA * nx) uA_3^x + (deltaA * ny) uA_3^y + (deltaA * nz) uA_3^z - 
  // (alphaB * nx) uB_0^x - (alphaB * ny) uB_0^y - (alphaB * nz) uB_0^z -
  // (betaB * nx) uB_1^x - (betaB * ny) uB_1^y - (betaB * nz) uB_1^z -
  // (gammaB * nx) uB_2^x - (gammaB * ny) uB_2^y - (gammaB * nz) uB_2^z -
  // (deltaB * nx) uB_3^x - (deltaB * ny) uB_3^y - (deltaB * nz) uB_3^z =
  // = (x_rest^B - x_rest^A) n - offset

  int numInvolvedDOFs = 24;
  int objects[24] = { objectA, objectA, objectA, 
                      objectA, objectA, objectA,
                      objectA, objectA, objectA,
                      objectA, objectA, objectA,
                      objectB, objectB, objectB, 
                      objectB, objectB, objectB,
                      objectB, objectB, objectB,
                      objectB, objectB, objectB };

  int dofs[24] = { 3 * vertexA[0] + 0, 3 * vertexA[0] + 1, 3 * vertexA[0] + 2, 
                   3 * vertexA[1] + 0, 3 * vertexA[1] + 1, 3 * vertexA[1] + 2, 
                   3 * vertexA[2] + 0, 3 * vertexA[2] + 1, 3 * vertexA[2] + 2, 
                   3 * vertexA[3] + 0, 3 * vertexA[3] + 1, 3 * vertexA[3] + 2,
                   3 * vertexB[0] + 0, 3 * vertexB[0] + 1, 3 * vertexB[0] + 2, 
                   3 * vertexB[1] + 0, 3 * vertexB[1] + 1, 3 * vertexB[1] + 2, 
                   3 * vertexB[2] + 0, 3 * vertexB[2] + 1, 3 * vertexB[2] + 2, 
                   3 * vertexB[3] + 0, 3 * vertexB[3] + 1, 3 * vertexB[3] + 2 };

  double weights[24] = { 
    barycentricWeightsA[0] * normal[0], barycentricWeightsA[0] * normal[1], barycentricWeightsA[0] * normal[2], 
    barycentricWeightsA[1] * normal[0], barycentricWeightsA[1] * normal[1], barycentricWeightsA[1] * normal[2], 
    barycentricWeightsA[2] * normal[0], barycentricWeightsA[2] * normal[1], barycentricWeightsA[2] * normal[2], 
    barycentricWeightsA[3] * normal[0], barycentricWeightsA[3] * normal[1], barycentricWeightsA[3] * normal[2],
    -barycentricWeightsB[0] * normal[0], -barycentricWeightsB[0] * normal[1], -barycentricWeightsB[0] * normal[2], 
    -barycentricWeightsB[1] * normal[0], -barycentricWeightsB[1] * normal[1], -barycentricWeightsB[1] * normal[2], 
    -barycentricWeightsB[2] * normal[0], -barycentricWeightsB[2] * normal[1], -barycentricWeightsB[2] * normal[2], 
    -barycentricWeightsB[3] * normal[0], -barycentricWeightsB[3] * normal[1], -barycentricWeightsB[3] * normal[2] };

  // compute the rest positions
  double restPositionA[3];
  double restPositionB[3];
  int columnStartA = columnStarts[objectA];
  int columnStartB = columnStarts[objectB];
  for(int i=0; i<3; i++)
  {
    restPositionA[i] = 0.0;
    restPositionB[i] = 0.0;
    for(int j=0; j<4; j++)
    {
      restPositionA[i] += barycentricWeightsA[j] * restPositions[columnStartA + 3 * vertexA[j] + i];
      restPositionB[i] += barycentricWeightsB[j] * restPositions[columnStartB + 3 * vertexB[j] + i];
    }
  }

  // compute rhs
  Vec3d normalv(normal);
  Vec3d restPositionAv(restPositionA);
  Vec3d restPositionBv(restPositionB);
  double b = dot(restPositionBv - restPositionAv, normalv) - offset;

  AddLinearConstraint(numInvolvedDOFs, objects, dofs, weights, b);
}

void Constraints::AddFixedDOFConstraint(int object, int dof, double value)
{
  if (JOutline == NULL) 
  {
    JOutline = new SparseMatrixOutline(1);
    b = (double*)malloc(sizeof(double) * 1);
  }
  else
  {
    JOutline->IncreaseNumRows(1);
    b = (double*)realloc(b, sizeof(double) * (JOutline->Getn()));
  }

  int column = 0;
  for(int objectID=0; objectID < object; objectID++)
  {
    column += r[objectID];
  }
  column += dof;

  int row = JOutline->Getn() - 1;
  JOutline->AddEntry(row, column, 1.0);

  b[row] = value - restPositions[column];
}

void Constraints::AddPointConstraint(int objectA, int vertexA, int objectB, int vertexB)
{
  if (JOutline == NULL)
  {
    JOutline = new SparseMatrixOutline(3);
    b = (double*)malloc(sizeof(double) * 3);
  }
  else
  {
    JOutline->IncreaseNumRows(3);
    b = (double*)realloc(b, sizeof(double) * JOutline->Getn());
  }

  int columnStart = columnStarts[objectA] + 3 * vertexA;
  int rowStart = JOutline->Getn() - 3;
  for(int i=0; i<3; i++)
    JOutline->AddEntry(rowStart+i, columnStart+i, 1.0);

  for(int i=0; i<3; i++)
    b[rowStart+i] = -restPositions[columnStart+i];

  columnStart = columnStarts[objectB] + 3 * vertexB;
  for(int i=0; i<3; i++)
    JOutline->AddEntry(rowStart+i, columnStart+i, -1.0);

  for(int i=0; i<3; i++)
    b[rowStart+i] += restPositions[columnStart+i];
}

//matrixB * PosB - matrixA * PosA = uAB
//matrixA, matrixB are row-major
void Constraints::AddPointConstraint(int objectA, int numVerticesA, int * verticesA,
    int objectB, int numVerticesB, int * verticesB, double uAB[3], double matrixA[9], double matrixB[9]) 
{
  if ((numVerticesA <= 0) || (numVerticesB <= 0))
    return;

  //mB * PB - mA * PA = uAB, PA = uA + rA, PB = uB + rB
  //so, mB * uB + mB * rB - mA * uA - mA * rA = uAB
  // => mA * uA - mB * uB = mB * rB - mA * rA - uAB

  if (JOutline == NULL)
  {
    JOutline = new SparseMatrixOutline(3);
    b = (double*)malloc(sizeof(double) * 3);
  }
  else
  {
    JOutline->IncreaseNumRows(3);
    b = (double*)realloc(b, sizeof(double) * JOutline->Getn());
  }

  int columnStart = columnStarts[objectA];
  int rowStart = JOutline->Getn() - 3;

  double restCenterPosA[3] = {0,0,0};
  for(int v = 0; v < numVerticesA; v++)
  {
    for(int i=0; i<3; i++)
    {
      if(matrixA)
        for(int j = 0; j < 3; j++)
          JOutline->AddEntry(rowStart+i, columnStart + 3*verticesA[v]+j, matrixA[i*3+j]/numVerticesA);
      else
        JOutline->AddEntry(rowStart+i, columnStart + 3*verticesA[v] + i, 1.0/numVerticesA);
      restCenterPosA[i] += restPositions[columnStart + 3*verticesA[v] + i];
    }
  }

  for(int i=0; i<3; i++)
  {
    if(matrixA)
      b[rowStart+i] = -(matrixA[i*3]*restCenterPosA[0]+matrixA[i*3+1]*restCenterPosA[1]+matrixA[i*3+2]*restCenterPosA[2]) / numVerticesA;
    else
      b[rowStart+i] = -restCenterPosA[i] / numVerticesA;
  }
  
  columnStart = columnStarts[objectB];
  
  double restCenterPosB[3] = {0,0,0};
  for(int v = 0; v < numVerticesB; v++)
  {
    for(int i=0; i<3; i++)
    {
      if(matrixB)
        for(int j = 0; j < 3; j++)
          JOutline->AddEntry(rowStart+i, columnStart + 3*verticesB[v]+j, -matrixB[i*3+j]/numVerticesB);
      else
        JOutline->AddEntry(rowStart+i, columnStart + 3*verticesB[v] + i, -1.0/numVerticesB);
      restCenterPosB[i] += restPositions[columnStart + 3*verticesB[v] + i];
    }
  }

  for(int i=0; i<3; i++)
  {
    if(matrixB)
      b[rowStart+i] += (matrixB[i*3] * restCenterPosB[0] + matrixB[i*3+1] * restCenterPosB[1] + matrixB[i*3+2] * restCenterPosB[2]) / numVerticesB;
    else
      b[rowStart+i] += restCenterPosB[i] / numVerticesB;

    if(uAB)
      b[rowStart+i] -= uAB[i];
  }
}

void Constraints::PrepareSkinningConstraint(int object, VolumetricMesh* mesh, int numSkinnedFrames, int numFrames, double* inputFramePos, int numSkinnedTets, int* skinnedTets)
{
  set<int> skinnedVerticesSet;

  int numVerticesPerElement = 4;
  if (mesh->getElementType() == VolumetricMesh::CUBIC)
    numVerticesPerElement = 8;
 
  for(int i=0; i<numSkinnedTets; i++)
  {
    for(int j=0; j<numVerticesPerElement; j++)
      skinnedVerticesSet.insert(mesh->getVertexIndex(skinnedTets[i],j));
  }

  numSkinnedVertices[object] = static_cast<int>(skinnedVerticesSet.size());

  delete(skinnedVertices[object]);
  delete(skinnedVertexPositions[object]);
  
  skinnedVertices[object] = (int*)malloc(sizeof(int) * numSkinnedVertices[object]);
  skinnedVertexPositions[object] = (double*)malloc(sizeof(double) * numSkinnedVertices[object] * 3);

  int counter = 0;
  for(set<int>::iterator it=skinnedVerticesSet.begin(); it!=skinnedVerticesSet.end(); it++)
  {
    skinnedVertices[object][counter] = *it;
    Vec3d vPos = mesh->getVertex(*it);
    for(int i=0; i<3; i++)
      skinnedVertexPositions[object][counter * 3 + i] = vPos[i];
    counter++;
  }

  delete(skinnings[object]);
  skinnings[object] = new Skinning(numSkinnedFrames, numFrames, inputFramePos, numSkinnedVertices[object], skinnedVertexPositions[object]);
}

void Constraints::ReleaseSkinningConstraint(int object)
{
  delete(skinnings[object]);
  skinnings[object] = NULL;

  delete(skinnedVertices[object]);
  skinnedVertices[object] = NULL;

  delete(skinnedVertexPositions[object]);
  skinnedVertexPositions[object] = NULL;
}

void Constraints::AddSkinningConstraint(int object, double* framex, double* frameA)
{
  if (skinnedVertices[object] == NULL)
  {
    printf("Skinning is not specified in advance for object %d.\n", object);
    exit(1);
  }

  double* targetPositions = (double*)malloc(sizeof(double) * numSkinnedVertices[object] * 3);

  double T[16] = {
    frameA[0], frameA[1], frameA[2], framex[0],
    frameA[3], frameA[4], frameA[5], framex[1],
    frameA[6], frameA[7], frameA[8], framex[2],
    0, 0, 0, 1.0
  };



  skinnings[object]->Skin(Skinning::LBS, T, targetPositions);

  for(int i=0; i<numSkinnedVertices[object]; i++)
  {
    AddFixedConstraint(object, skinnedVertices[object][i], &targetPositions[3*i]);
  }
}

void Constraints::AddFixedBarycentricConstraint3(int object, int vertex[3], double barycentricWeights[3], double pos[3])
{
  int numInvolvedDOFs = 3;
  int objects[3] = { object, object, object };
  int dofs[3] = { 3 * vertex[0], 3 * vertex[1], 3 * vertex[2] };

  // compute the rest position of this point
  double restPosition[3];
  int columnStart = columnStarts[object];
  for(int i=0; i<3; i++)
  {
    restPosition[i] = 0.0;
    for(int j=0; j<3; j++)
      restPosition[i] += barycentricWeights[j] * restPositions[columnStart + 3 * vertex[j] + i];
  }

  // constrain x,y,z DOFs
  for(int i=0; i<3; i++)
  {
    double b = pos[i] - restPosition[i];
    AddLinearConstraint(numInvolvedDOFs, objects, dofs, barycentricWeights, b);
    for(int j=0; j<3; j++)
      dofs[j] += 1;
  }
}

void Constraints::AddPointBarycentricConstraint3(int objectA, int vertexA[3], double barycentricWeightsA[3], int objectB, int vertexB[3], double barycentricWeightsB[3])
{
  int numInvolvedDOFs = 6;
  int objects[6] = { objectA, objectA, objectA, objectB, objectB, objectB };
  int dofs[6] = { 3 * vertexA[0], 3 * vertexA[1], 3 * vertexA[2], 
                  3 * vertexB[0], 3 * vertexB[1], 3 * vertexB[2] };
  double weights[6] = { barycentricWeightsA[0], barycentricWeightsA[1], barycentricWeightsA[2], 
                        -barycentricWeightsB[0], -barycentricWeightsB[1], -barycentricWeightsB[2] };

  // compute the rest positions of the two points
  double restPositionA[3];
  double restPositionB[3];
  int columnStartA = columnStarts[objectA];
  int columnStartB = columnStarts[objectB];
  for(int i=0; i<3; i++)
  {
    restPositionA[i] = 0.0;
    restPositionB[i] = 0.0;
    for(int j=0; j<3; j++)
    {
      restPositionA[i] += barycentricWeightsA[j] * restPositions[columnStartA + 3 * vertexA[j] + i];
      restPositionB[i] += barycentricWeightsB[j] * restPositions[columnStartB + 3 * vertexB[j] + i];
    }
  }

  // wA_0 * (restA0 + uA0) + wA_1 * (restA1 + uA1) + wA_2 * (restA2 + uA2) =
  // = wB_0 * (restB0 + uB0) + wB_1 * (restB1 + uB1) + wB_2 * (restB2 + uB2) 

  // constrain x,y,z DOFs
  for(int i=0; i<3; i++)
  {
    double b = restPositionB[i] - restPositionA[i];
    AddLinearConstraint(numInvolvedDOFs, objects, dofs, weights, b);
    for(int j=0; j<6; j++)
      dofs[j] += 1;
  }
}

void Constraints::AddTRACKSConstraint(int object, int numPatchVertices, int * patchVertices, double * patchVertexWeights, TetMesh * coarseMesh, int numCoarsePatchVertices, int * coarsePatchVertices, double * coarsePatchVertexWeights, double * uCoarse)
{
  if(JOutline == NULL)
  {
    JOutline = new SparseMatrixOutline(3);
    b = (double*)malloc(sizeof(double) * 3);
  }
  else
  {
    JOutline->IncreaseNumRows(3);
    b = (double*)realloc(b, sizeof(double) * JOutline->Getn());
  }

  int columnStart = columnStarts[object];
  int rowStart = JOutline->Getn() - 3;
 
  int numVertices = r[object]/3;
  int numCoarseVertices = coarseMesh->getNumVertices(); 
  
  double * CRow = (double*)malloc(sizeof(double) * r[object]);
  double * coarseCRow = (double*)malloc(sizeof(double) * 3 * numCoarseVertices);
  
  for (int row=0; row<3; row++)
  {
    memset(CRow, 0, sizeof(double) * r[object]);
  
    double sum = 0;
    for(int i=0; i<numVertices; i++)
    {
      for(int j=0; j<numPatchVertices; j++)
        CRow[i*3+row] += patchVertexWeights[numVertices*patchVertices[j]+i]; //row-indexed or column-indexed
    
      sum += CRow[i*3+row];
    }
  
    memset(coarseCRow, 0, sizeof(double) * 3 * numCoarseVertices);
  
    double coarseSum = 0;
    for(int i=0; i<numCoarseVertices; i++)
    {
      for(int j=0; j<numCoarsePatchVertices; j++)
        coarseCRow[i*3+row] += coarsePatchVertexWeights[numCoarseVertices*coarsePatchVertices[j]+i]; //row-indexed or column-indexed
  
      coarseSum += coarseCRow[i*3+row];
    }

    //normalization
    double scale = coarseSum/sum;
    for(int i=0; i<3*numVertices; i++)
    {
      CRow[i] *= scale;
    }
    
    for(int j=columnStart; j<r[object]+columnStart; j++)
      JOutline->AddEntry(rowStart+row, j, CRow[j-columnStart]);
  
    b[rowStart+row] = 0; 
    for(int i=0; i<numCoarseVertices*3; i++)
      b[rowStart+row] += coarseCRow[i] * uCoarse[i]; 
  }
}

void Constraints::FinalizeConstraints()
{
  delete(C);

  if (JOutline != NULL) 
  {
    int total = 0;
    for(int i = 0; i < numObjects; i++)
      total += r[i];

    // this entry is used to ensure that SparseMatrix::BuildSuperMatrixIndices is happy
    JOutline->AddEntry(0, total - 1, 0.0);
    
    C = new SparseMatrix(JOutline);
  }
  else
    C = NULL;
}

void Constraints::ComputeSkinnedTets(TetMesh * mesh, int numLineSegments, double * lineSegments, int * numSkinnedTets_, int ** skinnedTets_)
{
  /*set<int> skinnedTets;

  for(int i=0; i<numLineSegments; i++)
  {
    double * segment = &lineSegments[6*i];
    Vec3d start(&segment[0]);
    Vec3d end(&segment[3]);
    for(int el=0; el<mesh->getNumElements(); el++)
    {
      // collision detection between tet 'el' and the segment
      if (mesh->containsVertex(el, start) || mesh->containsVertex(el, end))
      {
	skinnedTets.insert(el);
      }
      else
      {
	Vec3d* tetVtx[4];
	for(int j=0; j<4; j++)
	  tetVtx[j] = mesh->getVertex(el, j);

	TriangleBasic* tri;
	Vec3d intersectionPoint;
        int code;
        
        #define TRIANGLE_INTERSECT(v1, v2, v3)\
        tri = new TriangleBasic(*tetVtx[(v1)], *tetVtx[(v2)], *tetVtx[v3]);\
	code = tri->lineSegmentIntersection(start, end, &intersectionPoint);\
	delete tri;\
	if(code == 1 || code == 2)\
	{\
          skinnedTets.insert(el);\
	  continue;\
	}
 	
        TRIANGLE_INTERSECT(0, 1, 2);
        TRIANGLE_INTERSECT(0, 1, 3);
        TRIANGLE_INTERSECT(0, 2, 3);
        TRIANGLE_INTERSECT(1, 2, 3);
      }
    }
  }

  *numSkinnedTets_ = (int) skinnedTets.size();
  *skinnedTets_ = (int*) malloc (sizeof(int) * *numSkinnedTets_);

  int counter = 0;
  for(set<int>::iterator it=skinnedTets.begin(); it!=skinnedTets.end(); it++)
  {
    *skinnedTets_[counter++] = *it;
  }*/
}


