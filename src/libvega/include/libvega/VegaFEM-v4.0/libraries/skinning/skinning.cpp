/*************************************************************************
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
 * Linear blend skinning                                                 *
 * Code authors: Jernej Barbic, Hongyi Xu, Danyong Zhao                  *
 *                                                                       *
 *************************************************************************/

 /*
   Implements linear blend skinning of a set of vertices (points) in 3D.
   The vertices are deformed via a set of coordinate frames (bones).
 */

#include "skinning.h"
#include "matrixMultiplyMacros.h"
#include "mat3d.h"
#include <stdio.h>
#include <float.h>
#include <cassert>
#include <iostream>
#include <algorithm>
using namespace std;

Skinning::Skinning(int numSkinnedFrames_, int numFrames_, const double * undeformedFramePositions, int numTargetVertices_,
    const double * undeformedVertexPositions_) : numSkinnedFrames(numSkinnedFrames_), numFrames(numFrames_), numVertices(numTargetVertices_),
    undeformedVertexPositions(undeformedVertexPositions_)
{
  if (numSkinnedFrames > numFrames)
  {
    printf("The number of skinned frames %d is more than the total number of frames %d.\n", numSkinnedFrames, numFrames);
    numSkinnedFrames = numFrames;
  }

  vtxFrameIDs.resize(numSkinnedFrames * numVertices);
  vtxFrameWeights.resize(numSkinnedFrames * numVertices);

  // build frame indices and frame weights
  for (int i = 0; i < numVertices; i++)
  {
    int * localFrameIndices = &vtxFrameIDs[numSkinnedFrames * i];
    double * localFrameWeights = &vtxFrameWeights[numSkinnedFrames * i];

    int j;
    for (j = 0; j < numSkinnedFrames; j++)
      localFrameWeights[j] = DBL_MAX;

    // compute the closest "numSkinnedFrames" frames; distances to them are stored in "localFrameWeights"
    // note: localFrameWeights are *not* sorted
    for (j = 0; j < numFrames; j++)
    {
      // compute distance^2 of vertex i to frame j
      double dist = 0;
      for (int dof = 0; dof < 3; dof++)
        dist += (undeformedVertexPositions[3 * i + dof] - undeformedFramePositions[3 * j + dof]) * (undeformedVertexPositions[3 * i + dof] - undeformedFramePositions[3 * j + dof]);

      // compute maximum local frame weight: max { localFrameWeights[0], ..., localFrameWeights[numSkinnedFrames-1] }
      double maxLocalFrameWeight = -1.0;
      int maxLocalFrameWeightIndex = 0;
      for (int k = 0; k < numSkinnedFrames; k++)
      {
        if (maxLocalFrameWeight < localFrameWeights[k])
        {
          maxLocalFrameWeightIndex = k;
          maxLocalFrameWeight = localFrameWeights[k];
        }
      }

      // check if this vertex is closer than maxLocalFrameWeight
      if (dist < maxLocalFrameWeight)
      {
        // replace largest weight with dist
        // note: localFrameWeights are *not* sorted
        localFrameIndices[maxLocalFrameWeightIndex] = j;
        localFrameWeights[maxLocalFrameWeightIndex] = dist;
      }
    }

    // make the weights inversely propotional to distance, and normalize them (so they sum to 1.0)
    double sum = 0.0;
    for(j=0; j<numSkinnedFrames; j++)
    {
      if(localFrameWeights[j] == 0.0)
      {
        // special case: vertex coincides exactly with the frame origin
        break;
      }
      else
        sum += 1.0 / localFrameWeights[j];
    }

    if (j == numSkinnedFrames)
    {
      // non-degenerate case
      // make weights proportional to inverse distance to coordinate frame
      for(j=0; j<numSkinnedFrames; j++)
        localFrameWeights[j] = (1.0 / localFrameWeights[j]) / sum; 
    }
    else
    {
      // special case: assign all weights 0, and weight of this vertex to be 1.0
      memset(localFrameWeights, 0, sizeof(double) * numSkinnedFrames);
      localFrameWeights[j] = 1.0;
    }
  }
}

Skinning::Skinning(int numSkinnedBones, int numBones, const double * undeformedBoneSegments, int numTargetVertices, const double * undeformedVertexPositions, int) :
  numSkinnedFrames(numSkinnedBones), numFrames(numBones), numVertices(numTargetVertices),
  undeformedVertexPositions(undeformedVertexPositions)
{
  if (numSkinnedFrames > numFrames)
  {
    printf("The number of skinned frames %d is more than the total number of frames %d.\n", numSkinnedFrames, numFrames);
    numSkinnedFrames = numFrames;
  }
  
  vtxFrameIDs.resize(numSkinnedFrames * numTargetVertices);
  vtxFrameWeights.resize(numSkinnedFrames * numTargetVertices);
  
  // build frame indices and frame weights
  for(int i=0; i<numTargetVertices; i++)
  {
    Vec3d vertexPosition(undeformedVertexPositions + i * 3);
    int * localFrameIndices = &vtxFrameIDs[numSkinnedFrames * i];
    double * localFrameWeights = &vtxFrameWeights[numSkinnedFrames * i];
  
    int j;
    for(j = 0; j < numSkinnedFrames; j++)
      localFrameWeights[j] = DBL_MAX;
  
    // compute the closest "numSkinnedFrames" frames; distances to them are stored in "localFrameWeights"
    // note: localFrameWeights are *not* sorted
    for(j=0; j<numFrames; j++)
    {
      // compute distance^2 of vertex i to frame j
      Vec3d p0(undeformedBoneSegments + j * 6);
      Vec3d p1(undeformedBoneSegments + j * 6 + 3);
      Vec3d dir = p1 - p0;
      double A = len2(dir);
      Vec3d diff = vertexPosition - p0;

      double B = dot(diff, dir);
      double t = B / A;

      Vec3d closestPt;

      if (t <= 1.0 && t >= 0.0)
      {
        closestPt = dir * t + p0;
      } else if (t < 0.0) {
        closestPt = p0;
      } else {
        closestPt = p1;
      }

      // compute distance
      diff = vertexPosition - closestPt;
      double dist = len(diff);

      // compute maximum local frame weight: max { localFrameWeights[0], ..., localFrameWeights[numSkinnedFrames-1] }
      double maxLocalFrameWeight = -1.0;
      int maxLocalFrameWeightIndex = 0;
      for(int k=0; k<numSkinnedFrames; k++)
      {
        if (maxLocalFrameWeight < localFrameWeights[k])
        {
          maxLocalFrameWeightIndex = k;
          maxLocalFrameWeight = localFrameWeights[k];
        }
      }

      // check if this vertex is closer than maxLocalFrameWeight
      if (dist < maxLocalFrameWeight)
      {
        // replace largest weight with dist
        // note: localFrameWeights are *not* sorted
        localFrameIndices[maxLocalFrameWeightIndex] = j;
        localFrameWeights[maxLocalFrameWeightIndex] = dist * dist;
      }
    }

    // make the weights inversely propotional to distance, and normalize them (so they sum to 1.0)
    double sum = 0.0;
    for (j = 0; j < numSkinnedFrames; j++)
    {
      if (localFrameWeights[j] == 0.0)
      {
        // special case: vertex coincides exactly with the frame origin
        break;
      }
      else
        sum += 1.0 / localFrameWeights[j];
    }

    if (j == numSkinnedFrames)
    {
      // non-degenerate case
      // make weights proportional to inverse distance to coordinate frame
      for (j = 0; j < numSkinnedFrames; j++)
        localFrameWeights[j] = (1.0 / localFrameWeights[j]) / sum;
    }
    else
    {
      // special case: assign all weights 0, and weight of this vertex to be 1.0
      memset(localFrameWeights, 0, sizeof(double) * numSkinnedFrames);
      localFrameWeights[j] = 1.0;
    }
  }
}

/*
Skinning::Skinning(int numSkinnedFrames, int numTargetVertices, const double * undeformedVertexPositions, const int * indices, const double * weights, const std::vector<int> & permutation)
  : numSkinnedFrames(numSkinnedFrames), numTargetVertices(numTargetVertices), undeformedVertexPositions(undeformedVertexPositions), permutation(permutation)
{
  frameIndices = (int*)malloc(sizeof(int) * numSkinnedFrames * numTargetVertices);
  frameWeights = (double*)malloc(sizeof(double) * numSkinnedFrames * numTargetVertices);

  memcpy(frameWeights, weights, sizeof(double) * numSkinnedFrames * numTargetVertices);
  for (int i = 0; i < numTargetVertices; i++)
    memcpy(frameIndices + numSkinnedFrames * i, indices, sizeof(int) * numSkinnedFrames);
}
*/

Skinning::Skinning(const double * undeformedVertexPositions, const SparseMatrix * weightMatrix, const double * restCorrections) :
    undeformedVertexPositions(undeformedVertexPositions), restCorrections(restCorrections)
{
  numVertices = weightMatrix->GetNumRows();
  numFrames = weightMatrix->GetNumColumns();
  numSkinnedFrames = 0;
  for (int i = 0; i < numVertices; i++)
    numSkinnedFrames = std::max(numSkinnedFrames, weightMatrix->GetRowLength(i));
  assert(numSkinnedFrames >= 2);

  vtxFrameIDs.resize(numSkinnedFrames * numVertices, 0.0);
  vtxFrameWeights.resize(numSkinnedFrames * numVertices, 0.0);
  // Copy skinning weights from SparseMatrix
  vector<pair<double, int>> sortBuffer;
  for (int vtxID = 0; vtxID < numVertices; vtxID++)
  {
    sortBuffer.clear();
    for (int j = 0; j < weightMatrix->GetRowLength(vtxID); j++)
    {
      int frameID = weightMatrix->GetColumnIndex(vtxID, j);
      double weight = weightMatrix->GetEntry(vtxID, j);
      sortBuffer.emplace_back(weight, frameID);
    }
    assert(sortBuffer.size() > 0);
    sort(sortBuffer.rbegin(), sortBuffer.rend()); // sort it in descending order using reverse_iterators
    for(size_t i = 0; i < sortBuffer.size(); i++)
    {
      vtxFrameIDs[vtxID * numSkinnedFrames + i] = sortBuffer[i].second;
      vtxFrameWeights[vtxID * numSkinnedFrames + i] = sortBuffer[i].first;
    }

    // when number of frames used on this vertex is smaller than numSkinnedFrames,
    // the remaining empty entries are initialized to zero because vector::resize(XX, 0.0)
  }
}

Skinning::Skinning(int numTargetVertices, const double * undeformedVertexPositions, int numFrames, const double * denseWeightMatrix) :
    numFrames(numFrames), numVertices(numTargetVertices), undeformedVertexPositions(undeformedVertexPositions)
{
  vector<int> framesAffected(numVertices, 0);
  for(int f = 0; f < numFrames; f++)
    for(int v = 0; v < numVertices; v++)
      if (denseWeightMatrix[f*numVertices+v] != 0.0)
        framesAffected[v]++;

  if (numVertices > 0)
  {
    numSkinnedFrames = *max_element(framesAffected.begin(), framesAffected.end());
    assert(numSkinnedFrames >= 2);

    vtxFrameIDs.resize(numSkinnedFrames * numVertices, 0.0);
    vtxFrameWeights.resize(numSkinnedFrames * numVertices, 0.0);
    // Copy skinning weights from SparseMatrix
    vector<vector<pair<double, int>>> vtxSortBuffer(numVertices);
    for(int f = 0; f < numFrames; f++)
      for (int vtxID = 0; vtxID < numVertices; vtxID++)
      {
        double value = denseWeightMatrix[f*numVertices+vtxID];
        if (value == 0.0)
          continue;

        vtxSortBuffer[vtxID].emplace_back(value, f);
      }

    for(int vtxID = 0; vtxID < numVertices; vtxID++)
    {
      sort(vtxSortBuffer[vtxID].rbegin(), vtxSortBuffer[vtxID].rend()); // sort it in descending order using reverse_iterators
      for(size_t i = 0; i < vtxSortBuffer[vtxID].size(); i++)
      {
        vtxFrameIDs[vtxID * numSkinnedFrames + i] = vtxSortBuffer[vtxID][i].second;
        vtxFrameWeights[vtxID * numSkinnedFrames + i] = vtxSortBuffer[vtxID][i].first;
      }

      // when number of frames used on this vertex is smaller than numSkinnedFrames,
      // the remaining empty entries are initialized to zero because vector::resize(XX, 0.0)
    }
  }
}

void Skinning::SkinOneVertex(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
  double * deformedPos)
{
  deformedPos[0] = skinRotation[0] * restPos[0] + skinRotation[1] * restPos[1] + skinRotation[2] * restPos[2] + skinTranslation[0];
  deformedPos[1] = skinRotation[3] * restPos[0] + skinRotation[4] * restPos[1] + skinRotation[5] * restPos[2] + skinTranslation[1];
  deformedPos[2] = skinRotation[6] * restPos[0] + skinRotation[7] * restPos[1] + skinRotation[8] * restPos[2] + skinTranslation[2];
}

void Skinning::SkinOneVertexWithRestCorrections(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
  double * deformedPos, const double * restCorrection)
{
  double rest[3];
  VECTOR_ADD3(restPos, restCorrection, rest);
  deformedPos[0] = skinRotation[0] * rest[0] + skinRotation[1] * rest[1] + skinRotation[2] * rest[2] + skinTranslation[0];
  deformedPos[1] = skinRotation[3] * rest[0] + skinRotation[4] * rest[1] + skinRotation[5] * rest[2] + skinTranslation[1];
  deformedPos[2] = skinRotation[6] * rest[0] + skinRotation[7] * rest[1] + skinRotation[8] * rest[2] + skinTranslation[2];
  //if (debug)
  //{
  //  LGI << "Skinning\n" << lg::Ary(skinRotation, 9, 3) << "\n" << lg::Ary(skinTranslation, 3, 3)
  //    << "\n" << lg::Ary(restPos, 3, 3)
  //    << "\n" << lg::Ary(deformedPos, 3, 3)
  //    << "\n" << lg::Ary(restCorrection, 3, 3);
  //  exit(0);
  //}
}

void Skinning::UnskinOneVertex(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
  const double * deformedPos, double * restVertexCorrection)
{
  double rhs[3];
  VECTOR_SUBTRACT3(deformedPos, skinTranslation, rhs);
  restVertexCorrection[0] = skinRotation[0] * rhs[0] + skinRotation[3] * rhs[1] + skinRotation[6] * rhs[2] - restPos[0];
  restVertexCorrection[1] = skinRotation[1] * rhs[0] + skinRotation[4] * rhs[1] + skinRotation[7] * rhs[2] - restPos[1];
  restVertexCorrection[2] = skinRotation[2] * rhs[0] + skinRotation[5] * rhs[1] + skinRotation[8] * rhs[2] - restPos[2];
}

void Skinning::UnskinOneVertexViaInverse(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
  const double * deformedPos, double * restVertexCorrection)
{
  double rhs[3], inverseRotation[9];
  VECTOR_SUBTRACT3(deformedPos, skinTranslation, rhs);
  inverse3x3(skinRotation, inverseRotation);
  restVertexCorrection[0] = inverseRotation[0] * rhs[0] + inverseRotation[1] * rhs[1] + inverseRotation[2] * rhs[2] - restPos[0];
  restVertexCorrection[1] = inverseRotation[3] * rhs[0] + inverseRotation[4] * rhs[1] + inverseRotation[5] * rhs[2] - restPos[1];
  restVertexCorrection[2] = inverseRotation[6] * rhs[0] + inverseRotation[7] * rhs[1] + inverseRotation[8] * rhs[2] - restPos[2];

  //if (debug)
  //{
  //  LGI << "Skinning\n" << lg::Ary(skinRotation, 9, 3) << "\n" << lg::Ary(skinTranslation, 3, 3)
  //    << "\n" << lg::Ary(restPos, 3, 3)
  //    << "\n" << lg::Ary(deformedPos, 3, 3)
  //    << "\n" << lg::Ary(restVertexCorrection, 3, 3);
  //}
}

void Skinning::SkinViaLBS(const double * frameTransforms, double * deformedVertexPositions, double * restVertexCorrections) const
{
  for (int vtxID = 0; vtxID < numVertices; vtxID++)
  {
    const int * localFrameIndices = &vtxFrameIDs[numSkinnedFrames * vtxID];
    const double * localFrameWeights = &vtxFrameWeights[numSkinnedFrames * vtxID];

    double skinRotation[9], skinTranslation[3];
    memset(skinRotation, 0, sizeof(skinRotation));
    memset(skinTranslation, 0, sizeof(skinTranslation));
    // transform vertex with respect to each governing frame, and blend the results with weights
    for (int j = 0; j < numSkinnedFrames; j++)
    {
      int frameIndex = localFrameIndices[j];
      double frameWeight = localFrameWeights[j];

      const double * A = &frameTransforms[16 * frameIndex];

      for (int ii = 0; ii < 3; ii++)
      {
        for (int jj = 0; jj < 3; jj++)
          skinRotation[3 * ii + jj] += A[4 * ii + jj] * frameWeight;
        skinTranslation[ii] += A[4 * ii + 3] * frameWeight;
      }
    }
    //debug = (i == 28887);
    if (restVertexCorrections == nullptr)
    {
      if (restCorrections == nullptr)
        SkinOneVertex(skinRotation, skinTranslation, &undeformedVertexPositions[3 * vtxID], &deformedVertexPositions[3 * vtxID]);
      else
        SkinOneVertexWithRestCorrections(skinRotation, skinTranslation, &undeformedVertexPositions[3 * vtxID], &deformedVertexPositions[3 * vtxID],
            &restCorrections[3 * vtxID]);
    }
    else
      UnskinOneVertexViaInverse(skinRotation, skinTranslation, &undeformedVertexPositions[3 * vtxID], &deformedVertexPositions[3 * vtxID],
          &restVertexCorrections[3 * vtxID]);
  }
}

void Skinning::SkinViaDQ(const DualDoubleQuaternion * dq, double * deformedVertexPositions, double * restVertexCorrections) const
{
  for (int i = 0; i < numVertices; i++)
  {
    const int * localFrameIndices = &vtxFrameIDs[numSkinnedFrames * i];
    const double * localFrameWeights = &vtxFrameWeights[numSkinnedFrames * i];

    DualDoubleQuaternion b = dq[localFrameIndices[0]] * localFrameWeights[0];
    for (int j = 1; j < numSkinnedFrames; j++)
    {
      if (dot(dq[localFrameIndices[0]].real(), dq[localFrameIndices[j]].real()) >= 0)
        b += dq[localFrameIndices[j]] * localFrameWeights[j];
      else
        b += dq[localFrameIndices[j]] * (-localFrameWeights[j]);
    }

    b.normalize();

    double skinRotation[9], skinTranslation[3]; 
    b.decompose(skinRotation, skinTranslation);

    if (restVertexCorrections == nullptr)
      SkinOneVertex(skinRotation, skinTranslation, &undeformedVertexPositions[3 * i], &deformedVertexPositions[3 * i]);
    else
      UnskinOneVertex(skinRotation, skinTranslation, &undeformedVertexPositions[3 * i], &deformedVertexPositions[3 * i], &restVertexCorrections[3 * i]);
  }
}

// set the indices of frames that affect each vertex (numSkinnedFrames x numTargetVertices)
void Skinning::SetFrameIndices(const int * frameIndices_)
{
  memcpy(vtxFrameIDs.data(), frameIndices_, sizeof(int) * numSkinnedFrames * numVertices);
}

// set the weights of frames, for each vertex (numSkinnedFrames x numTargetVertices)
void Skinning::SetFrameWeights(const double * frameWeights_)
{
  memcpy(vtxFrameWeights.data(), frameWeights_, sizeof(double) * numSkinnedFrames * numVertices);
}


void Skinning::Skin(SKIN_METHOD method, const double * frameTransforms, double * deformedVertexPositions, double * restVertexCorrections)
{
  switch (method)
  {
  case LBS:
    return SkinViaLBS(frameTransforms, deformedVertexPositions, restVertexCorrections);
  case DQ:
    dq.clear();
    for (int i = 0; i < numFrames; i++)
      dq.push_back(DualDoubleQuaternion(frameTransforms + 16 * i));
    return SkinViaDQ(dq.data(), deformedVertexPositions, restVertexCorrections);
  }
  assert(0); // should not reach here
}

