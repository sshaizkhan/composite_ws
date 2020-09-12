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
 * Code authors: Jernej Barbic, Hongyi Xu                                *
 *                                                                       *
 *************************************************************************/

/*
  Implements linear blend skinning of a set of vertices (points) in 3D.
  The vertices are deformed via a set of coordinate frames (bones).
*/

#ifndef _SKINNING_H_
#define _SKINNING_H_

#include "quaternion.h"
#include "dualQuaternion.h"
#include "sparseMatrix.h"
#include <vector>

class Skinning
{
public:
  // note: here "frame" means a coordinate frame (often called a "bone" in the context of skinning)
  // numSkinnedFrames: how many coordinate frames affect each vertex (good default: 4)
  // numFrames: total number of coordinate frames
  // undeformedFramePositions: the undeformed coordinate frame positions (3 x numFrames, column-major)
  // undeformedVertexPositions: the undeformed mesh vertex positions (3 x numTargetVertices, column-major)
  // this constructor automatically builds the skinning weights, based on the distance of undeformed vertex position to each frame;
  // only "numSkinnedFrames" closest frames will affect this vertex, with weights inversely proportional to distance
  Skinning(int numSkinnedFrames, int numFrames, const double * undeformedFramePositions, int numTargetVertices, const double * undeformedVertexPositions);

  Skinning(int numSkinnedBones, int numBones, const double *undeformedBoneSegments, int numTargetVertices, const double * undeformedVertexPositions, int);

  // load the skinning from a text file (as exported, say, from Maya)
  Skinning(const double * undeformedVertexPositions, const SparseMatrix * weightMatrix, const double * restCorrection = nullptr);

  // denseWeightMatrix: #targetVtx x #frames, column-major
  Skinning(int numTargetVertices, const double * undeformedVertexPositions, int numFrames, const double * denseWeightMatrix);

  enum SKIN_METHOD {LBS, DQ};
  // perform skinning
  // a coordinate frame transforms the world as X |--> framex + frameA * X
  // framex is 3 x numFrames (column-major)
  // frameA is 9 x numFrames (column-major), each 3x3 matrix is given as row-major
  // output: deformedVertexPositions
  //void Skin(int method, const double * framex, const double * frameA, double * deformedVertexPositions);
  // If restVertexCorrections is nullptr, ignore this parameter and compute regular skinning
  // Otherwise, compute restVertexCorrections so that the skinning result would be same as deformedVertexPositions
  // each frameTransform is 4x4, row-major, last row is always 0 0 0 1
  void Skin(SKIN_METHOD method, const double * frameTransforms, double * deformedVertexPositions, double * restVertexCorrections = nullptr);

  // manual assignment of indices of governing frames, and the skinning weights
  // set the indices of frames that affect each vertex (numSkinnedFrames x numTargetVertices)
  void SetFrameIndices(const int * frameIndices);
  // set the weights of frames, for each vertex (numSkinnedFrames x numTargetVertices)
  void SetFrameWeights(const double * frameWeights);

  inline int getNumTargetVertices() const { return numVertices; }
  inline int getNumSkinnedFrames() const { return numSkinnedFrames; }

  inline int getInfluencedJoint(const int vertex) const { return vtxFrameIDs[numSkinnedFrames * vertex]; }
  inline int getSecondInfluencedJoint(const int vertex) const { return vtxFrameIDs[numSkinnedFrames * vertex + 1]; }

  // i: [0, getNumSkinnedFrames)
  int getVertexFrameID(int vtxID, int i) const { return vtxFrameIDs[vtxID * numSkinnedFrames + i]; }
  double getVertexWeight(int vtxID, int i) const { return vtxFrameWeights[vtxID * numSkinnedFrames + i]; }
protected:
  int numSkinnedFrames = 0;
  int numFrames = 0;
  int numVertices = 0;
  const double * undeformedVertexPositions = nullptr;
  const double * restCorrections = nullptr;


  std::vector<int> vtxFrameIDs; // the indices of frames that affect each vertex (numSkinnedFrames x numTargetVertices)
  std::vector<double> vtxFrameWeights; // the weights of frames, for each vertex (numSkinnedFrames x numTargetVertices)
//  std::vector <int> permutation;
  std::vector<DualDoubleQuaternion> dq;
  void SkinViaLBS(const double * frameTransforms, double * deformedVertexPositions, double * restVertexCorrections) const;
  void SkinViaDQ(const DualDoubleQuaternion * q, double * deformedVertexPositions, double * restVertexCorrections) const;

  static void SkinOneVertex(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
    double * deformedPos);
  static void SkinOneVertexWithRestCorrections(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
    double * deformedPos, const double * restCorrection);

  static void UnskinOneVertexViaInverse(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
    const double * deformedPos, double * restVertexCorrection);
  static void UnskinOneVertex(const double skinRotation[9], const double skinTranslation[3], const double * restPos,
    const double * deformedPos, double * restVertexCorrection);
};

#endif

