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

#ifndef _CONSTRAINTS_H_
#define _CONSTRAINTS_H_

#include "sparseMatrix.h"
#include "tetMesh.h"
#include "skinning.h"

class Constraints
{
public:
  // init the class: set the number of objects
  // array r must be of length "numObjects"; r[i] is the number of deformable DOFs in each object
  // restPositions contains the rest configuration values of all the DOFs, in all objects
  Constraints(int numObjects, int const * r, double const * restPositions);
  ~Constraints();

  // clears all constraints
  void Reset(); 
  
  // vertex constraints
  void AddFixedConstraint(int object, int vertex, double pos[3]); // constrain a vertex to the given position (pos is a 3-vector)
  void AddFixedDOFConstraint(int object, int dof, double value); // constrain the given DOF to the given value
  void AddPointConstraint(int objectA, int vertexA, int objectB, int vertexB); // constrain vertexA of objectA to vertexB of objectB

  // impose a constraint of the form: matrixB * posB - matrixA * posA = uAB
  // if uAB is passed as NULL, uAB is assumed to be the zero vector
  // matrixA, matrixB are row-major 3x3 matrices
  // if either is passed as NULL, it is assumed to be identity
  void AddPointConstraint(int objectA, int numVerticesA, int * verticesA, int objectB, int numVerticesB, int * verticesB, double uAB[3] = NULL, double matrixA[9] = NULL, double matrixB[9] = NULL);

  // impose a general linear constraint, of the form:
  // sum_i^numInvolvedDOFs w_i u_{dof(i)} = b
  // where u_{dof(i)} is displacement (away from the rest configuration) of degree of freedom i
  // the DOFs can come from different objects
  // objects, dofs, weights are arrays of length "numInvolvedDOFs"
  void AddLinearConstraint(int numInvolvedDOFs, int * objects, int * dofs, double * weights, double b);

  // barycentric constraints
  void AddFixedBarycentricConstraint(int object, int vertex[4], double barycentricWeights[4], double pos[3]); // constrain point to a fixed position
  void AddFixedBarycentricTangentialConstraint(int object, const int vertex[4], const double barycentricWeights[4], const double planePoint[3], const double planeNormal[3]); // constrain point to the plane going through "planePoint" and with normal "planeNormal" (must be unit normal)
  void AddPointBarycentricConstraint(int objectA, int vertexA[4], double barycentricWeightsA[4], int objectB, int vertexB[4], double barycentricWeightsB[4]); // constrain point to point
  void AddPointBarycentricTangentialConstraint(int objectA, const int vertexA[4], const double barycentricWeightsA[4], int objectB, const int vertexB[4], const double barycentricWeightsB[4], 
    const double normal[3], double offset=0.0); // constrain point to point, but allow points to slide each in a plane that is normal to "normal", and with the two planes at a normal distance of "offset" (measured from plane A to plane B, in the direction of normal), with the two planes sliding along the normal direction without resistance: (xB - xA) dot normal = offset

  // triangle constraints (e.g., for cloth)
  void AddFixedBarycentricConstraint3(int object, int vertex[3], double barycentricWeights[3], double pos[3]); // constrain point to a fixed position
  void AddPointBarycentricConstraint3(int objectA, int vertexA[3], double barycentricWeightsA[3], int objectB, int vertexB[3], double barycentricWeightsB[3]); // constrain point to point

  // (advanced feature)
  // TRACKS-style constraint: constrain a patch of vertices, with the given weights, on this mesh, to a patch of vertices, with the given weights, on a coarse mesh
  void AddTRACKSConstraint(int object, int numPatchVertices, int * patchVertices, double * patchVertexWeights, TetMesh * coarseMesh, int numCoarsePatchVertices, int * coarsePatchVertices, double * coarsePatchVertexWeights, double * uCoarse); 

  // (advanced feature)
  // constrain tets to follow embedded line segments, i.e., "skinning" constraint
  static void ComputeSkinnedTets(TetMesh * mesh, int numLineSegments, double * lineSegments, int * numSkinnedTets, int ** skinnedTets);
  void PrepareSkinningConstraint(int object, VolumetricMesh * mesh, int numSkinnedFrames, int numFrames, double * inputFramePos, int numSkinnedTets, int * skinnedTets);
  void ReleaseSkinningConstraint(int object);
  void AddSkinningConstraint(int object, double * framex, double * frameA);

  // must call this function once, after adding all the constraints
  // after calling this function, constraints can no longer be added; to modify them, call "Reset" and start over
  void FinalizeConstraints();

  // constraint has the format: C * x = b
  // the following two functions will provide C and b, after "FinalizeConstraints" has been called
  SparseMatrix * GetConstraintMatrix() { return C; };
  double * GetConstraintRhs() { return b; };

protected:
  int numObjects; 
  int const * r;
  double const * restPositions;
  int * columnStarts;
  
  SparseMatrixOutline * JOutline;
  SparseMatrix * C;
  double * b;

  Skinning ** skinnings;
  int * numSkinnedVertices;
  int ** skinnedVertices;
  double ** skinnedVertexPositions;
};

#endif

