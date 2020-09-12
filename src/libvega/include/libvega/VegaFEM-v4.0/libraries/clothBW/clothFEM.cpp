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
 * Code authors: Bohan Wang, Jernej Barbic                               *
 *                                                                       *
 *************************************************************************/

#include "clothFEM.h"
#include "geometryQuery.h"
#include "sparseMatrix.h"
#include <vector>
#include <utility>
#include <map>
#include <iostream>
#include <cassert>

class ClothFEMImpl
{
public:
  ClothFEMImpl(int numVertices, const double * restPositions, const double * masses,
    int numTriangles, const int * triangles, 
    const int * triangleStretchAndShearMaterialIDs,
    const int * triangleBendMaterialIDs,
    int numStretchAndShearMaterials, 
    const ClothFEM::StretchAndShearMaterial * stretchAndShearMaterials,
    int numBendMaterials, 
    const ClothFEM::BendMaterial * bendMaterials);

  ClothFEMImpl(int numVertices, const double * restPositions, const double * masses,
    int numTriangles, const int * triangles, const double * triangleUVs,
    const int * triangleStretchAndShearMaterials,
    const int * triangleBendMaterials,
    int numStretchAndShearMaterials, 
    const ClothFEM::StretchAndShearMaterial * stretchAndShearMaterials,
    int numBendMaterials, 
    const ClothFEM::BendMaterial * bendMaterials);

  void GenerateUVs(const double * triangleUVs);
  void GenerateQuads();

  double ComputeBendStiffness(double E, double nu, double h);
  void ComputeBendStiffness();

  void ComputeShearAndStretchTerms(int tri, const double * u, double * E, double f[9], double K[81]) const;
  void ComputeBendTerms(const double * u, int quadId, double * energy, double f[12], double K[144]) const;

public:
  // store vtx and triangle indices for computing bend force
  struct Quad
  {
    int v[4];
    int tri[2];
  };

  std::vector<Vec3d> restPositions;
  std::vector<Vec3i> triangles;
  std::vector<Quad> quads;
  std::vector<int> triangleStretchAndShearMaterialIDs;
  std::vector<int> triangleBendMaterialIDs;

  std::vector<ClothFEM::StretchAndShearMaterial> stretchAndShearMaterials;
  std::vector<ClothFEM::BendMaterial> bendMaterials;
  std::vector<double> bendStiffness;

  std::vector<double> masses;

  std::vector<double> restPhiTheta;
  std::vector<double> restai;

  std::vector<double> ru, rv;
  std::vector<double> restTriangleAreas;

  int useRestAnglesForBendForces;

protected:
  double ComputePhiTheta(const Vec3d &n1, const Vec3d &n2, const Vec3d &e0) const;
  double Computeh(const Vec3d &p0, const Vec3d &p1, const Vec3d &p2, Vec3d &m) const;
};

ClothFEMImpl::ClothFEMImpl(int numVertices_, const double * restPositions_, const double * masses_,
  int numTriangles_, const int * triangles_, 
  const int * triangleStretchAndShearMaterialIDs_,
  const int * triangleBendMaterialIDs_,
  int numStretchAndShearMaterials_, 
  const ClothFEM::StretchAndShearMaterial * stretchAndShearMaterials_,
  int numBendMaterials_, 
  const ClothFEM::BendMaterial * bendMaterials_)
{
  restPositions.reserve(numVertices_);
  for (int i = 0; i < numVertices_; i++) 
  {
    restPositions.push_back(Vec3d(restPositions_ + i * 3));
  }

  triangles.reserve(numTriangles_);
  for (int i = 0; i < numTriangles_; i++) 
  {
    triangles.push_back(Vec3i(triangles_ + i * 3));
  }

  masses.assign(masses_, masses_ + numVertices_);
  triangleStretchAndShearMaterialIDs.assign(triangleStretchAndShearMaterialIDs_, triangleStretchAndShearMaterialIDs_ + numTriangles_);
  triangleBendMaterialIDs.assign(triangleBendMaterialIDs_, triangleBendMaterialIDs_ + numTriangles_);
  stretchAndShearMaterials.assign(stretchAndShearMaterials_, stretchAndShearMaterials_ + numStretchAndShearMaterials_);
  bendMaterials.assign(bendMaterials_, bendMaterials_ + numBendMaterials_);

  // generate UVs
  std::vector<double> triangleUVs(3 * 2 * triangles.size());
  for (int i = 0; i < static_cast<int>(triangles.size()); i++) 
  {
    int vertexA = triangles[i][0];
    int vertexB = triangles[i][1];
    int vertexC = triangles[i][2];

    triangleUVs[6 * i + 0] = 0.0;  // vertex A, u coordinate
    triangleUVs[6 * i + 1] = 0.0;  // vertex A, v coordinate

    Vec3d x0 = restPositions[vertexA] - restPositions[vertexB];  // vector from A to B

    double lengthAB = len(x0);
    triangleUVs[6 * i + 2] = lengthAB;  // vertex B, u coordinate
    triangleUVs[6 * i + 3] = 0.0;       // vertex B, v coordinate

    Vec3d x1 = restPositions[vertexA] - restPositions[vertexC];  // vector from A to C

    Vec3d xn0 = norm(x0);  // normalized vector from A to B
    double lengthAC = len(x1);
    triangleUVs[6 * i + 4] = dot(xn0, x1); // vertex C, u coordinate
    triangleUVs[6 * i + 5] = sqrt(lengthAC * lengthAC - triangleUVs[6 * i + 4] * triangleUVs[6 * i + 4]);  // vertex C, v coordinate
  }

  GenerateUVs(triangleUVs.data());
  GenerateQuads();
}

ClothFEMImpl::ClothFEMImpl(int numVertices_, const double * restPositions_, const double * masses_,
  int numTriangles_, const int * triangles_, const double * triangleUVs_,
  const int * triangleStretchAndShearMaterialIDs_,
  const int * triangleBendMaterialIDs_,
  int numStretchAndShearMaterials_, 
  const ClothFEM::StretchAndShearMaterial * stretchAndShearMaterials_,
  int numBendMaterials_, 
  const ClothFEM::BendMaterial * bendMaterials_)
{
  restPositions.reserve(numVertices_);
  for (int i = 0; i < numVertices_; i++) 
  {
    restPositions.push_back(Vec3d(restPositions_ + i * 3));
  }

  triangles.reserve(numTriangles_);
  for (int i = 0; i < numTriangles_; i++) 
  {
    triangles.push_back(Vec3i(triangles_ + i * 3));
  }

  masses.assign(masses_, masses_ + numVertices_);
  triangleStretchAndShearMaterialIDs.assign(triangleStretchAndShearMaterialIDs_, triangleStretchAndShearMaterialIDs_ + numTriangles_);
  triangleBendMaterialIDs.assign(triangleBendMaterialIDs_, triangleBendMaterialIDs_ + numTriangles_);
  stretchAndShearMaterials.assign(stretchAndShearMaterials_, stretchAndShearMaterials_ + numStretchAndShearMaterials_);
  bendMaterials.assign(bendMaterials_, bendMaterials_ + numBendMaterials_);

  GenerateUVs(triangleUVs_);
  GenerateQuads();
}

double ClothFEMImpl::ComputeBendStiffness(double E, double nu, double h)
{
  return E * h * h * h / 24.0 / (1.0 - nu * nu);
}

void ClothFEMImpl::GenerateUVs(const double *triangleUVs)
{
  ru.resize(triangles.size() * 3);
  rv.resize(triangles.size() * 3);
  restTriangleAreas.resize(triangles.size());

  for (int i = 0, k = 0; i < static_cast<int>(triangles.size()); i++) 
  {
    double ua = triangleUVs[k];
    double va = triangleUVs[k + 1];
    double ub = triangleUVs[k + 2];
    double vb = triangleUVs[k + 3];
    double uc = triangleUVs[k + 4];
    double vc = triangleUVs[k + 5];

    double vcb = vb - vc, vac = vc - va, vba = va - vb;
    double d = ua * vcb + ub * vac + uc * vba;
    double dinv = 1.0 / d;

    ru[i * 3 + 0] = dinv * vcb;
    ru[i * 3 + 1] = dinv * vac;
    ru[i * 3 + 2] = dinv * vba;
    rv[i * 3 + 0] = dinv * (uc - ub);
    rv[i * 3 + 1] = dinv * (ua - uc);
    rv[i * 3 + 2] = dinv * (ub - ua);

    restTriangleAreas[i] = fabs(d) * 0.5;

    k += 6;
  }
}

void ClothFEMImpl::GenerateQuads()
{
  int numVertices = static_cast<int>(restPositions.size());
  // index all edges that can bend

  /* New ordering below
           B         D
           1---------3
          / \       /
         /   \  5  /
        /  4  \   /
       /       \ /
      0---------2
      A         C
   */
  // SORTED ensures that edges are paired in ascending order
  #define SORTED(i, j) ((i) <= (j) ? std::make_pair((i), (j)) : std::make_pair((j), (i)))

  typedef std::pair<int, int> Edge;      // store sorted edge vtx index
  typedef std::pair<int, int> TriOpVtx;  // store the triangle data (triangle index, the non opposite vtx on the triangle) for an edge
  std::map<Edge, TriOpVtx> edgeInfo;     // map the edge to the <triangle index, opposite vtx> data
  std::map<Edge, TriOpVtx>::iterator iter;

  for (int tri = 0; tri < static_cast<int>(triangles.size()); tri++) 
  {
    const Vec3i &trivtx = triangles[tri];
    assert(trivtx[0] >= 0 && trivtx[1] >= 0 && trivtx[2] >= 0);
    assert(trivtx[0] < numVertices && trivtx[1] < numVertices && trivtx[2] < numVertices);

    for (int j = 0; j < 3; j++) 
    {
      Edge edge = SORTED(trivtx[j], trivtx[(j + 1) % 3]);
      int oppositeVtx = trivtx[(j + 2) % 3];

      if ((iter = edgeInfo.find(edge)) == edgeInfo.end())  // edge is not in edgeInfo, we insert it
        edgeInfo.insert(make_pair(edge, TriOpVtx(tri, oppositeVtx)));
      else  // find one edge. We can form a bend edge with the new one and the old one
      {
        Quad info;
        TriOpVtx &previousTri = iter->second;
        // If we already visited this bend edge before and created a BendInfo,
        // there's an edge in the mesh that has three neighboring triangles.
        if (previousTri.first < 0)
          throw 1;

        info.v[0] = previousTri.second;   // index of the first triangle's vtx that is opposite to the edge
        info.v[1] = edge.first;           // index of the first edge vtx
        info.v[2] = edge.second;          // index of the second edge vtx
        info.v[3] = oppositeVtx;          // index of the second triangle's vtx that is opposite to the edge
        info.tri[0] = previousTri.first;  // index of the first triangle
        info.tri[1] = tri;                // index of the second triangle
        quads.push_back(info);

        previousTri.first = -1;  // mark this edge as visited
      }
    }
  }

  ComputeBendStiffness();

  restPhiTheta.resize(quads.size());
  restai.resize(quads.size());

  std::vector<double> triangleAreas(triangles.size());
  for (int i = 0; i < static_cast<int>(triangles.size()); i++) 
  {
    int v0 = triangles[i][0];
    int v1 = triangles[i][1];
    int v2 = triangles[i][2];

    Vec3d p0 = restPositions[v0];
    Vec3d p1 = restPositions[v1];
    Vec3d p2 = restPositions[v2];

    Vec3d e1 = p1 - p0;
    Vec3d e2 = p2 - p0;

    Vec3d area = cross(e1, e2);
    triangleAreas[i] = len(area) * 0.5;
  }

  for (int i = 0; i < static_cast<int>(quads.size()); i++) 
  {
    Vec3d p0 = restPositions[quads[i].v[0]];
    Vec3d p1 = restPositions[quads[i].v[1]];
    Vec3d p2 = restPositions[quads[i].v[2]];
    Vec3d p3 = restPositions[quads[i].v[3]];

    Vec3d n1 = cross(p2 - p0, p1 - p0);
    Vec3d n2 = cross(p1 - p3, p2 - p3);
    n1.normalize();
    n2.normalize();
    Vec3d e0 = p1 - p2;

    double phiTheta = ComputePhiTheta(n1, n2, e0);
    restPhiTheta[i] = phiTheta;

    double Aibar = triangleAreas[quads[i].tri[0]] + triangleAreas[quads[i].tri[1]];
    Vec3d eibar = p1 - p2;
    double ai = 3.0 * dot(eibar, eibar) / Aibar;
    restai[i] = ai;
  }
}

void ClothFEMImpl::ComputeBendStiffness()
{
  bendStiffness.assign(quads.size(), 1.0);
  for (size_t qi = 0; qi < quads.size(); qi++) 
  {
    double E = (bendMaterials[triangleBendMaterialIDs[quads[qi].tri[0]]].E +
                bendMaterials[triangleBendMaterialIDs[quads[qi].tri[1]]].E) * 0.5;
    double nu = (bendMaterials[triangleBendMaterialIDs[quads[qi].tri[0]]].nu +
                 bendMaterials[triangleBendMaterialIDs[quads[qi].tri[1]]].nu) * 0.5;
    double h = (bendMaterials[triangleBendMaterialIDs[quads[qi].tri[0]]].h +
                bendMaterials[triangleBendMaterialIDs[quads[qi].tri[1]]].h) * 0.5;
    bendStiffness[qi] = ComputeBendStiffness(E, nu, h);
  }
}

double ClothFEMImpl::ComputePhiTheta(const Vec3d & n1, const Vec3d & n2, const Vec3d & e0) const
{
  double sinHalfTheta = len(n1 - n2);
  double cosHalfTheta = len(n1 + n2);

  double signNumber = dot(cross(n1, n2), e0);
  if (signNumber > 0.0)
    return sinHalfTheta / cosHalfTheta * 2.0;
  else
    return sinHalfTheta / cosHalfTheta * -2.0;
}

double ClothFEMImpl::Computeh(const Vec3d & p0, const Vec3d & p1, const Vec3d & p2, Vec3d & m) const
{
  // (t (p2 - p1) + p1 - p0) (p2 - p1) = 0
  Vec3d e21 = p2 - p1;
  Vec3d e10 = p1 - p0;
  double A = dot(e21, e21);
  double b = -dot(e10, e21);
  double t = b / A;

  m = e21 * t + e10;
  double L = len(m);
  m /= L;

  return L;
}

void ClothFEMImpl::ComputeBendTerms(const double *u, int quadId, double *energy, double f[12], double K[144]) const
{
  int qua = quadId;

  int p0 = quads[qua].v[0];
  int p1 = quads[qua].v[1];
  int p2 = quads[qua].v[2];
  int p3 = quads[qua].v[3];

  // vtx indices defined below
  //
  //       1---------3
  //      / \       /
  //     /   \  B  /
  //    /  A  \   /
  //   /       \ /
  //  0---------2

  Vec3d pos0 = restPositions[p0] + Vec3d(&u[3 * p0]);
  Vec3d pos1 = restPositions[p1] + Vec3d(&u[3 * p1]);
  Vec3d pos2 = restPositions[p2] + Vec3d(&u[3 * p2]);
  Vec3d pos3 = restPositions[p3] + Vec3d(&u[3 * p3]);

  // In the following, we re-number all variables to make them identical to "Tamstorf and Grinspun 2013, Discrete bending forces and their Jacobians".
  Vec3d x0 = pos0;
  Vec3d x1 = pos2;
  Vec3d x2 = pos1;
  Vec3d x3 = pos3;

  Vec3d n = cross(pos2 - pos0, pos1 - pos0);
  Vec3d n_tilde = cross(pos1 - pos3, pos2 - pos3);
  n.normalize();
  n_tilde.normalize();

  Vec3d e0 = x2 - x1;
  Vec3d e1 = x0 - x2;
  Vec3d e2 = x0 - x1;
  Vec3d e1_tilde = x3 - x2;
  Vec3d e2_tilde = x3 - x1;

  double phiTheta = ComputePhiTheta(n, n_tilde, e0);
  double phiTheta_bar = restPhiTheta[qua];

  if (useRestAnglesForBendForces == 0)
    phiTheta_bar = 0;

  if (energy) 
  {
    // bending energy = k a_i ( phi(theta) - phi(theta_bar) )^2 * 0.5
    *energy = bendStiffness[qua] * restai[qua] * (phiTheta - phiTheta_bar) * (phiTheta - phiTheta_bar) * 0.5;
  }

  if (f || K) 
  {
    Vec3d m[3], m_tilde[3];

    double h[3] = 
    {
      Computeh(x0, x1, x2, m[0]),
      Computeh(x1, x0, x2, m[1]),
      Computeh(x2, x0, x1, m[2])
    };

    double h_tilde[3] = 
    {
      Computeh(x3, x1, x2, m_tilde[0]),
      Computeh(x1, x3, x2, m_tilde[1]),
      Computeh(x2, x3, x1, m_tilde[2])
    };

    double e_dot[3] = 
    {
      dot(e0, e0),
      dot(e1, e1),
      dot(e2, e2),
    };

    double e_dot_tilde[3] = 
    {
      e_dot[0],
      dot(e1_tilde, e1_tilde),
      dot(e2_tilde, e2_tilde)
    };

    double e_len[3] = 
    {
      sqrt(e_dot[0]),
      sqrt(e_dot[1]),
      sqrt(e_dot[2])
    };

    double e_len_tilde[3] = 
    {
      e_len[0],
      sqrt(e_dot_tilde[1]),
      sqrt(e_dot_tilde[2])
    };

    double cos_alpha[3] = 
    {
      dot(e1, e2) / (e_len[1] * e_len[2]),
      dot(e0, e2) / (e_len[0] * e_len[2]),
      -dot(e0, e1) / (e_len[0] * e_len[1])
    };

    double cos_alpha_tilde[3] = 
    {
      dot(e1_tilde, e2_tilde) / (e_len_tilde[1] * e_len_tilde[2]),
      dot(e0, e2_tilde) / (e_len_tilde[0] * e_len_tilde[2]),
      -dot(e0, e1_tilde) / (e_len_tilde[0] * e_len_tilde[1])
    };

    // phi (theta) = 2 * tan (theta / 2)
    // dphi / dtheta = 1 / cos^2 (theta / 2) = 1.0 / (|| n + n_tilde || * 0.5)^2 = 4.0 / || n + n_tilde ||^2
    // double cosHalfTheta = sqrt(dot(n + n_tilde, n + n_tilde)) * 0.5;
    // double dphi_dtheta = 1.0 / (cosHalfTheta * cosHalfTheta);
    double dphi_dtheta = 4.0 / dot(n + n_tilde, n + n_tilde);

    // dE / dphi = k ai (phi - phi_bar)
    double dE_dphi = bendStiffness[qua] * restai[qua] * (phiTheta - phiTheta_bar);
    double dE_dtheta = dE_dphi * dphi_dtheta;

    Vec3d dtheta_dx0 = -1.0 / h[0] * n;
    Vec3d dtheta_dx1 = cos_alpha[2] / h[1] * n + cos_alpha_tilde[2] / h_tilde[1] * n_tilde;
    Vec3d dtheta_dx2 = cos_alpha[1] / h[2] * n + cos_alpha_tilde[1] / h_tilde[2] * n_tilde;
    Vec3d dtheta_dx3 = -1.0 / h_tilde[0] * n_tilde;

    double dtheta_du[12] = 
    {
      dtheta_dx0[0], dtheta_dx0[1], dtheta_dx0[2],
      dtheta_dx2[0], dtheta_dx2[1], dtheta_dx2[2],
      dtheta_dx1[0], dtheta_dx1[1], dtheta_dx1[2],
      dtheta_dx3[0], dtheta_dx3[1], dtheta_dx3[2]
    };

    if (f) 
    {
      // f = (dtheta / d_u) * (dE / dtheta)
      for (int i = 0; i < 12; i++)
        f[i] = dtheta_du[i] * dE_dtheta;
      // cblas_daxpy(12, 1.0 * dE_dtheta, dtheta_du, 1, f, 1);
    }

    if (K) 
    {
      Mat3d N[3], M[3];
      Mat3d N_tilde[3], M_tilde[3];
      for (int i = 0; i < 3; i++) 
      {
        M[i] = tensorProduct(n, m[i]);
        M_tilde[i] = tensorProduct(n_tilde, m_tilde[i]);

        N[i] = M[i] / e_dot[i];
        N_tilde[i] = M_tilde[i] / e_dot_tilde[i];
      }

      Mat3d P[3][3], P_tilde[3][3];
      Mat3d Q[3], Q_tilde[3];
      double w[3][3], w_tilde[3][3];

      std::swap(cos_alpha[1], cos_alpha[2]);
      std::swap(cos_alpha_tilde[1], cos_alpha_tilde[2]);

      for (int i = 0; i < 3; i++) 
      {
        for (int j = 0; j < 3; j++) 
        {
          w[i][j] = 1.0 / (h[i] * h[j]);
          w_tilde[i][j] = 1.0 / (h_tilde[i] * h_tilde[j]);

          P[i][j] = w[i][j] * cos_alpha[i] * trans(M[j]);
          P_tilde[i][j] = w_tilde[i][j] * cos_alpha_tilde[i] * trans(M_tilde[j]);
        }
      }

      for (int j = 0; j < 3; j++) 
      {
        Q[j] = w[0][j] * M[j];
        Q_tilde[j] = w_tilde[0][j] * M_tilde[j];
      }

      // #define S(A) (A + trans(A))
      auto S = [](const Mat3d &A) -> Mat3d 
      {
        return A + trans(A);
      };

      Mat3d H_theta[4][4];
      H_theta[0][0] = -1.0 * S(Q[0]);
      H_theta[1][1] = S(P[1][1]) - N[0] + S(P_tilde[1][1]) - N_tilde[0];
      H_theta[2][2] = S(P[2][2]) - N[0] + S(P_tilde[2][2]) - N_tilde[0];
      H_theta[3][3] = -1.0 * S(Q_tilde[0]);

      H_theta[1][0] = P[1][0] - Q[1];
      H_theta[1][2] = P[1][2] + trans(P[2][1]) + N[0] + P_tilde[1][2] + trans(P_tilde[2][1]) + N_tilde[0];
      H_theta[1][3] = P_tilde[1][0] - Q_tilde[1];

      H_theta[2][0] = P[2][0] - Q[2];
      H_theta[2][3] = P_tilde[2][0] - Q_tilde[2];

      Mat3d zeroM(0, 0, 0, 0, 0, 0, 0, 0, 0);
      H_theta[3][0] = zeroM;

      H_theta[0][1] = trans(H_theta[1][0]);
      H_theta[0][2] = trans(H_theta[2][0]);
      H_theta[0][3] = trans(H_theta[3][0]);

      H_theta[2][1] = trans(H_theta[1][2]);

      H_theta[3][1] = trans(H_theta[1][3]);
      H_theta[3][2] = trans(H_theta[2][3]);

      // dE / dtheta * H_theta
      int idxM[4] = { 0, 2, 1, 3 };
      for (int i = 0; i < 4; i++) 
      {
        int truei = idxM[i];
        for (int j = 0; j < 4; j++) 
        {
          int truej = idxM[j];

          for (int r = 0; r < 3; r++) 
          {
            for (int c = 0; c < 3; c++) 
            {
              int row = i * 3 + r;
              int col = j * 3 + c;
              K[col * 12 + row] = H_theta[truei][truej][r][c] * dE_dtheta;
            }  // end c
          }    // end r
        }      // end j
      }        // end i

      // d^2 E / dtheta^2 * dtheta_du^T * dtheta_du
      double ddE_dphi2 = bendStiffness[qua] * restai[qua];
      // ddphi / dtheta2 = 1.0 / cos^2(theta/2) * tan (theta/2)
      //                         = (dphi / dtheta) * (0.5 * phiTheta)
      double ddphi_dtheta2 = dphi_dtheta * (0.5 * phiTheta);
      double ddE_dtheta2 = ddE_dphi2 * dphi_dtheta * dphi_dtheta + dE_dphi * ddphi_dtheta2;
      for (int i = 0; i < 12; i++) 
      {
        for (int j = 0; j < 12; j++) 
        {
          K[j * 12 + i] += ddE_dtheta2 * dtheta_du[i] * dtheta_du[j];
        }
      }
    }  // end K
  }
}

void ClothFEMImpl::ComputeShearAndStretchTerms(int triIdx, const double *u, double *energy, double f[9], double K[81]) const
{
  // group index
  const int group = triangleStretchAndShearMaterialIDs[triIdx];
  // const int *vtxInd = triangles.data() + 3 * triIdx;

  // triangle indices, clockwise as A, B, C
  const int particleA = triangles[triIdx][0];
  const int particleB = triangles[triIdx][1];
  const int particleC = triangles[triIdx][2];

  //--- compute C(x) for Stretch and Shear
  Vec3d positionA = restPositions[particleA] + Vec3d(u + 3 * particleA);
  Vec3d positionB = restPositions[particleB] + Vec3d(u + 3 * particleB);
  Vec3d positionC = restPositions[particleC] + Vec3d(u + 3 * particleC);
  // Vec3d normal = cross(positionB - positionA, positionC - positionA);

  const double * triru = ru.data() + 3 * triIdx;
  const double * trirv = rv.data() + 3 * triIdx;

  Vec3d U = triru[0] * positionA + triru[1] * positionB + triru[2] * positionC;
  Vec3d V = trirv[0] * positionA + trirv[1] * positionB + trirv[2] * positionC;

  // compute in-plane strain: \epsilon_uu, \epsilon_vv, \epsilon_uv
  double strain[3], stress[3];
  strain[0] = (len2(U) - 1) / 2.0;
  strain[1] = (len2(V) - 1) / 2.0;
  strain[2] = dot(U, V);

  double e = stretchAndShearMaterials[group].E;
  double nu = stretchAndShearMaterials[group].nu;

  //linear strain-stress relationship
  //         e         [ 1 nu    0     ]
  // E =  --------     [nu  1    0     ]
  //      (1-nu^2)     [ 0  0 (1-nu)/2 ]
  double commonTerm = e / (1.0 - nu * nu);
  double E11 = commonTerm;
  double E12 = nu * commonTerm;
  double E33 = commonTerm * (1.0 - nu) / 2.0;

  // compute in-plane stress: \sigma_uu, \sigma_vv, \sigma_uv
  stress[0] = commonTerm * (strain[0] + nu * strain[1]);
  stress[1] = commonTerm * (nu * strain[0] + strain[1]);
  stress[2] = commonTerm * strain[2] * (1.0 - nu) / 2.0;

  // F_j = -(triangleArea) * (\sigma_uu * r_uj * U + \sigma_vv * r_vj * V + \sigma_uv * (r_uj * V + r_vj * U))
  // F_j = -(triangleArea) * ( (\sigma_uu * r_uj + \sigma_uv * r_vj) * U + (\sigma_vv * r_vj + \sigma_uv * r_uj) * V )

  double area = restTriangleAreas[triIdx];

  if (energy) 
  {
    Mat3d EMat(E11, E12, 0, E12, E11, 0, 0, 0, E33);
    Vec3d strainVec(strain);
    Vec3d Eeps = EMat * strainVec;
    *energy = area * dot(Eeps, strainVec) * 0.5;
  }

  if (f) 
  {
    for (int j = 0; j < 3; j++) 
    {
      Vec3d force = area * ((stress[0] * triru[j] + stress[2] * trirv[j]) * U + (stress[1] * trirv[j] + stress[2] * triru[j]) * V);
      f[3 * j] = force[0];
      f[3 * j + 1] = force[1];
      f[3 * j + 2] = force[2];
    }
  }

  if (K) 
  {
    Mat3d UUT = tensorProduct(U, U);
    Mat3d VVT = tensorProduct(V, V);
    Mat3d UVT = tensorProduct(U, V);
    Mat3d VUT = trans(UVT);
    Mat3d I(1.0);

    for (int j = 0; j < 3; j++) 
    {
      for (int i = 0; i < 3; i++) 
      {
        Mat3d pFjpPi = (E11 * triru[j] * triru[i] + E33 * trirv[j] * trirv[i]) * UUT + (E11 * trirv[j] * trirv[i] + E33 * triru[j] * triru[i]) * VVT + (E12 * triru[j] * trirv[i] + E33 * trirv[j] * triru[i]) * UVT + (E12 * trirv[j] * triru[i] + E33 * triru[j] * trirv[i]) * VUT + (stress[0] * triru[j] * triru[i] + stress[1] * trirv[j] * trirv[i] + stress[2] * (trirv[j] * triru[i] + triru[j] * trirv[i])) * I;

        pFjpPi *= area;
        for (int kk = 0; kk < 3; kk++) 
        {
          for (int ll = 0; ll < 3; ll++) 
          {
            int row = j * 3 + kk;
            int col = i * 3 + ll;
            K[col * 9 + row] = pFjpPi[kk][ll];
          }  // end for each ll
        }    // end for each kk
      }      // end for each i
    }        // end for each j
  }
}

ClothFEM::ClothFEM(int numVertices_, const double * restPositions_, const double * masses_,
  int numTriangles_, const int * triangles_, 
  const int * triangleStretchAndShearMaterialIDs_,
  const int * triangleBendMaterialIDs_,
  int numStretchAndShearMaterials_, 
  const StretchAndShearMaterial * stretchAndShearMaterials_,
  int numBendMaterials_, 
  const BendMaterial * bendMaterials_)
{
  impl = new ClothFEMImpl(numVertices_, restPositions_, masses_,
    numTriangles_, triangles_, 
    triangleStretchAndShearMaterialIDs_,
    triangleBendMaterialIDs_,
    numStretchAndShearMaterials_, stretchAndShearMaterials_, 
    numBendMaterials_, bendMaterials_);
}

ClothFEM::ClothFEM(int numVertices_, const double * restPositions_, const double * masses_,
  int numTriangles_, const int * triangles_, const double * triangleUVs_,
  const int * triangleStretchAndShearMaterialIDs_,
  const int * triangleBendMaterialIDs_,
  int numStretchAndShearMaterials_,
  const StretchAndShearMaterial * stretchAndShearMaterials_,
  int numBendMaterials_,
  const BendMaterial * bendMaterials_)
{
  impl = new ClothFEMImpl(numVertices_, restPositions_, masses_,
    numTriangles_, triangles_, triangleUVs_,
    triangleStretchAndShearMaterialIDs_,
    triangleBendMaterialIDs_,
    numStretchAndShearMaterials_, stretchAndShearMaterials_, 
    numBendMaterials_, bendMaterials_);
}

ClothFEM::~ClothFEM()
{
  delete impl;
}

int ClothFEM::GetNumVertices() const
{
  return static_cast<int>(impl->restPositions.size());
}
int ClothFEM::GetNumTriangles() const
{
  return static_cast<int>(impl->triangles.size());
}

int ClothFEM::GetNumQuads() const
{
  return static_cast<int>(impl->quads.size());
}

int ClothFEM::GetNumStretchAndShearMaterials() const
{
  return static_cast<int>(impl->stretchAndShearMaterials.size());
}

int ClothFEM::GetNumBendMaterials() const
{
  return static_cast<int>(impl->bendMaterials.size());
}

const Vec3i * ClothFEM::GetTriangles() const
{
  return impl->triangles.data();
}

const Vec3d * ClothFEM::GetRestPositions() const
{
  return impl->restPositions.data();
}

const int * ClothFEM::GetTriangleVertexIndices(int tri) const
{
  return impl->triangles[tri].data();
}

const int * ClothFEM::GetQuadVertexIndices(int quad) const
{
  return impl->quads[quad].v;
}

const int * ClothFEM::GetQuadTriangleIndices(int quad) const
{
  return impl->quads[quad].tri;
}

void ClothFEM::UseRestAnglesForBendForces(bool useRestAnglesForBendForces_)
{
  impl->useRestAnglesForBendForces = useRestAnglesForBendForces_ ? 1 : 0;
}

void ClothFEM::SetBendMaterial(double E, double nu, double h)
{
  BendMaterial bendMaterial(E, nu, h);
  SetBendMaterial(&bendMaterial);
}

void ClothFEM::SetBendMaterial(const BendMaterial * bendMaterial)
{
  // Set single material.
  impl->bendMaterials.clear();
  impl->bendMaterials.push_back(*bendMaterial);
  impl->triangleBendMaterialIDs.assign(impl->triangleBendMaterialIDs.size(), 0);
  impl->ComputeBendStiffness();
}

void ClothFEM::SetStretchAndShearMaterial(double E, double nu)
{
  StretchAndShearMaterial stretchAndShearMaterial(E, nu);
  SetStretchAndShearMaterial(&stretchAndShearMaterial);
}

void ClothFEM::SetStretchAndShearMaterial(const StretchAndShearMaterial * stretchAndShearMaterial)
{
  impl->stretchAndShearMaterials.clear();
  impl->stretchAndShearMaterials.push_back(*stretchAndShearMaterial);
  impl->triangleStretchAndShearMaterialIDs.assign(impl->triangleStretchAndShearMaterialIDs.size(), 0);
}

void ClothFEM::ComputeBendTerms(int quad, const double * u, double * E, double f[12], double K[144]) const
{
  impl->ComputeBendTerms(u, quad, E, f, K);
}

void ClothFEM::ComputeShearAndStretchTerms(int tri, const double * u, double * E, double f[9], double K[81]) const
{
  impl->ComputeShearAndStretchTerms(tri, u, E, f, K);
}

void ClothFEM::GenerateMassMatrix(SparseMatrix ** M, int expanded) const
{
  int nv = static_cast<int>(impl->restPositions.size());
  SparseMatrixOutline outline(expanded * nv);
  for (int i = 0; i < nv; i++)
    for (int j = 0; j < expanded; j++)
      outline.AddEntry(expanded * i + j, expanded * i + j, impl->masses[i]);
  *M = new SparseMatrix(&outline);
}

