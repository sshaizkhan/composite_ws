#include "clothFEM.h"
#include "objMesh.h"
#include "constraints.h"
#include "clothFEMStencilForceModel.h"
#include "forceModelAssembler.h"
#include "matrixIO.h"
#include "implicitEulerMultiObjectWithConstraintsSparse.h"

#include <memory>
#include <vector>
#include <chrono>
#include <numeric>

int main()
{
  const char meshName[] = "1.obj";
  const char resultFolder[] = "ret";

  ObjMesh mesh(meshName);

  double *vertices;
  int numVeritces;
  int *triangles;
  int numTriangles;
  mesh.exportGeometry(&numVeritces, &vertices, &numTriangles, &triangles);

  double density = 0.5;
  std::vector<double> masses(numVeritces);
  std::vector<double> groupSurfaceDensity(mesh.getNumGroups(), density);
  mesh.computeMassPerVertex(groupSurfaceDensity, masses);

  std::vector<int> triangleStretchAndShearMaterialIDs(numTriangles, 0), triangleBendMaterialIDs(numTriangles, 0);
  ClothFEM::StretchAndShearMaterial ssm;
  ssm.E = 3.3e4;
  ssm.nu = 0.3;

  ClothFEM::BendMaterial bm;
  bm.E = 1.1e8;
  bm.h = 0.0003;
  bm.nu = 0.3;

  ClothFEM fem(numVeritces, vertices, masses.data(), numTriangles, triangles,
    triangleStretchAndShearMaterialIDs.data(), triangleBendMaterialIDs.data(),
    1, &ssm, 1, &bm);

  int numObjects = 1;
  int n3 = numVeritces * 3;
  Constraints constraints(numObjects, &n3, vertices);

  double timestep = 0.001;
  SparseMatrix *M;
  fem.GenerateMassMatrix(&M, 3);

  ClothFEMStencilForceModel femForceModel(&fem);
  femForceModel.EnableKFilter(1);

  ForceModelAssembler fma(&femForceModel);

  ForceModel *forceModels[1] = { &fma };
  int numConstrainedDOFs[1] = { 0 };
  int *constraiendDOFs[1] = { nullptr };
  ImplicitEulerMultiObjectWithConstraintsSparse integrator(numObjects, &n3, timestep, &M, forceModels, numConstrainedDOFs, constraiendDOFs, 0.0, 0.02);

  int fixedSet[4][4] = {
    { 273, 3235, 3236, 4163 },   // A
    { 79, 4216, 8301, 11896 },   // B
    { 259, 3141, 3142, 4106 },   // C
    { 4080, 4089, 7401, 7457 },  // D
  };

  Vec3d positions[3][4] = {
    {
      { 0.725785, 0, 0.082555 },  // A0
      { 0.603548, 0, 1.21175 },   // B0
      { -0.23684, 0, 0.977905 },  // C0
      { 0, 0, 0 },                // D0
    },
    {
      { 0.709837, 0.032857, 0.127454 },   // A1
      { 0.509687, 0.01593, 1.257482 },    // B1
      { -0.313584, -0.015563, 0.97006 },  // C1
      { 0.0, 0.0, 0.0 },                  // D1
    },
    {
      { 0.567085, 0.1330165, 0.237639 },  // A2
      { 0.456382, 0.2627485, 1.11211 },   // B2
      { -0.219983, 0.2597755, 0.93504 },  // C2
      { 0.0, 0.0, 0.0 },                  // D2
    }
  };

  Vec3d dirs[2][4] = {
    {
      positions[1][0] - positions[0][0],
      positions[1][1] - positions[0][1],
      positions[1][2] - positions[0][2],
      positions[1][3] - positions[0][3],
    },
    {
      positions[2][0] - positions[1][0],
      positions[2][1] - positions[1][1],
      positions[2][2] - positions[1][2],
      positions[2][3] - positions[1][3],
    },
  };

  int gcounter = 0;

  std::vector<double> u(n3, 0);
  std::vector<double> durs;

  std::vector<double> fext(n3, 0);

  for (int i = 0; i < n3 / 3; i++)
    fext[i * 3 + 1] = -masses[i] * 9.81;

  //integrator.SetExternalForces(fext.data());

  auto runOneMotion = [&](const double dt[4], int stage, int numTimesteps) {
    for (int i = 0; i < numTimesteps; i++) {
      std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

      constraints.Reset();

      // for each set
      for (int seti = 0; seti < 4; seti++) {
        // for each vertex in the set
        for (int vi = 0; vi < 4; vi++) {
          Vec3d curu(u.data() + fixedSet[seti][vi] * 3);
          Vec3d restp(vertices + fixedSet[seti][vi] * 3);
          Vec3d curp = restp + curu;
          Vec3d newp = curp + dirs[stage][seti] * dt[seti];
          Vec3d newu = newp - restp;

          constraints.AddFixedConstraint(0, fixedSet[seti][vi], newp.data());
          //u[fixedSet[seti][vi] * 3 + 0] = newu[0];
          //u[fixedSet[seti][vi] * 3 + 1] = newu[1];
          //u[fixedSet[seti][vi] * 3 + 2] = newu[2];
        }
      }

      constraints.FinalizeConstraints();
      integrator.SetConstraintMatrix(constraints.GetConstraintMatrix());
      integrator.SetConstraintRhs(constraints.GetConstraintRhs());
      integrator.DoTimestep();
      memcpy(u.data(), integrator.Getq(), sizeof(double) * n3);

      std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

      durs.push_back(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());

      if (gcounter % 50 == 0) {
        char filename[64];
        sprintf(filename, "%s/f%05d.obj", resultFolder, gcounter / 50);
        //WriteMatrixToDisk(filename, n3, 1, u.data());

        ObjMesh outputMesh = mesh;
        for (int i = 0; i < mesh.getNumVertices(); i++) {
          outputMesh.setPosition(i, mesh.getPosition(i) + Vec3d(u.data() + i * 3));
        }
        outputMesh.save(filename);
        std::cout << gcounter << ' ' << std::flush;
      }

      gcounter++;
    }
  };

  double dt = timestep * 2;
  int numTimestepsPerSecond = (int)(1.0 / dt);

  // not sure why there is a 2.0 there, but the source code has
  double dt1[4] = { dt, dt, dt, dt };
  runOneMotion(dt1, 0, numTimestepsPerSecond);

  //std::cout << std::endl;
  //std::cout << std::accumulate(durs.begin(), durs.end(), 0) / (double)durs.size() << std::endl;

  double dt2[4] = { dt, 0, 0, 0 };
  runOneMotion(dt2, 1, numTimestepsPerSecond);

  double dt3[4] = { 0, dt, 0, 0 };
  runOneMotion(dt3, 1, numTimestepsPerSecond);

  double dt4[4] = { 0, 0, dt, 0 };
  runOneMotion(dt4, 1, numTimestepsPerSecond);

  for (int i = 0; i < 5; i++) {
    double dt5[4] = { 0, 0, 0, 0 };
    runOneMotion(dt5, 1, numTimestepsPerSecond);
  }

  std::cout << std::endl;
  std::cout << std::accumulate(durs.begin(), durs.end(), 0) / (double)durs.size() << std::endl;

  return 0;
}
