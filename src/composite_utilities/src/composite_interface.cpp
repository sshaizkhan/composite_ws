#include "composite_utilities/composite_interface.hpp"
#include <iostream>
#include <string>

using namespace std;

const int nIter = 1;


Composite::Composite(bool fem, bool debug):m_dampingMass(0.00),
m_dampingStiffness(0.01),m_surfaceDensity(0.047),
m_tensileStiffness(5000), m_shearStiffness(5000),
m_bendStiffnessU(0.001),m_bendStiffnessV(0.001),
m_sheet_thickness(0.001),m_sheet_poisson(0.3),m_shear_stretch_Material(1e5),m_bend_Material(1e10),
m_clothBW(NULL),m_clothFEM(NULL),m_sceneObjDeform(NULL), m_clothBWStencilForceModel(NULL), m_clothFEMStencilForceModel(NULL),
m_forceModelAssembler(NULL),m_objMesh(NULL), m_constraints(NULL),
m_clothBWFromObjMesh(NULL),m_implicitEulerMultiObjectWithConstraintsSparse(NULL),
m_forceModel(NULL),m_massMatrix(NULL),m_FEM_ON(fem),m_DEBUG(debug)
{
  if(m_FEM_ON)
    cout <<"======ClothFEM Selected======\n";
  else
    cout <<"======ClothBW Selected======\n";
}


void Composite::getParameters(){
  if(m_FEM_ON){
      cout << "FEM dampingMass: "<<m_dampingMass<<endl;
      cout << "FEM dampingStiffness: "<<m_dampingStiffness<<endl;
      cout << "FEM surfaceDensity: "<<m_surfaceDensity<<endl;
      cout << "FEM thickness: "<<m_sheet_thickness<<endl;
      cout << "FEM poisson's ratio: "<<m_sheet_poisson<<endl;
      cout << "FEM Shear and Stretch Material: "<<m_shear_stretch_Material<<endl;
      cout << "FEM bendMaterial: "<<m_bend_Material<<endl;
  }
  else{
      cout << "dampingMass: "<<m_dampingMass<<endl;
      cout << "dampingStiffness: "<<m_dampingStiffness<<endl;
      cout << "surfaceDensity: "<<m_surfaceDensity<<endl;
      cout << "tensileStiffness: "<<m_tensileStiffness<<endl;
      cout << "shearStiffness: "<<m_shearStiffness<<endl;
      cout << "bendStiffnessU: "<<m_bendStiffnessU<<endl;
      cout << "bendStiffnessV: "<<m_bendStiffnessV<<endl;
  }
}
void Composite::updateParam(const float* p){
  if(m_FEM_ON){
      m_shear_stretch_Material = p[0];
      m_bend_Material = p[1];

  }
  else{
    // cout <<"NO FEM\n";
      m_tensileStiffness = p[0];
      m_shearStiffness = p[1];
      m_bendStiffnessU = p[2];
      m_bendStiffnessV = p[2];
  }
      m_dampingMass = 0.0;
      m_dampingStiffness = 0.1;

}
void Composite::updateParam(const vector<double>& p){
  if(m_FEM_ON){
      m_shear_stretch_Material = p[0];
      m_bend_Material = p[1];
      m_dampingStiffness = p[2];
      cout << "Alec Updated SHEAR STIFFNESS:  " << m_shear_stretch_Material << endl;
      cout << "Alec Updated Bend Material:  " << m_bend_Material << endl;
      // cout << "Alec damping stiffness:  " << m_dampingStiffness << endl;

  }
  else{
    // cout <<"NO FEM\n"
    // cout << p[0] <<endl;
    // cout << p[1] <<endl;
    // cout << p[2] <<endl;
      m_tensileStiffness = p[0];
      m_shearStiffness = p[1];
      m_bendStiffnessU = p[2];
      m_bendStiffnessV = p[2];
      cout << "SHEAR STIFFNESS:  " << m_shearStiffness << endl;
      cout << "BEND STIFFNESS:  " << m_bendStiffnessU << endl;
  }
      m_dampingMass = 0.0;
      // m_dampingStiffness = 0.01;
}

void Composite::updateConfig(const char* objMeshname, const char* robot_data){
  if (m_objMesh != NULL)  
      delete m_objMesh;
    m_objMesh = new ObjMesh(objMeshname);

    if(m_fixGroups.size() != 0)
      m_fixGroups.clear();

    if(robot_data != NULL)
      parseRobotData(robot_data,m_objMesh, m_grasping_points,m_fixGroups);

    std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
    m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
    m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
    assert(m_masses.size() == m_numVertices);
    m_numDOFs = 3 * m_numVertices;

    // cout <<"Mesh loaded\n";
    // updateConstrain();

}

void Composite::updateConfig(const char* objMeshname, const std::vector<Vec3d>& robot_data){

    if (m_objMesh != NULL)  
      delete m_objMesh;
    m_objMesh = new ObjMesh(objMeshname);

    if(m_fixGroups.size() != 0)
      m_fixGroups.clear();

    parseRobotData(m_objMesh, robot_data,m_fixGroups);

    std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
    m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
    m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
    assert(m_masses.size() == m_numVertices);
    m_numDOFs = 3 * m_numVertices;
    if(m_DEBUG)
      cout <<"Mesh loaded\n";
}

void Composite::updateConfig(int numVertices, const double * vertices, int numTriangles, const int * triangles, const char* robot_data){
  if (m_objMesh != NULL)  
      delete m_objMesh;
    m_objMesh = new ObjMesh(numVertices, vertices, numTriangles, triangles);

    if(m_fixGroups.size() != 0)
      m_fixGroups.clear();

    if(robot_data != NULL)
      parseRobotData(robot_data,m_objMesh, m_grasping_points,m_fixGroups);

    std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
    m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
    m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
    assert(m_masses.size() == m_numVertices);
    m_numDOFs = 3 * m_numVertices;
    // updateConstrain();

}

void Composite::updateConfig(int numVertices, const double * vertices, int numTriangles, const int * triangles, const std::vector<Vec3d>& robot_data){
    if (m_objMesh != NULL)  
      delete m_objMesh;
    m_objMesh = new ObjMesh(numVertices, vertices, numTriangles, triangles);

    if(m_fixGroups.size() != 0)
      m_fixGroups.clear();

    parseRobotData(m_objMesh, robot_data,m_fixGroups);

    std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
    m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
    m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
    assert(m_masses.size() == m_numVertices);
    m_numDOFs = 3 * m_numVertices;
    if(m_DEBUG)
      cout <<"Mesh loaded\n";
    // updateConstrain();
}


void Composite::generateSheet(){

  // if (m_clothBWFromObjMesh != NULL)
  //     delete m_clothBWFromObjMesh;
  //   m_clothBWFromObjMesh = new ClothBWFromObjMesh();
  //   if (m_clothBW != NULL)
  //    delete m_clothBW;
  // // getParameters();
  //   ClothBW::MaterialGroup material;
  //   material.tensileStiffness = m_tensileStiffness;
  //   material.shearStiffness = m_shearStiffness;
  //   material.bendStiffnessU = m_bendStiffnessU;
  //   material.bendStiffnessV = m_bendStiffnessV;
  //   m_clothBW = m_clothBWFromObjMesh->GenerateClothBW(m_objMesh, m_surfaceDensity, material);
  //   // clothBW->SetGravity(1,gravityForce);
  //   // std::cout << gravityForce<<endl;
  //   if(m_massMatrix != NULL)
  //    delete m_massMatrix;
  //   m_clothBW->GenerateMassMatrix(&m_massMatrix);
  //   double totalMass = m_massMatrix->SumEntries() / 3.0;
 //    if(m_DEBUG)
  //      printf("Total cloth mass: %G\n", totalMass);
    if(m_FEM_ON){
    generateFEM();
    }
    else{
      generateBW();
        // m_clothBWStencilForceModel = new ClothBWStencilForceModel(m_clothBW);
        // m_forceModelAssembler = new ForceModelAssembler(m_clothBWStencilForceModel);
    }

    m_forceModel = m_forceModelAssembler; 
}

void Composite::generateFEM(){
  if (m_clothFEM)
    delete m_clothFEM;
  std::vector<int> triangleStretchAndShearMaterialIDs(m_numTriangles, 0); // assign each triangle to the first (and only) material group
  std::vector<int> triangleBendMaterialIDs(m_numTriangles, 0); // assign each triangle to the first (and only) material group
  const int numStretchAndShearMaterials = 1;
  ClothFEM::StretchAndShearMaterial stretchAndShearMaterial(m_shear_stretch_Material, m_sheet_poisson); // the stretch and shear material (E = 10^5 N / m, nu = 0.45)

  const int numBendMaterials = 1;
  ClothFEM::BendMaterial bendMaterial(m_bend_Material, m_sheet_poisson, m_sheet_thickness);// the bend material (E = 10^10 N / m^2, nu = 0.45, thickness = 1mm)
  m_clothFEM = new ClothFEM(m_numVertices, m_restVertexPositions, m_masses.data(),
    m_numTriangles, m_triangleVertexIndices, 
    triangleStretchAndShearMaterialIDs.data(), triangleBendMaterialIDs.data(),
    numStretchAndShearMaterials, &stretchAndShearMaterial,
    numBendMaterials, &bendMaterial);
  m_clothFEM->UseRestAnglesForBendForces(false);
  // cout <<"[clothFEM create]\n";
  if(m_massMatrix != NULL)
      delete m_massMatrix;
    m_clothFEM->GenerateMassMatrix(&m_massMatrix);
    double totalMass = m_massMatrix->SumEntries() / 3.0;
  m_clothFEMStencilForceModel = new ClothFEMStencilForceModel(m_clothFEM);
  m_clothFEMStencilForceModel->EnableKFilter(1);
  // cout << "step1\n";
  m_forceModelAssembler = new ForceModelAssembler(m_clothFEMStencilForceModel);
  // cout <<"[ForceModel create]\n";

}

void Composite::generateBW(){
  if (m_clothBWFromObjMesh != NULL)
      delete m_clothBWFromObjMesh;
    m_clothBWFromObjMesh = new ClothBWFromObjMesh();
    if (m_clothBW != NULL)
     delete m_clothBW;
  // getParameters();
    ClothBW::MaterialGroup material;
    material.tensileStiffness = m_tensileStiffness;
    material.shearStiffness = m_shearStiffness;
    material.bendStiffnessU = m_bendStiffnessU;
    material.bendStiffnessV = m_bendStiffnessV;
    m_clothBW = m_clothBWFromObjMesh->GenerateClothBW(m_objMesh, m_surfaceDensity, material);
    // clothBW->SetGravity(1,gravityForce);
    // std::cout << gravityForce<<endl;
    if(m_massMatrix != NULL)
      delete m_massMatrix;
    m_clothBW->GenerateMassMatrix(&m_massMatrix);
    double totalMass = m_massMatrix->SumEntries() / 3.0;
            m_clothBWStencilForceModel = new ClothBWStencilForceModel(m_clothBW);
        m_forceModelAssembler = new ForceModelAssembler(m_clothBWStencilForceModel);
}

void Composite::updateConstrain(){

 int numFixedVertices = m_fixGroups.size();
  
  m_numConstrainedDOFs = 3*numFixedVertices;
  if (m_constrainedDOFs)
    free(m_constrainedDOFs);
  m_constrainedDOFs = (int*) malloc (sizeof(int) * m_numConstrainedDOFs);
  for (int i=0; i<numFixedVertices; i++)
  {
    m_constrainedDOFs[3*i+0] = m_fixGroups[i] * 3 + 0;
    m_constrainedDOFs[3*i+1] = m_fixGroups[i] * 3 + 1;
    m_constrainedDOFs[3*i+2] = m_fixGroups[i] * 3 + 2;
  }

}

void Composite::getSolverInfo(){
  cout << "numVertices: "<<m_numVertices<<endl;
  cout << "timeStep: "<<m_timeStep<<endl;
  cout << "numConstrainedDOFs: "<<m_numConstrainedDOFs<<endl;
  cout << "dampingMass: "<<m_dampingMass<<endl;
  cout << "dampingStiffness: "<<m_dampingStiffness<<endl;
  cout << "numSolverThreads: "<<m_numSolverThreads<<endl;
}

void Composite::AddTriConstraints(int v1, int v2, int v3, double bary[4][3], int VID[4], std::vector<Vec3d>& constraintPts){
  // cout <<"objIndex: "<<objIndex<<endl;
  int vertx[3];
  double pos[3];
  Vec3d temp;
  vertx[0] = VID[v1];
  vertx[1] = VID[v2];
  vertx[2] = VID[v3];
  // cout <<"vertx: "<<vertx[0]<<", "<<vertx[1]<<", "<<vertx[2]<<endl;
  for(int i = 0; i<4 ; i++){
    temp = m_objMesh->getPosition(vertx[0])*bary[i][0]+
      m_objMesh->getPosition(vertx[1])*bary[i][1]+
      m_objMesh->getPosition(vertx[2])*bary[i][2];
      constraintPts.push_back(temp);
      // cout <<"temp: "<<temp<<endl;
      pos[0] = temp[0];
      pos[1] = temp[1];
      pos[2] = temp[2];
      
      m_constraints->AddFixedBarycentricConstraint3(0, vertx, bary[i], pos);

  }
}

void Composite::AddConstraintsFromFile(const char* robot_data){
  if(m_constraints != NULL)
  {
    parseData(robot_data,m_grasping_points);
    if(m_DEBUG)
      cout <<"[Input vertex number]: "<< m_grasping_points.size()<<endl;
    
      int vertxID[4];
    double baryW[4][3] = {{1.0/2,1.0/2,0},{1.0/2,0,1.0/2},{0,1.0/2,1.0/2},{1.0/3,1.0/3,1.0/3}};
      // cout <<"Grapsing size: "<<grasping_points.size()<<endl;
      vector<Vec3d> temp;
      double pos[3];
      //Do something to Constraints here
     for(int i=0; i< m_grasping_points.size()/4; i++){
        for(int j=0; j<4; j++){
          vertxID[j] = m_objMesh->getClosestVertex(m_grasping_points[4*i+j]);

          m_objMesh->getPosition(vertxID[j]).convertToArray(pos);
          temp.push_back(m_objMesh->getPosition(vertxID[j]));
          m_constraints->AddFixedConstraint(0,vertxID[j],pos);
        }

        AddTriConstraints(0,1,3,baryW,vertxID,temp);
        // AddTriConstraints(0,2,3,baryW,vertxID,temp);
        AddTriConstraints(1,2,3,baryW,vertxID,temp);
        // AddTriConstraints(0,1,2,baryW,vertxID,temp);  
      }

      m_grasping_points.clear();
      copy(temp.begin(), temp.end(), back_inserter(m_grasping_points));
      if(m_DEBUG){
              cout <<"Constraints added from file:"<< robot_data<< endl;
      cout << "[Total Constrainted Vertex Number]: "<< m_grasping_points.size()<<endl;
      }
 

  }
  else{
    cout <<"[Error: AddSurfaceConstraints] No Constraints obj to use...Please make sure sheet status has updated.\n";
  }

}
void Composite::AddConstraints(const std::vector<Vec3d>& robot_data){
    if(m_constraints != NULL)
  {
    for(int i=0; i<robot_data.size(); i++)
      m_grasping_points.push_back(robot_data[i]);
    if(m_DEBUG)
        cout <<"[Input vertex number]: "<< m_grasping_points.size()<<endl;
      int vertxID;
      double pos[3];
      //Do something to Constraints here
     for(int i=0; i< m_grasping_points.size(); i++){

          vertxID = m_objMesh->getClosestVertex(m_grasping_points[i]);

          m_objMesh->getPosition(vertxID).convertToArray(pos);
          
          m_constraints->AddFixedConstraint(0,vertxID,pos);
        
      }
      if(m_DEBUG)
        cout << "[Total Constrainted Vertex Number]: "<< m_grasping_points.size()<<endl; 

  }
  else{
    cout <<"[Error: AddSurfaceConstraints] No Constraints obj to use...Please make sure sheet status has updated.\n";
  }
}

void Composite::MoveSurfaceTo(const std::vector<Vec3d>& target_point, const int& dir, const double& dist){
  for(int i=0; i<target_point.size();i++){
    MoveVertexTo(target_point[i],dir,dist);
  }

}

void Composite::MoveSurfaceTo(const std::vector<int>& targetIDs, const int& dir, const double& dist){
  for(int i=0; i<targetIDs.size();i++){
    MoveVertexTo(targetIDs[i],dir,dist);
  }

}

void Composite::MoveVertexTo(const Vec3d& target_point, const int& dir, const double& dist){
  if(dir<3){
    double pos[3];
    int targetID=m_objMesh->getClosestVertex(target_point);
    m_sceneObjDeform->GetVertexPosition(targetID).convertToArray(pos);
    if(m_DEBUG)
      cout <<"Move Vertex# "<<targetID<<" from: "<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<endl;
    pos[dir]+=dist;
    m_constraints->AddFixedConstraint(0,targetID,pos);
    m_constraintGroups.push_back(targetID);
    if(m_DEBUG)
      cout <<"Move Vertex# "<<targetID<<" to: "<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<endl;
  }
  else{
    cout <<"[MoveVertexTo]Direction can only be 0,1 or 2\n";
  }

}


void Composite::MoveVertexTo(const int& targetID, const int& dir, const double& dist)
{
  //std::cout << "in MVT" << std::endl;
  if(dir<3){
    double pos[3];
    m_sceneObjDeform->GetVertexPosition(targetID).convertToArray(pos);
    if(m_DEBUG)
      cout <<"Move Vertex# "<<targetID<<" from: "<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<endl;
    pos[dir]+=dist;
    m_constraints->AddFixedConstraint(0,targetID,pos);
    m_constraintGroups.push_back(targetID);
    if(m_DEBUG)
      cout <<"Move Vertex# "<<targetID<<" to: "<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<endl;
  }
  else{
    cout <<"[MoveVertexTo]Direction can only be 0,1 or 2\n";
  }

}

void Composite::MoveSurfaceTo(const std::vector<Vec3d>& target_point, const std::vector<Vec3d>& destination){
  for(int i=0; i<target_point.size();i++){
    MoveVertexTo(target_point[i],destination[i]);
  }

}

void Composite::MoveVertexTo(const Vec3d& target_point, const Vec3d& destination){
  
    double target_pos[3];
    double dest_pos[3];
    double pos[3];
    destination.convertToArray(dest_pos);
    int targetID=m_objMesh->getClosestVertex(target_point);
    m_sceneObjDeform->GetVertexPosition(targetID).convertToArray(target_pos);
    if(m_DEBUG)
      cout <<"Move Vertex# "<<targetID<<" from: "<<target_pos[0]<<", "<<target_pos[1]<<", "<<target_pos[2]<<endl;
    for(int i=0; i<3; i++)
      pos[i]=dest_pos[i]-target_pos[i];
    m_constraints->AddFixedConstraint(0,targetID,pos);
    m_constraintGroups.push_back(targetID);
    if(m_DEBUG)
      cout <<"Move Vertex# "<<targetID<<" to: "<<dest_pos[0]<<", "<<dest_pos[1]<<", "<<dest_pos[2]<<endl;

}

double Composite::getGripDistance(int vecIDa, int vecIDb)
{
    double grip1[3];
    double grip2[3];
    double grip_dist = 0;
    double xD = 0;
    double yD = 0;
    double zD = 0;
    double xyzSum = 0;
    m_sceneObjDeform->GetVertexPosition(vecIDa).convertToArray(grip1);
    m_sceneObjDeform->GetVertexPosition(vecIDb).convertToArray(grip2);
    xD = pow(grip1[0] - grip2[0], 2);
    yD = pow(grip1[1] - grip2[1], 2);
    zD = pow(grip1[2] - grip2[2], 2);
    xyzSum = xD + yD + zD;
    grip_dist = sqrt(xyzSum);
    return grip_dist;
}

void Composite::Fun(const std::vector<int>& targetIDs,const std::vector<double>& dir)
{
  cout <<"FUN!!!!"<<endl;

  //AlecZach Added 0629
  // for(int i = 0; i < targetIDs.size(); i++)
  // {
    // MoveVertexTo(targetIDs[i],dir,dist);
  // }

}

void Composite::MoveSurfaceTo3D(const std::vector<int>& targetIDs, const std::vector<double>& dir, const double& dist, const int& x)
{
  //AlecZach Added 0629
  for(int i = 0; i < targetIDs.size(); i++)
  {
    MoveVertexTo3D(targetIDs[i],dir,dist,x);
  }

}

void Composite::MoveVertexTo3D(const int& targetID, const std::vector<double>& dir, const double& dist, const int& x)
{ //AlecZach Function 0629
  // std::cout << "in MVT" << std::endl;
  double pos[3];
    m_sceneObjDeform->GetVertexPosition(targetID).convertToArray(pos);
    if(m_DEBUG){
      cout <<"Move Vertex# "<<targetID<<" from: "<<pos[0]<<", "<<pos[1]<<", "<<pos[2]<<endl;
    }

    for(int i = 0; i < 3; i++)
    {
      pos[i] += dist * dir[i];
    }
    
    m_constraints->AddFixedConstraint(0,targetID,pos);
    m_constraintGroups.push_back(targetID);

}

void Composite::resetConstraints(){
      if(m_constraints!=NULL){
        m_constraints->Reset();
        m_constraintGroups.clear();
        m_grasping_points.clear();
        if(m_DEBUG)
          std::cout << "[Constraints Cleared]\n";
      } 
}



void Composite::finalizeAllConstraints(){
  if(m_constraints!=NULL){
    m_constraints->FinalizeConstraints();
    m_constraintMatrix= m_constraints->GetConstraintMatrix();
    m_constraintRhs = m_constraints->GetConstraintRhs();

    m_implicitEulerMultiObjectWithConstraintsSparse->SetConstraintMatrix(m_constraintMatrix);

    m_implicitEulerMultiObjectWithConstraintsSparse->SetConstraintRhs(m_constraintRhs);
    if(m_DEBUG)
       cout << "[Constraints Finalized]\n";
  }
  else
    cout <<"[Error: finalizeAllConstraints] No Constraints obj to use...Please update sheet status first.\n";
}


void Composite::clearPermanentConstraints(){
  clearPermanentFixedID();
  updateConstrain();
if (m_implicitEulerMultiObjectWithConstraintsSparse != NULL)
      delete m_implicitEulerMultiObjectWithConstraintsSparse;
    const int numObjects = 1;
    double dampingMassD = (double)m_dampingMass;
    double dampingStiffnessD = (double)m_dampingStiffness;
    m_implicitEulerMultiObjectWithConstraintsSparse = new ImplicitEulerMultiObjectWithConstraintsSparse(numObjects, &m_numDOFs, m_timeStep, &m_massMatrix, &m_forceModel, 
      &m_numConstrainedDOFs, &m_constrainedDOFs, &dampingMassD, &dampingStiffnessD, 1, 1E-5, m_numSolverThreads);
    // cout << "[Solver initialized]"<<endl;

    if (m_deform)
    free(m_deform);
  if (m_f_ext)
    free (m_f_ext);
  m_deform = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  m_f_ext = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  for(int i = 0 ; i < 3 * m_numVertices; i++)
  {
      m_deform[i] = 0.0;
      m_f_ext[i] = 0.0;
  }
  m_implicitEulerMultiObjectWithConstraintsSparse->SetState(m_deform);
  m_implicitEulerMultiObjectWithConstraintsSparse->SetExternalForces(m_f_ext);
}


void Composite::updateSheetStatus(const char* objMeshname, const char* robot_data, bool is_permanent){
  updateConfig(objMeshname, robot_data);

  if(is_permanent){
   updateConstrain();
   m_constraintGroups.clear();
   m_grasping_points.clear();
  }
  generateSheet();
  if (m_implicitEulerMultiObjectWithConstraintsSparse != NULL)
      delete m_implicitEulerMultiObjectWithConstraintsSparse;
    const int numObjects = 1;
    double dampingMassD = (double)m_dampingMass;
    double dampingStiffnessD = (double)m_dampingStiffness;

    std::cout << "======================================================\n";
    std::cout << m_numConstrainedDOFs << ',' << m_constrainedDOFs << std::endl;
    std::cout << dampingMassD << ',' << dampingStiffnessD << std::endl;
    std::cout << m_timeStep << std::endl;
    m_implicitEulerMultiObjectWithConstraintsSparse = new ImplicitEulerMultiObjectWithConstraintsSparse(numObjects, &m_numDOFs, m_timeStep, &m_massMatrix, &m_forceModel, 
      &m_numConstrainedDOFs, &m_constrainedDOFs, &dampingMassD, &dampingStiffnessD, nIter, 1E-5, m_numSolverThreads);
    // cout << "[Solver initialized]"<<endl;

    if (m_constraints != NULL)
      delete m_constraints;

  m_constraints = new Constraints(numObjects, &m_numDOFs, m_restVertexPositions);
  std::cout << m_numDOFs << ',' << 3 * m_numVertices << std::endl;

    if (m_deform)
    free(m_deform);
  if (m_f_ext)
    free (m_f_ext);
  m_deform = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  m_f_ext = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  for(int i = 0 ; i < 3 * m_numVertices; i++)
  {
      m_deform[i] = 0.0;
      m_f_ext[i] = 0.0;
  }
  m_implicitEulerMultiObjectWithConstraintsSparse->SetState(m_deform);
  m_implicitEulerMultiObjectWithConstraintsSparse->SetExternalForces(m_f_ext);

  if (m_sceneObjDeform != NULL)
      delete m_sceneObjDeform;
    m_sceneObjDeform = new SceneObjectDeformable(m_objMesh);
    if(m_DEBUG)
       cout <<"[Sheet status Updated]\n";

    if(robot_data != NULL && !is_permanent)
  {
    resetConstraints();
    AddConstraintsFromFile(robot_data);
    finalizeAllConstraints();
  }
}

void Composite::updateSheetStatus(const char* objMeshname, const std::vector<Vec3d>& robot_data, bool is_permanent){
  updateConfig(objMeshname, robot_data);

  if(is_permanent){
   updateConstrain();
   m_constraintGroups.clear();
   m_grasping_points.clear();
  }
  generateSheet();
  if (m_implicitEulerMultiObjectWithConstraintsSparse != NULL)
      delete m_implicitEulerMultiObjectWithConstraintsSparse;
    const int numObjects = 1;
    double dampingMassD = (double)m_dampingMass;
    double dampingStiffnessD = (double)m_dampingStiffness;
    m_implicitEulerMultiObjectWithConstraintsSparse = new ImplicitEulerMultiObjectWithConstraintsSparse(numObjects, &m_numDOFs, m_timeStep, &m_massMatrix, &m_forceModel, 
      &m_numConstrainedDOFs, &m_constrainedDOFs, &dampingMassD, &dampingStiffnessD, nIter, 1E-5, m_numSolverThreads);
    // cout << "[Solver initialized]"<<endl;

    if (m_constraints != NULL)
      delete m_constraints;

  m_constraints = new Constraints(numObjects, &m_numDOFs, m_restVertexPositions);

    if (m_deform)
    free(m_deform);
  if (m_f_ext)
    free (m_f_ext);
  m_deform = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  m_f_ext = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  for(int i = 0 ; i < 3 * m_numVertices; i++)
  {
      m_deform[i] = 0.0;
      m_f_ext[i] = 0.0;
  }
  m_implicitEulerMultiObjectWithConstraintsSparse->SetState(m_deform);
  m_implicitEulerMultiObjectWithConstraintsSparse->SetExternalForces(m_f_ext);

  if (m_sceneObjDeform != NULL)
      delete m_sceneObjDeform;
    m_sceneObjDeform = new SceneObjectDeformable(m_objMesh);
    if(m_DEBUG)
        cout <<"[Sheet status Updated]\n";

  if(!is_permanent)
  {
    resetConstraints();
    AddConstraints(robot_data);
    finalizeAllConstraints();
  }
}

void Composite::updateSheetStatus(int numVertices, const double * vertices, int numTriangles, const int * triangles, const char* robot_data, bool is_permanent){
  updateConfig(numVertices, vertices, numTriangles, triangles, robot_data);

  if(is_permanent){
   updateConstrain();
   m_constraintGroups.clear();
   m_grasping_points.clear();
  }
  generateSheet();
  if (m_implicitEulerMultiObjectWithConstraintsSparse != NULL)
      delete m_implicitEulerMultiObjectWithConstraintsSparse;
    const int numObjects = 1;
    double dampingMassD = (double)m_dampingMass;
    double dampingStiffnessD = (double)m_dampingStiffness;
    m_implicitEulerMultiObjectWithConstraintsSparse = new ImplicitEulerMultiObjectWithConstraintsSparse(numObjects, &m_numDOFs, m_timeStep, &m_massMatrix, &m_forceModel, 
      &m_numConstrainedDOFs, &m_constrainedDOFs, &dampingMassD, &dampingStiffnessD, nIter, 1E-5, m_numSolverThreads);
    // cout << "[Solver initialized]"<<endl;

    if (m_constraints != NULL)
      delete m_constraints;

  m_constraints = new Constraints(numObjects, &m_numDOFs, m_restVertexPositions);

    if (m_deform)
    free(m_deform);
  if (m_f_ext)
    free (m_f_ext);
  m_deform = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  m_f_ext = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  for(int i = 0 ; i < 3 * m_numVertices; i++)
  {
      m_deform[i] = 0.0;
      m_f_ext[i] = 0.0;
  }
  m_implicitEulerMultiObjectWithConstraintsSparse->SetState(m_deform);
  m_implicitEulerMultiObjectWithConstraintsSparse->SetExternalForces(m_f_ext);

  if (m_sceneObjDeform != NULL)
      delete m_sceneObjDeform;
    m_sceneObjDeform = new SceneObjectDeformable(m_objMesh);
    if(m_DEBUG)
        cout <<"[Sheet status Updated]\n";

    if(robot_data != NULL && !is_permanent)
  {
    resetConstraints();
    AddConstraintsFromFile(robot_data);
    finalizeAllConstraints();
  }

}

void Composite::updateSheetStatus(int numVertices, const double * vertices, int numTriangles, const int * triangles, const std::vector<Vec3d>& robot_data, bool is_permanent){
  updateConfig(numVertices, vertices, numTriangles, triangles, robot_data);
  if(is_permanent){
   updateConstrain();
   m_constraintGroups.clear();
   m_grasping_points.clear();
  }
  generateSheet();
  if (m_implicitEulerMultiObjectWithConstraintsSparse != NULL)
      delete m_implicitEulerMultiObjectWithConstraintsSparse;
    const int numObjects = 1;
    double dampingMassD = (double)m_dampingMass;
    double dampingStiffnessD = (double)m_dampingStiffness;

    std::cout << "+++++" << m_dampingMass << ',' << m_dampingStiffness << std::endl;
    std::cout << m_numConstrainedDOFs << ',' << m_constrainedDOFs << std::endl;
    m_implicitEulerMultiObjectWithConstraintsSparse = new ImplicitEulerMultiObjectWithConstraintsSparse(numObjects, &m_numDOFs, m_timeStep, &m_massMatrix, &m_forceModel, 
      &m_numConstrainedDOFs, &m_constrainedDOFs, &dampingMassD, &dampingStiffnessD, nIter, 1E-5, m_numSolverThreads);
    // cout << "[Solver initialized]"<<endl;

    if (m_constraints != NULL)
      delete m_constraints;

  m_constraints = new Constraints(numObjects, &m_numDOFs, m_restVertexPositions);

    if (m_deform)
    free(m_deform);
  if (m_f_ext)
    free (m_f_ext);
  m_deform = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  m_f_ext = (double*) malloc (sizeof(double) * 3 * m_numVertices);
  for(int i = 0 ; i < 3 * m_numVertices; i++)
  {
      m_deform[i] = 0.0;
      m_f_ext[i] = 0.0;
  }
  m_implicitEulerMultiObjectWithConstraintsSparse->SetState(m_deform);
  m_implicitEulerMultiObjectWithConstraintsSparse->SetExternalForces(m_f_ext);

  if (m_sceneObjDeform != NULL)
      delete m_sceneObjDeform;
    m_sceneObjDeform = new SceneObjectDeformable(m_objMesh);
    if(m_DEBUG)
        cout <<"[Sheet status Updated]\n";

  if(!is_permanent)
  {
    resetConstraints();
    AddConstraints(robot_data);
    finalizeAllConstraints();
  }

}

double* Composite::simulate(const int& iteration){

    // clear old external force
    m_implicitEulerMultiObjectWithConstraintsSparse->SetExternalForcesToZero();
       // add gravity
    memset(m_f_ext, 0.0, 3 * m_numVertices);
    for (int i = 0; i < m_numVertices; i++)
      m_f_ext[3 * i + 1] = m_gravityForce * -1.0 * m_masses[i];

    m_implicitEulerMultiObjectWithConstraintsSparse->AddExternalForces(m_f_ext);
  for(int i=0; i<iteration; i++)
    m_implicitEulerMultiObjectWithConstraintsSparse->DoTimestep();

  memcpy(m_deform, m_implicitEulerMultiObjectWithConstraintsSparse->Getq(), sizeof(double) * 3 * m_numVertices);

  m_sceneObjDeform->SetVertexDeformations(m_deform);
  return m_deform;
}

int Composite::getClosestVertex(Vec3d& querypoint, double*& dist){
  dist = new double(10);
  return m_sceneObjDeform->GetClosestVertex(querypoint,dist);
}

int Composite::getClosestVertex(const double querypoint[3], double*& dist){
  Vec3d q=Vec3d(querypoint);
  return getClosestVertex(q,dist);
  
}

void Composite::getDeformInfo(double& max_deform, double& avg_deform){
avg_deform=0.0;
max_deform=0.0;
double dist=0.0;
for(int i=0; i< m_numVertices;i++){
    dist = sqrt(pow(m_deform[3*i+0],2)+pow(m_deform[3*i+1],2)+pow(m_deform[3*i+2],2));
  avg_deform+=dist;
  if(dist>max_deform){
    max_deform = dist;
  }
}
avg_deform/=m_numVertices;

}