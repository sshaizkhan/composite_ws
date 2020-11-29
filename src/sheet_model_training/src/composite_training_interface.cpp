#include "sheet_model_training/composite_training_interface.hpp"
#include <iostream>
#include <string>

using namespace std;
Composite_Training::Composite_Training(bool fem):Composite(fem){
  // FEM_ON=fem;
  if(m_FEM_ON)
    cout<<"-----[Using FEM model]-----\n";
  else
    cout<<"-----[Using ClothBW model]-----\n";
    
setDensity(0.3); //0.047
setThickness(0.0003); //0.001
setPoisson(0.3);
setGravity(9.81);
}

void Composite_Training::loadMesh(const char* objMeshname, const char* training_data, const int& fixNum, const double& scale){
  //ALEC updated 070320 to only update mesh first time (fixNum=0)
  if(fixNum == 0){
     if (m_objMesh != NULL)  
         delete m_objMesh;
       cout << "Loaded mesh: " << objMeshname << endl;
       m_objMesh = new ObjMesh(objMeshname);
        for(int i=0; i<m_objMesh->getNumVertices();i++){
          m_objMesh->setPosition(i,m_objMesh->getPosition(i));
        }
      }
  if(m_training_fix.size() != 0)
    m_training_fix.clear();
  if(m_training_marker.size() != 0)
   m_training_marker.clear();
  if(m_fixGroups.size() != 0)
   m_fixGroups.clear();

  parseTrainingData(training_data, m_objMesh, m_training_fix, m_training_marker, m_fixGroups,fixNum, 1);

  if(fixNum==0){
    std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
    m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
    m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
    assert(m_masses.size() == m_numVertices);
    m_numDOFs = 3 * m_numVertices;
  }
}



void Composite_Training::updateConfig(const char* objMeshname, const char* robot_data){
  if (m_objMesh != NULL)  
      delete m_objMesh;
    m_objMesh = new ObjMesh(objMeshname);
    if(m_training_fix.size() != 0)
      m_training_fix.clear();
    if(m_fixGroups.size() != 0)
      m_fixGroups.clear();

    
    parseRobotData(robot_data,m_objMesh, m_training_fix, m_fixGroups);
      std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
  m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
    m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
  assert(m_masses.size() == m_numVertices);
  m_numDOFs = 3 * m_numVertices;
}


void Composite_Training::AddSurfaceConstraints(){
  if(m_constraints != NULL)
  {
    int vertxID[4];
    double baryW[4][3] = {{1.0/2,1.0/2,0},{1.0/2,0,1.0/2},{0,1.0/2,1.0/2},{1.0/3,1.0/3,1.0/3}};
    vector<Vec3d> temp;
    double pos[3];
    //Do something to Constraints here
   for(int i=0; i< m_training_fix.size()/4; i++)
   {
      for(int j=0; j<4; j++){
        vertxID[j] = m_objMesh->getClosestVertex(m_training_fix[4*i+j]);
        pos[0]=m_objMesh->getPosition(vertxID[j])[0];
        pos[1]=m_objMesh->getPosition(vertxID[j])[1];
        pos[2]=m_objMesh->getPosition(vertxID[j])[2];
        temp.push_back(m_objMesh->getPosition(vertxID[j]));
        m_constraints->AddFixedConstraint(0,vertxID[j],pos);
      }
      AddTriConstraints(0,2,3,baryW,vertxID,temp);
      AddTriConstraints(1,2,3,baryW,vertxID,temp); 
    }

    m_training_fix.clear();
    copy(temp.begin(), temp.end(), back_inserter(m_training_fix));
  }
  else{
    cout <<"[Error: AddSurfaceConstraints] No Constraints obj to use...Please make sure sheet status has updated.\n";
  }
}
void Composite_Training::updateSheetStatus(const char* objMeshname, const char* training_data, const std::vector<double>& param, const int& fixNum, bool is_permanent){
  if (fixNum > 0){
    loadMesh(objMeshname, training_data,fixNum);
  }
  else{
    loadMesh(objMeshname, training_data,fixNum);
    updateParam(param);
    if(is_permanent)
     updateConstrain();
    generateSheet();
    if (m_implicitEulerMultiObjectWithConstraintsSparse != NULL)
        delete m_implicitEulerMultiObjectWithConstraintsSparse;
      const int numObjects = 1;
      double dampingMassD = (double)m_dampingMass;
      double dampingStiffnessD = (double)m_dampingStiffness;
      m_implicitEulerMultiObjectWithConstraintsSparse = new ImplicitEulerMultiObjectWithConstraintsSparse(numObjects, &m_numDOFs, m_timeStep, &m_massMatrix, &m_forceModel, &m_numConstrainedDOFs, &m_constrainedDOFs, &dampingMassD, &dampingStiffnessD, 1, 1E-5, m_numSolverThreads);
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
      // cout <<"[Sheet status Updated]\n";

      if(training_data != NULL && !is_permanent)
    {
      resetConstraints();
      AddSurfaceConstraints();
      finalizeAllConstraints();
    }
  }
}


double Composite_Training::solve_err_with_constraints1(const char* objMeshname,const char* training_data,
  const std::vector<double>& param,const int& fixNum, bool is_permanent, const int& time)
{

  double zero= 0.00;
  std::vector<double> vec_0 {0,0,0};

  // =============================================== //

  // std::vector<int> fix0{11482,3511,12391,12390};  
  // std::vector<int> fix1{10215,2283,15410,16524};      
  // std::vector<int> fix2{7940,2050,15305,16228};    //T1_NewMeshTrial
  // std::vector<int> fix3{3954,11756,12207,12206};

  std::vector<int> fix0{1507,11878,12398,12397};  
  std::vector<int> fix1{16598,15412,3147,11163};      
  std::vector<int> fix2{11914,11915,2904,10947};    //T2_NewMeshTrial
  std::vector<int> fix3{2424,12227,12226,10400};

  // ==============================================//

  // std::vector<int> fix0Center{15539};  
  // std::vector<int> fix1Center{12132};      
  // std::vector<int> fix2Center{11922};    //T1_fix centers
  // std::vector<int> fix3Center{15447};

  // std::vector<int> fix0Center{16347};  
  // std::vector<int> fix1Center{12136};      
  // std::vector<int> fix2Center{15301};    //T2_fix centers
  // std::vector<int> fix3Center{15457};

  // ======================================//

  std::vector<int> fix_tn0= {248,253,284,0,20,13,82,64,49,273,263,271};  // order ABCD left mid right
  std::vector<int> fix_tn1= {48,30,11,279,252,281,200,232,246,4,22,3};  // order ABCD left mid right
  std::vector<int> fix_tn2= {288,316,332,2,31,0,48,64,47,333,307,320};  // order ABCD left mid right
  std::vector<int> fix_tn3= {51,19,13,260,257,280,203,234,244,4,30,3};  // order ABCD left mid right (234 is far, missing a point)
  std::vector<int> fix_tn4= {285,253,251,16,1,30,43,66,47};  // order ABCD left mid right last 3 241,265,239
  std::vector<std::vector<int>> fix_training_neighbors= {fix_tn0,fix_tn1,fix_tn2,fix_tn3,fix_tn4};

  // =========================================//

  // Vec3d v3d_A0 = Vec3d(0.258900, 0, 1.041991);   
  // Vec3d v3d_B0 = Vec3d(-0.647545, 0, 1.169960);  //T1_NewMeshTrial
  // Vec3d v3d_C0 = Vec3d(-0.716100, 0, 0.191913); 
  // Vec3d v3d_D0 = Vec3d(0,0,0);

  Vec3d v3d_A0 = Vec3d(0.198000, 0, 0.914022);   
  Vec3d v3d_B0 = Vec3d(-0.647508, 0, 1.169960);  //T2_NewMeshTrial
  Vec3d v3d_C0 = Vec3d(-0.777000, 0, 0.127929); 
  Vec3d v3d_D0 = Vec3d(0,0,0); 

  // ============================================//

  // Vec3d v3d_A1 = Vec3d(0.103416,-0.001622,1.039259); 
  // Vec3d v3d_A2 = Vec3d(0.079567,0.110935,0.965506);  
  // Vec3d v3d_B1 = Vec3d(-0.802412,0.033388,1.054125);
  // Vec3d v3d_B2 = Vec3d(-0.784751,0.119161,0.995961); //  T1_NewMeshTrial
  // Vec3d v3d_C1 = Vec3d(-0.73471,0.0131175,0.083230); 
  // Vec3d v3d_C2 = Vec3d(-0.731668,0.1049795,0.089480); 
  // Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  // Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

  Vec3d v3d_A1 = Vec3d(0.06288,-0.0036665,0.912126); 
  Vec3d v3d_A2 = Vec3d(0.050563,0.0675485,0.896538);  
  Vec3d v3d_B1 = Vec3d(-0.808377,-0.0010165,1.038368);
  Vec3d v3d_B2 = Vec3d(-0.787239,0.093844,0.99416); //  T2_NewMeshTrial
  Vec3d v3d_C1 = Vec3d(-0.767154,0.009518,0.003424); 
  Vec3d v3d_C2 = Vec3d(-0.71960,0.115937,0.010751); 
  Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

// =================================================//

  std::vector<Vec3d> v3d0= {v3d_A0,v3d_B0,v3d_C0,v3d_D0};
  std::vector<Vec3d> v3d1= {v3d_A1,v3d_B1,v3d_C1,v3d_D1};
  std::vector<Vec3d> v3d2= {v3d_A2,v3d_B2,v3d_C2,v3d_D2};
  std::vector<std::vector<Vec3d>> v3d = {v3d0, v3d1, v3d2};

  std::vector<std::vector<std::vector<double>>> vec;
  std::vector<std::vector<double>> temp_vec;
  std::vector<double> temp_vec_point;

  double val= 0.0;


  for(int i=0; i<2; i++)
  {
    for(int j=0; j<4; j++)
    {
      for(int k=0; k<3; k++)
      {
        val = v3d[i+1][j][k]-v3d[i][j][k];

        temp_vec_point.push_back(val);
      }
      temp_vec.push_back(temp_vec_point);
      temp_vec_point= {};
    }
    vec.push_back(temp_vec);
    temp_vec= {};
  }

  updateSheetStatus(objMeshname, training_data, param, fixNum, is_permanent);
  cout << "Training data file for Sheet 1: " << training_data << endl;
  // updateParam(vp);
  // getParameters();

  double t =  2*getTimeStep();  //20
  int loop = 50;
  int start_time=0;
  int stop_time=50;
  int extra_loops= 2;
  int extra_loops_counter= 0;
  for(int timeStepCount = 0; timeStepCount < loop; timeStepCount++)
  {
    resetConstraints();

    // Movements go here
    if(fixNum==0 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec[0][0],t,0);
      MoveSurfaceTo3D(fix1,vec[0][1],t,0);
      MoveSurfaceTo3D(fix2,vec[0][2],t,0);
      MoveSurfaceTo3D(fix3,vec[0][3],t,0);
    }
    else if(fixNum==1 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec[1][0],t,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    else if(fixNum==2 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec[1][1],t,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    else if(fixNum==3 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec[1][2],t,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    else if(fixNum==4 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
    }
    else{
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    cout << "." << flush;
    finalizeAllConstraints();
    simulate(1); //1
  }

    // Vec3d offset;
    double avg_err = 0;
    double max_err = 0;
    double* distPTR = new double(10);

    if (fixNum==0){
    m_sceneObjDeform->GetClosestVertex(v3d[1][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==1){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==2){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==3){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==4){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][2],distPTR); 
    cout << "  C: " << *distPTR << endl;

  }

  
  for(int i=0; i<m_training_marker.size(); i++){
    m_sceneObjDeform->GetClosestVertex(m_training_marker[i],distPTR); //capital G in Get
 
    if(*distPTR==0)
      *distPTR=0.1;
    if(*distPTR>max_err)
        max_err = *distPTR;

    avg_err+=*distPTR; 
  }
  avg_err/= m_training_marker.size();

  cout << "avg_err:" << avg_err << endl;
  cout << "max_err:" << max_err << endl;

  delete distPTR;
  // return (avg_err*1+max_err*0.5);
  return (avg_err*1+max_err*2.0);
}

double Composite_Training::solve_err_with_constraints2(const char* objMeshname,const char* training_data,
  const std::vector<double>& param,const int& fixNum, bool is_permanent, const int& time)
{

  double zero= 0.00;
  std::vector<double> vec_0 {0,0,0};

  // ================================================//

  // std::vector<int> fix0{1507,11878,12398,12397};  
  // std::vector<int> fix1{16598,15412,3147,11163};      
  // std::vector<int> fix2{11914,11915,2904,10947};    //T2_NewMeshTrial
  // std::vector<int> fix3{2424,12227,12226,10400};

  std::vector<int> fix0{2657,15541,16346,9154};  
  std::vector<int> fix1{16524,15410,2283,10215};      
  std::vector<int> fix2{11926,11523,900,11925};    //T3_NewMeshTrial2
  std::vector<int> fix3{15459,16605,11301,3308}; 

  // ================================================ //

  // std::vector<int> fix0Center{16347};  
  // std::vector<int> fix1Center{12136};      
  // std::vector<int> fix2Center{15301};    //T2_fix centers
  // std::vector<int> fix3Center{15457};

  // std::vector<int> fix0Center{12394};  
  // std::vector<int> fix1Center{12132};      
  // std::vector<int> fix2Center{16229};    //T3_fix centers
  // std::vector<int> fix3Center{12231};

  // ============================================= //

  std::vector<int> fix_tn0= {248,253,284,0,20,13,82,64,49,273,263,271};  // order ABCD left mid right
  std::vector<int> fix_tn1= {48,30,11,279,252,281,200,232,246,4,22,3};  // order ABCD left mid right
  std::vector<int> fix_tn2= {288,316,332,2,31,0,48,64,47,333,307,320};  // order ABCD left mid right
  std::vector<int> fix_tn3= {51,19,13,260,257,280,203,234,244,4,30,3};  // order ABCD left mid right (234 is far, missing a point)
  std::vector<int> fix_tn4= {285,253,251,16,1,30,43,66,47};  // order ABCD left mid right last 3 241,265,239
  std::vector<std::vector<int>> fix_training_neighbors= {fix_tn0,fix_tn1,fix_tn2,fix_tn3,fix_tn4};

  // ================================================= //

  // Vec3d v3d_A0 = Vec3d(0.198000, 0, 0.914022);   
  // Vec3d v3d_B0 = Vec3d(-0.647508, 0, 1.169960);  //T2_NewMeshTrial
  // Vec3d v3d_C0 = Vec3d(-0.777000, 0, 0.127929); 
  // Vec3d v3d_D0 = Vec3d(0,0,0);

  Vec3d v3d_A0 = Vec3d(0.129500,0,0.978007);   
  Vec3d v3d_B0 = Vec3d(-0.776945,0,1.169960);  //T3_NewMeshTrial2
  Vec3d v3d_C0 = Vec3d(-0.845500,0,0.255897); 
  Vec3d v3d_D0 = Vec3d(0,0,0); 

  // ================================================= //

  // Vec3d v3d_A1 = Vec3d(0.06288,-0.0036665,0.912126); 
  // Vec3d v3d_A2 = Vec3d(0.050563,0.0675485,0.896538);  
  // Vec3d v3d_B1 = Vec3d(-0.808377,-0.0010165,1.038368);
  // Vec3d v3d_B2 = Vec3d(-0.787239,0.093844,0.99416); //  T2_NewMeshTrial
  // Vec3d v3d_C1 = Vec3d(-0.767154,0.009518,0.003424); 
  // Vec3d v3d_C2 = Vec3d(-0.71960,0.115937,0.010751); 
  // Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  // Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

  Vec3d v3d_A1 = Vec3d(0.03418,0.008986,0.967035); 
  Vec3d v3d_A2 = Vec3d(-0.052287,0.0790045,0.924629);
  Vec3d v3d_B1 = Vec3d(-0.881043,0.030003,1.06678);
  Vec3d v3d_B2 = Vec3d(-0.836011,0.0909105,1.00588); //T3_NewMeshTrial
  Vec3d v3d_C1 = Vec3d(-0.866209,0.0086345,0.153300); 
  Vec3d v3d_C2 = Vec3d(-0.789734,0.135744,0.173395); 
  Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

  // ================================================== //

  std::vector<Vec3d> v3d0= {v3d_A0,v3d_B0,v3d_C0,v3d_D0};
  std::vector<Vec3d> v3d1= {v3d_A1,v3d_B1,v3d_C1,v3d_D1};
  std::vector<Vec3d> v3d2= {v3d_A2,v3d_B2,v3d_C2,v3d_D2};
  std::vector<std::vector<Vec3d>> v3d = {v3d0, v3d1, v3d2};

  std::vector<std::vector<std::vector<double>>> vec;
  std::vector<std::vector<double>> temp_vec;
  std::vector<double> temp_vec_point;

  double val= 0.0;


  for(int i=0; i<2; i++)
  {
    for(int j=0; j<4; j++)
    {
      for(int k=0; k<3; k++)
      {
        // val = pos[i+1][j][k]-pos[i][j][k];
        val = v3d[i+1][j][k]-v3d[i][j][k];
        // cout << val <<endl;
        temp_vec_point.push_back(val);
      }
      temp_vec.push_back(temp_vec_point);
      temp_vec_point= {};
    }
    vec.push_back(temp_vec);
    temp_vec= {};
  }

  updateSheetStatus(objMeshname, training_data, param, fixNum, is_permanent);
  cout << "Training data file for Sheet 2: " << training_data << endl;
  // updateParam(vp);
  // getParameters();

  double t =  2*getTimeStep();  //20
  int loop = 50;
  int start_time=0;
  int stop_time=50;
  int extra_loops= 2;
  int extra_loops_counter= 0;
  for(int timeStepCount = 0; timeStepCount < loop; timeStepCount++)
  {
    resetConstraints();
    // Movements go here
    if(fixNum==0 && timeStepCount<stop_time && timeStepCount>=start_time){
      // cout << "t:" << t << endl;
      MoveSurfaceTo3D(fix0,vec[0][0],t,0);
      MoveSurfaceTo3D(fix1,vec[0][1],t,0);
      MoveSurfaceTo3D(fix2,vec[0][2],t,0);
      MoveSurfaceTo3D(fix3,vec[0][3],t,0);
    }
    else if(fixNum==1 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec[1][0],t,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    else if(fixNum==2 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec[1][1],t,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    else if(fixNum==3 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec[1][2],t,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    else if(fixNum==4 && timeStepCount<stop_time && timeStepCount>=start_time){
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
    }
    else{
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }

    cout << "." << flush;
  
    finalizeAllConstraints();
    simulate(1); //1
  }

    // Vec3d offset;
    double avg_err = 0;
    double max_err = 0;
    double* distPTR = new double(10);

    if (fixNum==0){
    m_sceneObjDeform->GetClosestVertex(v3d[1][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==1){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==2){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==3){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==4){
    m_sceneObjDeform->GetClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    m_sceneObjDeform->GetClosestVertex(v3d[2][2],distPTR); 
    cout << "  C: " << *distPTR << endl;
  }

  
  for(int i=0; i<m_training_marker.size(); i++){
    m_sceneObjDeform->GetClosestVertex(m_training_marker[i],distPTR); //capital G in Get
    if(*distPTR==0)
      *distPTR=0.1;
    if(*distPTR>max_err)
        max_err = *distPTR;

    avg_err+=*distPTR; 
  }
  avg_err/= m_training_marker.size();

  cout << "avg_err:" << avg_err << endl;
  cout << "max_err:" << max_err << endl;


  
  delete distPTR;
  // return (avg_err*1+max_err*0.5);
  return (avg_err*1+max_err*2.0);
}

