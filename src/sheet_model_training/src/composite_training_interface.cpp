#include "sheet_model_training/composite_training_interface.hpp"
#include <iostream>
#include <string>
// template <typename T> std::string type_name(); //Alec 070320

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
// void Composite_Training::loadMesh(const char* objMeshname, const char* training_data, const int& fixNum, const double& scale){
// 	if (m_objMesh != NULL)	
// 	    delete m_objMesh;
// 	  m_objMesh = new ObjMesh(objMeshname);
//     for(int i=0; i<m_objMesh->getNumVertices();i++)
//       m_objMesh->setPosition(i,m_objMesh->getPosition(i)/scale);
//     if(m_training_fix.size() != 0)
//       m_training_fix.clear();
//   	if(m_training_marker.size() != 0)
//   		m_training_marker.clear();
//   	if(m_fixGroups.size() != 0)
//   		m_fixGroups.clear();
//   	// if(init_marker.size() != 0)
//   	// 	init_marker.clear();
  	
//   	parseTrainingData(training_data, m_objMesh, m_training_fix, m_training_marker, m_fixGroups,fixNum, 1);
//       std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
//   m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
//     m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
//   assert(m_masses.size() == m_numVertices);
//   m_numDOFs = 3 * m_numVertices;
//   	// updateConstrain();
//   // cout <<"[loadMesh] Mesh loaded.\nVertex Number: "<< m_numVertices<<endl;
//   // cout <<"Grasping location number: "<<fixNum<<endl;
//   // cout <<"Marker Number: "<<m_training_marker.size()<<endl;


// }
  void Composite_Training::loadMesh(const char* objMeshname, const char* training_data, const int& fixNum, const double& scale){
    //ALEC updated 070320 to only update mesh first time (fixNum=0)
    if(fixNum >= 0){
       if (m_objMesh != NULL)  
           delete m_objMesh;
         cout << "Loaded mesh: " << objMeshname << endl;
         m_objMesh = new ObjMesh(objMeshname);
          for(int i=0; i<m_objMesh->getNumVertices();i++){
            // m_objMesh->setPosition(i,m_objMesh->getPosition(i)/scale);
            m_objMesh->setPosition(i,m_objMesh->getPosition(i));
          }
        }
    if(m_training_fix.size() != 0)
      m_training_fix.clear();
    if(m_training_marker.size() != 0)
     m_training_marker.clear();
    if(m_fixGroups.size() != 0)
     m_fixGroups.clear();
   // if(init_marker.size() != 0)
   //  init_marker.clear();
    
    parseTrainingData(training_data, m_objMesh, m_training_fix, m_training_marker, m_fixGroups,fixNum, 1);

    if(fixNum==0){
      std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
      m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
      m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
      assert(m_masses.size() == m_numVertices);
      m_numDOFs = 3 * m_numVertices;
    }

   // updateConstrain();
  // cout <<"[loadMesh] Mesh loaded.\nVertex Number: "<< m_numVertices<<endl;
  // cout <<"Grasping location number: "<<fixNum<<endl;
  // cout <<"Marker Number: "<<m_training_marker.size()<<endl;
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
    // Compute vertex masses, under a constant surface density [kg / m^2].
      std::vector<double> groupSurfaceMassDensity(m_objMesh->getNumGroups(), m_surfaceDensity);
  m_objMesh->computeMassPerVertex(groupSurfaceMassDensity, m_masses);
    m_objMesh->exportGeometry(&m_numVertices, &m_restVertexPositions, &m_numTriangles, &m_triangleVertexIndices);
  assert(m_masses.size() == m_numVertices);
  m_numDOFs = 3 * m_numVertices;
  // cout <<"Mesh loaded\n";
}


void Composite_Training::AddSurfaceConstraints(){
    if(m_constraints != NULL)
  {
    // cout <<"[Input vertex number]: "<< m_training_fix.size()<<endl;
    
      int vertxID[4];
    double baryW[4][3] = {{1.0/2,1.0/2,0},{1.0/2,0,1.0/2},{0,1.0/2,1.0/2},{1.0/3,1.0/3,1.0/3}};
      // cout <<"Grapsing size: "<<grasping_points.size()<<endl;
      vector<Vec3d> temp;
      double pos[3];
      //Do something to Constraints here
     for(int i=0; i< m_training_fix.size()/4; i++){
        for(int j=0; j<4; j++){
          vertxID[j] = m_objMesh->getClosestVertex(m_training_fix[4*i+j]);
          pos[0]=m_objMesh->getPosition(vertxID[j])[0];
          pos[1]=m_objMesh->getPosition(vertxID[j])[1];
          pos[2]=m_objMesh->getPosition(vertxID[j])[2];
          temp.push_back(m_objMesh->getPosition(vertxID[j]));
          m_constraints->AddFixedConstraint(0,vertxID[j],pos);
          // constraints->AddFixedConstraint(objectIndex,vertxID[j],pos);
        }
        // cout <<"vertxID# "<<i+1<<": "<<vertxID[0]<<", "<<vertxID[1]<<", "<<vertxID[2]<<", "<<vertxID[3]<<endl;

        // AddTriConstraints(objectIndex,0,1,3,baryW,vertxID,temp);
        AddTriConstraints(0,2,3,baryW,vertxID,temp);
        AddTriConstraints(1,2,3,baryW,vertxID,temp);
        // AddTriConstraints(objectIndex,0,1,2,baryW,vertxID,temp);  
      }
    m_training_fix.clear();
      copy(temp.begin(), temp.end(), back_inserter(m_training_fix));
      
      // cout << "[Total Constrainted Vertex Number]: "<< m_training_fix.size()<<endl; 
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

  std::vector<int> fix0{11482,3511,12391,12390};  
  std::vector<int> fix1{10215,2283,15410,16524};      
  std::vector<int> fix2{7940,2050,15305,16228};    //T1_NewMeshTrial
  std::vector<int> fix3{3954,11756,12207,12206};

  // std::vector<int> fix0{1507,11878,12398,12397};  
  // std::vector<int> fix1{16598,15412,3147,11163};      
  // std::vector<int> fix2{11914,11915,2904,10947};    //T2_NewMeshTrial
  // std::vector<int> fix3{2424,12227,12226,10400}; 

  // std::vector<int> fix0{11482,3511,12391,12390};  
  // std::vector<int> fix1{10215,2283,15410,16524};      
  // std::vector<int> fix2{916,7976,15312,16232};    //T4_NewMeshTrial2
  // std::vector<int> fix3{2424,12227,12226,10400};

  // std::vector<int> fix0{273,3235,3236,4163};  //Alec 0630
  // std::vector<int> fix1{79,4216,8301,11896};      //Alec 0630
  // std::vector<int> fix2{259,3141,3142,4106};    //Alec 0630
  // std::vector<int> fix3{4080,4089,7401,7457};      //Alec 0630

  // ==============================================//
  std::vector<int> fix0Center{15539};  
  std::vector<int> fix1Center{12132};      
  std::vector<int> fix2Center{11922};    //T1_fix centers
  std::vector<int> fix3Center{15447};

  // std::vector<int> fix0Center{16347};  
  // std::vector<int> fix1Center{12136};      
  // std::vector<int> fix2Center{15301};    //T2_fix centers
  // std::vector<int> fix3Center{15457};

  // std::vector<int> fix0Center{15539};  
  // std::vector<int> fix1Center{12132};      
  // std::vector<int> fix2Center{11937};    //T4_fix centers
  // std::vector<int> fix3Center{15457};

  // ======================================//
  // std::vector<int> fix_tn0= {0,13,20,49,64,82,248,253,263,271,273,284};
  std::vector<int> fix_tn0= {248,253,284,0,20,13,82,64,49,273,263,271};  // order ABCD left mid right
  std::vector<int> fix_tn1= {48,30,11,279,252,281,200,232,246,4,22,3};  // order ABCD left mid right
  std::vector<int> fix_tn2= {288,316,332,2,31,0,48,64,47,333,307,320};  // order ABCD left mid right
  std::vector<int> fix_tn3= {51,19,13,260,257,280,203,234,244,4,30,3};  // order ABCD left mid right (234 is far, missing a point)
  std::vector<int> fix_tn4= {285,253,251,16,1,30,43,66,47};  // order ABCD left mid right last 3 241,265,239
  std::vector<std::vector<int>> fix_training_neighbors= {fix_tn0,fix_tn1,fix_tn2,fix_tn3,fix_tn4};

  // =========================================//

  Vec3d v3d_A0 = Vec3d(0.258900, 0, 1.041991);   
  Vec3d v3d_B0 = Vec3d(-0.647545, 0, 1.169960);  //T1_NewMeshTrial
  Vec3d v3d_C0 = Vec3d(-0.716100, 0, 0.191913); 
  Vec3d v3d_D0 = Vec3d(0,0,0); 

  // Vec3d v3d_A0 = Vec3d(0.198000, 0, 0.914022);   
  // Vec3d v3d_B0 = Vec3d(-0.647508, 0, 1.169960);  //T2_NewMeshTrial
  // Vec3d v3d_C0 = Vec3d(-0.777000, 0, 0.127929); 
  // Vec3d v3d_D0 = Vec3d(0,0,0); 

  // Vec3d v3d_A0 = Vec3d(0.198000,0,1.041991);   
  // Vec3d v3d_B0 = Vec3d(-0.708445,0,1.169960);  //T4_NewMeshTrial
  // Vec3d v3d_C0 = Vec3d(-0.777000,0,0.319882); 
  // Vec3d v3d_D0 = Vec3d(0,0,0);

  // Vec3d v3d_A0 = Vec3d(0.725785,0,0.082555);
  // Vec3d v3d_B0 = Vec3d(0.603548,0,1.21175);
  // Vec3d v3d_C0 = Vec3d(-0.23684,0,0.977905);
  // Vec3d v3d_D0 = Vec3d(0,0,0);

  // ============================================//

  Vec3d v3d_A1 = Vec3d(0.103416,-0.001622,1.039259); 
  Vec3d v3d_A2 = Vec3d(0.079567,0.110935,0.965506);  
  Vec3d v3d_B1 = Vec3d(-0.802412,0.033388,1.054125);
  Vec3d v3d_B2 = Vec3d(-0.784751,0.119161,0.995961); //  T1_NewMeshTrial
  Vec3d v3d_C1 = Vec3d(-0.73471,0.0131175,0.083230); 
  Vec3d v3d_C2 = Vec3d(-0.731668,0.1049795,0.089480); 
  Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

  // Vec3d v3d_A1 = Vec3d(0.06288,-0.0036665,0.912126); 
  // Vec3d v3d_A2 = Vec3d(0.050563,0.0675485,0.896538);  
  // Vec3d v3d_B1 = Vec3d(-0.808377,-0.0010165,1.038368);
  // Vec3d v3d_B2 = Vec3d(-0.787239,0.093844,0.99416); //  T2_NewMeshTrial
  // Vec3d v3d_C1 = Vec3d(-0.767154,0.009518,0.003424); 
  // Vec3d v3d_C2 = Vec3d(-0.71960,0.115937,0.010751); 
  // Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  // Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

  // Vec3d v3d_A1 = Vec3d(0.136423,-0.00327655,1.051020); 
  // Vec3d v3d_A2 = Vec3d(0.047424,0.08600245,1.054323);  
  // Vec3d v3d_B1 = Vec3d(-0.766079,-0.03148895,1.132309);
  // Vec3d v3d_B2 = Vec3d(-0.735901,0.09783145,1.072893); //T4_NewMeshTrial
  // Vec3d v3d_C1 = Vec3d(-0.813698,0.0133445,0.287432); 
  // Vec3d v3d_C2 = Vec3d(-0.678888,0.09244145,0.262918); 
  // Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  // Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

  // Vec3d v3d_A1 = Vec3d(0.709837,0.032857,0.127454);
  // Vec3d v3d_A2 = Vec3d(0.567085,0.1330165,0.237639);
  // Vec3d v3d_B1 = Vec3d(0.509687,0.01593,1.257482);
  // Vec3d v3d_B2 = Vec3d(0.456382,0.2627485,1.11211);
  // Vec3d v3d_C1 = Vec3d(-0.313584,-0.015563,0.97006);
  // Vec3d v3d_C2 = Vec3d(-0.219983,0.2597755,0.93504);
  // Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0);
  // Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

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
    // Variables
  // cout << "IM MADE IT: 3.5" << endl;
    // Hard coded fix points
////////////////////////////////////////////////////////////////////////////////////////////
  // double zero= 0.00;
  // std::vector<double> vec_0 {0,0,0};

  // std::vector<int> fix0{273,3235,3236,4163};  //Alec 0630
  // std::vector<int> fix1{79,4216,8301,11896};      //Alec 0630
  // std::vector<int> fix2{259,3141,3142,4106};    //Alec 0630
  // std::vector<int> fix3{4080,4089,7401,7457};      //Alec 0630

  // std::vector <double> pos_A0 = {0.725785,0,0.082555};
  // std::vector <double> pos_B0 = {0.603548,0,1.21175};
  // std::vector <double> pos_C0 = {-0.23684,0,0.977905};
  // std::vector <double> pos_D0 = {0,0,0};

  // std::vector <double> pos_A1_mm = {709.837,32.857,127.454};
  // std::vector <double> pos_A2_mm = {567.085,133.0165,237.639};
  // std::vector <double> pos_B1_mm = {509.687,15.93,1257.482};
  // std::vector <double> pos_B2_mm = {456.382,262.7485,1112.11};
  // std::vector <double> pos_C1_mm = {-313.584,-15.563,970.06};
  // std::vector <double> pos_C2_mm = {-219.983,259.7755,935.04};
  // std::vector <double> pos_D1_mm = {0,0,0};
  // std::vector <double> pos_D2_mm = {0,0,0}; //Drop it actually

  // std::vector <double> pos_A1 = {pos_A1_mm[0]/1000,pos_A1_mm[1]/1000,pos_A1_mm[2]/1000};
  // std::vector <double> pos_A2 = {pos_A2_mm[0]/1000,pos_A2_mm[1]/1000,pos_A2_mm[2]/1000};
  // std::vector <double> pos_B1 = {pos_B1_mm[0]/1000,pos_B1_mm[1]/1000,pos_B1_mm[2]/1000};
  // std::vector <double> pos_B2 = {pos_B2_mm[0]/1000,pos_B2_mm[1]/1000,pos_B2_mm[2]/1000};
  // std::vector <double> pos_C1 = {pos_C1_mm[0]/1000,pos_C1_mm[1]/1000,pos_C1_mm[2]/1000};
  // std::vector <double> pos_C2 = {pos_C2_mm[0]/1000,pos_C2_mm[1]/1000,pos_C2_mm[2]/1000};
  // std::vector <double> pos_D1 = {pos_D1_mm[0]/1000,pos_D1_mm[1]/1000,pos_D1_mm[2]/1000};
  // std::vector <double> pos_D2 = {pos_D2_mm[0]/1000,pos_D2_mm[1]/1000,pos_D2_mm[2]/1000};

  // std::vector<std::vector<double>> pos0= {pos_A0,pos_B0,pos_C0,pos_D0};
  // std::vector<std::vector<double>> pos1= {pos_A1,pos_B1,pos_C1,pos_D1};
  // std::vector<std::vector<double>> pos2= {pos_A2,pos_B2,pos_C2,pos_D2};
  // std::vector<std::vector<std::vector<double>>> pos = {pos0, pos1, pos2};

  // std::vector<std::vector<std::vector<double>>> vec;
  // std::vector<std::vector<double>> temp_vec;
  // std::vector<double> temp_vec_point;
  // double val= 0.00;
  // for(int i=0; i<2; i++)
  // {
  //   for(int j=0; j<4; j++)
  //   {
  //     for(int k=0; k<3; k++)
  //     {
  //       val = pos[i+1][j][k]-pos[i][j][k];
  //       // cout << val <<endl;
  //       temp_vec_point.push_back(val);
  //     }
  //     temp_vec.push_back(temp_vec_point);
  //     temp_vec_point= {};
  //   }
  //   vec.push_back(temp_vec);
  //   temp_vec= {};
  // }
///////////////////////////////////////////////////////////////////////////////////////////////////

  // I MIGHT NEED THIS more often???? but it looks like it regenerates a new mesh and whole new simulator
  //This function has been modified to just get loadMesh
  // cout<<"objMeshname:"<< objMeshname<< endl;
  // cout<<"training_data:"<< training_data << endl;




  // cout<<"fixNum:"<< fixNum<< endl;
  // cout<<"is_permanent:"<< is_permanent<< endl;
  // cout << "IM MADE IT: 222" << endl;
  // cout<<"Calling updateSheetStatus NOW"<< endl;
  // cout << "IM MADE IT: 333" << endl;
    // cout << " I MADE IT!!" << endl;

  updateSheetStatus(objMeshname, training_data, param, fixNum, is_permanent);
  cout << training_data << endl;
  // updateParam(vp);
  // getParameters();

  double t =  2*getTimeStep();  //20
  int loop = 50;
  int start_time=0;
  int stop_time=50;
  // if (fixNum == 4){
  //   loop*=3;
  //   stop_time*=3;
  // }
  int extra_loops= 2;
  int extra_loops_counter= 0;
  for(int timeStepCount = 0; timeStepCount < loop; timeStepCount++)
  {
    resetConstraints();
    // cout << "t" << t << endl;
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
      // if (timeStepCount==(stop_time-1) && extra_loops_counter<extra_loops){
      //   timeStepCount= 0;
      //   extra_loops_counter++;
      // }
    }
    else{
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }
    //   if(i<50 && i>=100){
    //   // MoveSurfaceTo3D(fix0,vec[1][0],0.001*,0);
    //   MoveSurfaceTo3D(fix1,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix1,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix2,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix3,vec_0,zero,0);
    // }
    // else{
    //   MoveSurfaceTo3D(fix0,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix1,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix2,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix3,vesc_0,zero,0);
    // }
      // MoveSurfaceTo(fixA,vec_A,0.001*i,0);
      // MoveSurfaceTo(fixB,vec_B,0.001*i,0);
      // MoveSurfaceTo(fixC,vec_C,0.001*i,0);
      // MoveSurfaceTo(fixD,vec_D,0.001*i,0);


    cout << "." << flush;
    // cout << timeStepCount << endl;
    finalizeAllConstraints();
    simulate(1); //1
    // simulate(1);
    // simulate(time);
    // cout << time << endl;
  }

    // Vec3d offset;
    double avg_err = 0;
    double max_err = 0;
    double* distPTR = new double(10); //what does this mean??

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
    // sheet->getClosestVertex(v3d[1][3],distPTR);
    // cout << "  D: " << *distPTR << endl;
  }

  
  for(int i=0; i<m_training_marker.size(); i++){
    // cout << "m_training_marker[i]:" << m_training_marker[i] << endl;

    // cout << "type" << typeid(m_training_marker[i]).name << endl;
    // cout << "type" << type_name<decltype(m_training_marker[i])>() << endl;
    // std::vector<double>
    m_sceneObjDeform->GetClosestVertex(m_training_marker[i],distPTR); //capital G in Get
    // m_sceneObjDeform->getClosestVertex(m_training_marker[i],distPTR);
    // cout << "distPTR:" << *distPTR << endl;
    if(*distPTR==0)
      *distPTR=0.1;
    if(*distPTR>max_err)
        max_err = *distPTR;

    avg_err+=*distPTR; 
  }
  avg_err/= m_training_marker.size();

  cout << "avg_err:" << avg_err << endl;
  cout << "max_err:" << max_err << endl;


  // std::cout << "err: "<<err<<std::endl;
  delete distPTR;
  // return (avg_err*1+max_err*0.5);
  return (avg_err*1+max_err*2.0);
  // return getError();
}

double Composite_Training::solve_err_with_constraints2(const char* objMeshname,const char* training_data,
  const std::vector<double>& param,const int& fixNum, bool is_permanent, const int& time)
{

  double zero= 0.00;
  std::vector<double> vec_0 {0,0,0};

  // std::vector<int> fix0{11482,3511,12391,12390};  
  // std::vector<int> fix1{10215,2283,15410,16524};      
  // std::vector<int> fix2{7940,2050,15305,16228};    //T1_NewMeshTrial
  // std::vector<int> fix3{3954,11756,12207,12206};

  std::vector<int> fix0{1507,11878,12398,12397};  
  std::vector<int> fix1{16598,15412,3147,11163};      
  std::vector<int> fix2{11914,11915,2904,10947};    //T2_NewMeshTrial
  std::vector<int> fix3{2424,12227,12226,10400}; 

  // std::vector<int> fix0{11482,3511,12391,12390};  
  // std::vector<int> fix1{10215,2283,15410,16524};      
  // std::vector<int> fix2{916,7976,15312,16232};    //T4_NewMeshTrial2
  // std::vector<int> fix3{2424,12227,12226,10400};

  // std::vector<int> fix0{273,3235,3236,4163};  //Alec 0630
  // std::vector<int> fix1{79,4216,8301,11896};      //Alec 0630
  // std::vector<int> fix2{259,3141,3142,4106};    //Alec 0630
  // std::vector<int> fix3{4080,4089,7401,7457};      //Alec 0630

  // ================================================ //

  // std::vector<int> fix0Center{15539};  
  // std::vector<int> fix1Center{12132};      
  // std::vector<int> fix2Center{11922};    //T1_fix centers
  // std::vector<int> fix3Center{15447};

  std::vector<int> fix0Center{16347};  
  std::vector<int> fix1Center{12136};      
  std::vector<int> fix2Center{15301};    //T2_fix centers
  std::vector<int> fix3Center{15457};

  // std::vector<int> fix0Center{15539};  
  // std::vector<int> fix1Center{12132};      
  // std::vector<int> fix2Center{11937};    //T4_fix centers
  // std::vector<int> fix3Center{15457};

  // ============================================= //

  // std::vector<int> fix_tn0= {0,13,20,49,64,82,248,253,263,271,273,284};
  std::vector<int> fix_tn0= {248,253,284,0,20,13,82,64,49,273,263,271};  // order ABCD left mid right
  std::vector<int> fix_tn1= {48,30,11,279,252,281,200,232,246,4,22,3};  // order ABCD left mid right
  std::vector<int> fix_tn2= {288,316,332,2,31,0,48,64,47,333,307,320};  // order ABCD left mid right
  std::vector<int> fix_tn3= {51,19,13,260,257,280,203,234,244,4,30,3};  // order ABCD left mid right (234 is far, missing a point)
  std::vector<int> fix_tn4= {285,253,251,16,1,30,43,66,47};  // order ABCD left mid right last 3 241,265,239
  std::vector<std::vector<int>> fix_training_neighbors= {fix_tn0,fix_tn1,fix_tn2,fix_tn3,fix_tn4};

  // ================================================= //

  // Vec3d v3d_A0 = Vec3d(0.258900, 0, 1.041991);   
  // Vec3d v3d_B0 = Vec3d(-0.647545, 0, 1.169960);  //T1_NewMeshTrial
  // Vec3d v3d_C0 = Vec3d(-0.716100, 0, 0.191913); 
  // Vec3d v3d_D0 = Vec3d(0,0,0); 

  Vec3d v3d_A0 = Vec3d(0.198000, 0, 0.914022);   
  Vec3d v3d_B0 = Vec3d(-0.647508, 0, 1.169960);  //T2_NewMeshTrial
  Vec3d v3d_C0 = Vec3d(-0.777000, 0, 0.127929); 
  Vec3d v3d_D0 = Vec3d(0,0,0); 

  // Vec3d v3d_A0 = Vec3d(0.198000,0,1.041991);   
  // Vec3d v3d_B0 = Vec3d(-0.708445,0,1.169960);  //T4_NewMeshTrial
  // Vec3d v3d_C0 = Vec3d(-0.777000,0,0.319882); 
  // Vec3d v3d_D0 = Vec3d(0,0,0);

  // Vec3d v3d_A0 = Vec3d(0.725785,0,0.082555);
  // Vec3d v3d_B0 = Vec3d(0.603548,0,1.21175);
  // Vec3d v3d_C0 = Vec3d(-0.23684,0,0.977905);
  // Vec3d v3d_D0 = Vec3d(0,0,0);

  // ================================================= //

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

  // Vec3d v3d_A1 = Vec3d(0.136423,-0.00327655,1.051020); 
  // Vec3d v3d_A2 = Vec3d(0.047424,0.08600245,1.054323);  
  // Vec3d v3d_B1 = Vec3d(-0.766079,-0.03148895,1.132309);
  // Vec3d v3d_B2 = Vec3d(-0.735901,0.09783145,1.072893); //T4_NewMeshTrial
  // Vec3d v3d_C1 = Vec3d(-0.813698,0.0133445,0.287432); 
  // Vec3d v3d_C2 = Vec3d(-0.678888,0.09244145,0.262918); 
  // Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0); 
  // Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

  // Vec3d v3d_A1 = Vec3d(0.709837,0.032857,0.127454);
  // Vec3d v3d_A2 = Vec3d(0.567085,0.1330165,0.237639);
  // Vec3d v3d_B1 = Vec3d(0.509687,0.01593,1.257482);
  // Vec3d v3d_B2 = Vec3d(0.456382,0.2627485,1.11211);
  // Vec3d v3d_C1 = Vec3d(-0.313584,-0.015563,0.97006);
  // Vec3d v3d_C2 = Vec3d(-0.219983,0.2597755,0.93504);
  // Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0);
  // Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it

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
    // Variables
  // cout << "IM MADE IT: 3.5" << endl;
    // Hard coded fix points
////////////////////////////////////////////////////////////////////////////////////////////
  // double zero= 0.00;
  // std::vector<double> vec_0 {0,0,0};

  // std::vector<int> fix0{273,3235,3236,4163};  //Alec 0630
  // std::vector<int> fix1{79,4216,8301,11896};      //Alec 0630
  // std::vector<int> fix2{259,3141,3142,4106};    //Alec 0630
  // std::vector<int> fix3{4080,4089,7401,7457};      //Alec 0630

  // std::vector <double> pos_A0 = {0.725785,0,0.082555};
  // std::vector <double> pos_B0 = {0.603548,0,1.21175};
  // std::vector <double> pos_C0 = {-0.23684,0,0.977905};
  // std::vector <double> pos_D0 = {0,0,0};

  // std::vector <double> pos_A1_mm = {709.837,32.857,127.454};
  // std::vector <double> pos_A2_mm = {567.085,133.0165,237.639};
  // std::vector <double> pos_B1_mm = {509.687,15.93,1257.482};
  // std::vector <double> pos_B2_mm = {456.382,262.7485,1112.11};
  // std::vector <double> pos_C1_mm = {-313.584,-15.563,970.06};
  // std::vector <double> pos_C2_mm = {-219.983,259.7755,935.04};
  // std::vector <double> pos_D1_mm = {0,0,0};
  // std::vector <double> pos_D2_mm = {0,0,0}; //Drop it actually

  // std::vector <double> pos_A1 = {pos_A1_mm[0]/1000,pos_A1_mm[1]/1000,pos_A1_mm[2]/1000};
  // std::vector <double> pos_A2 = {pos_A2_mm[0]/1000,pos_A2_mm[1]/1000,pos_A2_mm[2]/1000};
  // std::vector <double> pos_B1 = {pos_B1_mm[0]/1000,pos_B1_mm[1]/1000,pos_B1_mm[2]/1000};
  // std::vector <double> pos_B2 = {pos_B2_mm[0]/1000,pos_B2_mm[1]/1000,pos_B2_mm[2]/1000};
  // std::vector <double> pos_C1 = {pos_C1_mm[0]/1000,pos_C1_mm[1]/1000,pos_C1_mm[2]/1000};
  // std::vector <double> pos_C2 = {pos_C2_mm[0]/1000,pos_C2_mm[1]/1000,pos_C2_mm[2]/1000};
  // std::vector <double> pos_D1 = {pos_D1_mm[0]/1000,pos_D1_mm[1]/1000,pos_D1_mm[2]/1000};
  // std::vector <double> pos_D2 = {pos_D2_mm[0]/1000,pos_D2_mm[1]/1000,pos_D2_mm[2]/1000};

  // std::vector<std::vector<double>> pos0= {pos_A0,pos_B0,pos_C0,pos_D0};
  // std::vector<std::vector<double>> pos1= {pos_A1,pos_B1,pos_C1,pos_D1};
  // std::vector<std::vector<double>> pos2= {pos_A2,pos_B2,pos_C2,pos_D2};
  // std::vector<std::vector<std::vector<double>>> pos = {pos0, pos1, pos2};

  // std::vector<std::vector<std::vector<double>>> vec;
  // std::vector<std::vector<double>> temp_vec;
  // std::vector<double> temp_vec_point;
  // double val= 0.00;
  // for(int i=0; i<2; i++)
  // {
  //   for(int j=0; j<4; j++)
  //   {
  //     for(int k=0; k<3; k++)
  //     {
  //       val = pos[i+1][j][k]-pos[i][j][k];
  //       // cout << val <<endl;
  //       temp_vec_point.push_back(val);
  //     }
  //     temp_vec.push_back(temp_vec_point);
  //     temp_vec_point= {};
  //   }
  //   vec.push_back(temp_vec);
  //   temp_vec= {};
  // }
///////////////////////////////////////////////////////////////////////////////////////////////////

  // I MIGHT NEED THIS more often???? but it looks like it regenerates a new mesh and whole new simulator
  //This function has been modified to just get loadMesh
  // cout<<"objMeshname:"<< objMeshname<< endl;
  // cout<<"training_data:"<< training_data << endl;




  // cout<<"fixNum:"<< fixNum<< endl;
  // cout<<"is_permanent:"<< is_permanent<< endl;
  // cout << "IM MADE IT: 222" << endl;
  // cout<<"Calling updateSheetStatus NOW"<< endl;
  // cout << "IM MADE IT: 333" << endl;
    // cout << " I MADE IT!!" << endl;

  updateSheetStatus(objMeshname, training_data, param, fixNum, is_permanent);
  cout << training_data << endl;
  // updateParam(vp);
  // getParameters();

  double t =  2*getTimeStep();  //20
  int loop = 50;
  int start_time=0;
  int stop_time=50;
  // if (fixNum == 4){
  //   loop*=3;
  //   stop_time*=3;
  // }
  int extra_loops= 2;
  int extra_loops_counter= 0;
  for(int timeStepCount = 0; timeStepCount < loop; timeStepCount++)
  {
    resetConstraints();
    // cout << "t" << t << endl;
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
      // if (timeStepCount==(stop_time-1) && extra_loops_counter<extra_loops){
      //   timeStepCount= 0;
      //   extra_loops_counter++;
      // }
    }
    else{
      MoveSurfaceTo3D(fix0,vec_0,zero,0);
      MoveSurfaceTo3D(fix1,vec_0,zero,0);
      MoveSurfaceTo3D(fix2,vec_0,zero,0);
      MoveSurfaceTo3D(fix3,vec_0,zero,0);
    }
    //   if(i<50 && i>=100){
    //   // MoveSurfaceTo3D(fix0,vec[1][0],0.001*,0);
    //   MoveSurfaceTo3D(fix1,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix1,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix2,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix3,vec_0,zero,0);
    // }
    // else{
    //   MoveSurfaceTo3D(fix0,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix1,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix2,vec_0,zero,0);
    //   MoveSurfaceTo3D(fix3,vesc_0,zero,0);
    // }
      // MoveSurfaceTo(fixA,vec_A,0.001*i,0);
      // MoveSurfaceTo(fixB,vec_B,0.001*i,0);
      // MoveSurfaceTo(fixC,vec_C,0.001*i,0);
      // MoveSurfaceTo(fixD,vec_D,0.001*i,0);


    cout << "." << flush;
    // cout << timeStepCount << endl;
    finalizeAllConstraints();
    simulate(1); //1
    // simulate(1);
    // simulate(time);
    // cout << time << endl;
  }

    // Vec3d offset;
    double avg_err = 0;
    double max_err = 0;
    double* distPTR = new double(10); //what does this mean??

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
    // sheet->getClosestVertex(v3d[1][3],distPTR);
    // cout << "  D: " << *distPTR << endl;
  }

  
  for(int i=0; i<m_training_marker.size(); i++){
    // cout << "m_training_marker[i]:" << m_training_marker[i] << endl;

    // cout << "type" << typeid(m_training_marker[i]).name << endl;
    // cout << "type" << type_name<decltype(m_training_marker[i])>() << endl;
    // std::vector<double>
    m_sceneObjDeform->GetClosestVertex(m_training_marker[i],distPTR); //capital G in Get
    // m_sceneObjDeform->getClosestVertex(m_training_marker[i],distPTR);
    // cout << "distPTR:" << *distPTR << endl;
    if(*distPTR==0)
      *distPTR=0.1;
    if(*distPTR>max_err)
        max_err = *distPTR;

    avg_err+=*distPTR; 
  }
  avg_err/= m_training_marker.size();

  cout << "avg_err:" << avg_err << endl;
  cout << "max_err:" << max_err << endl;


  // std::cout << "err: "<<err<<std::endl;
  delete distPTR;
  // return (avg_err*1+max_err*0.5);
  return (avg_err*1+max_err*2.0);
  // return getError();
}

