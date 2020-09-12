// Alec Main Functions Breakdown- Simulator Test 
//////////////////    1     /////////////////////////////////////
std::vector<int> fix0{273,3235,3236,4163};  //Alec 0630
std::vector<int> fix1{79,4216,8301,11896};      //Alec 0630
std::vector<int> fix2{259,3141,3142,4106};    //Alec 0630
std::vector<int> fix3{4080,4089,7401,7457};      //Alec 0630
////////////////      2     ///////////////////////////////////////
Vec3d v3d_A0 = Vec3d(0.725785,0,0.082555);
Vec3d v3d_B0 = Vec3d(0.603548,0,1.21175);
Vec3d v3d_C0 = Vec3d(-0.23684,0,0.977905);
Vec3d v3d_D0 = Vec3d(0,0,0);

Vec3d v3d_A1 = Vec3d(0.709837,0.032857,0.127454);
Vec3d v3d_A2 = Vec3d(0.567085,0.1330165,0.237639);
Vec3d v3d_B1 = Vec3d(0.509687,0.01593,1.257482);
Vec3d v3d_B2 = Vec3d(0.456382,0.2627485,1.11211);
Vec3d v3d_C1 = Vec3d(-0.313584,-0.015563,0.97006);
Vec3d v3d_C2 = Vec3d(-0.219983,0.2597755,0.93504);
Vec3d v3d_D1 = Vec3d(0.0,0.0,0.0);
Vec3d v3d_D2 = Vec3d(0.0,0.0,0.0); //Drop it
////////////////      3     ///////////////////////////////////////
std::vector<Vec3d> v3d0= {v3d_A0,v3d_B0,v3d_C0,v3d_D0};
std::vector<Vec3d> v3d1= {v3d_A1,v3d_B1,v3d_C1,v3d_D1};
std::vector<Vec3d> v3d2= {v3d_A2,v3d_B2,v3d_C2,v3d_D2};
std::vector<std::vector<Vec3d>> v3d = {v3d0, v3d1, v3d2};

std::vector<std::vector<std::vector<double>>> vec;
std::vector<std::vector<double>> temp_vec;
std::vector<double> temp_vec_point;

void make_v3d()
{
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
  // return vec;
}
//////////////////    4     /////////////////////////////////////
void idleFunction(void)
{
  static int xt= 1; //multiplier to slow motion down
  static int timeStepCount = 0;
  double dt = 2*sheet->getTimeStep()/xt;
  int sim_action_time_step= (int)round(1/dt);
  double err;
  // cout<<"dt" << dt <<endl;
  // cout<<"sim_action_time_step"<< sim_action_time_step <<endl;
  /////////////////////////////////////////
  static int t1= 0*sim_action_time_step; //Start move 0
  static int t2= 1*sim_action_time_step; // Stop move
  static int t3= 1*sim_action_time_step; //Error 0 and //Start move 1
  static int t4= 2*sim_action_time_step;
  static int t5= 2*sim_action_time_step;
  static int t6= 3*sim_action_time_step;
  static int t7= 3*sim_action_time_step;
  static int t8= 4*sim_action_time_step;
  static int t9= 4*sim_action_time_step;
  static int t15=7*sim_action_time_step;
  static int t30=15*sim_action_time_step;
  static int tinf=1000*sim_action_time_step;

  if(timeStepCount==t3){
    get_err(0);
    cout << "Timesteps before error calc: " << t3 <<endl;
    cout << "------------------------------------"<< endl;
  }
  else if(timeStepCount==t5){
    get_err(1);
    cout << "Timesteps before error calc: " << t5 <<endl;
    cout << "------------------------------------"<< endl;
  }
  else if(timeStepCount==t7){
    get_err(2);
    cout << "Timesteps before error calc: " << t7 <<endl;
    cout << "------------------------------------"<< endl;
  }
  else if(timeStepCount==t9){
    get_err(3);
    cout << "Timesteps before error calc: " << t9 <<endl;
    cout << "------------------------------------"<< endl;
  }
  else if(timeStepCount==t15){
    get_err(4);
    cout << "Timesteps before error calc: " << t15 <<endl;
    cout << "------------------------------------"<< endl;
  }
  else if(timeStepCount==t30){
    get_err(4);
    cout << "Timesteps before error calc: " << t30 <<endl;
    cout << "------------------------------------"<< endl;
  }

  sheet->resetConstraints();

  if(timeStepCount<t2 && timeStepCount>=t1){
    sheet->MoveSurfaceTo3D(fix0,vec[0][0],dt,0);
    sheet->MoveSurfaceTo3D(fix1,vec[0][1],dt,0);
    sheet->MoveSurfaceTo3D(fix2,vec[0][2],dt,0);
    sheet->MoveSurfaceTo3D(fix3,vec[0][3],dt,0);
  }
  else if(timeStepCount<t4 && timeStepCount>=t3){
    sheet->MoveSurfaceTo3D(fix0,vec[1][0],dt,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
  }
  else if(timeStepCount<t6 && timeStepCount>=t5){
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec[1][1],dt,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
  }
  else if(timeStepCount<t8 && timeStepCount>=t7){
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec[1][2],dt,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
  }
  else if(timeStepCount<tinf && timeStepCount>=t9){
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
  }
  else{
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
  }

  sheet->finalizeAllConstraints();
  
  // if (timeStepCount % 1000 ==0){
    cout <<"timeStepCount: "<<timeStepCount<<endl;
  // }

  deform = sheet->simulate(iterations);
  // cout<< "ITERATIONS::::::" << iterations << endl;
  // cout <<"iterations: "<<iterations<<endl;
  // sheet->getDeformInfo(max_deform, avg_deform);
  // std::cout<<"Max: "<<max_deform<<"\nAvg: "<<avg_deform<<std::endl;
  // energy = sheet->getKineticEnergy();
  // std::cout<<"Energy: "<<energy*1000<<std::endl;
  // std::cout<<"qvel: "<<sheet->getIntegrator()->Getqvel()[0]<<std::endl;
  
  timeStepCount++;
  
  ros::spinOnce();

  glutPostRedisplay();  
}
//////////////////    5     /////////////////////////////////////
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
//////////////////    6     /////////////////////////////////////
void get_err(int fixNumber)
{

  int fixNum = fixNumber;

  std::vector<int> fix_tn_list= fix_training_neighbors[fixNum];

  for (const auto &x: fix_tn_list){
    // cout << x << endl;
    sort(fix_tn_list.begin(), fix_tn_list.end());
  }
  // for (const auto &x: fix_tn_list){
    // cout << "fix_tn_list_sorted:  " << x << endl;
  // }

  std::vector<std::string> training_data(5);
  training_data[0]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S1.txt";
  training_data[1]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S2.txt";
  training_data[2]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S3.txt";
  training_data[3]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S4.txt";
  training_data[4]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S5.txt";
  // cout << 'training_data:' << training_data[fixNum] << endl; 
  // Vec3d offset;
  if(m_training_marker.size() != 0){
    m_training_marker.clear();
  }
  parseData(training_data[fixNum].data(), dummie_outter_marker, m_training_marker, 5,1); //5 and 1 and dummie not used. m_training is output
  // parseData(training_data[fixNum], m_training_marker);
  // m_training_marker= get_training_points(fixNum)
  double avg_err = 0;
  double max_err = 0;
  double fix_neighbor_err = 0; 
  double* distPTR = new double(10); //what does this mean??
  int j=0;

  // Alec 0707 finding error of fixed points (should be VERY small)
  if (fixNum==0){
    sheet->getClosestVertex(v3d[1][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==1){
    sheet->getClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==2){
    sheet->getClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==3){
    sheet->getClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[2][2],distPTR); 
    cout << "  C: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[1][3],distPTR);
    cout << "  D: " << *distPTR << endl;
  }
    if (fixNum==4){
    sheet->getClosestVertex(v3d[2][0],distPTR);
    cout << "fix_pt_errors- A: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[2][1],distPTR); 
    cout << "  B: " << *distPTR;// << endl;
    sheet->getClosestVertex(v3d[2][2],distPTR); 
    cout << "  C: " << *distPTR << endl;
    // sheet->getClosestVertex(v3d[1][3],distPTR);
    // cout << "  D: " << *distPTR << endl;
  }
  for(int i=0; i<m_training_marker.size(); i++){
    // cout << "m_training_marker[i]:" << m_training_marker[i] << endl;
    // cout << "type" << typeid(m_training_marker[i]).name << endl;
    // cout << "type" << type_name<decltype(m_training_marker[i])>() << endl;
    // m_sceneObjDeform->GetClosestVertex(m_training_marker[i],distPTR); //capital G in Get
    sheet->getClosestVertex(m_training_marker[i],distPTR); //capital G in Get
    
    //Alec added 0707 finding error of fix point neighbors, should be small
    if(i==fix_tn_list[j]){
      // cout << "--------FIX NEIGHBOR HERE------" << fix_training_neighbors[j] << endl;
      fix_neighbor_err+= *distPTR;
      j++;
    }
    // m_sceneObjDeform->getClosestVertex(m_training_marker[i],distPTR);
    // cout << "distPTR:" << *distPTR << endl;
    if(*distPTR==0)
      *distPTR=0.1;
    if(*distPTR>max_err)
        max_err = *distPTR;

    avg_err+=*distPTR; 
  }
  avg_err/= m_training_marker.size();
  fix_neighbor_err/= fix_tn_list.size();
  cout << "avg_err:" << avg_err << endl;
  cout << "max_err:" << max_err << endl;
  cout << "fix_neighbor_err:" << fix_neighbor_err << endl;
  // std::cout << "err: "<<err<<std::endl;
  delete distPTR;
  cout << "avg_err*1+max_err*0.5:   " << avg_err*1+max_err*0.5 << endl;
  // return (avg_err*1+max_err*0.5);
  // return getError();
}