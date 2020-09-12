#include <ros/ros.h>
/***********************************************/
//SIMULATOR HEADERS
/************************************************/
#include "sheet_model_training/composite_training_interface.hpp"
#include "GL/glui.h"
#include "composite_utilities/initGraphics.h"

#include "configFile.h"
#include "listIO.h"

#include <algorithm> //for std::sort

#define Mm_PI 3.1415926
#define  MAX_FILE 4096


/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <fstream>
/*******************************************/
//ROS HEADERS
/********************************************/



using namespace std;




void initScene();

//glui
GLUI * glui = NULL;
GLUI_StaticText * systemSolveStaticText;
GLUI_StaticText * forceAssemblyStaticText;



// graphics
char windowTitleBase[4096] = "Composite Simulator";
int windowID;
int windowWidth = 800;
int windowHeight = 600;

//interactive control
double zNear = 0.01;               //default: 0.01
double zFar = 10.0;                //default:10.0;
double cameraRadius;
double focusPositionX, focusPositionY, focusPositionZ;
double cameraLongitude, cameraLatitude;
int g_iMenuId;      // mouse activity
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;
double forceAssemblyTime = 0.0;
double systemSolveTime = 0.0;

// start out paused, wire-frame view, scene unlocked (you can drag to add forces)
int runSimulation=0, renderWireframe=1, saveScreenToFile=0, dragForce = 0, pulledVertex = -1, lockScene = 0, axis = 1, pin = 0, sprite=0, renderNormals = 0, displayMenu = 0, useTextures = 1;
int renderFixedVertices = 1;
int shiftPressed=0;
int altPressed=0;
int ctrlPressed=0;

int dragStartX, dragStartY;
std::vector<int> pin_points;
int graphicsFrame = 0;
Lighting * light = NULL;


char configFilename[MAX_FILE];
char lightingFilename[MAX_FILE];
// camera
SphericalCamera * camera = NULL;

// files
SceneObject * extraSceneGeometry = NULL;
      
//composite interface////////////////////////////////////////
Composite *sheet;
bool is_FEM_on;
float density;
float timesteps;
int iterations;
float thickness;
float poisson;
float ShearAndStretchMaterial;       
float bendMaterial;
float tensileStiffness;       
float shearStiffness;         
float bendStiffnessUV;



// float density=0.47;
// float timesteps = 0.05;
// int iterations = 10; //10 alec 0702
// // float iterations=1.0;
// //FEM/////
// float thickness=0.001;
// float poisson=0.3;
// // float ShearAndStretchMaterial=1e5;       
// // float bendMaterial=1e10;
// // float ShearAndStretchMaterial=3.3e6;       
// // float bendMaterial=3.3e9;  
// float ShearAndStretchMaterial=3.3e4;       
// float bendMaterial=3.3e7;  
// // float ShearAndStretchMaterial=3.3e4;       
// // float bendMaterial=3.3e7;  
// // float stretch=3.3e6,float bend=3.3e9
// //NO_FEM///
// float tensileStiffness=8000.0;       
// float shearStiffness=8000.0;         
// float bendStiffnessUV=0.001;

std::vector<double>vp;
double* deform;
std::string meshfile, fix_file,compare_file;
std::vector<Vec3d> grippingPts;
vector<Vec3d> split_lo,split_hi;
Vec3d hight_color = Vec3d(1.0,0.0,1.0);

std::vector<Vec3d> inner_marker;
std::vector<Vec3d> outter_marker;

std::vector<Vec3d> dummie_outter_marker;  // Alec 070620
std::vector<Vec3d> m_training_marker; //Alec 070620

// std::vector<int> fix1{5583,8268,10292,10296};
// std::vector<int> fix2{20,3665,3894,4147};
// std::vector<int> fix3{1013,2154,2709,4070};
// std::vector<int> fix4{75,1436,3721,4183};

// std::vector<int> fix1{249,2144,3069,4069};  //Alec 0629
// std::vector<int> fix2{75,4181,8095,11832};      //Alec 0629
// std::vector<int> fix3{285,2599,3380,4210};    //Alec 0629
// std::vector<int> fix4{4136,4145,7725,7781};      //Alec 0629
double zero= 0.00;
std::vector<double> vec_0 {0,0,0};

std::vector<int> fix0{273,3235,3236,4163};  //Alec 0630
std::vector<int> fix1{79,4216,8301,11896};      //Alec 0630
std::vector<int> fix2{259,3141,3142,4106};    //Alec 0630
std::vector<int> fix3{4080,4089,7401,7457};      //Alec 0630

// std::vector<int> fix_tn0= {0,13,20,49,64,82,248,253,263,271,273,284};
std::vector<int> fix_tn0= {248,253,284,0,20,13,82,64,49,273,263,271};  // order ABCD left mid right
std::vector<int> fix_tn1= {48,30,11,279,252,281,200,232,246,4,22,3};  // order ABCD left mid right
std::vector<int> fix_tn2= {288,316,332,2,31,0,48,64,47,333,307,320};  // order ABCD left mid right
std::vector<int> fix_tn3= {51,19,13,260,257,280,203,234,244,4,30,3};  // order ABCD left mid right (234 is far, missing a point)
std::vector<int> fix_tn4= {285,253,251,16,1,30,43,66,47};  // order ABCD left mid right last 3 241,265,239
std::vector<std::vector<int>> fix_training_neighbors= {fix_tn0,fix_tn1,fix_tn2,fix_tn3,fix_tn4};

//////////////////////////////////////////////////////////////////////////////////////////////////////////
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
// std::vector <double> pos_D2_mm = {0,0,0}; //Drop it

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
///////////////////////////////////////////////////////////////////////////////////////////////////////////

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

std::vector<Vec3d> v3d0= {v3d_A0,v3d_B0,v3d_C0,v3d_D0};
std::vector<Vec3d> v3d1= {v3d_A1,v3d_B1,v3d_C1,v3d_D1};
std::vector<Vec3d> v3d2= {v3d_A2,v3d_B2,v3d_C2,v3d_D2};
std::vector<std::vector<Vec3d>> v3d = {v3d0, v3d1, v3d2};

std::vector<std::vector<std::vector<double>>> vec;
std::vector<std::vector<double>> temp_vec;
std::vector<double> temp_vec_point;

double val= 0.0;
// std::vector<std::vector<std::vector<double>>> vec;
// std::vector<std::vector<double>> temp_vec;
// std::vector<double> temp_vec_point;
// double val= 0.0;
// for(int i=0; i<2; i++)
// {
//   for(int j=0; j<4; j++)
//   {
//     for(int k=0; k<3; k++)
//     {
//       val = pos[i][j][k]-pos[i+1][j][k];
//       temp_vec_point.push_back(val);
//     }
//     temp_vec.push_back(temp_vec_point);
//   }
//   vec.push_back(temp_vec);
// }
// std::vector <double> vec_A0 = {pos_A1[0]-pos_A0[0],};
// std::vector <double> vec_B0 = pos_B1[1]-pos_A0;
// std::vector <double> vec_C0 = pos_A1-pos_A0;
// std::vector <double> vec_D0 = pos_A1-pos_A0;

// std::vector <double> vec_A1 = pos_A2-pos_A1;
// std::vector <double> vec_B1 = pos_A2-pos_A1;
// std::vector <double> vec_C1 = pos_A2-pos_A1;
// std::vector <double> vec_D1 = pos_A2-pos_A1;

// std::vector <double> vec_A = {110.185,-142.752,100.1595};
// std::vector <double> vec_A = {3*110.185,-3*142.752,3*100.1595};
// std::vector <double> vec_B = {-145.371,-53.305,246.8185};
// std::vector <double> vec_C = {-35.02,93.601,275.3385};
// std::vector <double> vec_C = {0.1*-35.02,0.1*93.601,0.1*275.3385};
// std::vector <double> vec_D = {0,0,0};


///////////////////////////////////////////////////////////

// This function specifies parameters (and default values, if applicable) for the
// configuration file. It then loads the config file and parses the options contained
// within. After parsing is complete, a list of parameters is printed to the terminal.
void initConfigurations()
{
  printf("Parsing configuration file %s...\n", configFilename);
  ConfigFile configFile;

  configFile.addOptionOptional("focusPositionX", &focusPositionX, 0.0);
  configFile.addOptionOptional("focusPositionY", &focusPositionY, 10.0);
  configFile.addOptionOptional("focusPositionZ", &focusPositionZ, 0.0);
  configFile.addOptionOptional("cameraRadius", &cameraRadius, 6.0);
  configFile.addOptionOptional("cameraLongitude", &cameraLongitude, -10.0);
  configFile.addOptionOptional("cameraLatitude", &cameraLatitude, 45.0);
  configFile.addOptionOptional("zBufferNear", &zNear, 0.01);
  configFile.addOptionOptional("zBufferFar", &zFar, 10.0);
  configFile.addOptionOptional("renderWireframe", &renderWireframe, renderWireframe);


  configFile.addOption("is_FEM_on", &is_FEM_on);
  configFile.addOption("density", &density);
  configFile.addOption("timesteps", &timesteps);
  configFile.addOption("iterations", &iterations);


      configFile.addOption("thickness", &thickness);
      configFile.addOption("poisson", &poisson);
      configFile.addOption("ShearAndStretchMaterial", &ShearAndStretchMaterial);
      configFile.addOption("bendMaterial", &bendMaterial);
  

  
      configFile.addOption("tensileStiffness", &tensileStiffness);
      configFile.addOption("shearStiffness", &shearStiffness);
      configFile.addOption("bendStiffnessUV", &bendStiffnessUV);
  


  
  // parse the configuration file
  if (configFile.parseOptions(configFilename) != 0)
  {
    printf("Error parsing options.\n");
    exit(1);
  }
 
  configFile.printOptions();

}


void drawString(const char * str) 
{
  glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT); // lighting and color mask
  glDisable(GL_LIGHTING);     // need to disable lighting for proper text color
  
  glColor3f(1.0, 1.0, 1.0); // set text color
  
  // loop all characters in the string
  while(*str)
  {
    glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *str);
    ++str;
  }
  
  glEnable(GL_LIGHTING);
  glPopAttrib();
}

void drawdata(){
    glDisable(GL_LIGHTING);

      for(int i=0; i<sheet->getGrapsingPoints().size(); i++)
      {
        glColor3f(1,0,1);
        glEnable(GL_POLYGON_OFFSET_POINT);
        glPolygonOffset(-1.0,-1.0);
        glPointSize(15.0);
        glBegin(GL_POINTS);
        glVertex3f(sheet->getGrapsingPoints()[i][0], sheet->getGrapsingPoints()[i][1], sheet->getGrapsingPoints()[i][2]);
        glEnd();
        glDisable(GL_POLYGON_OFFSET_FILL);
      }
      if(sheet->getPermanentFixedID().size()!=0){
        for(int i=0; i<sheet->getPermanentFixedID().size(); i++)
        {
          Vec3d temp = sheet->getSceneObj()->GetVertexPosition(sheet->getPermanentFixedID()[i]);
          glColor3f(1,0,0);
          glEnable(GL_POLYGON_OFFSET_POINT);
          glPolygonOffset(-1.0,-1.0);
          glPointSize(15.0);
          glBegin(GL_POINTS);
          glVertex3f(temp[0], temp[1], temp[2]);
          glEnd();
          glDisable(GL_POLYGON_OFFSET_FILL);
        }
      }
      if(outter_marker.size()!=0 && inner_marker.size()!=0){
              for(int i=0; i<outter_marker.size(); i++)
        {
          glColor3f(1,0,1);
          glEnable(GL_POLYGON_OFFSET_POINT);
          glPolygonOffset(-1.0,-1.0);
          glPointSize(15.0);
          glBegin(GL_POINTS);
          glVertex3f(outter_marker[i][0], outter_marker[i][1], outter_marker[i][2]);
          glEnd();
          glDisable(GL_POLYGON_OFFSET_FILL);
        }

        for(int i=0; i<inner_marker.size(); i++)
        {
          glColor3f(0,0,0);
          glEnable(GL_POLYGON_OFFSET_POINT);
          glPolygonOffset(-1.0,-1.0);
          glPointSize(15.0);
          glBegin(GL_POINTS);
          glVertex3f(inner_marker[i][0], inner_marker[i][1], inner_marker[i][2]);
          glEnd();
          glDisable(GL_POLYGON_OFFSET_FILL);
        }

        // for(int i=0; i<m_training_marker.size(); i++)
        // {  // ALEC added 070620
        //   glColor3f(0,0,0);
        //   glEnable(GL_POLYGON_OFFSET_POINT);
        //   glPolygonOffset(-1.0,-1.0);
        //   glPointSize(15.0);
        //   glBegin(GL_POINTS);
        //   glVertex3f(m_training_marker[i][0], m_training_marker[i][1], m_training_marker[i][2]);
        //   glEnd();
        //   glDisable(GL_POLYGON_OFFSET_FILL);
        // }
      }

    
    glEnable(GL_LIGHTING);
}

// this function does the following:
// (1) Clears display
// (2) Points camera at scene
// (3) Draws axes (if applicable) and sphere surrounding scene
// (4) Displays GUI menu
// (5) Sets lighting conditions
// (6) Builds surface normals for cloth (take this out to increase performance)
// (7) Renders cloth
// (8) Render pulled vertex in different color (if applicable)
void displayFunction()
{
  
  // std::cout <<"display enter"<<std::endl;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW); 
  glLoadIdentity(); 
  camera->Look();
  glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
  glStencilFunc(GL_ALWAYS, 0, ~(0u));

  // render any extra scene geometry
  glStencilFunc(GL_ALWAYS, 0, ~(0u));
  if (extraSceneGeometry != NULL)
  {

      glDisable(GL_LIGHTING);
      glColor3f(0.5,1,0.2);
      extraSceneGeometry->Render();
      glEnable(GL_LIGHTING);


      glDisable(GL_LIGHTING);
      glColor3f(0,0,0);
      extraSceneGeometry->EnableFlatFaces();
      extraSceneGeometry->RenderFacesAndEdges();
      glEnable(GL_LIGHTING);
  }

  if(axis)
  {
    glDisable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);

    glBegin(GL_LINES);
    for (int i = 0; i < 3; i++)
    {
      float color[3] = { 0, 0, 0 };
      color[i] = 1.0;
      glColor3fv(color);

      float vertex[3] = {0, 0, 0};
      vertex[i] = 1.0;
      glVertex3fv(vertex);
      glVertex3f(0, 0, 0);
    }
    glEnd();
    glEnable(GL_LIGHTING);
  }
 
  
  // render cloth
  if (sheet->getSceneObj() != NULL)
  { 

    glLineWidth(1.0);
    glStencilFunc(GL_ALWAYS, 1, ~(0u));

    

    sheet->getSceneObj()->SetLighting(light);

    sheet->getSceneObj()->BuildNormalsFancy();

    if (renderNormals)
    {
      glDisable(GL_LIGHTING);
      glColor3f(0,0,1);
      sheet->getSceneObj()->RenderNormals();
      glEnable(GL_LIGHTING);
    }

    // render fixed vertices
    glDisable(GL_LIGHTING);
    // if (renderFixedVertices)
    // {
    //   for(int i=0; i<sheet->getFixedGroups().size(); i++)
    //   {
    //     glColor3f(1,0,0);
    //     double fixedVertexPos[3];
    //     sheet->getSceneObj()->GetSingleVertexRestPosition(sheet->getFixedGroups()[i],
    //         &fixedVertexPos[0], &fixedVertexPos[1], &fixedVertexPos[2]);

    //     glEnable(GL_POLYGON_OFFSET_POINT);
    //     glPolygonOffset(-1.0,-1.0);
    //     glPointSize(12.0);
    //     glBegin(GL_POINTS);
    //     glVertex3f(fixedVertexPos[0], fixedVertexPos[1], fixedVertexPos[2]);
    //     glEnd();
    //     glDisable(GL_POLYGON_OFFSET_FILL);
    //   }
    // }
    drawdata();

    for(int i=0; i<sheet->getConstraintID().size(); i++)
      sheet->getSceneObj()->HighlightVertex(sheet->getConstraintID()[i],hight_color);

    glEnable(GL_LIGHTING);
    sheet->getSceneObj()->Render();

    if (renderWireframe)
    {
      glDisable(GL_LIGHTING);
      glColor3f(0,0,0);
      sheet->getSceneObj()->RenderEdges();
      glEnable(GL_LIGHTING);
    }
  }
  
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
  glutSwapBuffers();
}


void make_Vec()
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

// void get_training_points(int fixNumber){
//   int fixNum = fixNumber;
//   std::vector<std::string> training_data(5);
//   std::string training_data[0]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S1.txt";
//   std::string training_data[1]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S2.txt";
//   std::string training_data[2]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S3.txt";
//   std::string training_data[3]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S4.txt";
//   std::string training_data[4]= "/home/cam_sanding/composite_ws/src/sheet_model_training/data/AlecZach/T1S5.txt";
//   cout << 'training_data:' << training_data[fixNum] << endl; 
//   // parseData(training_data, m_training_fix, m_training_marker, m_fixGroups,fixNum, 1);
//   // parseData(compare_file.data(), outter_marker, inner_marker, 5,1);
//   // try
//   //   {std::vector<Vec3d>& point}
//   // if(m_training_marker.size() != 0){
//   //    m_training_marker.clear();
//   //  }
//   parseData(training_data[fixNum], dummie_outter_marker, m_training_marker, 5,1); //5 and 1 and dummie not used. m_training is output
//   return m_training_marker;
// }

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


// void parseData_Alec(const char* filename, std::vector<Vec3d>& fixed, std::vector<Vec3d>& point,const int& fixNum, const int& ifCenter){
//     std::fstream fin;
//     std::cout<<"file name:"<< filename << std::endl;
//       fin.open(filename);
//     if(!fin.is_open()){
//       std::cout<<"open failed"<<std::endl;
//     }
//     int fix_number = fixNum;
//       double unit_conversion = 1000; //Alec added 070320
//     fix_number*=4;
//     std::string line;
//     std::vector<std::string> coord;
//     std::string value;
//     // int count=0;
//     int i=0;
//     if(fixed.size()!=0)
//       fixed.clear();
//     if(point.size()!=0)
//       point.clear();
//     while(getline(fin,line)){
//         std::stringstream ss(line);
//         coord.clear();
//         while(getline(ss,value,' ')){
//           coord.push_back(value);
//           // std::cout<<value<<std::endl;
//         }
//         // Alec removed 070320 ///////////////////
//         // if(i>ifCenter-1 && i<fix_number+ifCenter)
//           // fixed.push_back(Vec3d(stod(coord[0])/unit_conversion,stod(coord[1])/unit_conversion,stod(coord[2])/unit_conversion));
//         // else if(i>ifCenter-1)
//         point.push_back(Vec3d(stod(coord[0])/unit_conversion,stod(coord[1])/unit_conversion,stod(coord[2])/unit_conversion));
//         i++;
//     }
// }

// void parseTrainingData_Alec(const char* filename, const ObjMesh* obj, std::vector<Vec3d>& training_fix, 
//                 std::vector<Vec3d>& training_marker, std::vector<int>& fixgroups,const int& fixNum, const int& ifCenter){

//   parseData_Alec(filename, training_fix, training_marker, fixNum, ifCenter);
//   for(int i=0; i< training_fix.size(); i++)
//     fixgroups.push_back(obj->getClosestVertex(training_fix[i]));
//   std::sort(fixgroups.begin(), fixgroups.end());
// }
// This function does the following:
//MoveSurfaceTo allows the user to move constraints along specified direction and distance
//MoveSurfaceTo(vector<Vec3d>,direction,distance)
void idleFunction(void)
{
  static int xt= 3;
  static int t1= 50*xt;
  static int t2= 100*xt;
  static int t3= 150*xt;
  static int t4= 200*xt;
  static int t5= 250*xt;
  static int t6= 300*xt;
  static int t7= 350*xt;
  static int t8= 400*xt;
  static int t9= 450*xt;
  static int t10=750*xt;
  static int t11=1500*xt;
  static int tinf=50000*xt;
  double err; 
  static int timeStepCount = 0;
  // if(timeStepCount==0){
  //   get_err(0);
  //   cout << "Timesteps before error calc: " << t3 <<endl;
  //   cout << "------------------------------------"<< endl;
  // }
  // get_err(3);
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
  else if(timeStepCount==t10){
    get_err(4);
    cout << "Timesteps before error calc: " << t10 <<endl;
    cout << "------------------------------------"<< endl;
    // alec_main()
  }
  else if(timeStepCount==t11){
    get_err(4);
    cout << "Timesteps before error calc: " << t11 <<endl;
    cout << "------------------------------------"<< endl;
    // alec_main()
  }
  double t =  2*sheet->getTimeStep()/xt;
  // double t =  sheet->getTimeStep()/xt;
  // cout << "t:" << t << endl;
  sheet->resetConstraints();

  if(timeStepCount<t2 && timeStepCount>=t1){
    // cout <<"X: "<<vec[0][0][0]<<"    Y: "<<vec[0][0][1]<<"   Z: "<<vec[0][0][2]<<endl;
    // cout <<"X: "<<vec[0][1][0]<<"    Y: "<<vec[0][1][1]<<"   Z: "<<vec[0][1][2]<<endl;
    // cout <<"X: "<<vec[0][2][0]<<"    Y: "<<vec[0][2][1]<<"   Z: "<<vec[0][2][2]<<endl;
    // cout <<"X: "<<vec[0][3][0]<<"    Y: "<<vec[0][3][1]<<"   Z: "<<vec[0][3][2]<<endl;
    // cout <<"X: "<<vec[1][0][0]<<"    Y: "<<vec[1][0][1]<<"   Z: "<<vec[1][0][2]<<endl;
    // cout <<"X: "<<vec[1][1][0]<<"    Y: "<<vec[1][1][1]<<"   Z: "<<vec[1][1][2]<<endl;
    // cout <<"X: "<<vec[1][2][0]<<"    Y: "<<vec[1][2][1]<<"   Z: "<<vec[1][2][2]<<endl;
    // cout <<"X: "<<vec[1][3][0]<<"    Y: "<<vec[1][3][1]<<"   Z: "<<vec[1][3][2]<<endl;
    sheet->MoveSurfaceTo3D(fix0,vec[0][0],t,0);
    sheet->MoveSurfaceTo3D(fix1,vec[0][1],t,0);
    sheet->MoveSurfaceTo3D(fix2,vec[0][2],t,0);
    sheet->MoveSurfaceTo3D(fix3,vec[0][3],t,0);
  }
  else if(timeStepCount<t4 && timeStepCount>=t3){
    sheet->MoveSurfaceTo3D(fix0,vec[1][0],t,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
  }

  else if(timeStepCount<t6 && timeStepCount>=t5){
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec[1][1],t,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
  }

  else if(timeStepCount<t8 && timeStepCount>=t7){
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec[1][2],t,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
  }

  else if(timeStepCount<tinf && timeStepCount>=t9){
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
  }
  else{
    // cout <<"stop"<<endl;
    // sheet->Fun(fix4,vec_D);
    sheet->MoveSurfaceTo3D(fix0,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix1,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix2,vec_0,zero,0);
    sheet->MoveSurfaceTo3D(fix3,vec_0,zero,0);
    // if(timeStepCount>=400)
      // timeStepCount=190;
  }




//   if(timeStepCount<100 && timeStepCount>=50){
//     // cout <<"push"<<endl;
// sheet->MoveSurfaceTo3D(fix1,vec_A,0.001*t,0);
// sheet->MoveSurfaceTo3D(fix2,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix3,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix4,vec_D,zero,0);
//   }
//   //vec[0][][]
//   if(timeStepCount<100 && timeStepCount>=50){
//     cout <<"push"<<endl;
//     cout <<vec[0][0][0]<<endl;
// sheet->MoveSurfaceTo3D(fix1,vec_A,0.001*t,0);
// sheet->MoveSurfaceTo3D(fix2,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix3,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix4,vec_D,zero,0);
//   }
//    else if(timeStepCount<200 && timeStepCount>=150){
// sheet->MoveSurfaceTo3D(fix1,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix2,vec_B,-0.001*t,0);
// sheet->MoveSurfaceTo3D(fix3,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix4,vec_D,zero,0);
// }
//    else if(timeStepCount<300 && timeStepCount>=250){
// sheet->MoveSurfaceTo3D(fix1,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix2,vec_D,zero,0);
// sheet->MoveSurfaceTo3D(fix3,vec_C,-0.001*t,0);
// sheet->MoveSurfaceTo3D(fix4,vec_D,zero,0);
// //     // cout <<"pull"<<endl;
// // sheet->MoveSurfaceTo(fix1,0,0);
// // sheet->MoveSurfaceTo(fix2,2,0.004*t);
// // sheet->MoveSurfaceTo(fix3,0,0);
// // sheet->MoveSurfaceTo(fix4,0,0);
//    }

//   else if(timeStepCount<270 && timeStepCount>=260){
// sheet->MoveSurfaceTo(fix1,0,0);
// sheet->MoveSurfaceTo(fix2,0,0);
// sheet->MoveSurfaceTo(fix3,0,0.004*t);
// sheet->MoveSurfaceTo(fix4,0,0);
//   }
//   else if(timeStepCount<300 && timeStepCount>=290){
// sheet->MoveSurfaceTo(fix1,0,0);
// sheet->MoveSurfaceTo(fix2,0,0);
// sheet->MoveSurfaceTo(fix3,0,0);
// sheet->MoveSurfaceTo(fix4,2,-0.004*t);
//   }


  sheet->finalizeAllConstraints();
  // cout <<"count: "<<timeStepCount<<endl;

  // double max_deform;
  // double avg_deform;
  // double energy;
  //simulate sheet behavior
  deform = sheet->simulate(iterations);
  // cout<< "ITERATIONS::::::" << iterations << endl;
  // cout <<"iterations: "<<iterations<<endl;
    //   sheet->getDeformInfo(max_deform, avg_deform);
    // std::cout<<"Max: "<<max_deform<<"\nAvg: "<<avg_deform<<std::endl;
    // energy = sheet->getKineticEnergy();
    // std::cout<<"Energy: "<<energy*1000<<std::endl;
    // std::cout<<"qvel: "<<sheet->getIntegrator()->Getqvel()[0]<<std::endl;
  
  timeStepCount++;
  
  ros::spinOnce();

  glutPostRedisplay();  
}


void reshape(int x,int y)
{
  std::cout<< "reshape"<<std::endl;
  glViewport(0,0,x,y);
  
  windowWidth = x;
  windowHeight = y;
  
  glMatrixMode(GL_PROJECTION); // Select The Projection Matrix
  glLoadIdentity(); // Reset The Projection Matrix
  
  // gluPerspective(90.0,1.0,0.01,1000.0);
  gluPerspective(60.0f, 1.0 * windowWidth / windowHeight, zNear, zFar);
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void exit_buttonCallBack(int code)
{
  // free memory
  std::cout<< "exit_buttonCallBack"<<std::endl;
  delete sheet;
  
  exit(0);
}

void keyboardFunction(unsigned char key, int x, int y)
{

}

// reacts to pressed "special" keys
void specialFunction(int key, int x, int y)
{
  switch (key)
  {
    case GLUT_KEY_LEFT:
      camera->MoveFocusRight(+0.1 * camera->GetRadius());
    break;

    case GLUT_KEY_RIGHT:
      camera->MoveFocusRight(-0.1 * camera->GetRadius());
    break;

    case GLUT_KEY_DOWN:
      camera->MoveFocusUp(+0.1 * camera->GetRadius());
    break;

    case GLUT_KEY_UP:
      camera->MoveFocusUp(-0.1 * camera->GetRadius());
    break;

    case GLUT_KEY_PAGE_UP:
      break;

    case GLUT_KEY_PAGE_DOWN:
      break;

    case GLUT_KEY_HOME:
      break;

    case GLUT_KEY_END:
      break;

    case GLUT_KEY_INSERT:
      break;

    default:
      break;
  }
}

void mouseMotion (int x, int y)
{
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}

void mouseButtonActivityFunction(int button, int state, int x, int y)
{
  switch (button)
  {
    case GLUT_LEFT_BUTTON:
      g_iLeftMouseButton = (state==GLUT_DOWN);
      shiftPressed = (glutGetModifiers() == GLUT_ACTIVE_SHIFT);
      altPressed = (glutGetModifiers() == GLUT_ACTIVE_ALT);
      ctrlPressed = (glutGetModifiers() == GLUT_ACTIVE_CTRL);
      if (g_iLeftMouseButton)
      {
        GLdouble model[16];
        glGetDoublev (GL_MODELVIEW_MATRIX, model);
        
        GLdouble proj[16];
        glGetDoublev (GL_PROJECTION_MATRIX, proj);
        
        GLint view[4];
        glGetIntegerv (GL_VIEWPORT, view);
        
        int winX = x;
        int winY = view[3]-1-y;
        
        float zValue;
        glReadPixels(winX,winY,1,1, GL_DEPTH_COMPONENT, GL_FLOAT, &zValue); 
        
        GLubyte stencilValue;
        glReadPixels(winX, winY, 1, 1, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, &stencilValue);
        
        GLdouble worldX, worldY, worldZ;
        gluUnProject (winX, winY, zValue, model, proj, view, &worldX, &worldY, &worldZ);
      }
      break;
    case GLUT_MIDDLE_BUTTON:
      g_iMiddleMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_RIGHT_BUTTON:
      g_iRightMouseButton = (state==GLUT_DOWN);
      break;
    case 3:
      g_iMiddleMouseButton=3;
      break;
    case 4:
      g_iMiddleMouseButton=4;
      break;
  }
  
  g_vMousePos[0] = x;
  g_vMousePos[1] = y; 
}

void mouseMotionFunction(int x, int y)
{
  int mouseDeltaX = x-g_vMousePos[0];
  int mouseDeltaY = y-g_vMousePos[1];
  
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
  
  if (g_iMiddleMouseButton) // handle camera rotations
  {
    const double factor = 0.1;
    camera->MoveRight(factor * mouseDeltaX);
    camera->MoveUp(factor * mouseDeltaY);
  } 
  
  if (g_iRightMouseButton) // handle zoom in/out
  {
    const double factor = 0.1;
    camera->ZoomIn(cameraRadius * factor * mouseDeltaX);
  }

}



// this function does the following:
// (1) Creates the camera
void initScene()
{
  if(camera != NULL)  
    delete(camera);
  
  double virtualToPhysicalPositionFactor = 1.0;
  
  initCamera(cameraRadius, cameraLongitude, cameraLatitude,
             focusPositionX, focusPositionY, focusPositionZ,
             1.0 / virtualToPhysicalPositionFactor,
             &zNear, &zFar, &camera);
  if (light != NULL)
    delete light;
  light = new Lighting(lightingFilename);
  std::cout<< "Camera set"<<std::endl;  
}

// int alec_main(){
  // vp.push_back(ShearAndStretchMaterial);
  // vp.push_back(bendMaterial);
// }

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simulator_from_file");
  // cout<<"argc: "<<argc<<endl;
  make_Vec();
  // get_err(0);
  int numFixedArgs = 6;
  if ( argc != numFixedArgs )
  {
    printf("=== Composite Simulator ===\n");
    printf("Usage: %s [config file]\n", argv[0]);
    printf("Please specify a configuration file\n");
    return 1;
  }
  else
  {
    strncpy(configFilename, argv[1], strlen(argv[1]));
    strncpy(lightingFilename, argv[2], strlen(argv[2]));
    meshfile = argv[3];
    cout <<"Mesh File: "<< meshfile<<endl;
    fix_file = argv[4];
    cout <<"Fix File: "<< fix_file<<endl;
    compare_file = argv[5];
    cout <<"Compare_file: "<<compare_file<<endl;
  }

  initGLUT(argc, argv, windowTitleBase, windowWidth, windowHeight, &windowID);
    // define background texture, set some openGL parameters
  initGraphics(windowWidth, windowHeight);
  initConfigurations();
  is_FEM_on=1;//1 // USE 1 for updated model
  sheet = new Composite(is_FEM_on);
  if(sheet->isFEM_ON()){
    vp.push_back(ShearAndStretchMaterial);
    vp.push_back(bendMaterial);
  }
  else{
    cout << "WHO NEEDS FEM" << endl;
    vp.push_back(tensileStiffness);
    vp.push_back(shearStiffness);
    vp.push_back(bendStiffnessUV);
  }
  
  sheet->setTimeStep(timesteps);
  // sheet->setDensity(density*10000);
  sheet->setDensity(density);
  sheet->setThickness(thickness);
  sheet->setPoisson(poisson);
  sheet->setGravity(9.81);

  sheet->updateParam(vp);
  sheet->getParameters();
  //ADDED ALEC//

  // iterations=0.1;
  //////////////

  // cout<<"JUST UPDATED TO THESE PARAMETERS"<<endl;
  // cout<<"JUST UPDATED PARAMETERS TO:"<<endl;
  // std::vector<Vec3d> f;

  //updateSheetStatus(objmeshfile, fix files, if the fix files are permanent fix points)
  //just specifed objmesh if no fixfile provided
  //if the last variable is set to 0, all given fixed point can be modified and moved.
  sheet->updateSheetStatus(meshfile.data(),fix_file.data(),0);

  sheet->resetConstraints();
  sheet->MoveSurfaceTo(fix0,0,0);
  sheet->MoveSurfaceTo(fix1,0,0);
  sheet->MoveSurfaceTo(fix2,0,0);
  sheet->MoveSurfaceTo(fix3,0,0);
  sheet->finalizeAllConstraints();
  // parseData(compare_file.data(), outter_marker, inner_marker, 5,1);


  initScene();
  // make window and size it properly
  glutMainLoop();
  return 0;
}