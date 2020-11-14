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
#include <vector>
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
char windowTitleBase[4096] = "CAM-Composite Simulator";
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
      
//composite interface/////
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

// *****************//
int vertexId;
std::vector<int> maxErrVertices;
double thresholdError = 0.04;
// *****************//

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

// std::vector<int> fix0{2657,15541,16346,9154};  
// std::vector<int> fix1{16524,15410,2283,10215};      
// std::vector<int> fix2{11926,11523,900,11925};    //T3_NewMeshTrial2
// std::vector<int> fix3{15459,16605,11301,3308};

// std::vector<int> fix0{11482,3511,12391,12390};  
// std::vector<int> fix1{10215,2283,15410,16524};      
// std::vector<int> fix2{916,7976,15312,16232};    //T4_NewMeshTrial2
// std::vector<int> fix3{2424,12227,12226,10400};


// fixed point center vertex ids:
// Shahwaz added on 1106

std::vector<int> fix0Center{15539};  
std::vector<int> fix1Center{12132};      
std::vector<int> fix2Center{11922};    //T1_fix centers
std::vector<int> fix3Center{15447};

// std::vector<int> fix0Center{16347};  
// std::vector<int> fix1Center{12136};      
// std::vector<int> fix2Center{15301};    //T2_fix centers
// std::vector<int> fix3Center{15457};

// std::vector<int> fix0Center{12394};  
// std::vector<int> fix1Center{12132};      
// std::vector<int> fix2Center{16229};    //T3_fix centers
// std::vector<int> fix3Center{12231};

// std::vector<int> fix0Center{15539};  
// std::vector<int> fix1Center{12132};      
// std::vector<int> fix2Center{11937};    //T4_fix centers
// std::vector<int> fix3Center{15457};

// std::vector<int> fix_tn0= {0,13,20,49,64,82,248,253,263,271,273,284};
std::vector<int> fix_tn0= {248,253,284,0,20,13,82,64,49,273,263,271};  // order ABCD left mid right
std::vector<int> fix_tn1= {48,30,11,279,252,281,200,232,246,4,22,3};  // order ABCD left mid right
std::vector<int> fix_tn2= {288,316,332,2,31,0,48,64,47,333,307,320};  // order ABCD left mid right
std::vector<int> fix_tn3= {51,19,13,260,257,280,203,234,244,4,30,3};  // order ABCD left mid right (234 is far, missing a point)
std::vector<int> fix_tn4= {285,253,251,16,1,30,43,66,47};  // order ABCD left mid right last 3 241,265,239
std::vector<std::vector<int>> fix_training_neighbors= {fix_tn0,fix_tn1,fix_tn2,fix_tn3,fix_tn4};


Vec3d v3d_A0 = Vec3d(0.258900, 0, 1.041991);   
Vec3d v3d_B0 = Vec3d(-0.647545, 0, 1.169960);  //T1_NewMeshTrial
Vec3d v3d_C0 = Vec3d(-0.716100, 0, 0.191913); 
Vec3d v3d_D0 = Vec3d(0,0,0); 

// Vec3d v3d_A0 = Vec3d(0.198000, 0, 0.914022);   
// Vec3d v3d_B0 = Vec3d(-0.647508, 0, 1.169960);  //T2_NewMeshTrial
// Vec3d v3d_C0 = Vec3d(-0.777000, 0, 0.127929); 
// Vec3d v3d_D0 = Vec3d(0,0,0); 

// Vec3d v3d_A0 = Vec3d(0.129500,0,0.978007);   
// Vec3d v3d_B0 = Vec3d(-0.776945,0,1.169960);  //T3_NewMeshTrial2
// Vec3d v3d_C0 = Vec3d(-0.845500,0,0.255897); 
// Vec3d v3d_D0 = Vec3d(0,0,0);

// Vec3d v3d_A0 = Vec3d(0.198000,0,1.041991);   
// Vec3d v3d_B0 = Vec3d(-0.708445,0,1.169960);  //T4_NewMeshTrial
// Vec3d v3d_C0 = Vec3d(-0.777000,0,0.319882); 
// Vec3d v3d_D0 = Vec3d(0,0,0);

// ========================================================//

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

// Vec3d v3d_A1 = Vec3d(0.03418,0.008986,0.967035); 
// Vec3d v3d_A2 = Vec3d(-0.052287,0.0790045,0.924629);
// Vec3d v3d_B1 = Vec3d(-0.881043,0.030003,1.06678);
// Vec3d v3d_B2 = Vec3d(-0.836011,0.0909105,1.00588); //T3_NewMeshTrial
// Vec3d v3d_C1 = Vec3d(-0.866209,0.0086345,0.153300); 
// Vec3d v3d_C2 = Vec3d(-0.789734,0.135744,0.173395); 
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


std::vector<Vec3d> v3d0= {v3d_A0,v3d_B0,v3d_C0,v3d_D0};
std::vector<Vec3d> v3d1= {v3d_A1,v3d_B1,v3d_C1,v3d_D1};
std::vector<Vec3d> v3d2= {v3d_A2,v3d_B2,v3d_C2,v3d_D2};
std::vector<std::vector<Vec3d>> v3d = {v3d0, v3d1, v3d2};

std::vector<std::vector<std::vector<double>>> vec;
std::vector<std::vector<double>> temp_vec;
std::vector<double> temp_vec_point;

double val= 0.0;


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

  training_data[0]= "/home/shah/composite_ws/src/sheet_model_training/data/T1/Txt_T1S1_Final.txt";
  training_data[1]= "/home/shah/composite_ws/src/sheet_model_training/data/T1/Txt_T1S2_Final.txt";
  training_data[2]= "/home/shah/composite_ws/src/sheet_model_training/data/T1/Txt_T1S3_Final.txt";  //T1
  training_data[3]= "/home/shah/composite_ws/src/sheet_model_training/data/T1/Txt_T1S4_Final.txt";
  training_data[4]= "/home/shah/composite_ws/src/sheet_model_training/data/T1/Txt_T1S5_Final.txt";

  // training_data[0]= "/home/shah/composite_ws/src/sheet_model_training/data/T2/Txt_T2S1_Final.txt";
  // training_data[1]= "/home/shah/composite_ws/src/sheet_model_training/data/T2/Txt_T2S2_Final.txt";
  // training_data[2]= "/home/shah/composite_ws/src/sheet_model_training/data/T2/Txt_T2S3_Final.txt";  //T2
  // training_data[3]= "/home/shah/composite_ws/src/sheet_model_training/data/T2/Txt_T2S4_Final.txt";
  // training_data[4]= "/home/shah/composite_ws/src/sheet_model_training/data/T2/Txt_T2S5_Final.txt";


  // training_data[0]= "/home/shah/composite_ws/src/sheet_model_training/data/T3/Txt_T3S1_Final.txt";
  // training_data[1]= "/home/shah/composite_ws/src/sheet_model_training/data/T3/Txt_T3S2_Final.txt";
  // training_data[2]= "/home/shah/composite_ws/src/sheet_model_training/data/T3/Txt_T3S3_Final.txt";  //T3
  // training_data[3]= "/home/shah/composite_ws/src/sheet_model_training/data/T3/Txt_T3S4_Final.txt";
  // training_data[4]= "/home/shah/composite_ws/src/sheet_model_training/data/T3/Txt_T3S5_Final.txt";

  // training_data[0]= "/home/shah/composite_ws/src/sheet_model_training/data/T4/Txt_T4S1_Final.txt";
  // training_data[1]= "/home/shah/composite_ws/src/sheet_model_training/data/T4/Txt_T4S2_Final.txt";
  // training_data[2]= "/home/shah/composite_ws/src/sheet_model_training/data/T4/Txt_T4S3_Final.txt";  //T4
  // training_data[3]= "/home/shah/composite_ws/src/sheet_model_training/data/T4/Txt_T4S4_Final.txt";
  // training_data[4]= "/home/shah/composite_ws/src/sheet_model_training/data/T4/Txt_T4S5_Final.txt";

  // // cout << 'training_data:' << training_data[fixNum] << endl; 
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
    // Zach added 1105 to check boundary conditions
    // double gripDist = 0;
    // // Check AB
    // gripDist = sheet->getGripDistance(fix0[0],fix1[0]);
    // cout << "Gripping Distance AB: " << gripDist << endl;
    // // Check BC
    // gripDist = sheet->getGripDistance(fix1[0],fix2[0]);
    // cout << "Gripping Distance BC: " << gripDist << endl;
    // // Check CD
    // gripDist = sheet->getGripDistance(fix2[0],fix3[0]);
    // cout << "Gripping Distance CD: " << gripDist << endl;
    // // Check AD
    // gripDist = sheet->getGripDistance(fix0[0],fix3[0]);
    // cout << "Gripping Distance AD: " << gripDist << endl;
    // // Check AC
    // gripDist = sheet->getGripDistance(fix0[0],fix2[0]);
    // cout << "Gripping Distance AC: " << gripDist << endl;
    // // Check BD
    // gripDist = sheet->getGripDistance(fix1[0],fix3[0]);
    // cout << "Gripping Distance BD: " << gripDist << endl;

  // Shahwaz added on 1106 to check refine boundary conditions
  double gripDist = 0;
    // Check AB
  gripDist = sheet->getGripDistance(fix0Center[0],fix1Center[0]);
  cout << "Gripping Distance AB: " << gripDist << endl;
    // Check AC
  gripDist = sheet->getGripDistance(fix0Center[0],fix2Center[0]);
  cout << "Gripping Distance AC: " << gripDist << endl;
    // Check AD
  gripDist = sheet->getGripDistance(fix0Center[0],fix3Center[0]);
  cout << "Gripping Distance AD: " << gripDist << endl;
    // Check BC
  gripDist = sheet->getGripDistance(fix1Center[0],fix2Center[0]);
  cout << "Gripping Distance BC: " << gripDist << endl;
    // Check BD
  gripDist = sheet->getGripDistance(fix1Center[0],fix3Center[0]);
  cout << "Gripping Distance BD: " << gripDist << endl;
    // Check CD
  gripDist = sheet->getGripDistance(fix2Center[0],fix3Center[0]);
  cout << "Gripping Distance CD: " << gripDist << endl;
    
  for(int i=0; i<m_training_marker.size(); i++)
  {
    // cout << "m_training_marker[i]:" << m_training_marker[i] << endl;
    // cout << "m_training_marker.size() = " << m_training_marker.size() << endl;
    // cout << "type" << typeid(m_training_marker[i]).name << endl;
    // cout << "type" << type_name<decltype(m_training_marker[i])>() << endl;
    // m_sceneObjDeform->GetClosestVertex(m_training_marker[i],distPTR); //capital G in Get
    sheet->getClosestVertex(m_training_marker[i],distPTR); //capital G in Get
      
      //Alec added 0707 finding error of fix point neighbors, should be small
    if(i==fix_tn_list[j])
    {
        // cout << "--------FIX NEIGHBOR HERE------" << fix_training_neighbors[j] << endl;
      fix_neighbor_err+= *distPTR;
      j++;
    }
      // m_sceneObjDeform->getClosestVertex(m_training_marker[i],distPTR);
      // cout << "distPTR:" << *distPTR << endl;
    if(*distPTR==0)
      *distPTR=0.1;
    if(*distPTR>max_err)
    {
      max_err = *distPTR;
      
      // if (thresholdError < max_err)
      // {
      //    // Shahwaz added on 11Nov2020
      //   vertexId = sheet->getVertexID(m_training_marker[i]);
      //   maxErrVertices.emplace_back(vertexId);
      // }
    } 
    avg_err+=*distPTR;

  }
// Shahwaz added on 11Nov2020
  
  // for (auto it = maxErrVertices.begin(); it != maxErrVertices.end(); ++it)
  // {
  //   cout << ' ' << *it;
  // }

  // cout << endl;
  // cout << "Vertex Id with max error:  " << vertexId<< endl;
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
  static int t15=5*sim_action_time_step; //8
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

    // sheet->MoveSurfaceTo3D(fix0,vec[0][0],0,0);
    // sheet->MoveSurfaceTo3D(fix1,vec[0][1],0,0);
    // sheet->MoveSurfaceTo3D(fix2,vec[0][2],0,0);
    // sheet->MoveSurfaceTo3D(fix3,vec[0][3],0,0);
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
  

  // if (timeStepCount % 100 ==0){
  //   cout <<"\ntimeStepCount: "<<timeStepCount<<endl;
  // }
  // else{
    cout << "." << flush;
    // cout.flush();
    // std::flush;
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
  // // Shahwaz added on 11Nov2020
  //  for (auto it = maxErrVertices.begin(); it != maxErrVertices.end(); ++it)
  // {
  //   sheet->MoveVertexTo(*it,0,0);
  // }
   
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
  make_v3d();
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

  double dampingStiffness= 0.02;
  initGraphics(windowWidth, windowHeight);
  initConfigurations();
  is_FEM_on=1;//1 // USE 1 for updated model
  sheet = new Composite(is_FEM_on);
  if(sheet->isFEM_ON()){
    vp.push_back(ShearAndStretchMaterial);
    vp.push_back(bendMaterial);
    vp.push_back(dampingStiffness);
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