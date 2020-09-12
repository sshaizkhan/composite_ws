#include <ros/ros.h>
/***********************************************/
//SIMULATOR HEADERS
/************************************************/
#include "sheet_model_training/composite_training_interface.hpp"
// #include "composite_utilities/composite_interface.hpp"
#include "GL/glui.h"
#include "composite_utilities/initGraphics.h"

#include "configFile.h"
#include "listIO.h"
#include <ros/package.h>

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
// #include <ros/ros.h>
// #include "std_msgs/String.h"
// #include <ros/package.h>
// #include <ros/console.h>


using namespace std;




void initScene();
void initGLUI();

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

string package_name="sheet_model_training";
string package_path=ros::package::getPath(package_name);
string sheet_type;
string configFilename;
string configFilePath;
// char configFilename[MAX_FILE];
char lightingFilename[MAX_FILE];
char trainingFilename[MAX_FILE];
char objMeshname[MAX_FILE] = "skirt.obj";
// camera
SphericalCamera * camera = NULL;

// files
SceneObject * extraSceneGeometry = NULL;
float ShearAndStretchMaterial1=1;       
float bendMaterial1=1;   
int ShearAndStretchMaterial2=5;       
int bendMaterial2=8;
float ShearAndStretchMaterial,bendMaterial;        
//composite interface////////////////////////////////////////
Composite_Training *sheet;
bool is_FEM_on,debug_mode;
float density=0.47;
//FEM/////
float thickness=0.001;
float poisson=0.3;
float timesteps = 0.05;
int iterations = 10;
// float p[2]={1e5, 1e10};
//NO_FEM///
float tensileStiffness=80000.0;       
float shearStiffness=80000.0;         
float bendStiffnessUV=0.09;
std::vector<double>vp;
double* deform;
// std::string meshfile, fix_file;
//GUI
int fixnum;
int moveSheet = -1;
int dataonly;
int meshonly;
///////////////////////////////////////////////////////////

// This function specifies parameters (and default values, if applicable) for the
// configuration file. It then loads the config file and parses the options contained
// within. After parsing is complete, a list of parameters is printed to the terminal.
void initConfigurations()
{
  configFilePath=package_path+"/config/"+sheet_type+"/"+configFilename;
  printf("Parsing configuration file %s...\n", configFilePath.data());
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


  configFile.addOptionOptional("is_FEM_on", &is_FEM_on,false);
  configFile.addOptionOptional("density", &density,0.47f);
  configFile.addOptionOptional("timesteps", &timesteps,0.01f);
  configFile.addOptionOptional("iterations", &iterations,1);


      configFile.addOptionOptional("thickness", &thickness,0.001f);
      configFile.addOptionOptional("poisson", &poisson,0.3f);
      configFile.addOptionOptional("ShearAndStretchMaterial1", &ShearAndStretchMaterial1,1.0f);
      configFile.addOptionOptional("bendMaterial1", &bendMaterial1,1.0f);
      configFile.addOptionOptional("ShearAndStretchMaterial2", &ShearAndStretchMaterial2,5);
      configFile.addOptionOptional("bendMaterial2", &bendMaterial2,8);
    // ShearAndStretchMaterial = ShearAndStretchMaterial1 * pow(10, ShearAndStretchMaterial2);
    // bendMaterial = bendMaterial1 * pow(10, bendMaterial2);
  
      configFile.addOptionOptional("tensileStiffness", &tensileStiffness,50000.0f);
      configFile.addOptionOptional("shearStiffness", &shearStiffness,50000.0f);
      configFile.addOptionOptional("bendStiffnessUV", &bendStiffnessUV,0.09f);

 configFile.addOption("objMeshname", objMeshname);
    configFile.addOption("fixnum", &fixnum);
  configFile.addOption("trainingFilename", trainingFilename);
  


  
  // parse the configuration file
  if (configFile.parseOptions(&configFilePath[0]) != 0)
  {
    printf("Error parsing options.\n");
    exit(1);
  }
 
  configFile.printOptions();

}

void Sync_GLUI()
{
  glui->sync_live();
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
      if(moveSheet == 1 || moveSheet == 2){
         for(int i=0; i<sheet->getTrainingFix().size(); i++)
        {
          glColor3f(1,0,1);
          glEnable(GL_POLYGON_OFFSET_POINT);
          glPolygonOffset(-1.0,-1.0);
          glPointSize(15.0);
          glBegin(GL_POINTS);
          glVertex3f(sheet->getTrainingFix()[i][0], sheet->getTrainingFix()[i][1], sheet->getTrainingFix()[i][2]);
          glEnd();
          glDisable(GL_POLYGON_OFFSET_FILL);
        }

        for(int i=0; i<sheet->getTrainingMarker().size(); i++)
        {
          glColor3f(0,0,0);
          glEnable(GL_POLYGON_OFFSET_POINT);
          glPolygonOffset(-1.0,-1.0);
          glPointSize(15.0);
          glBegin(GL_POINTS);
          glVertex3f(sheet->getTrainingMarker()[i][0], sheet->getTrainingMarker()[i][1], sheet->getTrainingMarker()[i][2]);
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
 
    if(meshonly!=1)
      drawdata();
  // render cloth
  if (sheet->getSceneObj() != NULL)
  { 
    if(!dataonly){
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

  }
  
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
  glutSwapBuffers();
}

// This function does the following:
//Predicts the sheet behavior based on input mesh and constraint points
//The Mesh can be imported using file format or by providing mesh parameters
//File format: .obj(only)
//Mesh parameter: numVertices, vertices, numTriangles, triangles (pass with msgs)
void idleFunction(void)
{
  double max_deform;
  double avg_deform;
  double energy;
  glutSetWindow(windowID);
  if(moveSheet == 1){
    deform = sheet->simulate((int)1/timesteps);
    // deform = sheet->simulate(1);
    sheet->getDeformInfo(max_deform, avg_deform);
    std::cout<<"Max: "<<max_deform<<"\nAvg: "<<avg_deform<<std::endl;
    energy = sheet->getKineticEnergy();
    std::cout<<"Energy: "<<energy<<std::endl;
    // deform = sheet->simulate(iterations);
    double *distPTR = new double(10);
    double err=0;
    double max=0;
    double avg=0;
    // std::cout <<"dist: " <<*dist<<std::endl;
    for(int i=0; i<sheet->getTrainingMarker().size(); i++){
      sheet->getClosestVertex(sheet->getTrainingMarker()[i],distPTR);
      // sheet->getSceneObj()->GetClosestVertex(sheet->getTrainingMarker()[i],distPTR);
      if(*distPTR==0)
        *distPTR = 0.1;
      if(*distPTR>max)
        max = *distPTR;
      err+=*distPTR; 
    }
    delete distPTR;
    avg = err/sheet->getTrainingMarker().size();
    std::cout << "================"<<std::endl;
    // std::cout << "total_err: "<<err<<std::endl;
    // std::cout << "size: "<<training_point.size()<<std::endl;
    std::cout << "avg_err:"<<avg*100<<std::endl;
    std::cout << "max_err:"<<max*100<<std::endl;
    std::cout << "opt_err:"<<(avg*1+max*0.5)*100<<std::endl;
    std::cout <<"Elastic Energy: "<<sheet->getElasticEnergy()<<endl;
    moveSheet=2;
  }
  Sync_GLUI();
  ros::spinOnce();

  glutPostRedisplay();  
}

void reshape(int x,int y)
{
  // std::cout<< "reshape"<<std::endl;
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
  // delete sheet;
  
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

// this function is called when the user
// changes the timestep parameter from cleancomposite
// within the simulator
void update_timestep(int code)
{
  sheet->getIntegrator()->SetTimestep(timesteps);
  glui->sync_live();
}

// this function is called when a button in the
// simulator is pressed
void update_simulation_status(int code)
{
  switch (code)
  {
    case 0: // initialize new parameters (need to re-initialize scene)
      moveSheet = -1;
      initScene();
      //runSimulation = 0;
      break;
      
    case 1: // load new config file
      initGraphics(windowWidth, windowHeight);
      initConfigurations();
      initScene();
      glui->close();
      initGLUI();
      //runSimulation = 0;
      break;
    case 2: // clear
    moveSheet = 0;
      initGraphics(windowWidth, windowHeight);
      initConfigurations();
      initScene();
      //runSimulation = 0;
      break;
    case 3: // moveSheet
      if(moveSheet ==0)
        moveSheet = 1;
      break;
    case 4: // reset cloth
      sheet->getIntegrator()->ResetToRest();
      memset(deform, 0, sizeof(double) * 3 * sheet->getSceneObj()->GetNumVertices());
      sheet->getSceneObj()->ResetDeformationToRest();
      moveSheet = 0;
      //runSimulation = 0;
      break;
    case 5:
      moveSheet=-1;
      initScene();
      glui->close();
      initGLUI();

      break;
    default:
      printf("Error. Invalid simulator update option.\n");
      break;
  }
  
  glui->sync_live();
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

  // if(sheet!=NULL)
  //   delete sheet;  
  sheet = new Composite_Training(is_FEM_on);
  // sheet = new Composite_Training(is_FEM_on,debug_mode);
  if(vp.size()!=0)
      vp.clear();
  if(sheet->isFEM_ON()){
    // vp.resize(2);
    ShearAndStretchMaterial = ShearAndStretchMaterial1 * pow(10, ShearAndStretchMaterial2);
    bendMaterial = bendMaterial1 * pow(10, bendMaterial2);

    vp.push_back(ShearAndStretchMaterial);
    vp.push_back(bendMaterial);
  }
  else{
    // cout <<"NO FEM"<<tensileStiffness<<endl;
    // vp.resize(3);
    vp.push_back(tensileStiffness);
    vp.push_back(shearStiffness);
    vp.push_back(bendStiffnessUV);
  }
  
  sheet->setTimeStep(timesteps);
  sheet->setDensity(density);
  sheet->setThickness(thickness);
  sheet->setPoisson(poisson);
  sheet->setGravity(9.81);

  // sheet->updateParam(vp);
  // sheet->getParameters();
  
  sheet->updateSheetStatus(objMeshname,trainingFilename,vp,fixnum);
  sheet->getParameters();
  moveSheet = 0;
}

// Create the GUI 
void initGLUI()
{
  // generate the UI
  glui = GLUI_Master.create_glui( "Controls",0,windowWidth + 100, 0);

  // scene option
  GLUI_Panel * scene_option_panel = glui->add_panel("Scene&Parameters",GLUI_PANEL_EMBOSSED);
  scene_option_panel->set_alignment(GLUI_ALIGN_LEFT);

  glui->add_checkbox_to_panel(scene_option_panel,"WireFrame", &renderWireframe);
  glui->add_checkbox_to_panel(scene_option_panel,"DataOnly", &dataonly);
  glui->add_checkbox_to_panel(scene_option_panel,"MeshOnly", &meshonly);
  
  //GLUI_RadioGroup * loadoption = glui->add_radiogroup_to_panel(scene_option_panel, &loadOption);
  
  GLUI_EditText * config_filename_text = glui->add_edittext_to_panel(scene_option_panel,"Config File Name",GLUI_EDITTEXT_TEXT,&configFilename[0]);
  config_filename_text->w = config_filename_text->w + 62;
  
  glui->add_button_to_panel(scene_option_panel,"Load New Config File",1, update_simulation_status);
  // GLUI_Spinner * data_spinner = glui->add_spinner_to_panel(scene_option_panel, "training_data", GLUI_SPINNER_INT, &dataNumber);
  // glui->add_button_to_panel(scene_option_panel,"Load New data File",0, update_simulation_status);
  // data_spinner->set_int_limits(1, 20);
  // data_spinner->set_speed(0.1);
  GLUI_Spinner * fem_spinner = glui->add_spinner_to_panel(scene_option_panel, "Cloth Model", GLUI_SPINNER_INT, &is_FEM_on);
   fem_spinner->set_int_limits(0, 1);
   fem_spinner->set_speed(0.1);
   glui->add_button_to_panel(scene_option_panel,"Switch Model",5, update_simulation_status);
  if(sheet->isFEM_ON()){
    GLUI_Panel * test_panel = glui->add_panel_to_panel(scene_option_panel,"Parameters",GLUI_PANEL_EMBOSSED);
    test_panel->set_alignment(GLUI_ALIGN_LEFT);
    test_panel->draw_name(0,0);
    glui->add_separator_to_panel(scene_option_panel);
    glui->add_edittext_to_panel(scene_option_panel,"surfaceDensity",GLUI_EDITTEXT_FLOAT,&density);
    glui->add_edittext_to_panel(scene_option_panel,"thickness",GLUI_EDITTEXT_FLOAT,&thickness);
    glui->add_edittext_to_panel(scene_option_panel,"poisson",GLUI_EDITTEXT_FLOAT,&poisson);
    glui->add_edittext_to_panel(test_panel,"ShearAndStretchMaterial",GLUI_EDITTEXT_FLOAT,&ShearAndStretchMaterial1);
    glui->add_edittext_to_panel(test_panel,"bendMaterial",GLUI_EDITTEXT_FLOAT,&bendMaterial1);
    glui->add_column_to_panel(test_panel, false);
    GLUI_Spinner * shear_spinner = glui->add_spinner_to_panel(test_panel, "E", GLUI_SPINNER_INT, &ShearAndStretchMaterial2);
    GLUI_Spinner * bend_spinner = glui->add_spinner_to_panel(test_panel, "E", GLUI_SPINNER_INT, &bendMaterial2);
    // cout <<"ShearAndStretchMaterial2: "<<ShearAndStretchMaterial2<<endl;
    // glui->add_edittext_to_panel(scene_option_panel, "dampingMass", GLUI_SPINNER_FLOAT, &dampingMass);
    // glui->add_edittext_to_panel(scene_option_panel, "dampingStiffness", GLUI_SPINNER_FLOAT, &dampingStiffness);

    shear_spinner->set_int_limits(1, 10);
    shear_spinner->set_speed(0.1);
    bend_spinner->set_int_limits(1, 15);
    bend_spinner->set_speed(0.1);
  }
  else{
     glui->add_edittext_to_panel(scene_option_panel,"surfaceDensity",GLUI_EDITTEXT_FLOAT,&density);
    GLUI_Spinner * P2_spinner = glui->add_spinner_to_panel(scene_option_panel, "tensileStiffness", GLUI_SPINNER_FLOAT, &tensileStiffness);
    GLUI_Spinner * P3_spinner = glui->add_spinner_to_panel(scene_option_panel, "shearStiffness", GLUI_SPINNER_FLOAT, &shearStiffness);
    GLUI_Spinner * P45_spinner = glui->add_spinner_to_panel(scene_option_panel, "bendStiffnessUV", GLUI_SPINNER_FLOAT, &bendStiffnessUV);
    // GLUI_Spinner * P4_spinner = glui->add_spinner_to_panel(scene_option_panel, "bendStiffnessU", GLUI_SPINNER_FLOAT, &bendStiffnessU);
    // GLUI_Spinner * P5_spinner = glui->add_spinner_to_panel(scene_option_panel, "bendStiffnessV", GLUI_SPINNER_FLOAT, &bendStiffnessV);
    // GLUI_Spinner * P6_spinner = glui->add_spinner_to_panel(scene_option_panel, "dampingMass", GLUI_SPINNER_FLOAT, &dampingMass);
    // GLUI_Spinner * P7_spinner = glui->add_spinner_to_panel(scene_option_panel, "dampingStiffness", GLUI_SPINNER_FLOAT, &dampingStiffness);
    
    P2_spinner->set_float_limits(100, 1e9);
    P3_spinner->set_float_limits(100, 1e9);
    P45_spinner->set_float_limits(1e-8, 10);
    // P4_spinner->set_float_limits(1e-4, 10);
    // P5_spinner->set_float_limits(1e-4, 0.1);
    // P6_spinner->set_float_limits(0.0, 0.0);
    // P7_spinner->set_float_limits(0.1, 10);
    
    P2_spinner->set_speed(1);
    P3_spinner->set_speed(1);
    P45_spinner->set_speed(0.1);
    // P4_spinner->set_speed(1);
    // P5_spinner->set_speed(1);
    // P6_spinner->set_speed(0.1);
    // P7_spinner->set_speed(0.1);
  }

  // glui->add_edittext_to_panel(scene_option_panel,"tensileStiffness",GLUI_EDITTEXT_FLOAT,&tensileStiffness);
  // glui->add_edittext_to_panel(scene_option_panel,"shearStiffness",GLUI_EDITTEXT_FLOAT,&shearStiffness);
  // glui->add_edittext_to_panel(scene_option_panel,"bendStiffnessU",GLUI_EDITTEXT_FLOAT,&bendStiffnessU);
  // glui->add_edittext_to_panel(scene_option_panel,"bendStiffnessV",GLUI_EDITTEXT_FLOAT,&bendStiffnessV);
  // glui->add_edittext_to_panel(scene_option_panel,"dampingMass",GLUI_EDITTEXT_FLOAT,&dampingMass);
  // glui->add_edittext_to_panel(scene_option_panel,"dampingStiffness",GLUI_EDITTEXT_FLOAT,&dampingStiffness);
  // glui->add_edittext_to_panel(scene_option_panel,"gravityForce",GLUI_EDITTEXT_FLOAT,&gravityForce);
  // glui->add_edittext_to_panel(scene_option_panel,"mousePower",GLUI_EDITTEXT_FLOAT,&mouseForceCoeff);
  // glui->add_checkbox_to_panel(scene_option_panel,"useRestAngles", &useRestAngles);
  glui->add_button_to_panel(scene_option_panel,"Upload Parameters",0, update_simulation_status);
  
  glui->add_separator();
  
  
  // simulation control 
  GLUI_Panel * simulation_panel = glui->add_panel("Simulation",GLUI_PANEL_EMBOSSED);
  simulation_panel->set_alignment(GLUI_ALIGN_LEFT);
  simulation_panel->draw_name(0,0);
  glui->add_button_to_panel(simulation_panel,"Move",3, update_simulation_status);
  glui->add_button_to_panel(simulation_panel,"Reset",4, update_simulation_status);
  glui->add_button_to_panel(simulation_panel,"Clear",2, update_simulation_status);

  glui->add_edittext_to_panel(simulation_panel,"Iteration ",GLUI_EDITTEXT_INT,&iterations,0,update_simulation_status);
  glui->add_edittext_to_panel(simulation_panel,"Timestep [sec]",GLUI_EDITTEXT_FLOAT,&timesteps,0,update_timestep);

  // systemSolveStaticText = glui->add_statictext("System solve: ");
  // forceAssemblyStaticText = glui->add_statictext("Force assembly: ");

  glui->add_separator();
  
  // end of the control
  glui->add_button("Quit", 0, exit_buttonCallBack);
  glui->sync_live();
  glui->set_main_gfx_window( windowID );
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simulator_prepreg_viz");

  int numFixedArgs = 4;
  if ( argc != numFixedArgs )
  {
    printf("=== Composite Simulator ===\n");
    printf("Usage: %s [config file]\n", argv[0]);
    printf("Please specify a configuration file\n");
    return 1;
  }
  else
  {
    sheet_type = argv[1];
    configFilename = argv[2];
    // strncpy(configFilename, argv[1], strlen(argv[1]));
    strncpy(lightingFilename, argv[3], strlen(argv[3]));
    // debug_mode = atoi(argv[4]);

  }
    
  // make window and size it properly
  initGLUT(argc, argv, windowTitleBase, windowWidth, windowHeight, &windowID);
  
  // define background texture, set some openGL parameters
  initGraphics(windowWidth, windowHeight);
  initConfigurations();

  // p[0] = ShearAndStretchMaterial;
  // p[1] = bendMaterial;
  // for(int i=0; i<(sizeof(p)/sizeof(*p));i++)
  //   vp.push_back(p[i]);
  // sheet = new Composite_Training(is_FEM_on);
  // if(sheet->isFEM_ON()){
  //   // vp.resize(2);
  //   vp.push_back(ShearAndStretchMaterial);
  //   vp.push_back(bendMaterial);
  // }
  // else{
  //   // cout <<"NO FEM"<<tensileStiffness<<endl;
  //   // vp.resize(3);
  //   vp.push_back(tensileStiffness);
  //   vp.push_back(shearStiffness);
  //   vp.push_back(bendStiffnessUV);
  // }
  
  // sheet->setTimeStep(timesteps);
  // sheet->setDensity(density);
  // sheet->setThickness(thickness);
  // sheet->setPoisson(poisson);
  // sheet->setGravity(9.81);

  // sheet->updateParam(vp);
  // sheet->getParameters();
  
  // sheet->updateSheetStatus(meshfile.data(),fix_file.data());

  // sheet->updateSheetStatus(meshfile.data());
  // sheet->resetConstraints();
  // sheet->AddSurfaceConstraints(fix_file.data());
  // sheet->finalizeAllConstraints();


  initScene();
  initGLUI();
  glutMainLoop();
  return 0;
}

