#include <ros/ros.h>
/***********************************************/
//SIMULATOR HEADERS
/************************************************/
#include "composite_utilities/composite_interface.hpp"
#include "GL/glui.h"
#include <GL/glut.h>

#include "composite_utilities/initGraphics.h"
#include "configFile.h"
#include "listIO.h"


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
float density=0.47;
float timesteps = 0.05;
int iterations = 10;
//FEM/////
float thickness=0.001;
float poisson=0.3;
float ShearAndStretchMaterial=1e5;       
float bendMaterial=1e10;   
//NO_FEM///
float tensileStiffness=8000.0;       
float shearStiffness=8000.0;         
float bendStiffnessUV=0.001;
std::vector<double>vp;
double* deform;
std::string meshfile, fix_file;


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
    if (renderFixedVertices)
    {
      for(int i=0; i<sheet->getPermanentFixedID().size(); i++)
      {
        glColor3f(1,0,0);
        double fixedVertexPos[3];
        sheet->getSceneObj()->GetSingleVertexRestPosition(sheet->getPermanentFixedID()[i],
            &fixedVertexPos[0], &fixedVertexPos[1], &fixedVertexPos[2]);

        glEnable(GL_POLYGON_OFFSET_POINT);
        glPolygonOffset(-1.0,-1.0);
        glPointSize(12.0);
        glBegin(GL_POINTS);
        glVertex3f(fixedVertexPos[0], fixedVertexPos[1], fixedVertexPos[2]);
        glEnd();
        glDisable(GL_POLYGON_OFFSET_FILL);
      }
    }
    
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

// This function does the following:
//Predicts the sheet behavior based on input mesh and constraint points
//The Mesh can be imported using file format or by providing mesh parameters
//File format: .obj(only)
//Mesh parameter: numVertices, vertices, numTriangles, triangles (pass with msgs)
void idleFunction(void)
{

  deform = sheet->simulate(iterations);
  
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

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "simulator_from_file");

  int numFixedArgs = 5;
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
  }
    
  // make window and size it properly
  initGLUT(argc, argv, windowTitleBase, windowWidth, windowHeight, &windowID);
  
  // define background texture, set some openGL parameters
  initGraphics(windowWidth, windowHeight);
  initConfigurations();


  sheet = new Composite(is_FEM_on);
   if(sheet->isFEM_ON()){
    vp.push_back(ShearAndStretchMaterial);
    vp.push_back(bendMaterial);
  }
  else{
    vp.push_back(tensileStiffness);
    vp.push_back(shearStiffness);
    vp.push_back(bendStiffnessUV);
  }
  
  
  sheet->setTimeStep(timesteps);
  sheet->setDensity(density);
  sheet->setThickness(thickness);
  sheet->setPoisson(poisson);
  sheet->setGravity(9.81);

  sheet->updateParam(vp);
  sheet->getParameters();
  
  sheet->updateSheetStatus(meshfile.data(),fix_file.data());
  
  initScene();
  glutMainLoop();
  return 0;
}

