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
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <ros/package.h>
#include <ros/console.h>

#ifdef mesh_tracking
#include "mesh_tracking/mesh.h"
#endif
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
float tensileStiffness=8000.0;       
float shearStiffness=8000.0;         
float bendStiffnessUV=0.001;
//composite interface////////////////////////////////////////
Composite *sheet;
float density=0.046;
float timesteps = 0.05;
int iterations = 10;
float p[3]={8575.38, 8751.91, 0.000900105};
std::vector<double>vp;
double* deform;
std::vector<std::string> meshnames, robot_data;
std::string meshfile, fix_file;
int iter=0;

int numVertices;
double * vertices;
int numTriangles;
int * triangles;

int numVertices_new;
double * vertices_new;
int numTriangles_new;
int * triangles_new;

bool mesh_ready=false;
///////////////////////////////////////////////////////////

#ifdef mesh_tracking
void getMesh(const mesh_tracking::mesh& msg)
{
  if(mesh_ready)
    return;
  if(vertices!=NULL)
    delete vertices;
  if(triangles != NULL)
    delete triangles;

  numVertices=msg.points.data.size()/3;
  numTriangles=msg.polys.data.size()/3;

  vertices = (double*)malloc(sizeof(double)*msg.points.data.size());
  triangles = (int*)malloc(msg.polys.data.size()*sizeof(int));
  
  for(int i=0;i<numVertices;i++)
  {
    vertices[3*i]=msg.points.data[3*i];
    vertices[3*i+1]=msg.points.data[3*i+1];
    vertices[3*i+2]=msg.points.data[3*i+2];
  }

  for(int i=0;i<numTriangles;i++)
  {
    triangles[3*i]=msg.polys.data[3*i];
    triangles[3*i+1]=msg.polys.data[3*i+1];
    triangles[3*i+2]=msg.polys.data[3*i+2];
  }


  mesh_ready=true;
  cout<<"Inside ROS "<<numVertices<<" "<<numTriangles<<endl;


}
#endif

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
  

  configFile.addOption("density", &density);
  configFile.addOption("timesteps", &timesteps);
  configFile.addOption("iterations", &iterations);

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
// (5) Sets cloth deformations
// (6) Sets lighting conditions
// (7) Builds surface normals for cloth (take this out to increase performance)
// (8) Renders cloth
// (9) Render pulled vertex in different color (if applicable)
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
  if (sheet->getClothBW() != NULL)
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
      for(int i=0; i<sheet->getFixedGroups().size(); i++)
      {
        glColor3f(1,0,0);
        double fixedVertexPos[3];
        sheet->getSceneObj()->GetSingleVertexRestPosition(sheet->getFixedGroups()[i],
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

// this function does the following:
//Predicts the sheet behavior based on input mesh and constraint points
//The Mesh can be imported using file format or by providing mesh parameters
//File format: .obj(only)
//Mesh parameter: numVertices, vertices, numTriangles, triangles (pass with msgs)
//The constraint points can be import as file only for now.
void idleFunction(void)
{
  ros::spinOnce();
  //////Reading Mesh through Camera [Package: mesh_tracking is REQUIRED]/////////////////////
  if(mesh_ready)
  {
    std::cout<<"Inside simulator "<<numVertices<<" "<<numTriangles<<endl;
    std::cout<<"Inside simulator "<<vertices[0]<<" "<<triangles[0]<<endl;
    std::cout<<"Size of the Arrays T "<<sizeof(triangles)<<std::endl;
    std::cout<<"Size of the Arrays "<<sizeof(vertices)<<std::endl;
    sheet->updateSheetStatus(numVertices, vertices, numTriangles, triangles);
    deform = sheet->simulate(1);
    mesh_ready=false;
  }
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

}

void mouseMotion (int x, int y)
{

}

void mouseButtonActivityFunction(int button, int state, int x, int y)
{

}

void mouseMotionFunction(int x, int y)
{
 

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
  ros::init(argc, argv, "simulator");
  #ifdef mesh_tracking
  ros::NodeHandle pnh("~");
  std::string mesh_tracking_package_path = ros::package::getPath("mesh_tracking");
  
  ros::Subscriber polygon_mesh = pnh.subscribe("/polygon_mesh",1,getMesh);
  #endif

  int numFixedArgs = 3;
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
  }
    
  // make window and size it properly
  initGLUT(argc, argv, windowTitleBase, windowWidth, windowHeight, &windowID);
  
  // define background texture, set some openGL parameters
  initGraphics(windowWidth, windowHeight);
  initConfigurations();

  p[0] = tensileStiffness;
  p[1] = shearStiffness;
  p[2] = bendStiffnessUV;
  for(int i=0; i<(sizeof(p)/sizeof(*p));i++)
    vp.push_back(p[i]);
  sheet->setTimeStep(timesteps);
  sheet->setDensity(density);
  sheet->setGravity(0);
  sheet->updateParam(vp);
  sheet->getParameters();

  initScene();

  // ros::spin();
  glutMainLoop();
  return 0;
}

