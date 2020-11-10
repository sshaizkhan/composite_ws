#ifndef COMPOSITE_INTERFACE_HPP
#define COMPOSITE_INTERFACE_HPP

#include <math.h>
#include <vector>
#include <string>
#include <sstream>
#include "objMesh.h"
#include "clothBWFromObjMesh.h"
#include "clothBWStencilForceModel.h"
#include "forceModelAssembler.h"
// #include "implicitNewmarkSparse.h"
// #include "implicitBackwardEulerSparse.h"
#include "implicitEulerMultiObjectWithConstraintsSparse.h"
#include "constraints.h"
#include "sceneObjectDeformable.h"
#include "minivector.h"
#include "clothFEM.h"
#include "clothFEMStencilForceModel.h"
#include "triMeshGeo.h"
#include <stack>
#include <ctime>
#include <fstream>
#include <iostream>



struct passtime{
	std::stack<clock_t> tictoc_stack;
	double diff;
	void tic() {
    tictoc_stack.push(clock());
	}

	void toc() {
		diff = (((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC)/5.;
    std::cout << "[Time elapsed(sec)]: "
              << diff
              << std::endl;

    tictoc_stack.pop();
	}
	// double get(){return diff;}
};

class Composite
{
protected:
	//Const Parameters
	float m_gravityForce=9.81;//9.81
	
	//solver parameters
	// int numVertices=0;
	float m_timeStep=0.05;
	int m_numConstrainedDOFs = 0; 
	int * m_constrainedDOFs = NULL;
	int m_numSolverThreads=2; // default = 0 (not multi-threaded)
	//Deformation Parameters
	double* m_deform = NULL;
	double* m_f_ext = NULL;
	//ClothBW objects
	ClothBW * m_clothBW;
	ClothBWFromObjMesh * m_clothBWFromObjMesh;
	ClothBWStencilForceModel * m_clothBWStencilForceModel;
	//BW Material Parameters
    float m_dampingMass;
	float m_dampingStiffness;
	float m_surfaceDensity;
	float m_tensileStiffness;
	float m_shearStiffness;
	float m_bendStiffnessU;
	float m_bendStiffnessV;
	//ClothFEM 
	bool m_FEM_ON=false;
	bool m_DEBUG=false;
	ClothFEM * m_clothFEM;
	ClothFEMStencilForceModel * m_clothFEMStencilForceModel;
	//FEM Material parameters
	double m_sheet_thickness=0.001;
	double m_sheet_poisson=0.3;
	double m_shear_stretch_Material;
	double m_bend_Material;
	ForceModelAssembler * m_forceModelAssembler;
	
	//Constraints
	Constraints * m_constraints;
	SparseMatrix * m_constraintMatrix=NULL;
	double * m_constraintRhs=NULL;

	//Integrator
	SparseMatrix * m_massMatrix;
	ForceModel * m_forceModel;

	//Constraint Integrator
	ImplicitEulerMultiObjectWithConstraintsSparse * m_implicitEulerMultiObjectWithConstraintsSparse;
	//Mesh
	ObjMesh * m_objMesh;
	int m_numVertices;
	int m_numDOFs;
  	double * m_restVertexPositions;
  	int m_numTriangles;
  	int * m_triangleVertexIndices;
  	std::vector<double> m_masses;
	SceneObjectDeformable * m_sceneObjDeform;
	//Grasping location
	std::vector<int> m_fixGroups;
	std::vector<int> m_constraintGroups;
	std::vector<Vec3d> m_grasping_points;
	
    //updateConfig Provide a series of method to load mesh and grasping data (unit: meter)
    virtual void updateConfig(const char* objMeshname, const char* robot_data=NULL);
    virtual void updateConfig(const char* objMeshname, const std::vector<Vec3d>& robot_data);
    
    virtual void updateConfig(int numVertices, const double * vertices, int numTriangles, const int * triangles, const char* robot_data=NULL);
    virtual void updateConfig(int numVertices, const double * vertices, int numTriangles, const int * triangles, const std::vector<Vec3d>& robot_data);
    //generateSheet
    //Constructs the clothBW class for the solver
    virtual void generateSheet();
    virtual void generateFEM();
    virtual void generateBW();
    //Use the fixed poisiton of data as constrain 
    void updateConstrain();
    void AddTriConstraints(int v1, int v2, int v3, double bary[4][3], int VID[4], std::vector<Vec3d>& constraintPts);
    inline void clearPermanentFixedID(){m_fixGroups.clear();}
public:
    /**************Data initilization************/
    //Initialize the simulator system
    //@param fem specifies if the model is construct using ClothFEM mode(slower but finer)
    //@param debug specifies if the model is in debug mode. Debug mode allows users 
    //to see informations that could be used for debugging 
    Composite(bool fem=false, bool debug=false);

    void setTimeStep(const float& step){m_timeStep = step;};
    void setDensity(const float& dens){m_surfaceDensity = dens;};
    void setGravity(const float& g){m_gravityForce = g;};
    inline bool isFEM_ON(){return m_FEM_ON;};
    inline void setThickness(const double& value){m_sheet_thickness = value;};
    inline void setPoisson(const double& value){m_sheet_poisson = value;};

    //updates the parameters for given model
    virtual void updateParam(const float* param);
    virtual void updateParam(const std::vector<double>& param);
    /*********************************************/
	
    /**************RealTime Simulation************/
    //Use below functions to create sheet model
    //Mesh src can be specified with .obj file or mesh geometry information.
    //Constraint src can be specified by .csv file or a vector.
    //Set is_permanent to true if the specified constraints are fixed through out the simulation.
    virtual void updateSheetStatus(const char* objMeshname, const char* robot_data=NULL, bool is_permanent=false);
    void updateSheetStatus(const char* objMeshname, const std::vector<Vec3d>& robot_data, bool is_permanent=false);  
    void updateSheetStatus(int numVertices, const double * vertices, int numTriangles, const int * triangles, const char* robot_data=NULL, bool is_permanent=false);
    void updateSheetStatus(int numVertices, const double * vertices, int numTriangles, const int * triangles, const std::vector<Vec3d>& robot_data, bool is_permanent=false);
    
    //Use this function to perform sheet simulation
    //Use @iteration to specify the stepping number for one simulation
    //Set iteration to 1 for real time simulation. to 10 for 10 fps simulation.
    double* simulate(const int& iteration);
    /*********************************************/

    /***************Constraints Methods************/
    //Make sure to call reset function before adding any "New" constraints
    void resetConstraints();

    //Use below function to load and add constraints
    void AddConstraintsFromFile(const char* robot_data);
    void AddConstraints(const std::vector<Vec3d>& robot_data);
    void MoveSurfaceTo(const std::vector<Vec3d>& target_point, const int& dir, const double& dist);
    void MoveSurfaceTo(const std::vector<int>& targetIDs, const int& dir, const double& dist);
    void MoveVertexTo(const Vec3d& target_point, const int& dir, const double& dist);
    void MoveVertexTo(const int& targetIDs, const int& dir, const double& dist);
    void MoveSurfaceTo(const std::vector<Vec3d>& target_point, const std::vector<Vec3d>& destination);
    void MoveVertexTo(const Vec3d& target_point, const Vec3d& destination);
    void Fun(const std::vector<int>& targetIDs,const std::vector<double>& dir);
    double getGripDistance(int vecIDa, int vecIDb);
    void MoveSurfaceTo3D(const std::vector<int>& targetIDs, const std::vector<double>& dir, const double& dist, const int& x); //AlecZach
    void MoveVertexTo3D(const int& targetID, const std::vector<double>& dir, const double& dist, const int& x);                //AlecZach
    //Make sure to finalize constraints once all constraints are added.
    //Note that new constraints after this function does not work.
    void finalizeAllConstraints();

    //Clears the permanent constraints
    void clearPermanentConstraints();
    /**********************************************/

    /******Data retrieve methods*********/
    virtual void getParameters();
    void getSolverInfo();
    void getDeformInfo(double& max_deform, double& avg_deform);
    inline SceneObjectDeformable * getSceneObj(){return m_sceneObjDeform;}
    inline ClothBW * getClothBW(){return m_clothBW;}
    inline int getVertexID(const Vec3d& point){return m_objMesh->getClosestVertex(point);}
    inline ImplicitEulerMultiObjectWithConstraintsSparse* getIntegrator(){return m_implicitEulerMultiObjectWithConstraintsSparse;}
    inline double getKineticEnergy(){return m_implicitEulerMultiObjectWithConstraintsSparse->GetKineticEnergy();}
    inline Constraints* getConstraints(){return m_constraints;}
    inline double* getDeform(){return m_deform;}
    inline float& getTimeStep(){return m_timeStep;}
    inline ForceModel* getForceModel(){return m_forceModel;}
    inline double getElasticEnergy(){return m_forceModel->GetElasticEnergy(m_deform);}
    inline std::vector<Vec3d>& getGrapsingPoints(){return m_grasping_points;}
    inline std::vector<int>& getPermanentFixedID(){return m_fixGroups;}
    inline std::vector<int>& getConstraintID(){return m_constraintGroups;}
    int getClosestVertex(Vec3d& querypoint, double*& dist);
    int getClosestVertex(const double querypoint[3], double*& dist);
    inline ObjMesh* getObj(){return m_objMesh;}
    inline void save(const char* filename){m_sceneObjDeform->GetMesh()->save(filename);}
    /*************************************/
    virtual ~Composite()
    {
    	if(m_implicitEulerMultiObjectWithConstraintsSparse != NULL)
	  		delete m_implicitEulerMultiObjectWithConstraintsSparse;

    	if(m_constraints!=NULL)
    		delete m_constraints;
		if (m_sceneObjDeform != NULL)
	    	delete m_sceneObjDeform;
	    if (m_forceModelAssembler != NULL)
	  		delete m_forceModelAssembler;
	    if(m_clothFEMStencilForceModel!=NULL)
	    	delete	m_clothFEMStencilForceModel;
	    if(m_clothFEM!=NULL)
	    	delete m_clothFEM;
		if (m_clothBWStencilForceModel != NULL)
	  		delete m_clothBWStencilForceModel;
	  	if (m_clothBWFromObjMesh != NULL)
	    	delete m_clothBWFromObjMesh;
	  	if (m_clothBW != NULL)
	    	delete m_clothBW;
	  	if (m_objMesh != NULL)
	    	delete m_objMesh;
	  	std::cout <<"sheet clear\n";

    }

};

//TODO: parse training points
//      save them into fixed points and mesh point
//void parseTrainingData(filename, &fixedpoints, &meshpoints)
//save as vector<Vec3d>

inline void parseData(const char* filename, std::vector<Vec3d>& fixed){
		std::fstream fin;
	  	fin.open(filename);
		if(!fin.is_open()){
			std::cout<<"open failed"<<std::endl;
		}
		
		// int fix_number = filename[113]-'0';
		// std::cout <<"fix:" <<fix_number<<std::endl;
		// fix_number*=4;
		std::string line;
		std::vector<std::string> coord;
		std::string value;
		// int count=0;
		
		if(fixed.size()!=0)
			fixed.clear();

		while(getline(fin,line)){
				std::stringstream ss(line);
				coord.clear();
				while(getline(ss,value,',')){
					coord.push_back(value);
				}

					fixed.push_back(Vec3d(stod(coord[0]),stod(coord[1]),stod(coord[2])));
				
		}
}



inline void parseRobotData(const char* filename, const ObjMesh* obj, std::vector<Vec3d>& fix_location, 
								std::vector<int>& fixgroups){

	parseData(filename, fix_location);
	std::cout << "Data Number: "<<fix_location.size()<<std::endl;

	for(int i=0; i< fix_location.size(); i++)
		fixgroups.push_back(obj->getClosestVertex(fix_location[i]));
	std::sort(fixgroups.begin(), fixgroups.end());

}

inline void parseRobotData(const ObjMesh* obj, const std::vector<Vec3d>& fix_location, 
								std::vector<int>& fixgroups){
	for(int i=0; i< fix_location.size(); i++)
		fixgroups.push_back(obj->getClosestVertex(fix_location[i]));
	std::sort(fixgroups.begin(), fixgroups.end());
}

 inline void parseMeshData(const char* filename, int& numVertices, double*& vertices, int& numTriangles, int *& triangles){

		std::fstream fin;
	  	fin.open(filename);
		if(!fin.is_open()){
			std::cout<<"open failed"<<std::endl;
		}
		
		// int fix_number = filename[113]-'0';
		// std::cout <<"fix:" <<fix_number<<std::endl;
		// fix_number*=4;
		std::string line;
		std::vector<std::string> coord;
		std::string value;
		// int count=0;
		if(vertices!=NULL)
			delete vertices;
		if(triangles != NULL)
			delete triangles;
		int i=0;
		int j=0;
		int k=0;
		while(getline(fin,line)){
				std::stringstream ss(line);
				coord.clear();
				while(getline(ss,value,' ')){
					coord.push_back(value);
				}
				if(i==0)
				{
					numVertices = stoi(coord[0]);
					numTriangles = stoi(coord[1]);
					vertices = (double*) malloc (sizeof(double) * 3 * numVertices);
					triangles = (int*) malloc (sizeof(int) * 3 * numTriangles);
					std::cout << "numVertices: "<<numVertices<<std::endl;
					std::cout << "numTriangles: "<<numTriangles<<std::endl;
				}
				else
				{
					if(j< numVertices){
						vertices[3*j] = stod(coord[0]);
						vertices[3*j+1] = stod(coord[1]);
						vertices[3*j+2] = stod(coord[2]);
						j++;

					}
					else
					{
						triangles[3*k] = stoi(coord[0]);
						triangles[3*k+1] = stoi(coord[1]);
						triangles[3*k+2] = stoi(coord[2]);
						k++;
					}

				}
				
				i++;
		}
		std::cout << "final vertices: "<<vertices[3*(numVertices-1)+2]<<std::endl;
		std::cout << "final triangle: "<<triangles[3*(numTriangles-1)+2]<<std::endl;
}


inline void parseMeshData(const char* filename, int& numVertices, std::vector<double>& vertices, int& numTriangles, std::vector<int>& triangles){

		std::fstream fin;
	  	fin.open(filename);
		if(!fin.is_open()){
			std::cout<<"open failed"<<std::endl;
		}
		
		// int fix_number = filename[113]-'0';
		// std::cout <<"fix:" <<fix_number<<std::endl;
		// fix_number*=4;
		std::string line;
		std::vector<std::string> coord;
		std::string value;
		// int count=0;
		
		int i=0;
		int j=0;
		int k=0;
		while(getline(fin,line)){
				std::stringstream ss(line);
				coord.clear();
				while(getline(ss,value,' ')){
					coord.push_back(value);
				}
				if(i==0)
				{
					numVertices = stoi(coord[0]);
					numTriangles = stoi(coord[1]);
					vertices.resize(3 * numVertices);
					triangles.resize(3 * numTriangles);

					std::cout << "numVertices: "<<numVertices<<std::endl;
					std::cout << "numTriangles: "<<numTriangles<<std::endl;
				}
				else
				{
					if(j< numVertices){
						vertices[3*j] = stod(coord[0]);
						vertices[3*j+1] = stod(coord[1]);
						vertices[3*j+2] = stod(coord[2]);
						j++;

					}
					else
					{
						triangles[3*k] = stoi(coord[0]);
						triangles[3*k+1] = stoi(coord[1]);
						triangles[3*k+2] = stoi(coord[2]);
						k++;
					}

				}
				
				i++;
		}
		std::cout << "final vertices: "<<vertices[3*(numVertices-1)+2]<<std::endl;
		std::cout << "final triangle: "<<triangles[3*(numTriangles-1)+2]<<std::endl;
}

inline void SaveToFile(const char* filename, std::vector<int> id){
	std::ofstream target;
	target.open(filename);
	for(int i =0; i< id.size(); i++)
	{
		target << id[i];
		if(i!= id.size()-1)
			target << ',';
	}
	target.close();
}

#endif