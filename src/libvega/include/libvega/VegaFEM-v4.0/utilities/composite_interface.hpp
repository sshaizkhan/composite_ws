#include <math.h>
#include <vector>

#include "objMesh.h"
#include "clothBWFromObjMesh.h"
#include "clothBWStencilForceModel.h"
#include "forceModelAssembler.h"
#include "implicitNewmarkSparse.h"
#include "implicitBackwardEulerSparse.h"


// #include "constrainedDOFs.h"
#include <iostream>

class Composite
{
private:
    static bool instanceFlag;
    static Composite *sheet;
    
    //Physics Parameters
    //The one that will be updated from ML 
    float dampingMass;
	float dampingStiffness;
	float surfaceDensity;
	float tensileStiffness;
	float shearStiffness;
	float bendStiffnessU;
	float bendStiffnessV;
	//Const Parameters
	const float gravityForce=9.81;//9.81
	//Simulation and Calculation Parameters
	float timeStep=0.01;
	//Computation parameters (default to computing everything)
	int computeStretchShearForce, computeStretchShearStiffness, computeBendForce, computeBendStiffness;
	int numInternalForceThreads; // default = 0 (not multi-threaded)
	int numSolverThreads=3; // default = 0 (not multi-threaded)
	//Deformation Parameters
	double* deform = NULL;
	double* f_ext = NULL;
	//Model objects
	ClothBW * clothBW;
	ClothBWStencilForceModel * clothBWStencilForceModel;
	ForceModelAssembler * forceModelAssembler;
	ObjMesh * objMesh;
	ClothBWFromObjMesh * clothBWFromObjMesh;
	IntegratorBase * integratorBase;
	ImplicitNewmarkSparse * implicitNewmarkSparse;
	ImplicitBackwardEulerSparse * implicitBackwardEulerSparse;
	ForceModel * forceModel;
	SparseMatrix * massMatrix;
	int numVertices=0;
	//Fixed Vertex
	std::vector<int> fixedVertices;
	int numConstrainedDOFs=0;
	int* constrainedDOFs=NULL;

	Composite();
public:
    static Composite* getInstance();

    void updateParam(const float* param);
    void loadMesh(const char* objMeshname);//ptnNum will be updated here
    void generateSheet(const char* objMeshname,const float* param=NULL);//add physical parameters and force model
    void updateConstrain(std::vector<int>& fix);
    

    //data retrieve methods
    void method();
    void getParameters();
    //this is the main function
    //@p is the parameter array
    //@meshname is the file name of sheet mesh (.obj)
    //@fixed is the fixed vertex index
    //Return @deform is the deformation information
    double* solve(const char* objMeshname,std::vector<int>& fixed,const float* param=NULL,const int& time=1000);

    ~Composite()
    {
        delete clothBW;
        delete objMesh;
        instanceFlag = false;

    }
};

