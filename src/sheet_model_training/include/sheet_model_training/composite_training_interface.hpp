#ifndef COMPOSITE_TRAINING_INTERFACE_HPP
#define COMPOSITE_TRAINING_INTERFACE_HPP

#include "composite_utilities/composite_interface.hpp"
#include "implicitEulerMultiObjectWithConstraintsSparse.h"
#include "constraints.h"
#include "clothFEM.h"
#include "clothFEMStencilForceModel.h"
#include "triMeshGeo.h"
class Composite_Training : public Composite
{
public:
	// Composite_Training(){};
	Composite_Training(bool fem);
	Composite_Training(bool fem, bool debug):Composite(fem,debug){};

		/**********************************Training*******************************/
	//Funcitons below are used for training purpose, to simulate the sheet please use above functions
	//[Training Used]loadMesh	load inital mesh files and training data (unit: mm)
    void loadMesh(const char* objMeshname, const char* training_data, const int& fixNum=4, const double& scale=1000.0);
    // void AddTriConstraints(const int& objIndex, int v1, int v2, int v3, double bary[4][3], int VID[4], std::vector<Vec3d>& constraintPts);
    virtual void updateConfig(const char* objMeshname, const char* robot_data) override;
    void AddSurfaceConstraints();
    // void updateParam_with_FEM(const std::vector<double>& p);
    // virtual void updateParam(const std::vector<double>& p) override;
	//solve_err		This function is used for training ONLY
    //construct integrater solver
    // void generateFEM();
    // void generateSheetBW();
    // double solve_err(const char* objMeshname,const char* training_data,const std::vector<double>& param,const int& time=100);
    virtual void updateSheetStatus(const char* objMeshname, const char* training_data, const std::vector<double>& param, const int& fixNum = 4, bool is_permanent=false);
    double solve_err_with_constraints(const char* objMeshname,const char* training_data,const std::vector<double>& param, const int& fixNum = 4,  bool is_permanent=false,const int& time=100);
    //show_err		err_type 0 for average err, 1 for weighted_err, 2 for std_err, 3 for maximum err, 4 for min_err
    //Error is defined as 0.5*offset + 0.5* maximum offset
    //Need to be used after solve_err() 
    // double show_err(const char* objMeshname,const char* training_data,const std::vector<double>& param,const int& time=100,const int& err_type=0);
    // void updateSheetStatus_with_constraints(const char* objMeshname, const char* robot_data);
    // double* simulate_with_constraints(const int& iteration);
    // double getError();
    // void getParameters_FEM();
    // void getParameters();
    inline std::vector<Vec3d>& getTrainingFix(){return m_training_fix;};
    inline std::vector<Vec3d>& getTrainingMarker(){return m_training_marker;};
    // inline bool isFEM_ON(){return FEM_ON;};

    // inline void setThickness(const double& value){sheet_thickness = value;};
    // inline void setPoisson(const double& value){sheet_poisson = value;};
    virtual ~Composite_Training(){
	std::cout << "Training sheet cleared\n";
    };
protected:
	//Training Parameters
	double m_avg_err = 0;
	double m_opt_err = 0;
	double m_max_err = 0;
	double m_min_err = 100;
	double m_std_err = 0;
	//constraint updates
	
	
	std::vector<Vec3d> m_training_fix;
	std::vector<Vec3d> m_training_marker;
	
	
	//clothFEM updates
	// int numVertices;
	// double * restVertexPositions;
	// int numTriangles;
	// int * triangleVertexIndices;
	// std::vector<double> masses;

	// ClothFEM * clothFEM = NULL;
	// ClothFEMStencilForceModel * clothFEMStencilForceModel = NULL;
	// ClothBW * clothBW=NULL;
	// ClothBWFromObjMesh * clothBWFromObjMesh=NULL;
	// ClothBWStencilForceModel * clothBWStencilForceModel=NULL;
	// ForceModelAssembler * forceModelAssembler=NULL;
	//FEM parameter
	// bool FEM_ON=false;
	// double sheet_thickness=0.001;
	// double sheet_poisson=0.3;
	// double shear_stretch_Material;
	// double bend_Material;

};
inline void  parseData(const char* filename, std::vector<Vec3d>& fixed, std::vector<Vec3d>& point,const int& fixNum, const int& ifCenter){
		std::fstream fin;
		// std::cout<<"file name:"<< filename << std::endl;
	  	fin.open(filename);
		if(!fin.is_open()){
			std::cout<<"open failed"<<std::endl;
		}
		int fix_number = fixNum;
	    double unit_conversion = 1000; //Alec added 070320
		fix_number*=4;
		std::string line;
		std::vector<std::string> coord;
		std::string value;
		// int count=0;
		int i=0;
		if(fixed.size()!=0)
			fixed.clear();
		if(point.size()!=0)
			point.clear();
		while(getline(fin,line)){
				std::stringstream ss(line);
				coord.clear();
				while(getline(ss,value,' ')){
					coord.push_back(value);
					// std::cout<<value<<std::endl;
				}
				// Alec removed 070320 ///////////////////
				// if(i>ifCenter-1 && i<fix_number+ifCenter)
					// fixed.push_back(Vec3d(stod(coord[0])/unit_conversion,stod(coord[1])/unit_conversion,stod(coord[2])/unit_conversion));
				// else if(i>ifCenter-1)
				point.push_back(Vec3d(stod(coord[0])/unit_conversion,stod(coord[1])/unit_conversion,stod(coord[2])/unit_conversion));
				i++;
		}
}

inline void parseTrainingData(const char* filename, const ObjMesh* obj, std::vector<Vec3d>& training_fix, 
								std::vector<Vec3d>& training_marker, std::vector<int>& fixgroups,const int& fixNum, const int& ifCenter){

	parseData(filename, training_fix, training_marker, fixNum, ifCenter);
	for(int i=0; i< training_fix.size(); i++)
		fixgroups.push_back(obj->getClosestVertex(training_fix[i]));
	std::sort(fixgroups.begin(), fixgroups.end());

}
#endif