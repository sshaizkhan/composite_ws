// #include "composite_interface.hpp"
#include"ros/ros.h"
#include "sheet_model_training/param_solver.hpp"
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <ros/package.h>
using namespace std;
// void save(std::vector<point> pts, char* filename)
// {
// 	std::ofstream myfile;
// 	myfile.open(filename);
// 	for(int i=0; i< pts.size(); i++)
// 		myfile<<pts[i];
// 	std::cout<<"file: "<<filename<<" saved"<<std::endl;
// 	myfile.close();
// }


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "training");
	// cout << "IM MADE IT: 1" << endl;
	// //TEST
	// //*surfaceDensity tensileStiffness shearStiffness bendStiffnessU bendStiffnessV dampingMass dampingStiffness
	std::vector<std::vector<double>>Lb;
	std::vector<std::vector<double>>Ub;
	std::vector<std::vector<double>>Sol;
	std::vector<double> Ltemp;
	std::vector<double> Utemp;
	std::vector<double> pass_t;

	Ltemp= {-0.5, -0.5};
	Utemp={0.5, 0.5};
	Lb.push_back(Ltemp);
	Ub.push_back(Utemp);
	// std::vector<double> Lb= {100, 5e7, 5e7, 10, 10, 0.0, 0.1};
	// std::vector<double> Ub={100, 5e7, 5e7, 100, 100, 0.0, 0.1};
	// float p[7]={0.00219,5e7,5e7,100,100,0.0,0.1};//good param
	// float p[7]={100,5e7,5e7,10,10,0.0,0.1}; //extreme param
	// float p[7]={4.601e-7, 550, 550, 505, 505, 0.0, 0.101}; //initial param
	// float p[3]={5000, 5000,0.001}; //initial param//init
	// float p[3]={6256.86,4951.3,0.000900619}; //boeing2
	// float p[3]={8575.38, 8751.91, 0.000900105};//LM
	// float p[2]={1000,800000};//boeing
	// float p[2]={-0.32,-0.14};//boeing
	// float p[2] = {0.28,-0.376};//boeing


	// float p[2] = {0.33,0.05}; //AlecZach
	float p[2] = {0.00,0.00}; //AlecZach



	// float p[7]={100,8500,100,0.01,0.01,0.0,0.001};
	// float p[7]={1,85,10,0.01,0.01,0.0,0.001};
	std::vector<double> vp;

	for(int i = 0; i < (sizeof(p)/sizeof(*p) ); i++)
	{
		vp.push_back(p[i]);
	}
	// std::vector<const char*> meshnames, fixindexfiles, testname;
	std::string package_name="sheet_model_training";
	std::string package_path=ros::package::getPath(package_name);
	std::vector<std::string> meshnames;//, training_data;
	std::string targetpath = package_path+"/data/boeing/output/param.txt";
	// std::vector<std::string> fixcoordfiles;
	const char* output = targetpath.c_str();
	
	std::string meshpath;
	std::string trainingpath;
    // std::vector<std::string> meshnames;
    meshnames.push_back(argv[3]);

    cout << "package path:" << package_path << endl;

    std::vector<std::string> training_data(5);

    training_data[0]= package_path+ "/data/T4/Txt_T4S1_Final.txt";
    training_data[1]= package_path+ "/data/T4/Txt_T4S2_Final.txt";
    training_data[2]= package_path+ "/data/T4/Txt_T4S3_Final.txt";
    training_data[3]= package_path+ "/data/T4/Txt_T4S4_Final.txt";
    training_data[4]= package_path+ "/data/T4/Txt_T4S5_Final.txt";

    // training_data[0]= argv[11];  // use parse data here
    // training_data[1]= argv[12];
    // training_data[2]= argv[13];
    // training_data[3]= argv[14];
    // training_data[4]= argv[15];
    cout << "training_data 0:" << training_data[0] << endl;
    cout << "training_data 1:" << training_data[1] << endl;
    cout << "training_data 2:" << training_data[2] << endl;
    cout << "training_data 3:" << training_data[3] << endl;
    cout << "training_data 4:" << training_data[4] << endl;
 
	
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
	Param_Solver* ps;
	nlopt::algorithm alg_type = nlopt::GN_ISRES;
	// nlopt::algorithm alg_type = nlopt::LD_LBFGS;
	
	// cout << "IM MADE IT: 2" << endl;
	     																						    //    ts  iterations
	ps = new Param_Solver(alg_type, 3000, meshnames, training_data, vp, 1e-8, 1e-8, 1e-8, Lb[0], Ub[0], 0.01, 100);
	// cout << "IM MADE IT: 3" << endl;
	ps->solveOPT();
	// cout << "IM MADE IT: 4" << endl;
	ps->checksol();
	ps->savesol(output);

	return 0;
}