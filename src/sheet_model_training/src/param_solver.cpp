#include "sheet_model_training/param_solver.hpp"
#include <algorithm>
#include "stdlib.h"
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <iostream>
#include <ros/package.h>
using namespace std;
double customminfunc(const std::vector<double>& x, std::vector<double>& grad, void* data) 
{
    // Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.
    Param_Solver *ps = (Param_Solver *) data;
    // cout <<"im here\n";
    return ps->ObjFun(x,grad);
}

Param_Solver::Param_Solver(nlopt::algorithm& Algo, const int& max_iter, const std::vector<std::string>& meshname,
                const std::vector<std::string>& training_data,
                const std::vector<double>& _X_init,const double& OptH,const double& OptXtolRel,const double& OptFtolRel,
                const std::vector<double> LBounds, const std::vector<double> UBounds,const double& timeStep, const unsigned int simulate_time)
{
	        //choose optimizer
        // alg_type = nlopt::LN_NEWUOA;
        // alg_type = nlopt::LN_NEWUOA_BOUND;
        // alg_type = nlopt::LD_MMA;
        // alg_type = nlopt::LD_TNEWTON_PRECOND_RESTART;
        // alg_type = nlopt::LN_BOBYQA;
        // alg_type = nlopt::LN_COBYLA;
        // alg_type = nlopt::LD_SLSQP;
        // alg_type = nlopt::LD_LBFGS;
        alg_type = nlopt::GN_ISRES;

        alg_type = Algo;
        maxeval = max_iter;
        sheet_timestep = timeStep;
        sheet = new Composite_Training(true); //True turns FEM on
        // sheet = new Composite_Training(false);
        sheet->setTimeStep(sheet_timestep);

    	objmeshname = meshname;
        Training_data_file = training_data;
	    // FixptsFile = fixpts;
	    // RealdataFile = dataname;
	    
	    X_init = _X_init;
        optH = OptH;
        optXtolRel = OptXtolRel;
        optFtolRel = OptFtolRel;
        //Optimization Parameters
        OptVarDim = _X_init.size();
        opt = nlopt::opt(alg_type, OptVarDim);
        OptVarlb.resize(OptVarDim);
        OptVarub.resize(OptVarDim);
        OptVarlb = LBounds;
        OptVarub = UBounds;
        cout << "[Param Dimention: "<< OptVarDim<<" ]" <<endl;
        Simulation_time = simulate_time;
        opt.set_xtol_rel(optXtolRel);//if the step doesn't change much in param
        opt.set_min_objective(customminfunc, this);
        opt.set_lower_bounds(OptVarlb);
        opt.set_upper_bounds(OptVarub);
        opt.set_ftol_rel(optFtolRel);//if the step doesn't change much in grad
        opt.set_population(50);
        opt.set_maxeval(maxeval);
        cout << "[Param_Solver constructed]"<<endl;
        iter = 0;
        // optx.resize(OptVarDim);

}

// double Param_Solver::ErrFun(const std::vector<double> &x)
// {
// 	// cout << "[Enter ERRFun]"<<endl;
// 	// double err;
// 	// double total_err=0;
//  //    // double max = 0;
// 	// // cout << "[Current parameter:";
// 	// // for(int i=0; i<x.size(); i++)
// 	// // 	cout <<" " <<x[i];
// 	// // cout << "]\n";

// 	// for(int i=0; i<RealdataFile.size();i++)
// 	// {
// 	// 	err = sheet->solve_err(objmeshname,FixptsFile,RealdataFile[i].data(),x,Simulation_time);
//  //        if(max<err)
//  //            max = err;
// 	// 	// cout << "#"<< i+1<<" err:"<< err<<endl;
// 	// 	total_err+=err;
// 	// }
	
// 	// total_err/=RealdataFile.size();
//  //    // total_err = total_err*0.75+max*0.25;
// 	// if(min_err>total_err){

// 	// 	min_err = total_err;		
// 	// 	savesol(x);
// 	// }
// 	// // std::cout <<"[ #"<<iter<<" ] 30 files done"<<std::endl;
// 	// sheet -> ResetInstance();
// 	// sheet = Composite::getInstance();
//  //    sheet->setTimeStep(sheet_timestep);
//  //    return total_err;
// }

double Param_Solver::ErrFun(const std::vector<double> &x)
{
    // cout << "[Enter ERRFun]"<<endl;
    double err1, err2;
    double total_err=0;
    double total_err1=0;
    double total_err2=0;
    // double min_err;
    // cout << min_err << endl;
    double max_err;
    int fix;
    std::vector<double> temp;
    temp.resize(3);
    // int count=0;
    // double max = 0;
    // // cout << "[Current parameter:";
    // // for(int i=0; i<x.size(); i++)
    // //   cout <<" " <<x[i];
    // // cout << "]\n";



    // if(alg_type == nlopt::LD_LBFGS){
    //     // temp[0] = 1000*x[0]+1000;
    //     // temp[1] = 200000*x[1]+800000;
    //     temp[0] = 10000000*x[0];
    //     temp[1] = (10000000000*x[0])+(4000000000*x[1]);
    //     cout <<temp[0]<<" " <<temp[1]<<endl;
    // }
    // else if(alg_type == nlopt::GN_ISRES){
    //     // temp[0] = 1000*x[0]+1000;
    //     // temp[1] = 200000*x[1]+800000;
    //     temp[0] = 10000000*x[0];
    //     temp[1] = (10000000000*x[0])+(4000000000*x[1]);
    //     cout <<temp[0]<<" " <<temp[1]<<endl;
    // }
    // double shear= 3.3e2;
    // double bend= 3.3e8;



    // double shear= 619.842;
    // double bend= 729.371e5;
    // # 9 :[ Current Sol(0.0841121):  0.149637 0.30045]
    
    // min_err: 0.0746245
    // best values so far.. Shear: 984.64    Bend: 1.54765e+08

    // min_err: 0.0733688
    // best values so far.. Shear: 888.214    Bend: 1.72827e+08


    double shear= 2846.86;
    double bend= 3.68391e07;

    // min_err: 0.0719615
    // best values so far.. Shear: 784.905    Bend: 1.33168e+08




    // min_err: 0.229569
    // best values so far.. Shear: 704.483    Bend: 1.10711e+08
    // [ #1104 ]1                  files done ERR: 0.34484







    temp[0]= shear*pow((1+x[0]),7);
    temp[1]= bend*pow((1+x[1]),7);


    double dampingStiffness= 0.02;
    temp[2]= dampingStiffness;

    // if(alg_type == nlopt::LD_LBFGS){
    //     err = sheet->solve_err_with_constraints(objmeshname[i].data(),Training_data_file[i].data(),temp,fix,1,Simulation_time);
    // }
    // else{
    //     err = sheet->solve_err_with_constraints(objmeshname[i].data(),Training_data_file[i].data(),temp,fix,1,Simulation_time);
    //     // err = sheet->solve_err_with_constraints(objmeshname[i].data(),Training_data_file[i].data(),x,fix,1,Simulation_time);
    // }
    //  sheet->getParameters();
    //  if(sheet->getElasticEnergy() > 10){}
    //     err*=100;
    //  cout << "#"<< i+1<<" err:"<< err<<endl;
    //  total_err+=err;
    // }

    // for(int i=0; i<objmeshname.size();i++)
    for(int i=0; i<Training_data_file.size()-5;i++)
    {
        fix= i;
        if(alg_type == nlopt::LD_LBFGS){
            err1 = sheet->solve_err_with_constraints1(objmeshname[0].data(),Training_data_file[i].data(),temp,fix,1,Simulation_time);
            err2 = sheet->solve_err_with_constraints2(objmeshname[1].data(),Training_data_file[i+5].data(),temp,fix,1,Simulation_time);
        }
        else{
            err1 = sheet->solve_err_with_constraints1(objmeshname[0].data(),Training_data_file[i].data(),temp,fix,1,Simulation_time);
            err2 = sheet->solve_err_with_constraints2(objmeshname[1].data(),Training_data_file[i+5].data(),temp,fix,1,Simulation_time);
            // err = sheet->solve_err_with_constraints(objmeshname[i].data(),Training_data_file[i].data(),x,fix,1,Simulation_time);
        }
        // cout << err << endl;
        // sheet->getParameters();
            // if(err>max)
            //     max = err;
        if(sheet->getElasticEnergy() > 10){
            err1*=100;
            err2*=100;
            cout << "ELASTIC ENERGY for Sheet 1!!!!!!!!!!!!!!!!! Too high, multiply error by 100:"<< err1 <<endl;
            cout << "ELASTIC ENERGY for Sheet 2!!!!!!!!!!!!!!!!! Too high, multiply error by 100:"<< err2 <<endl;
        }
        else {
            // cout << "Elastic Energy is good! Err:  "<< err <<endl;  
        }
         cout << "#"<< i+1<<" err1:"<< err1 <<endl;
         cout << "#"<< i+1<<" err2:"<< err2 <<endl;
         total_err1+=err1;
         total_err2+=err2;
    }
    total_err1/=Training_data_file.size()-5;
    total_err2/=Training_data_file.size()-5;
    std::cout << "total_err1: "<<total_err1<<std::endl;
    std::cout << "total_err2: "<<total_err2<<std::endl;



    // total_err-=max_err;
    // std::cout << "total_err: "<<total_err<<std::endl;
    // total_err/=Training_data_file.size();
    // total_err = total_err*0.75+max*0.25;


    
    // if(min_err==0.0){
        // cout << "Min error is 0" <<endl;
        // min_err=total_err;
    // }






    if(min_err >  total_err1 || min_err > total_err2){
         cout << "Updating MIN Error and SolX" << endl;
         min_err = std::min(total_err1, total_err2); 
         // alecx=temp; 
         alecx[0]=temp[0];
         alecx[1]=temp[1];     
         // savesol(x);
         // solx = x;
    }
    else{
        std::cout << "min_err: "<<min_err<<std::endl;
        std::cout << "best values so far.. Shear: "<<alecx[0] << "    Bend: " << alecx[1] << std::endl;
    }






    // std::cout <<"[ #"<<iter<<" ] 30 files done"<<std::endl;
    // std::cout << "total_err: "<<total_err<<std::endl;
    
    // delete sheet;
    // sheet->setTimeStep(sheet_timestep);
    total_err = std::max(total_err1, total_err2);
    return total_err;
}

// gradient computation:
// Forward Difference Method
double Param_Solver::ObjFun(const std::vector<double> &x, std::vector<double> &grad)
{
	// cout << "[Enter ObjFun]"<<endl;
	cout << "[Current parameter:";
	for(int i=0; i<x.size(); i++)
		cout <<" " <<x[i];
	cout << "]\n";
    double err = ErrFun(x);
    // cout <<"grad empty"<<endl;
    if (!grad.empty()) {
        std::vector<double> xph = x;
        // cout <<"grad:";
        for (uint i=0; i < x.size(); ++i)
        {
            
                xph[i] += optH;
                grad[i] = (ErrFun(xph)-err)/optH;
                xph[i] -= optH;
            

            // cout << " " <<grad[i];
        }
        cout <<"grad :"<<endl;
        for(uint i=0; i<grad.size(); i++){
        	cout << " " <<grad[i];
        }
        cout <<"\n";
        

    }    
    iter++;

    std::cout <<"[ #"<<iter<<" ]" << objmeshname.size()<<"                  files done ERR: "<<err <<std::endl;
    return err;
};


bool Param_Solver::solveOPT()
{
    solx = X_init;
    bool successFlag = false;
    // cout << "IM MADE IT: 3.1" << endl;
    try{
        
        nlopt::result result = opt.optimize(solx, solminf);
        // cout << "IM MADE IT: 3.2" << endl;
        cout <<"Result: "<<result<<endl;
        if(result == 3)
        	cout<<"NLOPT_FTOL_REACHED\n";
        else if(result == 4)
        	cout<<"NLOPT_XTOL_REACHED\n";
        successFlag = true;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }
    return successFlag;
};

double Param_Solver::get_solminf()
{
    return solminf;
};

std::vector<double> Param_Solver::get_solx()
{
    return solx;
};

void Param_Solver::checksol(){
	cout <<"[ Sol:" ;
	for(int i=0; i<solx.size();i++)
		cout <<" "<<solx[i];
	cout <<"]\n" ;
}

void Param_Solver::savesol(const char* Outputfile){
		ofstream fout;
            string package_name="sheet_model_training";
string package_path=ros::package::getPath(package_name);
string targetpath =package_path+"/data/boeing/output/param_globe.txt";
		if(alg_type != nlopt::LD_LBFGS)
			Outputfile =targetpath.c_str();
		fout<<"# "<< iter<<" :";
		fout <<"[ Sol:" ;
		for(int i=0; i<solx.size();i++)
			fout <<" "<<solx[i];
		fout <<"]\n" ;
		cout <<"file saved to "<< Outputfile<<endl;

}

void Param_Solver::savesol(const std::vector<double> &x){
		ofstream fout;
                    string package_name="sheet_model_training";
string package_path=ros::package::getPath(package_name);
string targetpath;
		if(alg_type == nlopt::LD_LBFGS){
            targetpath = package_path+"/data/boeing/output/param.txt";

			fout.open(targetpath.c_str());
        }
		else{
            targetpath = package_path+"/data/boeing/output/param_globe.txt";
			fout.open(targetpath.c_str());
        }
		fout<<"# "<< iter<<" :";
		fout <<"[ Current Sol(" << min_err<<"): ";
		for(int i=0; i<x.size();i++)
			fout <<" "<<x[i];
		fout <<"]\n" ;
		cout <<"file saved *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *  *"<<endl;	
}