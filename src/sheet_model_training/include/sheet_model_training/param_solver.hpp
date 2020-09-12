#ifndef PARAM_SOLVER
#define PARAM_SOLVER

#include "sheet_model_training/composite_training_interface.hpp"
#include "nlopt.hpp"
class Param_Solver{
	public:
		Param_Solver(nlopt::algorithm& Algo, const int& max_iter,  const std::vector<std::string>& meshname,
                const std::vector<std::string>& training_data,
                const std::vector<double>& _X_init,const double& OptH,const double& OptXtolRel,const double& OptFtolRel,
                const std::vector<double> LBounds, const std::vector<double> UBounds,const double& timeStep, const unsigned int simulate_time=100);
		~Param_Solver(){};

			Composite_Training* sheet;

		    std::vector<double> X_init;
            
            std::vector<std::string> objmeshname;
            std::vector<std::string> Training_data_file;

               
            // optim solver
            nlopt::opt opt;
            int OptVarDim;
            double optXtolRel;
            double optFtolRel;
            double optH;
            int iter;
            int maxeval;
            std::vector<double> OptVarlb;
            std::vector<double> OptVarub; 
            double sheet_timestep;
            double min_err = 10000;
            // std::vector<double> optx;
            std::vector<double> solx;
            std::vector<double> alecx {0.0,0.0};
            double solminf;
            nlopt::algorithm alg_type;
            unsigned int Simulation_time;
            // std::vector<double> OptVarlb;
            // std::vector<double> OptVarub;   
            double ErrFun(const std::vector<double> &x);
            // double NLConFun(const std::vector<double> &x);
            double ObjFun(const std::vector<double> &x, std::vector<double> &grad);
            // double ConFun(const std::vector<double> &x, std::vector<double> &grad);
            bool solveOPT();
            double get_solminf();
            std::vector<double> get_solx();

            void checksol();
            void savesol(const char* Outputfile);
            void savesol(const std::vector<double> &x);
            // Eigen::MatrixXd get_IK();
    
};

#endif