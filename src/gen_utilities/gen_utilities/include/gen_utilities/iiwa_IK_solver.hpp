#ifndef OPT_OBJ_RW
#define OPT_OBJ_RW

#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <vector>

namespace iiwa_IK_solver
{
    class iiwa_IK_solver
    {
        private:

        public:
            // constructor and distructor
            iiwa_IK_solver(Eigen::MatrixXd Theta, Eigen::MatrixXd Point, Eigen::MatrixXd Robot_ree_T_tee, 
                std::vector<double> _X_init, double OptH, double OptXtolRel, int Rob_version, 
                std::vector<double> LBounds= {-2.9321506,-2.0594867,-2.9321506,-2.0594867,-2.9321506,-2.0594867,-3.0194170},
                std::vector<double> UBounds= {2.9321506,2.0594867,2.9321506,2.0594867,2.9321506,2.0594867,3.0194170}, double wiggle_angle=30);

            ~iiwa_IK_solver();

            // other variables
            Eigen::MatrixXd theta;
            Eigen::MatrixXd point;
            Eigen::MatrixXd robot_ree_T_tee;
            
            std::vector<double> X_init;
            
            // optim solver
            nlopt::opt opt;
            int OptVarDim;
            double optXtolRel;
            double optH;
            int status;
            int rob_version;
            double tolerance[4];
            double Wiggle_angle;
            std::vector<double> optx;
            std::vector<double> solx;
            double solminf;
            nlopt::algorithm alg_type;
            std::vector<double> OptVarlb;
            std::vector<double> OptVarub;   
            double ErrFun(const std::vector<double> &x);
            double NLConFun(const std::vector<double> &x);
            double ObjFun(const std::vector<double> &x, std::vector<double> &grad);
            double ConFun(const std::vector<double> &x, std::vector<double> &grad);
            bool solveOPT();
            double get_solminf();
            std::vector<double> get_solx();
            Eigen::MatrixXd get_IK();
    };
}
#endif