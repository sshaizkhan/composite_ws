#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include "gen_utilities/iiwa_utilities.hpp"
#include "gen_utilities/iiwa7_FK_all_joints_mex.hpp"
#include "gen_utilities/iiwa14_FK_all_joints_mex.hpp"

namespace iiwa
{
    Eigen::MatrixXd compute_iiwa7_FK_all(Eigen::MatrixXd joint_angles, Eigen::MatrixXd rob_base_T)
    {
        // need to include the iiwa_FK_all_joints_mex.hpp file
        double joint_angles_arr[7];
        double *ptr_joint_angles;
        ptr_joint_angles = &joint_angles_arr[0]; 
        Eigen::Map<Eigen::MatrixXd>(ptr_joint_angles,joint_angles.rows(),joint_angles.cols()) = joint_angles;
        double rob_base_T_arr[16];
        double *ptr_rob_base_T;
        ptr_rob_base_T = &rob_base_T_arr[0];
        Eigen::Map<Eigen::MatrixXd>(ptr_rob_base_T,rob_base_T.rows(),rob_base_T.cols()) = rob_base_T;
        double FK_arr[144];
        iiwa7_FK_all_joints_mex( ptr_joint_angles, ptr_rob_base_T, FK_arr);
        Eigen::MatrixXd FK = Eigen::Map<Eigen::MatrixXd>(FK_arr,36,4);
        return FK;
    }

    Eigen::MatrixXd compute_iiwa7_FK_all(std::vector<double> joint_angles, Eigen::MatrixXd rob_base_T)
    {
        // need to include the iiwa_FK_all_joints_mex.hpp file
        double *ptr_joint_angles = &joint_angles[0];
        double rob_base_T_arr[16];
        double *ptr_rob_base_T;
        ptr_rob_base_T = &rob_base_T_arr[0];
        Eigen::Map<Eigen::MatrixXd>(ptr_rob_base_T,rob_base_T.rows(),rob_base_T.cols()) = rob_base_T;
        double FK_arr[144];
        iiwa7_FK_all_joints_mex( ptr_joint_angles, ptr_rob_base_T, FK_arr);
        Eigen::MatrixXd FK = Eigen::Map<Eigen::MatrixXd>(FK_arr,36,4);
        return FK;
    }

    Eigen::MatrixXd compute_iiwa14_FK_all(Eigen::MatrixXd joint_angles, Eigen::MatrixXd rob_base_T)
    {
        // need to include the iiwa_FK_all_joints_mex.hpp file
        double joint_angles_arr[7];
        double *ptr_joint_angles;
        ptr_joint_angles = &joint_angles_arr[0]; 
        Eigen::Map<Eigen::MatrixXd>(ptr_joint_angles,joint_angles.rows(),joint_angles.cols()) = joint_angles;
        double rob_base_T_arr[16];
        double *ptr_rob_base_T;
        ptr_rob_base_T = &rob_base_T_arr[0];
        Eigen::Map<Eigen::MatrixXd>(ptr_rob_base_T,rob_base_T.rows(),rob_base_T.cols()) = rob_base_T;
        double FK_arr[144];
        iiwa14_FK_all_joints_mex( ptr_joint_angles, ptr_rob_base_T, FK_arr);
        Eigen::MatrixXd FK = Eigen::Map<Eigen::MatrixXd>(FK_arr,36,4);
        return FK;
    }

    Eigen::MatrixXd compute_iiwa14_FK_all(std::vector<double> joint_angles, Eigen::MatrixXd rob_base_T)
    {
        // need to include the iiwa_FK_all_joints_mex.hpp file
        double *ptr_joint_angles = &joint_angles[0];
        double rob_base_T_arr[16];
        double *ptr_rob_base_T;
        ptr_rob_base_T = &rob_base_T_arr[0];
        Eigen::Map<Eigen::MatrixXd>(ptr_rob_base_T,rob_base_T.rows(),rob_base_T.cols()) = rob_base_T;
        double FK_arr[144];
        iiwa14_FK_all_joints_mex( ptr_joint_angles, ptr_rob_base_T, FK_arr);
        Eigen::MatrixXd FK = Eigen::Map<Eigen::MatrixXd>(FK_arr,36,4);
        return FK;
    }
}