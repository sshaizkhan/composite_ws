#ifndef IIWA_UTILITIES_HPP
#define IIWA_UTILITIES_HPP

#include <Eigen/Eigen>
#include <vector>
#include <string>

namespace iiwa
{
	Eigen::MatrixXd compute_iiwa7_FK(Eigen::MatrixXd, Eigen::MatrixXd);
	Eigen::MatrixXd compute_iiwa7_FK(std::vector<double>, Eigen::MatrixXd);
	Eigen::MatrixXd compute_iiwa7_FK_all(Eigen::MatrixXd, Eigen::MatrixXd);
	Eigen::MatrixXd compute_iiwa7_FK_all(std::vector<double>, Eigen::MatrixXd);
	Eigen::MatrixXd compute_iiwa14_FK_all(Eigen::MatrixXd, Eigen::MatrixXd);
	Eigen::MatrixXd compute_iiwa14_FK_all(std::vector<double>, Eigen::MatrixXd);
};

#endif