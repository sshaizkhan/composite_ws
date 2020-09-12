#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <chrono>

namespace ut
{
	//! get unique rows in a vector
	std::vector<std::vector<int> > get_unique_rows(std::vector<std::vector<int> >);
	std::vector<std::vector<float> > get_unique_rows(std::vector<std::vector<float> >);
	std::vector<std::vector<double> > get_unique_rows(std::vector<std::vector<double> >);
	
	//! vector to matrix conversion
	Eigen::MatrixXi vec_to_mat(std::vector<std::vector<int> >);
	Eigen::MatrixXf vec_to_mat(std::vector<std::vector<float> >);
	Eigen::MatrixXd vec_to_mat(std::vector<std::vector<double> >);
	
	
	//! matrix to vector conversion
	std::vector<std::vector<int> > mat_to_vec(Eigen::MatrixXi);
	std::vector<std::vector<float> > mat_to_vec(Eigen::MatrixXf);
	std::vector<std::vector<double> > mat_to_vec(Eigen::MatrixXd);

	//! print vector
	void disp_vec(std::vector<std::vector<int> >);
	void disp_vec(std::vector<std::vector<float> >);
	void disp_vec(std::vector<std::vector<double> >);
	
	//! Compute bxbybz (nx9 vector) from normals
	Eigen::MatrixXd compute_TCP(Eigen::MatrixXd, Eigen::MatrixXd);
	double get_pt_to_lsf_plane_dist(Eigen::MatrixXd, Eigen::MatrixXd);
	Eigen::MatrixXd get_traj_wrt_tcp(Eigen::Matrix4d, std::vector<std::vector<double> >);

	//! sort matrix based on the input column
	std::vector<std::vector<int> > SortRows(std::vector<std::vector<int> >, int);
	std::vector<std::vector<float> > SortRows(std::vector<std::vector<float> >, int);
	std::vector<std::vector<double> > SortRows(std::vector<std::vector<double> >, int);
	
	//! check if row/vector is a member of matrix/2D vector
	std::vector<int> ismember(std::vector<std::vector<int> >, std::vector<int>);
	std::vector<int> ismember(std::vector<std::vector<float> >, std::vector<float>);
	std::vector<int> ismember(std::vector<std::vector<double> >, std::vector<double>);
	std::vector<int> ismember(Eigen::MatrixXi, Eigen::MatrixXi);
	std::vector<int> ismember(Eigen::MatrixXf, Eigen::MatrixXf);
	std::vector<int> ismember(Eigen::MatrixXd, Eigen::MatrixXd);

	//! find rowwise mean
	Eigen::MatrixXd mean(Eigen::MatrixXi);
	Eigen::MatrixXd mean(Eigen::MatrixXf);
	Eigen::MatrixXd mean(Eigen::MatrixXd);
	
	//! find median
	double median(std::vector<int>);
	double median(std::vector<float>);
	double median(std::vector<double>);

	//! linearly spaced points between start and end point
	Eigen::VectorXd linsp(double, double, double);

	//! inpolygon function to check if point/s is/are inside polygon 
	Eigen::MatrixXd InPoly(Eigen::MatrixXd, Eigen::MatrixXd);
	void InPoly(const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::MatrixXi&);
	void InPoly(const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::MatrixXd&);
	bool lines_intersect(double l1[2][2], double l2[2][2]);

	//! find the index of non-zero elements
	std::vector<int> find_idx(Eigen::VectorXi);
	std::vector<int> find_idx(Eigen::VectorXf);
	std::vector<int> find_idx(Eigen::VectorXd);
	std::vector<int> find_idx(Eigen::MatrixXd);
	std::vector<int> find_idx(Eigen::MatrixXi);
	std::vector<int> find_idx(std::vector<int>);
	std::vector<int> find_idx(std::vector<float>);
	std::vector<int> find_idx(std::vector<double>);

	//! get the norm of vector or 1-d matrix
	double vec_norm(Eigen::MatrixXd);
	double vec_norm(std::vector<double>);
	double vec_norm(std::vector<int>);

	//! generating uniform pointcloud on the mesh model
	Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, double, double);
	Eigen::MatrixXd add_pts(Eigen::MatrixXd, Eigen::MatrixXd);
	Eigen::MatrixXd generate_grid_points(double, double, double, double, double, double);

	//! timer for execution time evaluation
	void timer_start();
	void timer_end(std::string time_unit="millisec");

	//! get intersetcion of 2 lines defined by their end points
	Eigen::MatrixXd intersect_lines(Eigen::MatrixXd , Eigen::MatrixXd , Eigen::MatrixXd , Eigen::MatrixXd);
};

#endif
