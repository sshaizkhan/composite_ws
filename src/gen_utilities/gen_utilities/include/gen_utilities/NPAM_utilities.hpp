#ifndef NPAM_UTILITIES_HPP
#define NPAM_UTILITIES_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>

class NPAM
{
private:
	static Eigen::MatrixXd align_pts(const Eigen::MatrixXd&);
	static Eigen::MatrixXd smoothened_traj_by_pts_skip(const Eigen::MatrixXd&, double);
	static Eigen::MatrixXd skip_toolpath_lines(const Eigen::MatrixXd&, int);
	
public:

	//! apply rotation to points about their centroid
	static Eigen::MatrixXd rotate_pts(Eigen::MatrixXd, double, double, double);
	
	//! find number of layers
	static int number_of_layers(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, double);
	
	//! generate uniform grid points on a plane 
	static Eigen::MatrixXd generate_grid_points(double, double, double, double, double, double, double);
	
	//! identify bottom layer from stl file data
	static Eigen::MatrixXd identify_bottom_layer(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&);

	//! identify top layer from stl file data
	static Eigen::MatrixXd identify_top_layer(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&);

	//! project data on the bottom plane
	static Eigen::MatrixXd project_grid_points(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, double, double, double);

	//! generate the hatching path with points
	static Eigen::MatrixXd Infill_Path(const Eigen::MatrixXd&, bool, double, double, double, double, int);
	static Eigen::MatrixXd Infill_Path_with_Normals(const Eigen::MatrixXd&, bool, double, double, double, double, int);
	static Eigen::MatrixXd Infill_Path_with_bxbybz(const Eigen::MatrixXd&, bool, double, double, double, double, int);
	static Eigen::MatrixXd Infill_Path_with_euler(const Eigen::MatrixXd&, bool, double, double, double, double, int);
};

#endif