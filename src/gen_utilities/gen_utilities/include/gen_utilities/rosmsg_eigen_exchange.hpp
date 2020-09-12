#ifndef ROSMSG_EIGEN_EXCHANGE
#define ROSMSG_EIGEN_EXCHANGE

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/PointCloud.h"
#include "gen_utilities/XYZ_bxbybz.h"
#include "gen_utilities/Waypoints.h"
#include "gen_utilities/KukaJointAnglesSet.h"

//! Methods for conversion between ros msgs and eigen matrices

namespace ros_eigen_exch
{
	//! convert ros pose message to 1D Eigen::Matrix
	inline Eigen::MatrixXd pose_to_eigen(geometry_msgs::Pose&);
	//! convert ros pose arrays message to Eigen::Matrix rows of pose
	inline Eigen::MatrixXd pose_array_to_eigen(geometry_msgs::PoseArray& );
	//! convert 1D Eigen::Matrix to ros pose message
	inline geometry_msgs::Pose eigen_to_pose(Eigen::MatrixXd);
	//! convert Eigen::Matrix rows of ros poses to pose arrays message
	inline geometry_msgs::PoseArray eigen_to_pose_array(Eigen::MatrixXd);
	//! convert ros joint angles state message to Eigen::Matrix
	inline Eigen::MatrixXd joint_state_to_eigen(sensor_msgs::JointState&);
	//! convert Eigen::Matrix to ros joint angles state message
	inline sensor_msgs::JointState eigen_to_joint_state(Eigen::MatrixXd);
	//! convert custom ros waypoints in x-y-z-bx-by-bz message format to Eigen::Matrix
	inline Eigen::MatrixXd xyz_bxbybz_to_eigen(gen_utilities::Waypoints&);
	//! convert Eigen::Matrix to custom ros waypoints in x-y-z-bx-by-bz format
	inline gen_utilities::Waypoints eigen_to_xyz_bxbybz(Eigen::MatrixXd);
	//! convert custom ros waypoints in x-y-z-c-b-a message format to Eigen::Matrix
	inline Eigen::MatrixXd xyz_cba_to_eigen(gen_utilities::Waypoints&);
	//! convert Eigen::Matrix to custom ros waypoints in x-y-z-c-b-a format
	inline gen_utilities::Waypoints eigen_to_xyz_cba(Eigen::MatrixXd);
	//! convert custom ros kuka joint angles message to Eigen::Matrix
	inline Eigen::MatrixXd joint_angles_to_eigen(gen_utilities::KukaJointAnglesSet&);
	//! convert Eigen::Matrix to custom ros kuka joint angles message
	inline gen_utilities::KukaJointAnglesSet eigen_to_joint_angles(Eigen::MatrixXd);
	//! convert custom ros group index message to Eigen::Matrix
	inline Eigen::MatrixXd group_idx_to_eigen(gen_utilities::KukaJointAnglesSet&);
	//! convert Eigen::Matrix to custom ros group index message
	inline gen_utilities::KukaJointAnglesSet eigen_to_group_idx(Eigen::MatrixXd);
	//! convert Eigen::Matrix to ros pointcloud message
	inline sensor_msgs::PointCloud eigen_to_pointcloud(Eigen::MatrixXd);
	//! convert ros pointcloud message to Eigen::Matrix
	inline Eigen::MatrixXd pointcloud_to_eigen(sensor_msgs::PointCloud);

};

#ifndef ROSMSG_EIGEN_EXCHANGE_CPP
#  include "../../src/rosmsg_eigen_exchange.cpp"
#endif

#endif