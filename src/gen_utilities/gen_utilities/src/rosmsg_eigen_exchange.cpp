#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/PointCloud.h"
#include "gen_utilities/XYZ_bxbybz.h"
#include "gen_utilities/Waypoints.h"
#include "gen_utilities/KukaJointAnglesSet.h"

///////////////////////////////////////////////////////////

namespace ros_eigen_exch 
{
	inline Eigen::MatrixXd pose_to_eigen(geometry_msgs::Pose& pos)
	{	
		Eigen::MatrixXd mat(1,7);
	    mat(0,0) = pos.position.x;
	    mat(0,1) = pos.position.y;
	    mat(0,2) = pos.position.z;
	    mat(0,3) = pos.orientation.x;
	    mat(0,4) = pos.orientation.y;
	    mat(0,5) = pos.orientation.z;
	    mat(0,6) = pos.orientation.w;
	    return mat;
	}

	///////////////////////////////////////////////////////////

	inline Eigen::MatrixXd pose_array_to_eigen(geometry_msgs::PoseArray& pos)
	{	
		Eigen::MatrixXd mat(pos.poses.size(),7);
	    for (int i=0;i<mat.rows();++i)
	    {
		    mat(i,0) = pos.poses[i].position.x;
		    mat(i,1) = pos.poses[i].position.y;
		    mat(i,2) = pos.poses[i].position.z;
		    mat(i,3) = pos.poses[i].orientation.x;
		    mat(i,4) = pos.poses[i].orientation.y;
		    mat(i,5) = pos.poses[i].orientation.z;
		    mat(i,6) = pos.poses[i].orientation.w;
	    }
	    return mat;
	}

	///////////////////////////////////////////////////////////

	inline geometry_msgs::Pose eigen_to_pose(Eigen::MatrixXd mat)
	{	
		geometry_msgs::Pose pos;
		pos.position.x = mat(0,0);
	    pos.position.y = mat(0,1);
	    pos.position.z = mat(0,2);
	    pos.orientation.x = mat(0,3);
	    pos.orientation.y = mat(0,4);
	    pos.orientation.z = mat(0,5);
	    pos.orientation.w = mat(0,6);
	    return pos;
	}

	///////////////////////////////////////////////////////////

	inline geometry_msgs::PoseArray eigen_to_pose_array(Eigen::MatrixXd mat)
	{	
		geometry_msgs::PoseArray pos;
		pos.poses.resize(mat.rows());
	    for (int i=0;i<mat.rows();++i)
	    {
	    pos.poses[i].position.x = mat(i,0);
	    pos.poses[i].position.y = mat(i,1);
	    pos.poses[i].position.z = mat(i,2);
	    pos.poses[i].orientation.x = mat(i,3);
	    pos.poses[i].orientation.y = mat(i,4);
	    pos.poses[i].orientation.z = mat(i,5);
	    pos.poses[i].orientation.w = mat(i,6);
	    }
	    return pos;
	}

	///////////////////////////////////////////////////////////

	inline Eigen::MatrixXd joint_state_to_eigen(sensor_msgs::JointState& j)
	{	
		Eigen::MatrixXd mat(1,j.position.size());
	    for (int i=0;i<mat.cols();++i)
	    {
	    	mat(0,i) = j.position[i];
	    }
	    return mat;
	}

	///////////////////////////////////////////////////////////

	inline sensor_msgs::JointState eigen_to_joint_state(Eigen::MatrixXd mat)
	{	
		sensor_msgs::JointState j;
		j.position.resize(mat.cols());
		for (int i=0;i<mat.cols();++i)
	    {
	    	j.position[i] = mat(0,i);
	    }
	    return j;
	}

	///////////////////////////////////////////////////////////

	inline Eigen::MatrixXd xyz_bxbybz_to_eigen(gen_utilities::Waypoints& wp)
	{	
		Eigen::MatrixXd mat(wp.xyz_bxbybz.size(),12);
		for (int i=0;i<mat.rows();++i)
		{
			mat(i,0) = wp.xyz_bxbybz[i].x;
			mat(i,1) = wp.xyz_bxbybz[i].y;
			mat(i,2) = wp.xyz_bxbybz[i].z;
			mat(i,3) = wp.xyz_bxbybz[i].bx_i;
			mat(i,4) = wp.xyz_bxbybz[i].bx_j;
			mat(i,5) = wp.xyz_bxbybz[i].bx_k;
			mat(i,6) = wp.xyz_bxbybz[i].by_i;
			mat(i,7) = wp.xyz_bxbybz[i].by_j;
			mat(i,8) = wp.xyz_bxbybz[i].by_k;
			mat(i,9) = wp.xyz_bxbybz[i].bz_i;
			mat(i,10) = wp.xyz_bxbybz[i].bz_j;
			mat(i,11) = wp.xyz_bxbybz[i].bz_k;
		}
	    return mat;
	}

	///////////////////////////////////////////////////////////

	inline gen_utilities::Waypoints eigen_to_xyz_bxbybz(Eigen::MatrixXd mat)
	{	
		gen_utilities::Waypoints wp;
		wp.xyz_bxbybz.resize(mat.rows());
		for (int i=0;i<mat.rows();++i)
		{
			wp.xyz_bxbybz[i].x = mat(i,0);
			wp.xyz_bxbybz[i].y = mat(i,1);
			wp.xyz_bxbybz[i].z = mat(i,2);
			wp.xyz_bxbybz[i].bx_i = mat(i,3);
			wp.xyz_bxbybz[i].bx_j = mat(i,4);
			wp.xyz_bxbybz[i].bx_k = mat(i,5);
			wp.xyz_bxbybz[i].by_i = mat(i,6);
			wp.xyz_bxbybz[i].by_j = mat(i,7);
			wp.xyz_bxbybz[i].by_k = mat(i,8);
			wp.xyz_bxbybz[i].bz_i = mat(i,9);
			wp.xyz_bxbybz[i].bz_j = mat(i,10);
			wp.xyz_bxbybz[i].bz_k = mat(i,11);
		}
		return wp; 
	}

	///////////////////////////////////////////////////////////

	inline Eigen::MatrixXd xyz_cba_to_eigen(gen_utilities::Waypoints& wp)
	{	
		Eigen::MatrixXd mat(wp.xyz_cba.size(),6);
		for (int i=0;i<mat.rows();++i)
		{
			mat(i,0) = wp.xyz_cba[i].x;
			mat(i,1) = wp.xyz_cba[i].y;
			mat(i,2) = wp.xyz_cba[i].z;
			mat(i,3) = wp.xyz_cba[i].c;
			mat(i,4) = wp.xyz_cba[i].b;
			mat(i,5) = wp.xyz_cba[i].a;
		}
	    return mat;
	}

	///////////////////////////////////////////////////////////

	inline gen_utilities::Waypoints eigen_to_xyz_cba(Eigen::MatrixXd mat)
	{	
		gen_utilities::Waypoints wp;
		wp.xyz_cba.resize(mat.rows());
		for (int i=0;i<mat.rows();++i)
		{
			wp.xyz_cba[i].x = mat(i,0);
			wp.xyz_cba[i].y = mat(i,1);
			wp.xyz_cba[i].z = mat(i,2);
			wp.xyz_cba[i].c = mat(i,3);
			wp.xyz_cba[i].b = mat(i,4);
			wp.xyz_cba[i].a = mat(i,5);
		}
		return wp; 
	}

	///////////////////////////////////////////////////////////

	inline Eigen::MatrixXd joint_angles_to_eigen(gen_utilities::KukaJointAnglesSet& j)
	{	
		Eigen::MatrixXd mat(j.joint_angles_set.size(),7);
		for (int i=0;i<mat.rows();++i)
		{
			mat(i,0) = j.joint_angles_set[i].j1;
			mat(i,1) = j.joint_angles_set[i].j2;
			mat(i,2) = j.joint_angles_set[i].j3;
			mat(i,3) = j.joint_angles_set[i].j4;
			mat(i,4) = j.joint_angles_set[i].j5;
			mat(i,5) = j.joint_angles_set[i].j6;
			mat(i,6) = j.joint_angles_set[i].j7;
		}
	    return mat;
	}

	///////////////////////////////////////////////////////////

	inline gen_utilities::KukaJointAnglesSet eigen_to_joint_angles(Eigen::MatrixXd mat)
	{	
		gen_utilities::KukaJointAnglesSet j;
		j.joint_angles_set.resize(mat.rows());
		for (int i=0;i<mat.rows();++i)
		{
			j.joint_angles_set[i].j1 = mat(i,0);
			j.joint_angles_set[i].j2 = mat(i,1);
			j.joint_angles_set[i].j3 = mat(i,2);
			j.joint_angles_set[i].j4 = mat(i,3);
			j.joint_angles_set[i].j5 = mat(i,4);
			j.joint_angles_set[i].j6 = mat(i,5);
			j.joint_angles_set[i].j7 = mat(i,6);
		}
	    return j;
	}

	///////////////////////////////////////////////////////////

	inline Eigen::MatrixXd group_idx_to_eigen(gen_utilities::KukaJointAnglesSet& j)
	{	
		Eigen::MatrixXd mat(j.group_idx.size(),2);
		for (int i=0;i<mat.rows();++i)
		{
			mat(i,0) = j.group_idx[i].s;
			mat(i,1) = j.group_idx[i].e;
		}
	    return mat;
	}

	///////////////////////////////////////////////////////////

	inline gen_utilities::KukaJointAnglesSet eigen_to_group_idx(Eigen::MatrixXd mat)
	{	
		gen_utilities::KukaJointAnglesSet j;
		j.group_idx.resize(mat.rows());
		for (int i=0;i<mat.rows();++i)
		{
			j.group_idx[i].s = mat(i,0);
			j.group_idx[i].e = mat(i,1);
		}
	    return j;
	}

	///////////////////////////////////////////////////////////

	inline sensor_msgs::PointCloud eigen_to_pointcloud(Eigen::MatrixXd mat)
	{
		sensor_msgs::PointCloud ptcloud;
		ptcloud.points.resize(mat.rows());
		for (long i=0;i<mat.rows();++i)
		{
			ptcloud.points[i].x = mat(i,0);
			ptcloud.points[i].y = mat(i,1);
			ptcloud.points[i].z = mat(i,2);
		}	
		return ptcloud;
	}

	///////////////////////////////////////////////////////////

	inline Eigen::MatrixXd pointcloud_to_eigen(sensor_msgs::PointCloud ptcloud)
	{
		Eigen::MatrixXd mat(ptcloud.points.size(),3);
		for (long i=0;i<mat.rows();++i)
		{
			mat(i,0) = ptcloud.points[i].x;
			mat(i,1) = ptcloud.points[i].y;
			mat(i,2) = ptcloud.points[i].z;
		}	
		return mat;
	}

}