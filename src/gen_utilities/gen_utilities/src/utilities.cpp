#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <chrono>
#include "gen_utilities/file_rw.hpp"
#include "gen_utilities/transformation_utilities.hpp"
#include "gen_utilities/utilities.hpp"

namespace ut
{
    std::chrono::high_resolution_clock::time_point clock_start;
    std::chrono::high_resolution_clock::time_point clock_end;

    std::vector<std::vector<int> > get_unique_rows(std::vector<std::vector<int> > input)
    {
        // if want sorted
        // std::sort(input.begin(), input.end());
        input.erase(std::unique(input.begin(), input.end()), input.end());
        return input;   
    }

    ///////////////////////////////////////////////////////////

    std::vector<std::vector<float> > get_unique_rows(std::vector<std::vector<float> > input)
    {
        // if want sorted
        // std::sort(input.begin(), input.end());
        input.erase(std::unique(input.begin(), input.end()), input.end());
        return input;   
    }

    ///////////////////////////////////////////////////////////

    std::vector<std::vector<double> > get_unique_rows(std::vector<std::vector<double> > input)
    {
        // if want sorted
        // std::sort(input.begin(), input.end());
        input.erase(std::unique(input.begin(), input.end()), input.end());
        return input;   
    }

    ///////////////////////////////////////////////////////////

    Eigen::MatrixXi vec_to_mat(std::vector<std::vector<int> > vec)
    {
        Eigen::MatrixXi mat(vec.size(), vec[0].size());
        for (long int i = 0; i < vec.size(); ++i)
            mat.row(i) = Eigen::VectorXi::Map(&vec[i][0], vec[0].size());
        return mat;
    }

    ///////////////////////////////////////////////////////////

    Eigen::MatrixXf vec_to_mat(std::vector<std::vector<float> > vec)
    {
        Eigen::MatrixXf mat(vec.size(), vec[0].size());
        for (long int i = 0; i < vec.size(); ++i)
            mat.row(i) = Eigen::VectorXf::Map(&vec[i][0], vec[0].size());
        return mat;
    }

    ///////////////////////////////////////////////////////////

    Eigen::MatrixXd vec_to_mat(std::vector<std::vector<double> > vec)
    {
        Eigen::MatrixXd mat(vec.size(), vec[0].size());
        for (long int i = 0; i < vec.size(); ++i)
            mat.row(i) = Eigen::VectorXd::Map(&vec[i][0], vec[0].size());
        return mat;
    }

    ////////////////////////////////////////////////////////////

    std::vector<std::vector<int> > mat_to_vec(Eigen::MatrixXi mat)
    {
        std::vector<std::vector<int> > vec;
        for (long int i = 0; i < mat.rows(); ++i)
        {
            std::vector<int> vec_row;
            for (long int j = 0; j < mat.cols(); ++j)
            {
                vec_row.push_back(mat(i,j));
            }
            vec.push_back(vec_row);
        }
        return vec;
    }

    ////////////////////////////////////////////////////////////

    std::vector<std::vector<float> > mat_to_vec(Eigen::MatrixXf mat)
    {
        std::vector<std::vector<float> > vec;
        for (long int i = 0; i < mat.rows(); ++i)
        {
            std::vector<float> vec_row;
            for (long int j = 0; j < mat.cols(); ++j)
            {
                vec_row.push_back(mat(i,j));
            }
            vec.push_back(vec_row);
        }
        return vec;
    }

    ////////////////////////////////////////////////////////////

    std::vector<std::vector<double> > mat_to_vec(Eigen::MatrixXd mat)
    {
        std::vector<std::vector<double> > vec;
        for (long int i = 0; i < mat.rows(); ++i)
        {
            std::vector<double> vec_row;
            for (long int j = 0; j < mat.cols(); ++j)
            {
                vec_row.push_back(mat(i,j));
            }
            vec.push_back(vec_row);
        }
        return vec;
    }

    ////////////////////////////////////////////////////////////

    void disp_vec(std::vector<std::vector<int> > vec)
    {
        for (long int i=0;i<vec.size();++i)
        {
            for (long int j=0;j<vec[0].size();++j)
            {
                if (j!=vec[0].size()-1)
                {
                    std::cout << vec[i][j] << ",";    
                }
                else
                {
                    std::cout << vec[i][j] << std::endl;
                }
                
            }
        }
    }

    ////////////////////////////////////////////////////////////

    void disp_vec(std::vector<std::vector<float> > vec)
    {
        for (long int i=0;i<vec.size();++i)
        {
            for (long int j=0;j<vec[0].size();++j)
            {
                if (j!=vec[0].size()-1)
                {
                    std::cout << vec[i][j] << ",";    
                }
                else
                {
                    std::cout << vec[i][j] << std::endl;
                }
                
            }
        }
    }

    ////////////////////////////////////////////////////////////

    void disp_vec(std::vector<std::vector<double> > vec)
    {
        for (long int i=0;i<vec.size();++i)
        {
            for (long int j=0;j<vec[0].size();++j)
            {
                if (j!=vec[0].size()-1)
                {
                    std::cout << vec[i][j] << ",";    
                }
                else
                {
                    std::cout << vec[i][j] << std::endl;
                }
                
            }
        }
    }

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd compute_TCP(Eigen::MatrixXd data_points, Eigen::MatrixXd normals)
    {
        // tcp computation...bx,by,bz
        Eigen::Vector3d tool_x;
        Eigen::Vector3d tool_y;
        Eigen::Vector3d tool_z;
        Eigen::Vector3d dir_vec; 
        Eigen::Vector3d direction;
        Eigen::MatrixXd bxbybz = Eigen::MatrixXd::Constant(data_points.rows(),9,0);
        for (unsigned int i=0; i<data_points.rows(); ++i)
        {
            if (i!=data_points.rows()-1)
            {
            // calculating direction vector from sequence of points
                // direction = (data_points.row(i+1).array() - data_points.row(i).array()).transpose(); 
            // OR
            // applying constant direction vector
                direction << 0, 1, 0;
                dir_vec = direction.array()/direction.norm();    
            }
            tool_z << -normals.row(i).transpose();
            tool_x = dir_vec.cross(tool_z);
            tool_x = tool_x.array()/tool_x.norm();
            tool_y = tool_z.cross(tool_x);
            tool_y = tool_y.array()/tool_y.norm();
            bxbybz.block(i,0,1,3) = tool_x.transpose();
            bxbybz.block(i,3,1,3) = tool_y.transpose();
            bxbybz.block(i,6,1,3) = tool_z.transpose();
        }
        return bxbybz;
    }

    ////////////////////////////////////////////////////////////

    double get_pt_to_lsf_plane_dist(Eigen::MatrixXd pt, Eigen::MatrixXd pts_for_plane)
    {
        Eigen::MatrixXd x_vec(pts_for_plane.rows(),1);
        Eigen::MatrixXd y_vec(pts_for_plane.rows(),1);
        Eigen::MatrixXd z_vec(pts_for_plane.rows(),1);
        double x_avg;   
        double y_avg;   
        double z_avg;
        double L00; 
        double L11; 
        double L01; 
        double R0;  
        double R1;
        double A;   
        double B;  
        double C;   
        double D;
        x_vec = pts_for_plane.block(0,0,pts_for_plane.rows(),1);
        y_vec = pts_for_plane.block(0,1,pts_for_plane.rows(),1);
        z_vec = pts_for_plane.block(0,2,pts_for_plane.rows(),1); 
        x_avg = x_vec.sum()/pts_for_plane.rows();
        y_avg = y_vec.sum()/pts_for_plane.rows();
        z_avg = z_vec.sum()/pts_for_plane.rows();
        L00 = ((x_vec.array() - x_avg).array().pow(2)).sum();
        L01 = ((x_vec.array() - x_avg).array()*(y_vec.array() - y_avg).array()).sum(); 
        L11 = ((y_vec.array() - y_avg).array().pow(2)).sum();
        R0 = ((z_vec.array() - z_avg).array()*(x_vec.array() - x_avg).array()).sum(); 
        R1 = ((z_vec.array() - z_avg).array()*(y_vec.array() - y_avg).array()).sum(); 
        A = -((L11*R0-L01*R1)/(L00*L11-L01*L01));
        B = -((L00*R1-L01*R0)/(L00*L11-L01*L01));
        C = 1;
        D = -(z_avg+A*x_avg+B*y_avg);
        return fabs(A*pt(0,0)+B*pt(0,1)+C*pt(0,2)+D)/(sqrt(A*A+B*B+C*C));
    }

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd get_traj_wrt_tcp(Eigen::Matrix4d tool_F_T_tcp, std::vector<std::vector<double> > vec)
    {
        std::vector<std::vector<double> > traj_from_kuka_scanning_vec;
        traj_from_kuka_scanning_vec = get_unique_rows(vec);
        Eigen::MatrixXd pts_from_tcp_publisher(traj_from_kuka_scanning_vec.size(),traj_from_kuka_scanning_vec[0].size());
        pts_from_tcp_publisher = vec_to_mat(traj_from_kuka_scanning_vec);
        Eigen::Vector3d b_t_Flange;
        Eigen::MatrixXd b_eul_Flange(1,3);
        Eigen::Matrix3d b_r_Flange;
        Eigen::Matrix4d b_T_Flange;
        Eigen::Matrix4d b_T_tcp;    
        
        // angles must be radians
        Eigen::MatrixXd transformed_pt = Eigen::MatrixXd::Constant(pts_from_tcp_publisher.rows(),pts_from_tcp_publisher.cols(),0);
        for (int i=0;i<pts_from_tcp_publisher.rows();++i)
        {
            b_t_Flange << pts_from_tcp_publisher(i,0), pts_from_tcp_publisher(i,1), pts_from_tcp_publisher(i,2);
            b_eul_Flange << pts_from_tcp_publisher(i,3), pts_from_tcp_publisher(i,4), pts_from_tcp_publisher(i,5);
            b_r_Flange = rtf::eul2rot(b_eul_Flange);
            b_T_Flange = rtf::hom_T(b_t_Flange,b_r_Flange);
            b_T_tcp = b_T_Flange*tool_F_T_tcp;
            transformed_pt(i,0) = b_T_tcp(0,3);
            transformed_pt(i,1) = b_T_tcp(1,3);
            transformed_pt(i,2) = b_T_tcp(2,3);
        }

        Eigen::MatrixXd scan_traj_wrt_tcp(transformed_pt.rows(),transformed_pt.cols());
        // saves only xyz and not euler angles
        scan_traj_wrt_tcp = transformed_pt.block(0,0,transformed_pt.rows(),3);
        return scan_traj_wrt_tcp;
    }

    ////////////////////////////////////////////////////////////

    std::vector<std::vector<int> > SortRows(std::vector<std::vector<int> > input, int col)
    {
        for (long int i=0;i<input.size();++i)
        {
            int temp = input[i][0];
            input[i][0] = input[i][col];
            input[i][col] = temp; 
        }
        std::sort(input.begin(), input.end());
        for (long int i=0;i<input.size();++i)
        {
            int temp = input[i][0];
            input[i][0] = input[i][col];
            input[i][col] = temp; 
        }
        return input;   
    }

    ////////////////////////////////////////////////////////////

    std::vector<std::vector<float> > SortRows(std::vector<std::vector<float> > input, int col)
    {
        for (long int i=0;i<input.size();++i)
        {
            float temp = input[i][0];
            input[i][0] = input[i][col];
            input[i][col] = temp; 
        }
        std::sort(input.begin(), input.end());
        for (long int i=0;i<input.size();++i)
        {
            float temp = input[i][0];
            input[i][0] = input[i][col];
            input[i][col] = temp; 
        }
        return input;   
    }

    ////////////////////////////////////////////////////////////

    std::vector<std::vector<double> > SortRows(std::vector<std::vector<double> > input, int col)
    {
        for (long int i=0;i<input.size();++i)
        {
            double temp = input[i][0];
            input[i][0] = input[i][col];
            input[i][col] = temp; 
        }
        std::sort(input.begin(), input.end());
        for (long int i=0;i<input.size();++i)
        {
            double temp = input[i][0];
            input[i][0] = input[i][col];
            input[i][col] = temp; 
        }
        return input;   
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> ismember(std::vector<std::vector<int> > vec, std::vector<int> row_vec)
    {
        std::vector<int> v;
        for (long int i=0;i<vec.size();++i)
        {
            if ((vec[i][0]==row_vec[0]) && (vec[i][1]==row_vec[1]) && (vec[i][2]==row_vec[2]))
            {
                v.push_back(1);
            }
            else
            {
                v.push_back(0);
            }
        }
        return v;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> ismember(std::vector<std::vector<float> > vec, std::vector<float> row_vec)
    {
        std::vector<int> v;
        for (long int i=0;i<vec.size();++i)
        {
            if ((vec[i][0]==row_vec[0]) && (vec[i][1]==row_vec[1]) && (vec[i][2]==row_vec[2]))
            {
                v.push_back(1);
            }
            else
            {
                v.push_back(0);
            }
        }
        return v;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> ismember(std::vector<std::vector<double> > vec, std::vector<double> row_vec)
    {
        std::vector<int> v;
        for (long int i=0;i<vec.size();++i)
        {
            if ((vec[i][0]==row_vec[0]) && (vec[i][1]==row_vec[1]) && (vec[i][2]==row_vec[2]))
            {
                v.push_back(1);
            }
            else
            {
                v.push_back(0);
            }
        }
        return v;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> ismember(Eigen::MatrixXi mat, Eigen::MatrixXi row_vec)
    {
        std::vector<int> v;
        for (long int i=0;i<mat.rows();++i)
        {
            if (mat.row(i)==row_vec)
            {
                v.push_back(1);
            }
            else
            {
                v.push_back(0);
            }
        }
        return v;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> ismember(Eigen::MatrixXf mat, Eigen::MatrixXf row_vec)
    {
        std::vector<int> v;
        for (long int i=0;i<mat.rows();++i)
        {
            if (mat.row(i)==row_vec)
            {
                v.push_back(1);
            }
            else
            {
                v.push_back(0);
            }
        }
        return v;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> ismember(Eigen::MatrixXd mat, Eigen::MatrixXd row_vec)
    {
        std::vector<int> v;
        for (long int i=0;i<mat.rows();++i)
        {
            if (mat.row(i)==row_vec)
            {
                v.push_back(1);
            }
            else
            {
                v.push_back(0);
            }
        }
        return v;
    }

    ///////////////////////////////////////////////////////////

    Eigen::MatrixXd mean(Eigen::MatrixXi mat)
    {
        Eigen::VectorXd vec(mat.cols());
        for (int i=0;i<mat.cols();++i)
        {
            vec(i) =  mat.block(0,i,mat.rows(),1).sum()/mat.rows();
        }
        return vec.transpose();
    }

    ///////////////////////////////////////////////////////////

    Eigen::MatrixXd mean(Eigen::MatrixXf mat)
    {
        Eigen::VectorXd vec(mat.cols());
        for (int i=0;i<mat.cols();++i)
        {
            vec(i) =  mat.block(0,i,mat.rows(),1).sum()/mat.rows();
        }
        return vec.transpose();
    }

    ///////////////////////////////////////////////////////////

    Eigen::MatrixXd mean(Eigen::MatrixXd mat)
    {
        Eigen::VectorXd vec(mat.cols());
        for (int i=0;i<mat.cols();++i)
        {
            vec(i) =  mat.block(0,i,mat.rows(),1).sum()/mat.rows();
        }
        return vec.transpose();
    }

    ////////////////////////////////////////////////////////////

    double median(std::vector<int> vec)
    {
        std::sort(vec.begin(),vec.end());
        if (vec.size()%2==1)    // odd
        {
            return vec[vec.size()/2];
        }
        else                    // even
        {
            return (vec[(vec.size()/2)-1]+vec[vec.size()/2])/2; 
        }
    }

    ////////////////////////////////////////////////////////////

    double median(std::vector<float> vec)
    {
        std::sort(vec.begin(),vec.end());
        if (vec.size()%2==1)    // odd
        {
            return vec[vec.size()/2];
        }
        else                    // even
        {
            return (vec[(vec.size()/2)-1]+vec[vec.size()/2])/2; 
        }
    }

    ////////////////////////////////////////////////////////////

    double median(std::vector<double> vec)
    {
        std::sort(vec.begin(),vec.end());
        if (vec.size()%2==1)    // odd
        {
            return vec[vec.size()/2];
        }
        else                    // even
        {
            return (vec[(vec.size()/2)-1]+vec[vec.size()/2])/2; 
        }
    }

    ////////////////////////////////////////////////////////////

    Eigen::VectorXd linsp(double strt, double end, double stp)
    {
        int sz;
        if (strt<=end && stp>0 || strt>=end && stp<0)
        {
            sz = int((end-strt)/stp)+1;
        }
        else
        {
            if (strt>=end)
            {
                std::cerr << "start value is greater than the end value for incement!" << std::endl;
                std::terminate();   
            }
            else
            {
                std::cerr << "start value is less than the end value for decrement!" << std::endl;
                std::terminate();   
            }
        }
        return Eigen::VectorXd::LinSpaced(sz,strt,strt+stp*(sz-1));
    }

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd InPoly(Eigen::MatrixXd q, Eigen::MatrixXd p)
    {
        double l1[2][2];
        double l2[2][2];

        Eigen::MatrixXd in = Eigen::VectorXd::Constant(q.rows(),1,0);

        double xmin = p.col(0).minCoeff();
        double xmax = p.col(0).maxCoeff();
        double ymin = p.col(1).minCoeff();
        double ymax = p.col(1).maxCoeff();

        for (long i=0;i<q.rows();++i)
        {
            // bounding box test
            if (q(i,0)<xmin || q(i,0)>xmax || q(i,1)<ymin || q(i,1)>ymax)
            {
                continue;
            }
            int intersection_count = 0;
            Eigen::MatrixXd cont_lines = Eigen::MatrixXd::Constant(p.rows(),1,0);
            for (int j=0;j<p.rows();++j)
            {
                if (j==0)
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(p.rows()-1,0);l2[0][1] = p(p.rows()-1,1);
                    l2[1][0] = p(j,0);l2[1][1] = p(j,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                    }   
                }
                else
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(j,0);l2[0][1] = p(j,1);
                    l2[1][0] = p(j-1,0);l2[1][1] = p(j-1,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                        if (cont_lines(j-1,0)==1)
                        {
                            if (p(j-1,1)==q(i,1))
                            {
                                if (j-1==0)
                                {
                                    if (!((p(p.rows()-1,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(p.rows()-1,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                                else
                                {
                                    if (!((p(j-2,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(j-2,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (intersection_count%2==1)
            {
                in(i,0) = 1;
            }
        }
        return in;
    }

    ////////////////////////////////////////////////////////////

    void InPoly(const Eigen::MatrixXd& q, const Eigen::MatrixXd& p, Eigen::MatrixXi& in)
    {
        double l1[2][2];
        double l2[2][2];

        double xmin = p.col(0).minCoeff();
        double xmax = p.col(0).maxCoeff();
        double ymin = p.col(1).minCoeff();
        double ymax = p.col(1).maxCoeff();

        for (long i=0;i<q.rows();++i)
        {
            // bounding box test
            if (q(i,0)<xmin || q(i,0)>xmax || q(i,1)<ymin || q(i,1)>ymax)
            {
                continue;
            }
            int intersection_count = 0;
            Eigen::MatrixXd cont_lines = Eigen::MatrixXd::Constant(p.rows(),1,0);
            for (int j=0;j<p.rows();++j)
            {
                if (j==0)
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(p.rows()-1,0);l2[0][1] = p(p.rows()-1,1);
                    l2[1][0] = p(j,0);l2[1][1] = p(j,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                    }   
                }
                else
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(j,0);l2[0][1] = p(j,1);
                    l2[1][0] = p(j-1,0);l2[1][1] = p(j-1,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                        if (cont_lines(j-1,0)==1)
                        {
                            if (p(j-1,1)==q(i,1))
                            {
                                if (j-1==0)
                                {
                                    if (!((p(p.rows()-1,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(p.rows()-1,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                                else
                                {
                                    if (!((p(j-2,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(j-2,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (intersection_count%2==1)
            {
                in(i,0) = 1;
            }
            else
            {
                in(i,0) = 0;
            }
        }
    }

    ////////////////////////////////////////////////////////////
    
    void InPoly(const Eigen::MatrixXd& q, const Eigen::MatrixXd& p, Eigen::MatrixXd& in)
    {
        double l1[2][2];
        double l2[2][2];

        double xmin = p.col(0).minCoeff();
        double xmax = p.col(0).maxCoeff();
        double ymin = p.col(1).minCoeff();
        double ymax = p.col(1).maxCoeff();

        for (long i=0;i<q.rows();++i)
        {
            // bounding box test
            if (q(i,0)<xmin || q(i,0)>xmax || q(i,1)<ymin || q(i,1)>ymax)
            {
                continue;
            }
            int intersection_count = 0;
            Eigen::MatrixXd cont_lines = Eigen::MatrixXd::Constant(p.rows(),1,0);
            for (int j=0;j<p.rows();++j)
            {
                if (j==0)
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(p.rows()-1,0);l2[0][1] = p(p.rows()-1,1);
                    l2[1][0] = p(j,0);l2[1][1] = p(j,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                    }   
                }
                else
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(j,0);l2[0][1] = p(j,1);
                    l2[1][0] = p(j-1,0);l2[1][1] = p(j-1,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                        if (cont_lines(j-1,0)==1)
                        {
                            if (p(j-1,1)==q(i,1))
                            {
                                if (j-1==0)
                                {
                                    if (!((p(p.rows()-1,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(p.rows()-1,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                                else
                                {
                                    if (!((p(j-2,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(j-2,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (intersection_count%2==1)
            {
                in(i,0) = 1;
            }
            else
            {
                in(i,0) = 0;
            }
        }
    }

    ////////////////////////////////////////////////////////////

    bool lines_intersect(double l1[2][2], double l2[2][2])
    {
        // l1 for horizontal ray line...slope is always zero

        // checking if other slope is zero
        if (l2[0][1]==l2[1][1])
        {
            return false;
        }
        else
        {
            // checking both pts of second line above first line
            if ((l2[0][1]>l1[0][1] && l2[1][1]>l1[0][1]) || (l2[0][1]<l1[0][1] && l2[1][1]<l1[0][1]))
            {
                return false;
            }
            else
            {
                // checking both pts of second line either on right or on left of fist line
                if ((l2[0][0]<l1[0][0] && l2[1][0]<l1[0][0]) || (l2[0][0]>l1[1][0] && l2[1][0]>l1[1][0]))
                {
                    return false;
                }
                else
                {
                    // checking if other line is vertical
                    if (l2[0][0]== l2[1][0])
                    {
                        return true;
                    }
                    else
                    {
                        // getting intersection point
                        double m2 = (l2[1][1]-l2[0][1])/(l2[1][0]-l2[0][0]);        
                        double x = (l1[0][1]+m2*l2[0][0]-l2[0][1])/m2;
                        // checking if intersection point lies on the first line
                        if ((x>l1[0][0] || std::abs(x-l1[0][0])<1e-9) && (x<l1[1][0] || std::abs(x-l1[1][0])<1e-9))
                        {
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                    }
                }
            }
        } 
        return false;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(Eigen::VectorXi vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.rows();++i)
        {
            if (vec(i,0)!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(Eigen::VectorXf vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.rows();++i)
        {
            if (vec(i,0)!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(Eigen::VectorXd vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.rows();++i)
        {
            if (vec(i,0)!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(Eigen::MatrixXd vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.rows();++i)
        {
            if (vec(i,0)!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(Eigen::MatrixXi vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.rows();++i)
        {
            if (vec(i,0)!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(std::vector<int> vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.size();++i)
        {
            if (vec[i]!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(std::vector<float> vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.size();++i)
        {
            if (vec[i]!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    std::vector<int> find_idx(std::vector<double> vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.size();++i)
        {
            if (vec[i]!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    ////////////////////////////////////////////////////////////

    double vec_norm(Eigen::MatrixXd vec)
    {
        if (vec.rows()==0)
        {
            std::cout << "Empty vector" << std::endl;
            return 0;
        }
        else
        {
            if (vec.cols()==1)
            {
                double sum = 0;
                for (int i=0;i<vec.rows();++i)
                {
                    sum += vec(i,0)*vec(i,0);
                }
                return std::sqrt(sum);
            }
            else if (vec.rows()==1)
            {
                double sum = 0;
                for (int i=0;i<vec.cols();++i)
                {
                    sum += vec(0,i)*vec(0,i);
                }
                return std::sqrt(sum);    
            }
            else
            {
                std::cout << "input should be either row matrix or column matrix" << std::endl;
                return 0;
            }
        }
    }

    ////////////////////////////////////////////////////////////

    double vec_norm(std::vector<double> vec)
    {
        if (vec.size()==0)
        {
            std::cout << "Empty vector" << std::endl;
            return 0;
        }
        else
        {
            double sum = 0;
            for (int i=0;i<vec.size();++i)
            {
                sum += vec[i]*vec[i];
            }
            return std::sqrt(sum);
        }
    }

    ////////////////////////////////////////////////////////////

    double vec_norm(std::vector<int> vec)
    {
        if (vec.size()==0)
        {
            std::cout << "Empty vector" << std::endl;
            return 0;
        }
        else
        {
            double sum = 0;
            for (int i=0;i<vec.size();++i)
            {
                sum += vec[i]*vec[i];
            }
            return std::sqrt(sum);
        }
    }

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd v, Eigen::MatrixXd f, Eigen::MatrixXd n, double gap_x, double gap_y)
    {
        long max_pts_size = 100000000;
        Eigen::MatrixXd ptcloud_w_normals(max_pts_size,6);
        // Eigen::MatrixXd ptcloud_normals;
        long idx_start = 0;
        double alpha, beta;
        Eigen::MatrixXd tri(3,3);
        Eigen::MatrixXd tri2(3,3);
        Eigen::MatrixXd tri3(3,3);
        Eigen::MatrixXd tri4(3,3);
        Eigen::MatrixXd nn(1,3);
        Eigen::MatrixXd nn2(1,3);
        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4); 
        Eigen::MatrixXd T2 = Eigen::MatrixXd::Identity(4,4); 
        Eigen::MatrixXd T2_inv(4,4); 
        Eigen::MatrixXd T_inv(4,4);    

        for (long idx=0;idx<f.rows();++idx)
        {
            tri.row(0) << v.row(f(idx,0)-1);
            tri.row(1) << v.row(f(idx,1)-1);
            tri.row(2) << v.row(f(idx,2)-1);
            nn = n.row(idx);
            tri2.row(0) << 0,0,0;
            tri2.row(1) << tri.row(1).array() - tri.row(0).array();
            tri2.row(2) << tri.row(2).array() - tri.row(0).array();
            if (nn(0,1)==0 && nn(0,2)==0)
            {
                alpha = 0;
            }
            else
            {
                alpha = atan(nn(0,1)/nn(0,2));
            }

            T.block(0,0,3,3) = rtf::rot_x(alpha);
            nn2 = rtf::apply_transformation(nn,T);
            tri3 = rtf::apply_transformation(tri2,T);
            if (nn2(0,0)==0 && nn2(0,2)==0)
            {
                beta = 0;
            }
            else
            {
                beta = -atan(nn2(0,0)/nn2(0,2));
            }
            T2.block(0,0,3,3) = rtf::rot_y(beta); 
            tri4 = rtf::apply_transformation(tri3,T2);
            Eigen::MatrixXd grid_pts = generate_grid_points(gap_x, gap_y, tri4.col(0).minCoeff(), tri4.col(1).minCoeff(), tri4.col(0).maxCoeff(), tri4.col(1).maxCoeff());
            Eigen::MatrixXd pts = add_pts(tri4, grid_pts);
            if (pts.rows()!=0)
            {
            	T2_inv = T2.inverse();
            	T_inv = T.inverse();
                Eigen::MatrixXd pts2 = rtf::apply_transformation(pts,T2_inv);
                Eigen::MatrixXd pts3 = rtf::apply_transformation(pts2,T_inv);
                pts3.col(0) = pts3.col(0).array() + tri(0,0);
                pts3.col(1) = pts3.col(1).array() + tri(0,1);
                pts3.col(2) = pts3.col(2).array() + tri(0,2);

                if (idx_start+pts3.rows() < max_pts_size)
                {
                    ptcloud_w_normals.block(idx_start,0,pts3.rows(),3) = pts3;
                    Eigen::MatrixXd nn_mat = Eigen::MatrixXd::Constant(pts3.rows(),3,1);
                    nn_mat.col(0) = nn_mat.col(0).array()*nn(0,0);
                    nn_mat.col(1) = nn_mat.col(1).array()*nn(0,1);
                    nn_mat.col(2) = nn_mat.col(2).array()*nn(0,2);
                    ptcloud_w_normals.block(idx_start,3,pts3.rows(),3) = nn_mat;
                    idx_start = idx_start + pts3.rows();
                }
                else
                {
                    std::cout << "ERROR : Pointcloud size limit exceeded!" << std::endl;
                    return ptcloud_w_normals.block(0,0,--idx_start,6);    
                }
            }
        }
        return ptcloud_w_normals.block(0,0,idx_start,6);
    }

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd add_pts(Eigen::MatrixXd tri, Eigen::MatrixXd grid_pts)
    {
        Eigen::MatrixXd in = InPoly(grid_pts, tri);
        std::vector<int> loc = find_idx(in);
        Eigen::MatrixXd pts = Eigen::MatrixXd::Constant(loc.size(),3,0);
        for (int i=0;i<loc.size();++i)
        {
            pts(i,0) = grid_pts(loc[i],0);
            pts(i,1) = grid_pts(loc[i],1);
        }
        return pts;
    }

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd generate_grid_points(double pathgap_x, double pathgap_y, double xmin, double ymin, double xmax, double ymax)
    {
        // Function to generate the uniform mesh grid of points along the x-y plane
        // INPUT = gap between the adjacent points and maximum value in x and y direction
        // OUTPUT = All points consisting the uniform grid
        Eigen::VectorXd j = linsp(floor(ymin),ceil(ymax),pathgap_y);
        Eigen::VectorXd i_val = linsp(floor(xmin),ceil(xmax),pathgap_x);
        Eigen::MatrixXd pts = Eigen::MatrixXd::Constant(j.rows()*i_val.rows(),2,0);
        long int st_pt = 0;
        for (long int i=0;i<i_val.rows();++i)
        {
            pts.block(st_pt,0,j.rows(),1) = i_val(i)*Eigen::MatrixXd::Constant(j.rows(),1,1);
            pts.block(st_pt,1,j.rows(),1) = j.block(0,0,j.rows(),j.cols());
            st_pt = st_pt + j.rows();
        }
        return pts; 
    }

    ////////////////////////////////////////////////////////////

    void timer_start()
    {
        clock_start = std::chrono::high_resolution_clock::now();
    }

    ////////////////////////////////////////////////////////////

    void timer_end(std::string time_unit)
    {
        clock_end = std::chrono::high_resolution_clock::now();
        if (time_unit.compare("nanosec")==0)
        {
            std::cout << "Time elapsed is : " << std::chrono::duration_cast<std::chrono::nanoseconds>(clock_end - clock_start).count() << " nanoseconds.\n";        
        }
        if (time_unit.compare("microsec")==0)
        {
            std::cout << "Time elapsed is : " << std::chrono::duration_cast<std::chrono::microseconds>(clock_end - clock_start).count() << " microseconds.\n";        
        }
        if (time_unit.compare("millisec")==0)
        {
            std::cout << "Time elapsed is : " << std::chrono::duration_cast<std::chrono::milliseconds>(clock_end - clock_start).count() << " milliseconds.\n";        
        }
        if (time_unit.compare("sec")==0)
        {
            std::cout << "Time elapsed is : " << std::chrono::duration_cast<std::chrono::seconds>(clock_end - clock_start).count() << " seconds.\n";        
        }
    }

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd intersect_lines(Eigen::MatrixXd p1, Eigen::MatrixXd p2, Eigen::MatrixXd p3, Eigen::MatrixXd p4)
    {
        double L1_x1 = p1(0,0);
        double L1_y1 = p1(0,1);
        double L1_x2 = p2(0,0);
        double L1_y2 = p2(0,1);
        double L2_x1 = p3(0,0);
        double L2_y1 = p3(0,1);
        double L2_x2 = p4(0,0);
        double L2_y2 = p4(0,1);
        double Dx12 = L1_x1 - L1_x2;
        double Dx34 = L2_x1 - L2_x2;
        double Dy12 = L1_y1 - L1_y2;
        double Dy34 = L2_y1 - L2_y2;
        double Dx24 = L1_x2 - L2_x2;
        double Dy24 = L1_y2 - L2_y2;
        Eigen::MatrixXd A(2,2);
        A(0,0) = Dx12;
        A(0,1) = -Dx34;
        A(1,0) = Dy12;
        A(1,1) = -Dy34;
        Eigen::MatrixXd B(2,1);
        B(0,0) = -Dx24;
        B(1,0) = -Dy24;
        Eigen::MatrixXd ts = A.inverse()*B;
        Eigen::MatrixXd pt(1,2);
        pt(0,0) = ts(0,0)*L1_x1 + ((1-ts(0,0))*L1_x2);
        pt(0,1) = ts(0,0)*L1_y1 + ((1-ts(0,0))*L1_y2);
        return pt;
    };

}