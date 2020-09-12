#ifndef FILE_RW
#define FILE_RW

#include <string>
#include <vector>
#include <Eigen/Eigen>

namespace file_rw
{
    //! read data from file and store in vector
    std::vector< std::vector<double> > file_read_vec(std::string);

    //! read data from file and store in matrix
    Eigen::MatrixXd file_read_mat(std::string);

    //! write data to file from vector
    void file_write(std::string, const std::vector< std::vector<int> >&);
    void file_write(std::string, const std::vector< std::vector<float> >&);
    void file_write(std::string, const std::vector< std::vector<double> >&);
    
    //! write data to file from matrix
    void file_write(std::string, const Eigen::MatrixXd&);
    void file_write(std::string, const Eigen::MatrixXi&);
    void file_write(std::string, const Eigen::MatrixXf&);
};

#endif