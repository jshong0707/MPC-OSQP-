#ifndef filter_H_
#define filter_H_

#include "globals.hpp"

#include <eigen-master/Eigen/Core>
#include <eigen-master/Eigen/Dense>

using namespace Eigen;

class filter
{
private:


    Vector3d euler = Vector3d::Zero();
public:
    filter(/* args */);
    ~filter();
    MatrixXd pinv(const MatrixXd &A);

    double tustin_derivative(double input, double input_old, double output_old, double cutoff_freq);
    double lowpassfilter(double input, double input_old, double output_old, double cutoff_freq);
    Vector3d quat2euler(double qw, double qx, double qy, double qz);
    Vector3d quat2rpy(Vector4d quat);
    Matrix3d skew(Vector3d& v);

};




#endif