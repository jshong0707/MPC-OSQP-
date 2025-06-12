#include "filter.hpp"

filter::filter(/* args */)
{
}

filter::~filter()
{
}

MatrixXd filter::pinv(const MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

double filter::tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * PI * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

double filter::lowpassfilter(double input, double input_old, double output_old, double cutoff_freq) 
{
    double time_const = 1 / (2 * PI * cutoff_freq);
    double output = 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);
    return output;
}

Vector3d filter::quat2euler(double qw, double qx, double qy, double qz)
{
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    euler[0] = PI / 2 + atan2(sinr_cosp, cosr_cosp);

    double sinp = sqrt(1 + 2 * (qw * qy - qx * qz));
    double cosp = sqrt(1 - 2 * (qw * qy - qx * qz));
    euler[1] = 2 * atan2(sinp, cosp) - PI / 2;

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    euler[2] = atan2(siny_cosp, cosy_cosp);
    // printf("alpha: %f \t beta: %f \t gamma: %f\n", euler[0], euler[1], euler[2]);
    
    return euler;

}

Vector3d filter::quat2rpy(Vector4d quat) {
  
  Vector3d rpy;

    rpy(0) = std::atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]),
                        1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    
    rpy(1) = std::asin(2.0 * (quat[0] * quat[2] - quat[3] * quat[1]));

    rpy(2) = std::atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                        1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    

    // cout << rpy << endl;
    return rpy;
}


Matrix3d filter::skew(Vector3d& v)
{
    Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1), v(0),   0;

    return S;
}