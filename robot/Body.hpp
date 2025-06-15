#pragma once

#include "globals.hpp"

class filter;

class Body
{
private:
    Matrix3d I;
    double M;

    int N_Task;
    filter *F;
    VectorXd x0;
    Matrix3d R = Matrix3d::Identity(); // Rotation Matrix
    Vector3d RPY = Vector3d::Zero();
    Vector3d YPR = Vector3d::Zero();
    
    VectorXd x_ref;
    VectorXd z_ref; // mpc
    
    Vector3d omega_IMU = Vector3d::Zero();
    Matrix3d omega_IMU_hat; // = Matrix3d::Zero();
    Vector3d u = Vector3d::Zero();
    Matrix3d u_hat = Matrix3d::Zero();
    double theta;
    Vector3d CoM_pos_W = Vector3d::Zero();
    Vector3d r_W[4];

    Matrix3d exp_omega_dt;
    int horizon;
    int nx = 12; 
    double MPC_dt = 0.;
    double t = 0.;
    double dt = 0.001;


public:
    Body();
    ~Body();

    void sensor_measure(const mjModel* m, mjData* d);
    void foot_vector(const mjModel* m, mjData* d);
    VectorXd get_z_ref(double t);
    VectorXd get_x_ref(double t);

    void init_x_ref_mpc(int horizon, double sampling_time);
    VectorXd get_x0(){return x0;}
    Matrix3d get_R();
    int get_Task(){return N_Task;}
    Matrix3d get_Body_I(){return I;}
    double get_Body_M(){return M;}
    Vector3d get_r_W(int leg){return r_W[leg];}

};

