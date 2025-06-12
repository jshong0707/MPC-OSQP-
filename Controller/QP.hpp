// #ifndef QP_H_
// #define QP_H_

// #include "globals.hpp"
// #include "qpOASES.hpp"

// using namespace qpOASES;

// class filter;
// class Kinematics;

// class QP
// {
// private:
//     filter* F;

//     Kinematics K_FL;
//     Kinematics K_FR;
//     Kinematics K_RL;
//     Kinematics K_RR;
    
//     real_t xOpt[2];
//     double W = 0.25;
//     Vector3d y = Vector3d::Zero();
//     double x;
//     double M = 40; 
//     double g = 9.81;
//     double L = 0.25;

//     VectorXd P = VectorXd::Zero(6);
//     VectorXd D = VectorXd::Zero(6);

//     VectorXd grf;

//     /* CoM State*/
//         VectorXd error_pos = VectorXd::Zero(6);
//         VectorXd error_vel = VectorXd::Zero(6);
//         VectorXd posCoM = VectorXd::Zero(6);
//         VectorXd velCoM = VectorXd::Zero(6);
//         VectorXd velCoM_old = VectorXd::Zero(6);
//         VectorXd accCoM_des = VectorXd::Zero(6);

//         Vector3d body_rpy = Vector3d::Zero();
//         Vector4d body_quat = Vector4d::Zero();

//     /* Optimal GRF */
//         Vector3d opt_GRF[4];
        
//     /* Foot vector*/
//         Vector3d p_FL = Vector3d::Zero();
//         Vector3d p_FR = Vector3d::Zero();
//         Vector3d p_RL = Vector3d::Zero();
//         Vector3d p_RR = Vector3d::Zero();
        
//     // error_pos
//     double total_mass;
    
    



// public:
//     QP(Body &B, Kinematics &K_FL, Kinematics &K_FR, Kinematics &K_RL, Kinematics &K_RR);
//     ~QP();

//     void Cal_Centroid(const mjModel* m, mjData* d, double t);
//     void optCentroid2(const mjModel* m, mjData* d, bool* Contact_signal);

//     Vector3d get_opt_GRF(int Leg_num){return opt_GRF[Leg_num];}

// };


// #endif