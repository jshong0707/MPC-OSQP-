// #include "QP.hpp"

// QP::QP(Body &B, Kinematics &K_FL, Kinematics &K_FR, Kinematics &K_RL, Kinematics &K_RR)
// :B(B), K_FL(K_FL), K_FR(K_FR), K_RL(K_RL), K_RR(K_RR)
// {

//     grf.setZero(12);

//     P << 200, 200, 200, 2000, 1000, 200;
//     D << 1800, 1800, 1000, 1800, 1800, 1000;
//     // P << 100, 100, 100, 10, 10, 10;
//     // D << 1800, 1800, 100, 0, 150, 0;
//     // P << 10, 10, 1500, 1000, 2500, 100;
//     // D << 200, 200, 2000, 500, 500, 500;
//     // D << 500, 500, 1500, 300, 300, 500;
    
// }


// QP::~QP(){}

// void QP::Cal_Centroid(const mjModel* m, mjData* d, double t)
// {
//     B.Footstate(m, d);
//     velCoM_old = velCoM;

//     total_mass = B.get_m_total();
    
//     mj_fwdPosition(m, d);
//     mj_fwdVelocity(m, d);

//     // position of total CoM
//     for(int i = 0; i < 3; i++)
//     {
//         posCoM(i) = d->subtree_com[i];
//         velCoM(i) = d->sensordata[34 + i];
//     }

//     // cout << posCoM.segment(0,3) << endl;
//     body_quat << d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6];
//     body_rpy = F->quat2rpy(body_quat);

//     posCoM.segment(3,3) = body_rpy;
//     velCoM.segment(3,3) << d->sensordata[68], d->sensordata[69], d->sensordata[70];


//     // calculate errors  x,y,z, roll,pitch,yaw 방향
//         error_pos = B.get_pos_des(t) - posCoM;
//         error_vel = B.get_vel_des(t) - velCoM;
//         // cout << B.get_vel_des(t) << endl;
//         // cout << error_pos[0] << "   " << error_pos[1] << "   " << error_pos[2] << "   " << error_pos[3] << "   " << error_pos[4] << "   " << error_pos[5] << endl;
//         // cout << error_vel[0] << "   " << error_vel[1] << "   " << error_vel[2] << "   " << error_vel[3] << "   " << error_vel[4] << "   " << error_vel[5] << endl; 

         

// }
  
// void QP::optCentroid2(const mjModel* m, mjData* d, bool* Contact_signal)
// {

//     bool* s = Contact_signal;
//     // cout << s[0] << "  " << s[1] << "  " << s[2] << "  " << s[3] << "  " << endl; 
    
//     accCoM_des = P.cwiseProduct(error_pos) + D.cwiseProduct(error_vel); 

//     // cout << accCoM_des << endl;
//     // s[0] = 1;
//     // s[1] = 1;
//     // s[2] = 1;
//     // s[3] = 1;
    
//     const int nV = 12;   // number of states
//     const int nC = 16;   // number of constraints

//     MatrixXd S = MatrixXd::Identity(6,6) * 10;   // weight for error

//     S(0, 0) =  5.;  // x
//     S(1, 1) =  5.;  // y
//     S(2, 2) =  20.;  // z
//     S(3, 3) =  35.;  // roll
//     S(4, 4) =  20.;  // pitch
//     S(5, 5) =  5.;  // yaw
    
//     p_FL = B.getFLfoot_vector();
//     p_FR = B.getFRfoot_vector();
//     p_RL = B.getRLfoot_vector();
//     p_RR = B.getRRfoot_vector();
    
//     MatrixXd matA(6, 12);
//     matA << 1., 0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 0.,
//             0., 1., 0., 0., 1., 0., 0., 1., 0., 0., 1., 0.,
//             0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 0., 1.,
//             0., -p_FL[2], p_FL[1], 0., -p_FR[2], p_FR[1], 0., -p_RL[2], p_RL[1], 0., -p_RR[2], p_RR[1],
//             p_FL[2], 0., -p_FL[0], p_FR[2], 0., -p_FR[0], p_RL[2], 0., -p_RL[0], p_RR[2], 0., -p_RR[0],
//             -p_FL[1], p_FL[0], 0., -p_FR[1], p_FR[0], 0., -p_RL[1], p_RL[0], 0., -p_RR[1], p_RR[0], 0.;
            
//     VectorXd b(6);
//     b(0) = total_mass * accCoM_des(0);
//     b(1) = total_mass * accCoM_des(1);
//     b(2) = total_mass * (accCoM_des(2) + g);

//     Matrix3d Body_Inertia;

//         Body_Inertia <<   0.603324, 0.00197572, -0.0327175,
//                         0.00197572,    2.67741, -0.0001082,
//                         -0.0327175, -0.0001082,    2.79587;

//     b.segment(3,3) = 1.2 * accCoM_des.segment(3,3);
//     // b.segment(3,3) = Body_Inertia * accCoM_des.segment(3,3);
//     // cout << "Desired Force\n" << b << endl;
//     MatrixXd alpha = MatrixXd::Identity(12,12) * 1;   // weight for state
//     MatrixXd beta = MatrixXd::Identity(12,12) * 0.05;

//     MatrixXd matH = 2 * (matA.transpose() * S * matA + alpha + beta);
//     VectorXd matG = - 2 * b.transpose() * S * matA - 2 * grf.transpose()*beta;
    
//     // cout << grf.transpose()*matH*grf << endl;
//     // cout << matA.transpose() * S * matA << endl;
//     qpOASES::real_t H[nV*nV] = { 0 };
//     qpOASES::real_t g[nV] = { 0 };

//     for(int i = 0; i < 12; i++)
//     {
//         for(int j = 0; j < 12; j++) H[i*12 + j] = matH(i, j);
//         g[i] = matG(i);
//     }

//     // friction cone (inequality constraint)
//     // ubA <= AF <= lbA
//     double mu = 0.6;
//     qpOASES::real_t A[nC*nV] = { 0 };
//     A[0] = 1.0;  A[2] = mu;
//     A[12 + 0] = 1.0;  A[12 + 2] = -mu;
//     A[24 + 1] = 1.0;  A[24 + 2] = mu;
//     A[36 + 1] = 1.0;  A[36 + 2] = -mu;
//     A[48 + 3] = 1.0;  A[48 + 5] = mu;
//     A[60 + 3] = 1.0;  A[60 + 5] = -mu;
//     A[72 + 4] = 1.0;  A[72 + 5] = mu;
//     A[84 + 4] = 1.0;  A[84 + 5] = -mu;
//     A[96 + 6] = 1.0;  A[96 + 8] = mu;
//     A[108 + 6] = 1.0;  A[108 + 8] = -mu;
//     A[120 + 7] = 1.0;  A[120 + 8] = mu;
//     A[132 + 7] = 1.0;  A[132 + 8] = -mu;
//     A[144 + 9] = 1.0;  A[144 + 11] = mu;
//     A[156 + 9] = 1.0;  A[156 + 11] = -mu;
//     A[168 + 10] = 1.0; A[168 + 11] = mu;
//     A[180 + 10] = 1.0; A[180 + 11] = -mu;
//     qpOASES::real_t lbA[nC] = {0.0, -1e20, 0.0, -1e20, 0.0, -1e20, 0.0, -1e20, 0.0, -1e20, 0.0, -1e20, 0.0, -1e20, 0.0, -1e20};
//     qpOASES::real_t ubA[nC] = {1e20, 0.0, 1e20, 0.0, 1e20, 0.0, 1e20, 0.0, 1e20, 0.0, 1e20, 0.0, 1e20, 0.0, 1e20, 0.0};

//     // lb <= F <= ub
    
//     qpOASES::real_t lb[nV] = {-353.0*s[0], -353.0*s[0], -353.0*s[0], -353.0*s[1], -353.0*s[1], -353.0*s[1], -353.0*s[2], -353.0*s[2], -353.0*s[2], -353.0*s[3], -353.0*s[3], -353.0*s[3]}; // calculated by the limit of actuator and friction coeff
//     qpOASES::real_t ub[nV] = { 353.0*s[0],  353.0*s[0],  353.0*s[0],  353.0*s[1],  353.0*s[1],  353.0*s[1],  353.0*s[2],  353.0*s[2],  353.0*s[2],  353.0*s[3],  353.0*s[3],  353.0*s[3]}; // mu/(l*sqrt(1 + mu^2))*tau
        
    

//     qpOASES::QProblem qp(nV, nC);
//     qpOASES::Options options;
//     options.printLevel = qpOASES::PL_NONE;
//     qp.setOptions(options);
    
//     int nWSR = 30;
//     qp.init(H, g, A, lb, ub, lbA, ubA, nWSR);

//     qpOASES::real_t xOpt[nV];
//     qp.getPrimalSolution(xOpt);
//     for(int i = 0; i < nV; i++) grf(i) = xOpt[i];

//     // for(int i = 0; i < 6; i++)
//     // cout << i << "  AF: " << (matA * grf)[i] << "\t b: "  << b[i] << "\t AF - b: "  << (matA*grf - b)[i]<< endl;
//     // // // cout << "cost: " <<  (0.5*grf.transpose()*matH*grf + matG.transpose()*grf) << endl;

//     // cout << "F_FL: " << xOpt[0] << "\t" << xOpt[1] << "\t" << xOpt[2] << endl;
//     // cout << "F_FR: " << xOpt[3] << "\t" << xOpt[4] << "\t" << xOpt[5] << endl;
//     // cout << "F_RL: " << xOpt[6] << "\t" << xOpt[7] << "\t" << xOpt[8] << endl;
//     // cout << "F_RR: " << xOpt[9] << "\t" << xOpt[10] << "\t" << xOpt[11] << endl;

//     opt_GRF[0] = grf.segment(0, 3);  // 1, 2, 3
//     opt_GRF[1] = grf.segment(3, 3);  // 4, 5, 6
//     opt_GRF[2] = grf.segment(6, 3);  // 7, 8, 9
//     opt_GRF[3] = grf.segment(9, 3);  // 10, 11, 12
  

// }

