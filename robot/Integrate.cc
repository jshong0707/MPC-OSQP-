#include "Integrate.hpp"


Integrate::Integrate(F_Kinematics &K_FL, F_Kinematics &K_FR, F_Kinematics &K_RL, F_Kinematics &K_RR,
Trajectory &Traj, Body &B, MPC &M, Controller &C, FSM &FSM_)
:K_FL(K_FL), K_FR(K_FR), K_RL(K_RL), K_RR(K_RR), Traj(Traj), B(B), M(M), C(C), FSM_(FSM_)
{
    leg_pos_ref = VectorXd::Zero(12);
    leg_pos = VectorXd::Zero(12);
    
    mpc_dt = M.get_dt();
    opt_u.resize(12); opt_u.setZero();
}

Integrate::~Integrate()
{
}

void Integrate::sensor_measure(const mjModel* m, mjData* d)
{
    t = d->time;

    K_FL.sensor_measure(m, d);
    K_FR.sensor_measure(m, d);
    K_RL.sensor_measure(m, d);
    K_RR.sensor_measure(m, d);

    B.sensor_measure(m, d);
    M.foot_vector(m, d);
    

}

void Integrate::Leg_controller()
{
    /* State Machine */
    is_contact = FSM_.contactschedule();

    /* FeedBack Controller */
    for(int leg = 0; leg < 4; leg++)
    {
        if(is_contact[leg] == false)
            FB_input[leg] = C.FB_controller(pos_err[leg], pos_err_old[leg], leg);
        else
            FB_input[leg] = Vector3d::Zero();
    }
        // for(int leg = 0; leg < 4; leg++)
        //     FB_input[leg] = C.FB_controller(pos_err[leg], pos_err_old[leg], leg);
        

    /* MPC */
    if(t >= optimization_t)
    {
        M.Dynamics();
        M.SolveQP();

        opt_u = M.get_opt_u();
        
        if(t < 0.00000001)
            opt_u = VectorXd::Zero(12);

        for(int i = 0; i < 4; i ++)
        {
            opt_u[3*i] = - opt_u[3*i];
            opt_u[3*i + 1] = - opt_u[3*i + 1];
            opt_GRF[i] = B.get_R().transpose() * opt_u.segment(3*i,3);
        }
        // cout << "///////////////////////////////////////////////////////////\n" << endl;
        // cout << is_contact << endl;
        // cout << opt_GRF[0][0] << "   " << opt_GRF[1][0] << "   " << opt_GRF[2][0] << "   " << opt_GRF[3][0] << endl; 
        // cout << opt_GRF[0][1] << "   " << opt_GRF[1][1] << "   " << opt_GRF[2][1] << "   " << opt_GRF[3][1] << endl; 
        // cout << opt_GRF[0][2] << "   " << opt_GRF[1][2] << "   " << opt_GRF[2][2] << "   " << opt_GRF[3][2] << endl; 
        
        optimization_t += mpc_dt;
    }

    /* Joint Input */

        F_Joint_input[0] = K_FL.get_Jacb().transpose() * (opt_GRF[0] + FB_input[0]);
        F_Joint_input[1] = K_FR.get_Jacb().transpose() * (opt_GRF[1] + FB_input[1]);
        B_Joint_input[0] = K_RL.get_Jacb().transpose() * (opt_GRF[2] + FB_input[2]);
        B_Joint_input[1] = K_RR.get_Jacb().transpose() * (opt_GRF[3] + FB_input[3]);


}

void Integrate::Cal_Kinematics()
{
    
    K_FL.Cal_Kinematics();
    K_FR.Cal_Kinematics();
    K_RL.Cal_Kinematics();
    K_RR.Cal_Kinematics();
    
}

void Integrate::get_error(double t)
{
    leg_pos_ref = Traj.Traj(t);
    
    // cout << leg_pos_ref << endl;
    pos_err[0] = K_FL.get_error(leg_pos_ref.segment(0,3));
    pos_err[1] = K_FR.get_error(leg_pos_ref.segment(3,3));
    pos_err[2] = K_RL.get_error(leg_pos_ref.segment(6,3));
    pos_err[3] = K_RR.get_error(leg_pos_ref.segment(9,3));
    
    pos_err_old[0] = K_FL.get_err_old();
    pos_err_old[1] = K_FR.get_err_old();
    pos_err_old[2] = K_RL.get_err_old();
    pos_err_old[3] = K_RR.get_err_old();

}

void Integrate::Data_log()
{
    leg_pos << K_FL.get_pos(), K_FR.get_pos(), K_RL.get_pos(), K_RR.get_pos();

}   
