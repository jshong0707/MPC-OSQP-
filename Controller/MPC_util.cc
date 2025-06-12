#include "MPC_util.hpp"

MPC_util::MPC_util(int Nx, int Nu, int Horizon)
{
    nx = Nx;
    nu = Nu;
    horizon = Horizon;
    
}

MPC_util::~MPC_util()
{
}


void MPC_util::x_fill_horizon_block(MatrixXd& M, MatrixXd& m)
{
        MatrixXd m_horizon;
        m_horizon.resize(horizon * m.rows(), nx);
        m_horizon.setZero();

        // int M_rows_old = M.rows() - nx * horizon;
        
        for(int k = 0; k < horizon; k++)
            m_horizon.block(k*nx, 0, m.rows(), m.cols()) = m;
        
        M.block(M_rows_old, 0, m_horizon.rows(), m_horizon.cols()) = m_horizon;      

}

void MPC_util::u_fill_horizon_block(MatrixXd& M, MatrixXd& m)
{
        MatrixXd m_horizon;
        m_horizon.resize(horizon * m.rows(), nu);
        m_horizon.setZero();
        
        for(int k = 0; k < horizon; k++)
        {
            m_horizon.block(k * m.rows(), 0, m.rows(), m.cols()) = m;
        }
        
        M.block(M_rows_old, nx * horizon, m_horizon.rows(), m_horizon.cols()) = m_horizon;

}

void MPC_util:: fill_horizon_vector(VectorXd& V, VectorXd& v)
{
        VectorXd v_horizon;
        v_horizon.resize(horizon * v.rows());
        v_horizon.setZero();

        for(int k = 0; k < horizon; k++)
            v_horizon.block(k * v.rows(), 0, v.rows(), v.cols()) = v;
        
        V.block(V_rows_old, 0, v_horizon.rows(), v_horizon.cols()) = v_horizon;

}

void MPC_util::add_block(MatrixXd& M, int extraRows)
{
    int rows = M.rows();
    int cols = M.cols();
    
    M_rows_old = M.rows();

    M.conservativeResize(rows + extraRows, cols);
    M.block(rows, 0, extraRows, cols).setZero();
    

}

void MPC_util::add_rows(VectorXd& V, int extraRows)
{
    int rows = V.rows();

    V_rows_old = V.rows();

    V.conservativeResize(rows + extraRows);
    V.segment(rows, extraRows).setZero();

}

// Matrix3d MPC_util::skew(const Vector3d& v) {
//     Matrix3d S;
//     S <<  0,    -v(2),  v(1),
//           v(2),  0,    -v(0),
//          -v(1), v(0),   0;
//     return S;
// }