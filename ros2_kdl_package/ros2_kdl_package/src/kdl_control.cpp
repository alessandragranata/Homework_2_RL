#include "kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;

    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    
    KDL::Frame x = robot_->getEEFrame();        // Current pose
    KDL::Twist x_dot = robot_->getEEVelocity(); // Current velocity
    
   
    Vector6d ep;
    
    Vector6d ep_dot;
    
    
    computeErrors(_desPos,x,_desVel,x_dot,ep,ep_dot);
 
    // Step 3: Set up gain matrices
    Matrix6d KP, KD;
    KP.setZero();
    KD.setZero();
    
    for(int i=0; i<3; ++i)
        KP(i,i)=_Kpp;
    for(int i=3; i<6; ++i)
        KP(i,i)=_Kpo;
    
    for(int i=0; i<3; ++i)
        KD(i,i)=_Kdp;
    for(int i=3; i<6; ++i)
        KD(i,i)=_Kdo;
     

    Eigen::MatrixXd J = robot_->getEEJacobian().data;  // Jacobian  
    Eigen::MatrixXd J_dot = robot_->getEEJaDot(); // Time derivative of Jacobian
    Eigen::MatrixXd B = robot_->getJsim(); // Joint space inertia matrix 
    
    Eigen::VectorXd n = robot_->getCoriolis(); // Nonlinear effects ( with no gravity beacuse of the world)
    
    Eigen::MatrixXd q_dot = robot_->getJntVelocities(); // Current joint velocities
    
    
    // Step 7: Compute y (operational space control input)
    Eigen::VectorXd y;
    y =  pseudoinverse(J) * (toEigen(_desAcc) + KD * ep_dot  + KP * ep - J_dot * q_dot);
 
    // Step 8: Compute torques
    Eigen::VectorXd tau = B * y + n;
 
    return tau; // Return computed torques 
    

   
}

//

