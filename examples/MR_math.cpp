#include <iostream>
#include <Eigen/Eigen>
#include "MR.h"

using namespace std;

int main(int argc, char** argv){    

    cout << fixed;
    cout.precision(6);

/*****CH3. Rigid-Body Motion *****/
//                               //
    /***RotInv(R)***/
    // Matrix3d inM;
    // inM<< 0.0,-1.0, 0.0,
    //       0.0, 0.0,-1.0,
    //       1.0, 0.0, 0.0;
    // cout << MR::RotInv(inM) << endl;

    /***VecToso3(omg) ***/
    // Vector3d inVec;
    // inVec << 1.123, 2.456, 3.789;
    // cout << MR::VecToso3(inVec) << endl;

    /***so3ToVec(so3mat)***/
    // inM<< 0.0,-1.0, 2.0,
    //       1.0, 0.0,-3.0,
    //      -2.0, 3.0, 0.0;
    // cout << MR::so3ToVec(inM) << endl;

    /***AxisAng3(expc3)***/
    // inVec << 1.0, 2.0, 3.0;
    // cout << MR::AxisAng3(inVec) << endl;

    /***MatrixExp3(so3mat)***/
    // inM<< 0.0,-3.0, 2.0,
    //       3.0, 0.0,-1.0,
    //      -2.0, 1.0, 0.0;
    // inM<< 0.0, 0.0, 0.0,
    //       0.0, 0.0, 0.0,
    //       0.0, 0.0, 0.0;
    // cout << MR::MatrixExp3(inM) << endl;

    /***MatrixLog3(so3mat)***/
    // inM<< 0.0, 0.0, 1.0,
    //       1.0, 0.0, 0.0,
    //       0.0, 1.0, 0.0;
    // cout << MR::MatrixLog3(inM) << endl;

    /***RpToTrans(R,p)***/
    // cout << MR::RpToTrans(inM, inVec) << endl;

    /***TransToRp(T)***/    
    // MR::TransToRp(MR::RpToTrans(inM, inVec));
    // cout << MR::_R << endl;
    // cout << MR::_p << endl;

    /***TransToInv(T)***/
    // Matrix4d T, invT;
    // inM<< 0.0, 0.0, 1.0,
    //       1.0, 0.0, 0.0,
    //       0.0, 1.0, 0.0;
    // inVec << 1.0, 2.0, 3.0;    
    // T = MR::RpToTrans(inM, inVec);
    // invT = MR::TransToInv(T);
    // cout << "invT: \n" << invT << endl;
    // cout << "\nT*T^-1: \n" << T*invT << endl;

    /***VecTose3(V)***/    
    // VectorXd V(6);
    // V << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
    // cout << "VecTose3: \n" << MR::VecTose3(V) << endl;

    /***se3ToVec(se3mat)***/    
    // Matrix4d se3;
    // se3 = MR::VecTose3(V);
    // cout << "se3ToVec: \n" << MR::se3ToVec(se3) << endl; 

    /***Adjoint(T)***/
    // Matrix4d T, invT;
    // T << 1.0, 0.0, 0.0, 0.0,
    //      0.0, 0.0,-1.0, 0.0,
    //      0.0, 1.0, 0.0, 3.0,
    //      0.0, 0.0, 0.0, 1.0;
    // cout << "Adjoint: \n" << MR::Adjoint(T) << endl; 

    /***ScrewToAxis(q,s,h)***/
    // Vector3d q,s;
    // double h;
    // q << 3, 0, 0;
    // s << 0, 0, 1;
    // h = 2;
    // cout << MR::ScrewToAxis(q,s,h) << endl;

    /***AxisAng(expc6)***/
    // VectorXd expc6(6);
    // expc6 << 1, 0, 0, 1, 2, 3;
    // cout << MR::AxisAng6(expc6) << endl;

    /***MatrixExp6(se3mat)***/    
    // Matrix4d se3mat;
    // se3mat << 0,      0,       0,      0,
    //           0,      0, -1.5708, 2.3562,
    //           0, 1.5708,       0, 2.3562,
    //           0,      0,       0,      0;
    // cout << MR::MatrixExp6(se3mat) << endl;

    /***MatrixLog6(T)***/    
    // Matrix4d T;
    // T << 
    // 1, 0, 0, 0,
    // 0, 0, -1, 0,
    // 0, 1, 0, 3,
    // 0, 0, 0, 1;
    // cout << MR::MatrixLog6(T) << endl;
/***********************************************/

//----------------------------------------------
//----------------------------------------------

/********** CH4. Forward Kinematics **********/
//                                           //
    // Matrix4d M_S, M_B, Ts;
    // MatrixXd Slist(6,3), Blist(6,3);
    // VectorXd thetalist_S(3), thetalist_B(3);

    // M_S << -1.0,  0.0,  0.0,  0.0,
    //       0.0,  1.0,  0.0,  6.0,
    //       0.0,  0.0, -1.0,  2.0,
    //       0.0,  0.0,  0.0,  1.0;
    // Slist.col(0) << 0.0, 0.0, 1.0, 4.0, 0.0, 0.0;
    // Slist.col(1) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    // Slist.col(2) << 0.0, 0.0, -1.0, -6.0, 0.0, -0.1;
    // thetalist_S << PI/2.0, 3.0, PI;

    // Ts = MR::FKinSpace(M_S, Slist, thetalist_S);
    // cout << "FKinSpace\n"<< Ts << endl;

    // M_B << -1, 0, 0, 0,
    //         0, 1, 0, 6,
    //         0, 0, -1, 2,
    //         0, 0, 0, 1;
    // Blist.col(0) << 0, 0, -1, 2, 0, 0;
    // Blist.col(1) << 0, 0, 0, 0, 1, 0;
    // Blist.col(2) << 0, 0, 1, 0, 0, 0.1;
    // thetalist_B << PI/2, 3, PI;
    // cout << MR::FKinBody(M_B, Blist, thetalist_B)<< endl;
/*********************************************/

//----------------------------------------------
//----------------------------------------------
   
/********** CH5. Velocity Kinematics **********/
//                                            //
    // MatrixXd Slist(6,4);
    // VectorXd thetalist(4);
    // Slist.col(0)<< 0, 0, 1, 0, 0.2, 0.2;
    // Slist.col(1)<< 1, 0, 0, 2, 0, 3;
    // Slist.col(2)<< 0, 1, 0, 0, 2, 1;
    // Slist.col(3)<< 1, 0, 0, 0.2, 0.3, 0.4;
    // thetalist << 0.2, 1.1, 0.1, 1.2;

    // cout << MR::JacobianSpace(Slist,thetalist) << endl;


    // MatrixXd Blist(6,4);
    // VectorXd thetalist(4);
    // Blist.col(0)<< 0, 0, 1, 0, 0.2, 0.2;
    // Blist.col(1)<< 1, 0, 0, 2, 0, 3;
    // Blist.col(2)<< 0, 1, 0, 0, 2, 1;
    // Blist.col(3)<< 1, 0, 0, 0.2, 0.3, 0.4;
    // thetalist << 0.2, 1.1, 0.1, 1.2;
    
    // cout << MR::JacobianBody(Blist,thetalist) << endl;
/**********************************************/

//----------------------------------------------
//----------------------------------------------

/********** CH6. Inverse Kinematics **********/
//                                           //
    // MatrixXd Slist(6,3);
    // Matrix4d M, T;
    // VectorXd thetalist0(3);
    // double eomg, ev;
    // tuple<VectorXd,bool> IK_Space;

    // Slist.col(0) << 0.0, 0.0, 1.0, 4.0, 0.0, 0.0;
    // Slist.col(1) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    // Slist.col(2) << 0.0, 0.0,-1.0,-6.0, 0.0,-0.1;
    // M <<-1.0,  0.0,  0.0,  0.0,
    //      0.0,  1.0,  0.0,  6.0,
    //      0.0,  0.0, -1.0,  2.0,
    //      0.0,  0.0,  0.0,  1.0;
    // T << 0.0, 1.0,  0.0, -5.0,
    //      1.0, 0.0,  0.0,  4.0,
    //      0.0, 0.0, -1.0,  1.6858,
    //      0.0, 0.0,  0.0,  1.0;
    // thetalist0 << 1.5, 2.5, 3.0;
    // eomg = 0.01;
    // ev = 0.001;

    // IK_Space = MR::IKinSpace(Slist,M,T,thetalist0,eomg,ev);
    // cout << "theta: \n"<< get<0>(IK_Space) <<endl;
    // cout << "SUCCESS: "<< get<1>(IK_Space) <<endl;


    // MatrixXd Blist(6,3);
    // Matrix4d M, T;
    // VectorXd thetalist0(3);
    // double eomg, ev;
    // tuple<VectorXd,bool> IK_Body;

    // Blist.col(0) << 0.0, 0.0, -1.0, 2.0, 0.0, 0.0;
    // Blist.col(1) << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    // Blist.col(2) << 0.0, 0.0, 1.0, 0.0, 0.0, 0.1;
    // M <<-1.0,  0.0,  0.0,  0.0,
    //      0.0,  1.0,  0.0,  6.0,
    //      0.0,  0.0, -1.0,  2.0,
    //      0.0,  0.0,  0.0,  1.0;
    // T << 0.0, 1.0,  0.0, -5.0,
    //      1.0, 0.0,  0.0,  4.0,
    //      0.0, 0.0, -1.0,  1.6858,
    //      0.0, 0.0,  0.0,  1.0;
    // thetalist0 << 1.5, 2.5, 3.0;
    // eomg = 0.01;
    // ev = 0.001;

    // IK_Body = MR::IKinBody(Blist,M,T,thetalist0,eomg,ev);
    // cout << "theta: \n"<< get<0>(IK_Body) <<endl;
    // cout << "SUCCESS: "<< get<1>(IK_Body) <<endl;
/*********************************************/

//----------------------------------------------
//----------------------------------------------

/********** CH8. Open-loop Dynamics **********/
//                                           //
/*********************************************/

    return 0;
}