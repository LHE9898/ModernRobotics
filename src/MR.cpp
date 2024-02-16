#include <iostream>
#include <Eigen/Eigen>
#include <cmath>
#include <tuple>

#include "MR.h"

// I =
//      1 0 0
//      0 1 0
//      0 0 1
// Matrix3d I = Matrix3d::Identity();
Matrix3d I = Matrix3d::Identity();
Matrix3d ZERO = Matrix3d::Zero();
Vector3d V_ZERO = Vector3d::Zero();

bool MR::NearZero(double val){

    if (val < NEAR){
        return true;
    }
    else{
        return false;
    }
}

// RotInv(R)
Matrix3d MR::RotInv(const Matrix3d& R){

    Eigen::Matrix3d M;

    M = R.inverse();

    return M;
}

// VecToso3(omega)
Matrix3d MR::VecToso3(const Vector3d& w){
    Eigen::Matrix3d M;
    
    M <<  0.0,-w(2), w(1),
         w(2),  0.0,-w(0),
        -w(1), w(0),  0.0;

    return M;
}

// so3ToVec(so3mat)
Vector3d MR::so3ToVec(const Matrix3d& so3mat){
    Vector3d V;
    
    V << so3mat(2,1), so3mat(0,2), so3mat(1,0);

    return V;
}

// AxisAng3(expc3)
Vector4d MR::AxisAng3(const Vector3d& expc3){
    Vector4d V;
    double omghat[3], theta;

    theta = expc3.norm();
    omghat[0] = expc3(0) / theta;
    omghat[1] = expc3(1) / theta;
    omghat[2] = expc3(2) / theta;

    V << omghat[0], omghat[1], omghat[2], theta;
    
    return V;
}

// MatrixExp3(so3mat)
Matrix3d MR::MatrixExp3(const Matrix3d& so3mat){
    Matrix3d R;
    Vector4d V;
    double w1,w2,w3, theta;

    V = AxisAng3(so3ToVec(so3mat));
    w1 = V(0); 
    w2 = V(1);
    w3 = V(2);
    theta = V(3);

    if (0 >= theta){
        R = I;
    }
    else{
        R << cos(theta)+pow(w1,2)*(1-cos(theta)),  w1*w2*(1-cos(theta))-w3*sin(theta),  w1*w3*(1-cos(theta))+w2*sin(theta),
              w1*w2*(1-cos(theta))+w3*sin(theta), cos(theta)+pow(w2,2)*(1-cos(theta)),  w2*w3*(1-cos(theta))-w1*sin(theta),
              w1*w3*(1-cos(theta))-w2*sin(theta),  w2*w3*(1-cos(theta))+w1*sin(theta), cos(theta)+pow(w3,2)*(1-cos(theta));
    }

    return R ;
}

// MatrixLog3(R)
Matrix3d MR::MatrixLog3(const Matrix3d& R){
    Matrix3d so3mat;
    Vector3d omg;
    double theta;
    double trR = R(0,0) + R(1,1) + R(2,2);
    double acos_in = (trR-1.0)/2.0;

    // cos(theta) >= 1
    // R = I
    if (acos_in >= 1.0){
        so3mat.setZero(3,3);
    }
    // cos(theta) <= -1 
    else if (acos_in <= -1.0){
        // w = [0, 0, 1]
        if(!NearZero(1.0+R(2,2))){
            omg <<        1.0/sqrt(2.0*(1.0+R(2,2)))*R(0,2),
                          1.0/sqrt(2.0*(1.0+R(2,2)))*R(1,2),
                    1.0/sqrt(2.0*(1.0+R(2,2)))*(1.0+R(2,2));
        }
        // w = [0, 1, 0]
        else if(!NearZero(1.0+R(1,1))){
            omg <<        1.0/sqrt(2.0*(1.0+R(1,1)))*R(0,1),
                    1.0/sqrt(2.0*(1.0+R(1,1)))*(1.0+R(1,1)),
                          1.0/sqrt(2.0*(1.0+R(1,1)))*R(2,1);
        }
        // w = [1, 0, 0]
        else {
            omg <<  1.0/sqrt(2.0*(1.0+R(0,0)))*(1.0+R(0,0)),
                          1.0/sqrt(2.0*(1.0+R(0,0)))*R(1,0),
                          1.0/sqrt(2.0*(1.0+R(0,0)))*R(2,0);
        }
        so3mat = VecToso3(PI*omg);
    }
    // -1 < cos(theta) < 1 
    else{
        theta = acos(acos_in);
        so3mat = (1.0/(2.0*sin(theta))*(R-R.transpose()))*theta;
    }

    return so3mat;
}

// RpToTrans(R, p)
Matrix4d MR::RpToTrans(const Matrix3d& R, const Vector3d& p){
    Matrix4d T;
    T << R, p,
         0.0, 0.0, 0.0, 1.0; 
    // _T.block<3, 3>(0, 0) = R;
    // _T.block<3, 1>(0, 3) = p;
    // _T.row(3) << 0, 0, 0, 1;
    
    return T;
}

// TransToRp(T)
vector<MatrixXd> MR::TransToRp(const Matrix4d& T){
    vector<MatrixXd> Rp;
    Matrix3d R;
    Vector3d p;
    R = T.block<3, 3>(0, 0);
    p = T.block<3, 1>(0, 3);

    Rp.push_back(R);
    Rp.push_back(p);

    return Rp;
}

// TransToInv(T)
Matrix4d MR::TransToInv(const Matrix4d& T){
    Matrix4d invT;
    invT = T.inverse();

    return invT;
}

// VecTose3(V)
Matrix4d MR::VecTose3(const VectorXd& V){
    Matrix4d se3mat;
    Vector3d w,v;
    w << V(0) , V(1), V(2);
    v << V(3) , V(4), V(5);

    se3mat << VecToso3(w), v,
              0.0, 0.0, 0.0, 0.0;
    
    return se3mat;
}

// se3ToVec(se3mat)
VectorXd MR::se3ToVec(const Matrix4d& se3mat){
    VectorXd V(6);
    // V << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    VectorXd w(3),v(3);
    Matrix3d w_matrix;
    
    w_matrix = se3mat.block<3, 3>(0, 0);
    w = so3ToVec(w_matrix);     
    v = se3mat.block<3, 1>(0, 3);

    V.segment(0,3) = w;
    V.segment(3,3) = v;

    return V;
}

// Adjoint(T)
MatrixXd MR::Adjoint(const Matrix4d& T){
    MatrixXd Ad_T(6,6);
    Matrix3d zeroMat = zeroMat.setZero();
    Matrix3d p_mat;
    vector<MatrixXd> Rp = TransToRp(T);
    

    p_mat = VecToso3(Rp[1]);

    // Ad_T.block<3, 3>(0, 0) = Rp[0];
    // Ad_T.block<3, 3>(0, 3) = zeroMat;
    // Ad_T.block<3, 3>(3, 0) = p_mat*Rp[0];
    // Ad_T.block<3, 3>(3, 3) = Rp[0];
    Ad_T << Rp[0], zeroMat,
            p_mat*Rp[0], Rp[0];

    return Ad_T;
}

VectorXd MR::ScrewToAxis(const Vector3d& q,const Vector3d& s,const double h){
    VectorXd axis(6);
    axis.segment(0, 3) = s;
    axis.segment(3, 3) = q.cross(s) + (h * s);

    return axis;
}
VectorXd MR::AxisAng6(const VectorXd& expc6){
    VectorXd v_ret(7);
    double theta = Vector3d(expc6(0), expc6(1), expc6(2)).norm();
    
    if (NearZero(theta))
        theta = Vector3d(expc6(3), expc6(4), expc6(5)).norm();
    
    v_ret << expc6 / theta, theta;
    
    return v_ret;
}

// MatrixExp6(se3mat)
Matrix4d MR::MatrixExp6(const Matrix4d& se3mat){
    Matrix4d T;
    Matrix3d omgmat, G_theta;
    Vector3d omgtheta, v;
    double theta;
    
    omgtheta = so3ToVec(se3mat.block<3, 3>(0, 0));
    theta = (AxisAng3(omgtheta))(3);
    
    if(NearZero(omgtheta.norm())){
        T << I, se3mat.block<3,1>(0,3),
             0.0, 0.0, 0.0, 1.0;
    }
    else{
        omgmat = se3mat.block<3, 3>(0, 0) / theta;
        v <<se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
        G_theta = I*theta + (1.0-cos(theta))*omgmat + (theta-sin(theta))*(omgmat*omgmat);

        T << MatrixExp3(omgmat*theta), G_theta*v/theta, 
             0.0, 0.0, 0.0, 1.0;
    }

    return T;
}

// MatrixLog6(T)
Matrix4d MR::MatrixLog6(const Matrix4d& T){
    Matrix4d se3mat;
    Matrix3d so3mat, G_inv_mat;
    vector<MatrixXd> Rp = TransToRp(T);
    Matrix3d R = Rp[0];
    Vector3d p = Rp[1];
    double trR = R.trace();
    double theta = acos((trR-1.0)/2.0);

    so3mat = MatrixLog3(R);
    if(NearZero(so3mat.norm())){
        se3mat << ZERO, p,
                  0.0, 0.0, 0.0, 0.0;
    }
    else{
        G_inv_mat = (I - 0.5*so3mat + (1.0/theta-0.5/tan(theta/2.0))*so3mat*so3mat/theta);
        se3mat << so3mat, G_inv_mat*p,
                  0.0, 0.0, 0.0, 0.0;
    }
    return se3mat;
}

Matrix4d MR::FKinSpace(const Matrix4d& M,
                       const MatrixXd& Slist,
                       const VectorXd& thetalist){
    Matrix4d T;
    int list_size = thetalist.size();

    T = M;
    for(int idx=(list_size-1); idx > -1; idx--){
        // [w; v]*theta
        T = MatrixExp6(VecTose3(Slist.col(idx) * thetalist(idx)))*T;
    }
    
    return T;
}

Matrix4d MR::FKinBody(const Matrix4d& M,
                      const MatrixXd& Blist,
                      const VectorXd& thetalist){
    Matrix4d T;
    
    int list_size = thetalist.size();

    T = M;
    for(int idx=0; idx < list_size; idx++){
        // [w; v]*theta
        T = T*MatrixExp6(VecTose3(Blist.col(idx) * thetalist(idx)));
    }

    return T;
}

MatrixXd MR::JacobianSpace(const MatrixXd& Slist,
                           const VectorXd& thetalist){
    int list_size = thetalist.size();
    MatrixXd J = Slist;
    Matrix4d T;
    
    T << I, V_ZERO,
         0.0, 0.0, 0.0, 1.0;
    
    for(int idx=0; idx<list_size; idx++){
        T = T*MatrixExp6(VecTose3(Slist.col(idx)*thetalist(idx)));
        J.col(idx) = Adjoint(T)*Slist.col(idx);
    }    

    return J;
}

MatrixXd MR::JacobianBody(const MatrixXd& Blist,
                          const VectorXd& thetalist){
    int list_size = thetalist.size();
    MatrixXd J = Blist;
    Matrix4d T, se3mat;
    
    T << I, V_ZERO,
         0.0, 0.0, 0.0, 1.0;

    for(int idx=list_size-2; idx>-1; idx--){
        T = T*MatrixExp6(VecTose3(-1.0*Blist.col(idx+1)*thetalist(idx+1)));
        J.col(idx) = Adjoint(T)*Blist.col(idx);
    }  

    return J;
}

// IKinSpace(Slist,M,T,thetalist0,eomg,ev)
tuple<VectorXd,bool> MR::IKinSpace(const MatrixXd& Slist, 
                        const Matrix4d& M, 
                        const Matrix4d& T, 
                        const VectorXd& thetalist0, 
                        const double eomg, 
                        const double ev){
    MatrixXd J, Jpinv;
    Matrix4d Tsb;
    VectorXd Vs;
    Vector3d thetalist, w, v;

    thetalist = thetalist0;
    
    Tsb = FKinSpace(M,Slist,thetalist);
    Vs = Adjoint(Tsb)*se3ToVec(MatrixLog6(Tsb.inverse()*T));
    w << Vs(0),Vs(1),Vs(2); 
    v << Vs(3),Vs(4),Vs(5); 

    bool err = (w.norm() > eomg || v.norm() > ev);
    int iter = 0;
    while(err && iter < IK_ITER_MAX ){
        J = JacobianSpace(Slist, thetalist);
        Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
        thetalist = thetalist + Jpinv*Vs;

        iter ++;
        Tsb = FKinSpace(M,Slist,thetalist);
        Vs = Adjoint(Tsb)*se3ToVec(MatrixLog6(Tsb.inverse()*T));

        w << Vs(0),Vs(1),Vs(2); 
        v << Vs(3),Vs(4),Vs(5); 
        err = (w.norm() > eomg || v.norm() > ev);
    }

    // cout << "iter: " << iter << endl;
    return make_tuple(thetalist,!err);
}

// IKinBody(Blist,M,T,thetalist0,eomg,ev)
tuple<VectorXd,bool> MR::IKinBody(
                        const MatrixXd& Blist, 
                        const Matrix4d& M, 
                        const Matrix4d& T, 
                        const VectorXd& thetalist0, 
                        const double eomg, 
                        const double ev){
    MatrixXd J, Jpinv;
    Matrix4d Tsb;
    VectorXd Vb;
    Vector3d thetalist, w, v;

    thetalist = thetalist0;
    Tsb = FKinBody(M,Blist,thetalist);
    Vb = se3ToVec(MatrixLog6(Tsb.inverse()*T));
    w << Vb(0),Vb(1),Vb(2); 
    v << Vb(3),Vb(4),Vb(5); 

    bool err = (w.norm() > eomg || v.norm() > ev);
    int iter = 0;
    while(err && iter < IK_ITER_MAX ){
        J = JacobianBody(Blist, thetalist);
        Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
        thetalist = thetalist + Jpinv*Vb;

        iter ++;
        Tsb = FKinBody(M,Blist,thetalist);
        Vb = se3ToVec(MatrixLog6(Tsb.inverse()*T));

        w << Vb(0),Vb(1),Vb(2); 
        v << Vb(3),Vb(4),Vb(5); 
        err = (w.norm() > eomg || v.norm() > ev);
    }

    // cout << "iter: " << iter << endl;
    return make_tuple(thetalist,!err);                        
}