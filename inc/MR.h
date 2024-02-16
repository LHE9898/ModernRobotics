#ifndef __MR_H__
#define __MR_H__

#define PI 3.14159265358979323846264338327950288419716939937510582097494459230781
#define NEAR 1e-6
#define IK_ITER_MAX 20

using namespace std;
using namespace Eigen;

namespace MR{
    
    bool NearZero(double val);

    // CH3. Rigid-Body Motion 
    Matrix3d RotInv(const Matrix3d& R);           // R^-1
    Matrix3d VecToso3(const Vector3d& w);         // w=[w_1, w_2, w_3]^T -> [w]=[3x3]
    Vector3d so3ToVec(const Matrix3d& so3mat);    // [w]=[3x3] -> w=[w_1, w_2, w_3]^T 
    Vector4d AxisAng3(const Vector3d& expc3);     // [w1,w2,w3,theta]
    Matrix3d MatrixExp3(const Matrix3d& so3mat);  // so(3) -> SO(3)
    Matrix3d MatrixLog3(const Matrix3d& R);       // SO(3) -> so(3)
    Matrix4d RpToTrans(const Matrix3d& R,         // T = [R p 
                       const Vector3d& p);        //      0 1]
    vector<MatrixXd> TransToRp(const Matrix4d& T);// R = Rp[0], p=Rp[1]      
    Matrix4d TransToInv(const Matrix4d& T);
    Matrix4d VecTose3(const VectorXd& V);         // [w v] -> [[w] v; 0 0]
    VectorXd se3ToVec(const Matrix4d& se3mat);    // [[w] v; 0 0] -> [w v]
    MatrixXd Adjoint(const Matrix4d& T);          // Ad_T = [R, 0; [p]R, R]
    VectorXd ScrewToAxis(const Vector3d& q,
                         const Vector3d& s,
                         const double h);         // q,s,h -> [w v]
    VectorXd AxisAng6(const VectorXd& expc6);     // [w1,w2,w3,v1,v2,v3,theta]
    Matrix4d MatrixExp6(const Matrix4d& se3mat);  // se(3) -> SE(3)
    Matrix4d MatrixLog6(const Matrix4d& T);       // SE(3) -> se(3)

    // CH4. Forward Kinematics
    Matrix4d FKinBody(const Matrix4d& M,
                      const MatrixXd& Blist,
                      const VectorXd& thetalist); // [M, S, theta] -> T_bs
    Matrix4d FKinSpace(const Matrix4d& M,
                       const MatrixXd& Slist,
                       const VectorXd& thetalist); // [M, S, theta] -> T_sb

    // CH5. Velocity Kinematics
    MatrixXd JacobianSpace(const MatrixXd& Slist,
                           const VectorXd& thetalist);
    MatrixXd JacobianBody(const MatrixXd& Blist,
                          const VectorXd& thetalist);
    
    // CH6. Inverse Kinematics
    tuple<VectorXd,bool> IKinBody(
                        const MatrixXd& Blist, 
                        const Matrix4d& M, 
                        const Matrix4d& T, 
                        const VectorXd& thetalist0, 
                        const double eomg, 
                        const double ev);          // tuple 
    tuple<VectorXd,bool> IKinSpace(
                        const MatrixXd& Slist, 
                        const Matrix4d& M, 
                        const Matrix4d& T, 
                        const VectorXd& thetalist0, 
                        const double eomg, 
                        const double ev);

    // CH8. Open-loop Dynamics

}

#endif  