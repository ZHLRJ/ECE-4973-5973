#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

using namespace std;

int main( int argc, char** argv )
{

    // rotate 90 around z-axis
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SO3d SO3_R(R);               // initialize Sophus::SO(3) by R

//    Sophus::poi axisAngle{0,0,1,M_PI/2};
//    Sophus::SO3d SO3_v(axisAngle);

    Eigen::Quaternion<double> q (R);            // quaternion
    Sophus::SO3d SO3_q(q);
//    above methods are equal
//
    cout<<"SO(3) from matrix: "<<SO3_R.matrix()<<endl;
//    cout<<"SO(3) from vector: "<<SO3_v.matrix()<<endl;
    cout<<"SO(3) from quaternion :"<<SO3_q.matrix()<<endl;
//
//    lie algebra
    Eigen::Vector3d so3 = SO3_R.log();
    cout<<"so3 = "<<so3.transpose()<<endl;
//    vector --> skew-symmetric matrix
    cout<<"so3 hat=\n"<<Sophus::SO3d::hat(so3)<<endl;
//    skew-symmetric matrix --> vector
    cout<<"so3 hat vee= "<<Sophus::SO3d::vee( Sophus::SO3d::hat(so3) ).transpose()<<endl;
//
//    small Perturbation
    Eigen::Vector3d update_so3(1e-4, 0, 0); //delta R
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3)* SO3_R;
    cout<<"update_so3 = "<<Sophus::SO3d::exp(update_so3).matrix()<<endl;
    cout<<"SO3 updated = "<<SO3_updated.matrix()<<endl;


    //  SE(3)
    Eigen::Vector3d t(1,0,0);           // translate
    Sophus::SE3d SE3_Rt(R, t);           // R,t --> SE(3)
    Sophus::SE3d SE3_qt(q,t);            // q,t --> SE(3)
//    cout<<Sophus::SE3d(R,t)<<endl;
    cout<<"SE3 from R,t= "<<endl<<SE3_Rt.matrix()<<endl;
    cout<<"SE3 from q,t= "<<endl<<SE3_qt.matrix()<<endl;
    cout<<"SE3 Rotate matrix= "<<endl<<SE3_qt.rotationMatrix()<<endl;
//     Lie algebra of se(3) ---6D vector
    typedef Eigen::Matrix<double,6,1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout<<"se3 = "<<se3.transpose()<<endl;
    // in Sophus, se(3) translate + rotation
    // hat  vee
    cout<<"se3 hat = "<<endl<<Sophus::SE3d::hat(se3)<<endl;
    cout<<"se3 hat vee = "<<Sophus::SE3d::vee( Sophus::SE3d::hat(se3) ).transpose()<<endl;
//
//    update se3
    Vector6d update_se3; //samll perturbation
    update_se3.setZero();
    update_se3(0,0) = 1e-4;
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3)*SE3_Rt;
    cout<<"SE3 updated = "<<endl<<SE3_updated.matrix()<<endl;

    return 0;
}