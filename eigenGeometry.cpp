#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen Geometry
#include <Eigen/Geometry>
using namespace std;

int main ( int argc, char** argv )
{
    // Eigen/Geometry
    // 3D rotation  Matrix3d or Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    // using AngleAxis, can treat as matrix, Operator overloading
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //z-axis 45 degree
    cout .precision(3);
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;                //show rotation matrix
    // other way to initialize a rotation matrix
    rotation_matrix = rotation_vector.toRotationMatrix();
    cout<<"The Rotation Matrix is : \n"<<rotation_matrix<<endl;
    // using AngleAxis directly
    Eigen::Vector3d v ( 1,0,0 );
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"The v_rotated matrix = \n"<<v_rotated<<endl;

    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    // using rotation matrix
    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    // Euler angle   rotation matrix --> Euler angle
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 ); // ZYX，roll pitch yaw
    cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;

    //3D transformations in Homogeneous  Eigen::Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();                // name Isometry3d，but 4＊4 matrix
    cout<<"The T = \n"<<T.matrix()<<endl;
    T.rotate ( rotation_vector );                                     // rotation_vector
    T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );      // translation (1,3,4)
    cout << "Transform matrix = \n" << T.matrix() <<endl;

    // transformation
    Eigen::Vector3d v_transformed = T*v;                              // R*v+t
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;

    // you also can use Eigen::Affine3d , Eigen::Projective3d ....

    // Quaternion
    // initializing Quaternion  by AngleAxis
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;   // Attention ! coeffs order (x,y,z,w),w is real part
    // initializing Quaternion  by rotation matrix
    q = Eigen::Quaterniond ( rotation_matrix );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;
    // using quaternion to rotate
    v_rotated = q*v; // mathematically is qvq^{-1}
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    // The following parts are for the test
    Eigen::Quaterniond test1=Eigen::Quaterniond(0.55, 0.3, 0.2, 0.2);
    Eigen::Matrix3d test1_rotate=test1.normalized().toRotationMatrix();
    Eigen::Matrix3d test2_rotate=test1_rotate.transpose();
    Eigen::Isometry3d Test1=Eigen::Isometry3d::Identity();                // 4*4 matrix
    Test1.rotate ( test2_rotate );
    Eigen::Vector3d vec ( 0.7,1.1,0.2 );
    Eigen::Vector3d vec_inverse=-test2_rotate*vec;
//  Eigen::Vector3d vector1=test1_rotate.transpose()*vec*(-1);
    Test1.pretranslate ( vec_inverse );
//  Eigen::Matrix4d Test1_inverse=Test1.inverse();
    cout<<"The inverse of Test1 = \n"<<Test1.matrix()<<endl;

    Eigen::Quaterniond test=Eigen::Quaterniond(-0.1,0.3,-0.7,0.2);
    Eigen::Matrix3d test_rotate=test.normalized().toRotationMatrix();
    Eigen::Isometry3d Test=Eigen::Isometry3d::Identity();                // 4*4 matrix
    Test.rotate ( test_rotate );
    Test.pretranslate ( Eigen::Vector3d ( -0.1,0.4,0.8 ) );
    Eigen::Vector3d point (0.5,-0.1,0.2);
    Eigen::Vector3d point_transformed = Test*Test1*point;


    cout << "Transform matrix = \n" << T.matrix() <<endl;
    cout<<"The test = "<<test.coeffs()<<endl;
    cout<<"The test_rotate is= \n"<<Test.matrix()<<endl;
    cout<<"The new vector is \n"<<point_transformed<<endl;
    return 0;
}
