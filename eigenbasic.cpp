// Created by haoliang_zhang
//
#include <iostream>
// Eigen Core
#include <Eigen/Core>
// algebraic operation of dense matrix
#include <Eigen/Dense>

//using namespace Eigen;

#define MATRIX_SIZE 50

int main(int argc, char **argv) {
    //<typename Scalar_, int Rows_, int Cols_>
    Eigen::Matrix<float,2,3> matrix_23;

    //Eigen::Vector3d is  Eigen::Matrix<>
    Eigen::Vector3d v_3d;
    // same as
    Eigen::Matrix<float, 3, 1> vd_3d;

    // Matrix3d   Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); // initial to zero
    // dynamic-size objects
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    // or
    Eigen::MatrixXd matrix_x;


    // input
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // output
    std::cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << std::endl;

    // visit by ()
    std::cout << "print matrix 2x3: " << std::endl;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) std::cout << matrix_23(i, j) << "\t";
        std::cout << std::endl;
    }

    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    // you cannot mix different type
    // Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;

    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    std::cout << "[1,2,3;4,5,6]*[3,2,1]=" << result.transpose() << std::endl;

    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    std::cout << "[1,2,3;4,5,6]*[4,5,6]: " << result2.transpose() << std::endl;

    // Do not mess up dimension
    // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    matrix_33 = Eigen::Matrix3d::Random();      // random matrix
    std::cout << "random matrix: \n" << matrix_33 << std::endl;
    std::cout << "transpose: \n" << matrix_33.transpose() << std::endl;      // transpose
    std::cout << "sum: " << matrix_33.sum() << std::endl;            // sum of all
    std::cout << "trace: " << matrix_33.trace() << std::endl;          // trace
    std::cout << "times 10: \n" << 10 * matrix_33 << std::endl;               // multiply
    std::cout << "inverse: \n" << matrix_33.inverse() << std::endl;        // inverse
    std::cout << "det: " << matrix_33.determinant() << std::endl;    // determinanat

    // Eigen value/vector
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    std::cout << "Eigen values = \n" << eigen_solver.eigenvalues() << std::endl;
    std::cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << std::endl;

    // solve equation
    // solve matrix_NN * x = v_Nd


    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
            = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();  // make sure semi-positive
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock(); // time
    //
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    std::cout << "time of normal inverse is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
//    std::cout << "x = " << x.transpose() << std::endl;

    // QR, acclerate
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    std::cout << "time of Qr decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
//    std::cout << "x = " << x.transpose() << std::endl;

    // for PD matrix, can use cholesky
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    std::cout << "time of ldlt decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << std::endl;
//    std::cout << "x = " << x.transpose() << std::endl;

    return 0;
}

