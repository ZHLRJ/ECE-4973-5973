//
// Created by Mars_Zhang on 2020/8/27.
//
//The test for 3D-3D ICP
#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <unistd.h>
#include <pangolin/pangolin.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

// path to trajectory file
// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {return 0;}
    bool write ( ostream& out ) const {return 0;}
protected:
    Eigen::Vector3d _point;
};
void bundleAdjustment (const vector< Eigen::Vector3d >& pts1,const vector<Eigen::Vector3d >& pts2,Eigen::Isometry3d &T);

void DrawTrajectory(vector< Eigen::Vector3d >& poses,vector< Eigen::Vector3d >& poses2);


int main(int argc, char **argv) {
    string trajectory_file = "./compare.txt";

    vector<Eigen::Vector3d> position_estimation,position_groundtruth,position_trans;

    // implement pose reading code
    ifstream fin(trajectory_file);  //从文件中读取数据
    double time_e,tx_e,ty_e,tz_e,qx_e,qy_e,qz_e,qw_e,time_g,tx_g,ty_g,tz_g,qx_g,qy_g,qz_g,qw_g;
    string line;

    int i=0;

    while(getline(fin,line))
    {
//        cout<<line<<endl;
        istringstream record(line);    //从string读取数据

        record>>time_e>>tx_e>>ty_e>>tz_e>>qx_e>>qy_e>>qz_e>>qw_e>>time_g>>tx_g>>ty_g>>tz_g>>qx_g>>qy_g>>qz_g>>qw_g;
        Eigen::Vector3d t_e(tx_e,ty_e,tz_e);
        position_estimation.push_back(t_e);


        Eigen::Vector3d t_g(tx_g,ty_g,tz_g);
        position_groundtruth.push_back(t_g);

    }
    Eigen::Isometry3d T_trans;
//    cout<<"position_estimation: "<<position_estimation.size()<<endl<<position_estimation[1]<<endl;
    bundleAdjustment (position_groundtruth,position_estimation,T_trans);
    cout<<"The T_trans : "<<T_trans.matrix()<<endl;
    cout<<"position_groundtruth : "<<position_groundtruth[0]<<endl;
    cout<<"position_estimation : "<<position_estimation[0]<<endl;
    cout<<"position_refine : "<<T_trans*position_estimation[0]<<endl;
    for (auto &P:position_estimation){
        position_trans.push_back(T_trans*P);
    }

    DrawTrajectory(position_groundtruth,position_trans);

}

void bundleAdjustment (
        const vector< Eigen::Vector3d >& pts1,
        const vector<Eigen::Vector3d >& pts2,
        Eigen::Isometry3d &T)
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( unique_ptr<Block::LinearSolverType>(linearSolver) );      // 矩阵块求解器
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( unique_ptr<Block>(solver_ptr ));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
            Eigen::Matrix3d::Identity(),
            Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
                pts2[i] );
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( pts1[i] );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;
//    T_trans=Sophus::SE3( Eigen::Isometry3d(pose->estimate()).rotation(),Eigen::Isometry3d(pose->estimate()).translation());
//    cout<<endl<<"after optimization:"<<endl;
//    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate()).matrix()<<endl;
    T=(Eigen::Isometry3d( pose->estimate()).matrix());

}
void DrawTrajectory(vector< Eigen::Vector3d >& poses,vector< Eigen::Vector3d >& poses2) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
//            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glColor3f(0,0,1);
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }

        for (size_t i = 0; i < poses.size() - 1; i++) {
//            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glColor3f(1,0,0);
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1[0], p1[1], p1[2]);
            glVertex3d(p2[0], p2[1], p2[2]);
            glEnd();
        }

        pangolin::FinishFrame();
//        usleep(5000);   // sleep 5 ms
    }

}