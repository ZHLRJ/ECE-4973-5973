#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <opencv2/core/core.hpp>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;
using namespace cv;


// path to trajectory file
string trajectory_file = "../../data/compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> VeVector3d;
//typedef vector<Sophus::SE3d> VeVector3d;


void DrawTrajectory(VeVector3d ,VeVector3d);
void pose_estimation_3d3d(
        const VeVector3d &pts1,
        const VeVector3d &pts2,
        Mat &R, Mat &t
);
int main(int argc, char **argv) {
//    cout<<"###################here"<<endl;
    std::ifstream inStream(trajectory_file, std::ios::binary);
    if (!inStream) throw std::runtime_error("couldn't open file " + trajectory_file);

    VeVector3d poses_e,poses_g;

    // implement pose reading code
    ifstream fin(trajectory_file);  //从文件中读取数据

    double time_e,tx_e,ty_e,tz_e,qx_e,qy_e,qz_e,qw_e,time_g,tx_g,ty_g,tz_g,qx_g,qy_g,qz_g,qw_g;
    string line;

    while(getline(fin,line))
    {
//        cout<<line<<endl;
        istringstream record(line);    //从string读取数据
        record>>time_e>>tx_e>>ty_e>>tz_e>>qx_e>>qy_e>>qz_e>>qw_e>>time_g>>tx_g>>ty_g>>tz_g>>qx_g>>qy_g>>qz_g>>qw_g;
        Eigen::Vector3d t_e(tx_e,ty_e,tz_e);
        Eigen::Quaterniond q_e = Eigen::Quaterniond(qw_e,qx_e,qy_e,qz_e).normalized();  //四元数的顺序要注意
        Sophus::SE3d SE3_e(q_e,t_e);
        poses_e.push_back(SE3_e);


        Eigen::Vector3d t_g(tx_g,ty_g,tz_g);
        Eigen::Quaterniond q_g = Eigen::Quaterniond(qw_g,qx_g,qy_g,qz_g).normalized();  //四元数的顺序要注意
        Sophus::SE3d SE3_g(q_g,t_g);
        Sophus::SE3d T_trans;
//        if (i==0){
//            T_trans=SE3_g*SE3_e.inverse();
//            i=1;
//        }
        poses_g.push_back(T_trans*SE3_g);

    }
    Mat R, t;
//    pose_estimation_3d3d (poses_e, poses_g, R, t );
    // draw trajectory in pangolin
    DrawTrajectory(poses_e,poses_g);
//
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(VeVector3d poses,VeVector3d poses2)
{
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
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < poses.size() - 1; i++) {
//            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glColor3f(1,0,0);
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}

VeVector3d pose_estimation_3d3d(const vector<Point3f> &pts1,const vector<Point3f> &pts2,Mat &R, Mat &t)
{
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for (int i = 0; i < N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f(Vec3f(p1) / N);
    p2 = Point3f(Vec3f(p2) / N);
    vector<Point3f> q1(N), q2(N); // remove the center
    for (int i = 0; i < N; i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
    }
    cout << "W=" << W << endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    Eigen::Matrix3d R_ = U * (V.transpose());
    if (R_.determinant() < 0) {
        R_ = -R_;
    }
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

    // convert to cv::Mat
    R = (Mat_<double>(3, 3) <<
                            R_(0, 0), R_(0, 1), R_(0, 2),
            R_(1, 0), R_(1, 1), R_(1, 2),
            R_(2, 0), R_(2, 1), R_(2, 2)
    );
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}