//
// Created by mars_zhang on 12/10/20.
//
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <chrono>
using namespace std;
using namespace cv;

int main ( int argc, char** argv ) {

    string imgname="../../data/Zugpsitze_mountain.jpg";

    Mat img_1 = imread(imgname, IMREAD_COLOR);
    cout<<img_1.size<<endl;
    std::vector<KeyPoint> keypoints_1;


    int num_feature=10;

    Ptr<BRISK> detector_BRISK = BRISK::create(num_feature);
    Ptr<ORB> detector_ORB = ORB::create(num_feature);
    Ptr<xfeatures2d::SURF> detector_SURF = xfeatures2d::SURF::create(num_feature);
    Ptr<SIFT> detector_SIFT = SIFT::create(num_feature);


    chrono::steady_clock::time_point t1,t2;
    chrono::duration<double> time_used;

    // BRISK
    t1 = chrono::steady_clock::now();
    detector_BRISK->detect(img_1, keypoints_1);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "detector_BRISK costs time : " << time_used.count() * 1000 << " millisecond." << endl;



    // detector_ORB

    t1 = chrono::steady_clock::now();
    detector_ORB->detect(img_1, keypoints_1);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "detector_ORB costs time : " << time_used.count() * 1000 << " millisecond." << endl;


    // detector_SURF

    t1 = chrono::steady_clock::now();
    detector_SURF->detect(img_1, keypoints_1);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "detector_SURF costs time : " << time_used.count() * 1000 << " millisecond." << endl;

    // detector_SIFT

    std::vector<KeyPoint> keypoints_sift;
    keypoints_sift.resize(num_feature);
    t1 = chrono::steady_clock::now();
    detector_SIFT->detect(img_1, keypoints_sift);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "detector_SIFT costs time : " << time_used.count() * 1000 << " millisecond." << endl;

    return 0;
}