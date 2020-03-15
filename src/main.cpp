#include "algorithm.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include <ctime>
#include <stdint.h>
#include <chrono>
#include <ctime>
using namespace std;
using namespace HybridAStar;
using namespace cv;

void setColor(cv::Mat& mat, int row, int col, u_char r, u_char g, u_char b){
    mat.at<cv::Vec3b>(row, col)[0] = b;
    mat.at<cv::Vec3b>(row, col)[1] = g;
    mat.at<cv::Vec3b>(row, col)[2] = r;
}

std::time_t getTimeStamp()
{
	std::chrono::time_point<std::chrono::system_clock,std::chrono::microseconds> tp =
	std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
	auto tmp=std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
	std::time_t timestamp = tmp.count();
		//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
	return timestamp;
}

Constants::config collisionLookup[Constants::headings * Constants::positions];
/// A lookup of analytical solutions (Dubin's paths)
float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
/// The collission detection for testing specific configurations
CollisionDetection configurationSpace;

int main(){
    if(Constants::dubinsLookup) Lookup::dubinsLookup(dubinsLookup);
    Lookup::collisionLookup(collisionLookup);

    double x, y, a;
    cout << "Set StartPoint(x,y,a): ";
    cin >> x >> y >> a;

    cv::Mat view = cv::Mat::zeros(401, 151, CV_8UC3);
    cv::Mat cvmmp = cv::imread("/mnt/c/Users/milesching/Desktop/1.png");
    cv::resize(cvmmp, cvmmp, cv::Size(151, 401));

    u_char ucmap[401][151] = {0};
    memset(ucmap, 0x7f, sizeof(ucmap));
    for(int y = 0; y < 401; ++y)
        for(int x = 0; x < 151; ++x)
            if(cvmmp.at<cv::Vec3b>(400 - y, x)[0] > 200)
                ucmap[y][x] = 0;
    configurationSpace.updateMap(make_pair((u_char*)(void*)ucmap, make_pair(151, 401)));

    int width = 151;
    int height = 401;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();
    // set theta to a value (0,2PI]
    a = Helper::normalizeHeadingRad(a);
    const Node3D nGoal(x, y, a, 0, 0, nullptr);
    Node3D nStart(75, 100, 1.57, 0, 0, nullptr);
    double tstart = getTimeStamp();
    Node3D* nSolution = HybridAStar::Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup);
    double tms = (getTimeStamp() - tstart) / 1000;
    cout << "规划时间：" << tms << endl;
    cout << "规划结果：[" << " ms" << endl;
    int cnt = 0;
    while(nSolution){
        cout << nSolution->getX() << ", " << nSolution->getY() << ", " << nSolution->getT() << "," << endl;
        nSolution = (Node3D*)(void*)nSolution->getPred();
        ++cnt;
    }
    cout << "]" << endl;
    cout << "路径长度：" << cnt * 0.7068582 << " m" << endl;

    return 0;
}