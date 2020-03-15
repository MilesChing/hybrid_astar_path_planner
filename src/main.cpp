#include "algorithm.h"
#include <iostream>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace HybridAStar;
using namespace cv;

void setColor(cv::Mat& mat, int row, int col, u_char r, u_char g, u_char b){
    mat.at<cv::Vec3b>(row, col)[0] = b;
    mat.at<cv::Vec3b>(row, col)[1] = g;
    mat.at<cv::Vec3b>(row, col)[2] = r;
}

Constants::config collisionLookup[Constants::headings * Constants::positions];
/// A lookup of analytical solutions (Dubin's paths)
float* dubinsLookup = new float [Constants::headings * Constants::headings * Constants::dubinsWidth * Constants::dubinsWidth];
/// The collission detection for testing specific configurations
CollisionDetection configurationSpace;

int main(){
    if(Constants::dubinsLookup) Lookup::dubinsLookup(dubinsLookup);
    Lookup::collisionLookup(collisionLookup);

    cv::namedWindow("HybridA* Test", cv::WINDOW_KEEPRATIO);
    double x, y, a;
    cout << "Set StartPoint(x,y,a): ";
    cin >> x >> y >> a;

    cv::Mat view = cv::Mat::zeros(401, 151, CV_8UC3);
    cv::Mat cvmmp = cv::imread("/mnt/c/Users/milesching/Desktop/1.png");
    cv::resize(cvmmp, cvmmp, cv::Size(151, 401));

    u_char ucmap[401][151] = {0};
    memset(ucmap, 0x7f, sizeof(ucmap));
    for(int i = 0; i < 401; ++i)
        for(int j = 0; j < 151; ++j)
            if(cvmmp.at<cv::Vec3b>(i, j)[0] > 200)
                ucmap[i][j] = 0;
    configurationSpace.updateMap(make_pair((u_char*)(void*)ucmap, make_pair(401, 151)));

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
    Node3D* nSolution = HybridAStar::Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height,
        configurationSpace, dubinsLookup);
    cout << "[" << endl;
    while(nSolution){
        cout << nSolution->getX() << ", " << nSolution->getY() << ", " << nSolution->getT() << "," << endl;
        nSolution = (Node3D*)(void*)nSolution->getPred();
    }
    cout << "]" << endl;

    return 0;
}