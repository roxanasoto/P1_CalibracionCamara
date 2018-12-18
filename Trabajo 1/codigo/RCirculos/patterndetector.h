#ifndef PATTERNDETECTOR_H
#define PATTERNDETECTOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "mainwindow.h"
#include "KFTracking.h"
#include "trackingGrid.h"

using namespace std;
using namespace cv;

// Parametros para visualizacion
#define PROCFIN 0
#define PROC1   1
#define PROC2   2
#define PROC3   3
#define PROC4   4
#define PROC5   5
#define PROC6   6
#define PROC7   7
#define PROC8   8
#define PROC9   9

// Parametros de seleccion de patron
#define PATT_CIRCLE 10
#define PATT_RING   11


class MainWindow;

class PatternDetector
{
private:
    Mat img;
    unsigned int currentPattern;
    unsigned int numCols;
    unsigned int numRows;
    MainWindow *visualizer;


    KFTracking* kfTracking;
    TrackingGrid* trackGrid;
    double dtKFTrac;
    double radioOptimo;

    bool foundFirstPattern;
    vector<Point2f> patternBefore;
    vector<Point2f> patternActual;
    pair<float,float> centNextPred;

    Mat adaptiveThresholdIntegralImage(Mat input);
    vector<Point2f> cleanNoiseCenters(vector<Point2f> vCenters, vector<pair<float, int> > vRadius, int maxError=0);
    vector<Point2f> findROI_circles(Mat image, Mat &imgOut);
    vector<Point2f> findROI_rings(Mat image, Mat &imgOut);
    vector<Point2f> findFinalCenters_circles(vector<Point2f> keypoints, Mat image);
    vector<Point2f> cleanNoiseUsingDistances(vector<Point2f> keypoints, Mat &imgOut);
    bool trackingCirclePoints(std::vector<Point2f>&);
    bool trackingRingsPoints(std::vector<Point2f>&);
    void drawZigZagPattern(vector<Point2f>,vector<StruSegme>,Mat,int,int);
    void sortPoints(vector<StruSegme>,vector<Point2f>&,int);

public:
    PatternDetector();

    void setCurrentPattern(unsigned int pattType);
    unsigned int getCurrentPattern();
    void setImage(Mat im);
    void setVisualizer(MainWindow *vis);
    void setSizePattern(int nRows, int nCols);
    bool processingCirclesPattern(std::vector<Point2f> &keypoints);
    bool processingRingsPattern(std::vector<Point2f> &keypoints);
    void initProcessing(unsigned int pattSelected);
};

#endif // PATTERNDETECTOR_H
