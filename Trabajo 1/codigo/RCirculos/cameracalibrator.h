#ifndef CAMERACALIBRATOR_H
#define CAMERACALIBRATOR_H

#include <string>
#include <opencv2/opencv.hpp>
#include "patterndetector.h"

using namespace std;
using namespace cv;

class MainWindow;
class PatternDetector;

// Tipos de calibrador
#define CALIB_NONE      20
#define CALIB_OPENCV    21
#define CALIB_ANKUR     22

// Tipos de selector de frames
#define FRAMESEL_MANUAL     30
#define FRAMESEL_INTERVAL   31
#define FRAMESEL_RANSAC     32

class CameraCalibrator
{
private:
    PatternDetector *pattDetector;
    string pathVideo;
    string folderOutVideo;
    unsigned int numCols;
    unsigned int numRows;
    VideoCapture video;
    MainWindow *visualizer;
    bool actived;

    // Additional parameters to calibration
    bool isCalibrated;      // Estado de la calibracion
    bool showUndistort;     // Mostrar el video no distorsionado
    unsigned int currCalib; // Calibrador actual (Ninguno, Opencv o Ankur)
    unsigned int currFrameSelector;     // Selector de frames actual (Manual, por intervalos, ransac)
    bool saveCamParams;     // Flag para indicar si se guarda o no los parametros y otras salidas del proceso de calibracion

    float distanceKeypoints;
    string pathOutput;
    unsigned int numFramesToCalibration;
    bool calibFixPrincipalPoint;
    bool calibZeroTangentDist;
    bool calibFixAspectRatio;

    // Parameters to compute camera distance
    bool distanceActived;
    Mat paramCameraMatrix;
    Mat paramDistCoeffs;

    void processingPattern();
    void processingCirclesGrid();
    void processingRingsGrid();
    vector<Point3f> calcDistanceInWorld();
    void selectFrames(map<uint, vector<Point2f> > mapFrames, vector<Mat> &frames, vector<vector<Point2f> > &centers, Size &imageSize);
    double runOpenCVCalibration(Size imageSize, vector<vector<Point2f> > imagePoints, vector<vector<Point3f> > objectsPoints, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecs, vector<Mat> &tvecs, bool resetParams=true);
    double runAnkurCalibrationSinRotTras(Size imageSize, vector<vector<Point2f> > imagePoints, vector<vector<Point3f> > objectsPoints, vector<Mat> frames, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecs, vector<Mat> &tvecs);
    double runCalibrationAndSave(Size imageSize, vector<vector<Point2f> > imagePoints, vector<Mat> frames, Mat &cameraMatrix, Mat &distCoeffs);
    void saveCameraParameters(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecs, vector<Mat> &tvecs, vector<vector<Point2f> > imagePoints, double rms);
    double paramsOptimizationAnkurLMSinRotTrasWithLessFNC(vector<vector<Point2f> >, vector<vector<Point3f> >,Mat&,Mat&,vector<Mat>,vector<Mat>);
    bool readCameraParameters(string path);
    float runComputeDistance(vector<Point2f> imagePoints, Mat cameraMatrix, Mat distCoeffs);
    void RANSAC(Size imageSize,vector<vector<Point2f> > frames,double RMS,vector<int> framesId);

public:
    CameraCalibrator();

    void setVisualizer(MainWindow *vis);
    void setActived(bool act);
    void setCurrentCalibrator(unsigned int calib);
    void setTypeFrameSelector(unsigned int selector);
    void setShowUndistort(bool status);
    void setSaveCamParams(bool status);
    void setCalcDistance(bool status);
    void setFixPrincipalPoint(bool status);
    void setZeroTangentDist(bool status);
    void setFixAspectRatio(bool status);
    void setDistanceKeypoints(float size);
    void setPathOutput(string path);
    void setPathCameraParams(string path);
    void setNumFramesToCalibration(unsigned int numFrames);
    void setSizePattern(int nRows, int nCols);
    bool loadVideo(string path);
    void initProcessing(unsigned int pattSelected);
    void clearCalibrationInputs();
};

#endif // CAMERACALIBRATOR_H
