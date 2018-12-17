#ifndef KFTRACKING_H
#define KFTRACKING_H

#include <opencv2/video/tracking.hpp>
#include <vector>

using namespace std;
using namespace cv;

#define STATE_SIZE 6
#define MEASURE_SIZE 4
#define CONTROL_SIZE 0
#define TYPE_KF CV_32F

class KFTracking{
    public:
        int stateSize;
        int measSize;
        int jumpTime;
        unsigned int type;
        vector<bool> firstFound;

        KFTracking(int);
        ~KFTracking();
        void setStateInit(vector<Mat>*);
        void reboot();
        vector<Mat>* kalmanCorrection(vector<Mat>*);
        vector<Mat>* predict(double);
        vector<Mat>* futureNTime(int);

    private:
        vector<KalmanFilter>* arrayKF;
        int contrSize;

    };


#endif // KFTRACKING_H
