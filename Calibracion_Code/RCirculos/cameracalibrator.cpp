#include "cameracalibrator.h"
#include "image.h"
#include <time.h>
#include <iterator>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

#include <minpack.h>
#define realMP __minpack_real__

#define INTERVAL_INC    7
#define OFFSET_ANKUR    30
#define TOL_ANKUR       0.0001
#define MAX_INTER_ANKUR 20


#define INF (1<<30)
#define dbg(x) cout<<#x<<"="<<x<<endl
#define dbg2(x,y) cout<<#x<<"="<<x<<" "<<#y<<"="<<y<<endl

// Convertir floats a strings
template<typename T>
string num2str(T x) { stringstream ss; ss << x; return ss.str();}

template<typename T>
void saveValuesToCSV(string nameFile, vector<T> distances)
{
    ofstream file(nameFile.c_str(), ofstream::out);
    if( !file ) {
        cout << "No se pudo abrir el archivo :C" << endl;
        return;
    }

    file << "Distances\n";
    for(size_t i = 0; i < distances.size(); i++)
    {
        file << distances[i] << endl;
    }
    file.close();
}

CameraCalibrator::CameraCalibrator()
{
    pattDetector = new PatternDetector();
    actived = true;
    clearCalibrationInputs();
}

///
/// \brief CameraCalibrator::setVisualizer asigna el visualizador que se usara para el procesamiento
/// \param vis  MainWindow donde se visualizara los videos
///
void CameraCalibrator::setVisualizer(MainWindow *vis)
{
    visualizer = vis;
    pattDetector->setVisualizer(vis);
}

///
/// \brief CameraCalibrator::setActived setea el estado del procesamiento
/// \param act  Nuevo estado del procesamiento
///
void CameraCalibrator::setActived(bool act)
{
    actived = act;
}

void CameraCalibrator::setCurrentCalibrator(unsigned int calib)
{
    currCalib = calib;
}

void CameraCalibrator::setTypeFrameSelector(unsigned int selector)
{
    currFrameSelector = selector;
}

void CameraCalibrator::setShowUndistort(bool status)
{
    showUndistort = status;
}

void CameraCalibrator::setSaveCamParams(bool status)
{
    saveCamParams = status;
}

void CameraCalibrator::setSizePattern(int nRows, int nCols)
{
    numRows = nRows;//num de Filas del patron
    numCols = nCols;//num de columnas del patron
    pattDetector->setSizePattern(nRows, nCols);//seteamos el tamaño del patron
}

bool CameraCalibrator::loadVideo(std::string path)
{
    pathVideo = path;
    video.open(path);
    if (!video.isOpened())
        return false;
    return true;
}


bool isOnArray(vector<vector<int> > v,int indexVideo, int frame){
    return binary_search(v[indexVideo].begin(),v[indexVideo].end(),frame);
}

vector<vector<Point2f> > genCenters2D;
vector<vector<Point3f> > genCenters3D;
vector<Mat> vecGenRot;
vector<Mat> vecGenTras;

Mat genParamCameraMatrix;
Mat genDistorsioMatrix;
Mat genVecRot;
Mat genVecTras;

int primeraVez = 0;
int cantVeces = 1;
int cantActualizaciones = 0;
realMP* xAnte;
realMP* xAnteOri;
Size genImageSize;

int iteracionesGen = 0;

double distBetwenPoints(Point2f a, Point2f b){
//    return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
    return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
//    return (abs(a.x-b.x) + abs(a.y-b.y))*(abs(a.x-b.x) + abs(a.y-b.y));
}

void fcnWithMoreFcnSinTrRot(const int*,const int*,const realMP*,realMP*,int*);

void fcnWithMoreFcnSinTrRot(const int* m, const int* n, const realMP* x, realMP* fvec, int* iflag){
    Mat cameraMatrix = genParamCameraMatrix;
    Mat distCoef = genDistorsioMatrix;

    // seteando los valores de camara matrix
    cameraMatrix.at<double>(0,0) = x[0];
    cameraMatrix.at<double>(0,2) = x[1];
    cameraMatrix.at<double>(1,1) = x[2];
    cameraMatrix.at<double>(1,2) = x[3];

    // seteando los coeficientes de distorsion
    distCoef.at<double>(0) = x[4];
    distCoef.at<double>(1) = x[5];
    distCoef.at<double>(2) = x[6];
    distCoef.at<double>(3) = x[7];
    distCoef.at<double>(4) = x[8];

    int nFrames = genCenters2D.size();
    int nPoints = genCenters2D[0].size();

//    for(int iFrame = 0; iFrame < nFrames; iFrame++){
//        // proyectamos los puntos 3d(reales) a puntos 2d usando parametros intrisicos y coeficientes de distorsion
//        vector<Point2f> project3D2D;
//        projectPoints(genCenters3D[iFrame], vecGenRot[iFrame], vecGenTras[iFrame], cameraMatrix, distCoef, project3D2D);
//        realMP temp = 0.0;
//        for(int i = 0; i < nPoints; i++){
//            temp += distBetwenPoints(genCenters2D[iFrame][i], project3D2D[i]);
//        }
//        fvec[iFrame] = temp;
//    }

    for(int iFrame = 0; iFrame < nFrames; iFrame++){
        // proyectamos los puntos 3d(reales) a puntos 2d usando parametros intrisicos y coeficientes de distorsion
        vector<Point2f> project3D2D;
        projectPoints(genCenters3D[iFrame], vecGenRot[iFrame], vecGenTras[iFrame], cameraMatrix, distCoef, project3D2D);
        realMP err = norm(Mat(genCenters2D[iFrame]),Mat(project3D2D),NORM_L2);

//        fvec[iFrame] = (err*err)/nPoints; // f1 funciono con C03 - no funciono
//        fvec[iFrame] = err*err/nPoints*nFrames; // no funciono
//        fvec[iFrame] = err*err/sqrt(nPoints*nFrames); // f3 no funciono
        fvec[iFrame] = err/sqrt(nPoints*nFrames); // f4 no funciono no funciono de 0.295502 -> 0.314084
//        fvec[iFrame] = err; // f5 no funciono de 0.295502 -> 0.314085
        if(cantVeces == 1)
            cout << fvec[iFrame] << ",";
    }
    if(cantVeces  == 1)
        cout << endl;
    cantVeces++;

}
///
/// \brief CameraCalibrator::processingPattern realiza el procesamiento para la calibracion del patrón
///
void CameraCalibrator::processingPattern()
{
    cout<<"Procesing Pattern"<<endl;
    // Variables auxiliares
    Mat img, tmp;
    int framesTotal = 0, framesAnalyzed = 0;
    int framesPattern = 0;
    bool status = false;
    map<uint, vector<Point2f> > mapFrames;
    vector<float> distances;

    double auxtime = 0;
    double e1,e2;
    TickMeter tm;

    while (true && actived) {
        // Lectura de cada frame
        video >> img;
        if (!img.data)
            break;
        framesTotal++;
        e1 = (double)cv::getTickCount();
        vector<Point2f> keypoints; //vector vacio d keypoints
        tmp = img.clone();
        pattDetector->setImage(tmp);
        switch (pattDetector->getCurrentPattern()) {
            case PATT_CIRCLE:
                tm.start();
                status = pattDetector->processingCirclesPattern(keypoints);
                tm.stop();
                break;
            case PATT_RING:
                tm.start();
                status = pattDetector->processingRingsPattern(keypoints);
                tm.stop();
                break;
        }
         //e2 = (double)cv::getTickCount();

        // preguntamos si encontro el patron
        if(status) {
            //cout<<"time in actual frame"<<(e2-e1)/getTickFrequency();
            //auxtime = auxtime + (e2-e1);
            framesPattern++;
//            cout<<"time"<<auxtime<<endl;
            framesAnalyzed++;
            cout<<"framesAnalized"<<framesAnalyzed;
            mapFrames[framesTotal] = keypoints;
            //imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + ".png", tmp);
         }
          else {
            //imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + "_mal.png", tmp);
            continue; // Pasamos al siguiente frame
          }

        // Evaluamos si esta activada la opcion de calcular la distancia de la camara al patron
        /*if(distanceActived) {
            distances.push_back(runComputeDistance(keypoints, paramCameraMatrix, paramDistCoeffs));
        }*/
        if (waitKey(10) >= 0)
            break;
    }
    double average_time = tm.getTimeMilli() / tm.getCounter();
    auxtime = auxtime*1000;
    cout << "=====================\n";
    cout << "Total Frames: " << framesTotal << "\nFrames Analizados: " << framesAnalyzed << "\n Analisis: " << (framesAnalyzed * 1.0 / framesTotal) << "\n AVG: "<<average_time<< endl;
    cout << "=====================\n";
    cout << "tiempo promedio "<<auxtime /framesTotal;

    if(actived) {
        if(distanceActived)
            saveValuesToCSV<float>(folderOutVideo + "_Distances.csv", distances);

        // Evaluamos si se encuentra activada la opcion de calibracion de camara
        if(currCalib != CALIB_NONE) {
            vector<Mat> frames;
            vector<vector<Point2f> > centers;
            Size imageSize;
            double rms;
            // Obtenemos los frames que seran usados para la calibracion
            visualizer->visualizeMsg("Selecting frames ...");
            //selectFrames(mapFrames, frames, centers, imageSize); // Retorna los frames y los centros
            visualizer->visualizeMsg("Camera calibration ...");
            //rms = runCalibrationAndSave(imageSize, centers, frames, paramCameraMatrix, paramDistCoeffs);
            //visualizer->visualizeValue("RMS", rms);

            /*if(showUndistort) {
                Mat img, tmp;
                loadVideo(pathVideo);
                int id = 1;
                while (actived) {
                    video >> img;
                    if (!img.data)
                        break;
                    tmp = img.clone();
                    undistort(img, tmp, paramCameraMatrix, paramDistCoeffs);
                    visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(tmp), "UNDISTORT IMAGE");                   
                    string path = "/home/asran/Documentos/videosCalibracion/AnilloCirculo03/Ankur/";

                    //imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + "orifpng", tmp);
                    imwrite(path + num2str<int>(id++) + ".png", tmp);
                    if (waitKey(20) >= 0)
                        break;
                }
            }*/
        }
    }
}

///
/// \brief CameraCalibrator::initProcessing inicia el proceso de calibración de la cámara
///
void CameraCalibrator::initProcessing(unsigned int pattSelected)
{
    visualizer->cleanImage(PROC1);
    visualizer->cleanImage(PROC2);
    visualizer->cleanImage(PROC3);
    visualizer->cleanImage(PROC4);

    //visualizer->cleanImage(PROC5);
    /*visualizer->cleanImage(PROC6);
    visualizer->cleanImage(PROC7);
    visualizer->cleanImage(PROC8);
    visualizer->cleanImage(PROC9);*/

    visualizer->cleanImage(PROCFIN);

    pattDetector->setCurrentPattern(pattSelected);

    switch (pattSelected) {
    case PATT_CIRCLE:
        folderOutVideo = pathVideo.substr(pathVideo.size()-21, 17); //cojemos el substring del path
        break;
    case PATT_RING:
        folderOutVideo = pathVideo.substr(pathVideo.size()-20, 16);
        break;
    }
    processingPattern();
    clearCalibrationInputs();
}

///
/// \brief CameraCalibrator::clearCalibrationInputs limpia los parámetros de calibración
///
void CameraCalibrator::clearCalibrationInputs()
{
    isCalibrated = false;
    distanceKeypoints = 0;
    numFramesToCalibration = 0;
    pathOutput = "";
    calibFixPrincipalPoint = false;
    calibFixAspectRatio = false;
    calibZeroTangentDist = true;
    currCalib = CALIB_NONE;
    currFrameSelector = FRAMESEL_MANUAL;
    saveCamParams = false;
    showUndistort = false;
}
