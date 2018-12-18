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



void CameraCalibrator::setFixAspectRatio(bool status)
{
    calibFixAspectRatio = status;
}

void CameraCalibrator::setFixPrincipalPoint(bool status)
{
    calibFixPrincipalPoint = status;
}

void CameraCalibrator::setZeroTangentDist(bool status)
{
    calibZeroTangentDist = status;
}


///
/// \brief CameraCalibrator::setSizePattern setea el tamaño del patrón
/// \param nRows    Numero de filas que tiene el patron
/// \param nCols    Numero de columnas que tiene el patron
///
void CameraCalibrator::setSizePattern(int nRows, int nCols)
{
    numRows = nRows;
    numCols = nCols;
    pattDetector->setSizePattern(nRows, nCols);
}

///
/// \brief CameraCalibrator::loadVideo se carga el video a partir de la ruta asignada
/// \param path     Ruta donde se encuentra el video
/// \return   Bool que indica si el video se cargo correctamente o no
///
bool CameraCalibrator::loadVideo(std::string path)
{
    pathVideo = path;
    video.open(path);
    if (!video.isOpened())
        return false;
    return true;
}

void ReadFiles(vector<vector<int> > &FramesIndex){
    ifstream read;
    read.open("IndexOut.txt",ios::in);
    int size,nro;
    while(read >> size){
      vector<int> v2;
      for(int i=0;i<size;i++){
        read>>nro;
        v2.push_back(nro);
      }
      FramesIndex.push_back(v2);
    }
    read.close();
}

void ReadFilesCircle(vector<vector<int> > &FramesIndex){
    ifstream read;
    read.open("IndexOutCircle.txt",ios::in);
    int size,nro;
    while(read >> size){
      vector<int> v2;
      for(int i=0;i<size;i++){
        read>>nro;
        v2.push_back(nro);
      }
      FramesIndex.push_back(v2);
    }
    read.close();
}

bool isOnArray(vector<vector<int> > v,int indexVideo, int frame){
    return binary_search(v[indexVideo].begin(),v[indexVideo].end(),frame);
}


///
/// \brief CameraCalibrator::calcDistanceInWorld calcula las posiciones 3D reales en el mundo de los centros de los patrones
/// \return  Vector de las posiciones reales de los centros (puntos 3D)
///
vector<Point3f> CameraCalibrator::calcDistanceInWorld()
{
    vector<Point3f> realCenters;

    switch (pattDetector->getCurrentPattern()) {
    case PATT_CIRCLE:
        for (size_t i = 0; i < numCols; i++)
            for (size_t j = 0; j < numRows; j++)
                realCenters.push_back(Point3f(float((2 * j + i % 2)*distanceKeypoints), float(i*distanceKeypoints), 0));
        break;
    case PATT_RING:
        for (size_t i = 0; i < numCols; ++i)
            for (size_t j = 0; j < numRows; ++j)
                realCenters.push_back(Point3f(float(j * distanceKeypoints), float(i * distanceKeypoints), 0));
        break;
    }
    return realCenters;
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
//realMP* xAnte;
//realMP* xAnteOri;
Size genImageSize;

int iteracionesGen = 0;


///
/// \brief CameraCalibrator::processingPattern realiza el procesamiento para la calibracion del patrón
///
void CameraCalibrator::processingPattern()
{
    //Variable tiempo
    TickMeter tm;
    // Variables auxiliares
    Mat img, tmp;
    int framesTotal = 0, framesAnalyzed = 0;
    bool status = false;
    map<uint, vector<Point2f> > mapFrames;
    vector<float> distances;

    cout<<"actived antes del while"<<actived<<endl;
    while (actived){
        // Lectura de cada frame
        cout<<"entro al whiile"<<endl;;
        video >> img;

        if (!img.data)
            break;
        framesTotal++;

        vector<Point2f> keypoints;
        tmp = img.clone();
        pattDetector->setImage(tmp);
        switch (pattDetector->getCurrentPattern()) {
            case PATT_CIRCLE:
                //tm.start();
                status = pattDetector->processingCirclesPattern(keypoints);
                //tm.stop();
                break;
            case PATT_RING:
                tm.start();
                status = pattDetector->processingRingsPattern(keypoints);
                tm.stop();
                visualizer->visualizetimeReal(tm.getTimeMilli() / tm.getCounter());
                break;
        }
         cout<<"status0: "<<status<<endl;
        // preguntamos si encontro el patron
        if(status ) {
            //imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + "_mal.png", tmp);
            cout<<"ACTIVATE0: "<<actived<<endl;//1
            //actived = 0;
            cout<<"status: "<<status<<endl;
            framesAnalyzed++;
            visualizer->visualizaframesReal(framesAnalyzed);
            mapFrames[framesTotal] = keypoints;
            // Pasamos al siguiente frame
//            video >> img;
            cout<<"ACTIVATE1: "<<actived<<endl;
        }
        else {

            cout<<"actived del else "<<actived<<endl;
            //status = true;
            //setActived(true);
            //actived= true;
            continue;         //imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + ".png", tmp);
        }

        if (waitKey(10) >= 0)
            break;
    }
    cout<<"TM get counter"<<tm.getCounter();
    double average_time = tm.getTimeMilli() / tm.getCounter();

    cout << "=====================\n";
    cout << "Total Frames: " << framesTotal << "\nFrames Analizados: " << framesAnalyzed << "\n% Analisis: " << (framesAnalyzed * 1.0 / framesTotal) << "\n AVG: "<<average_time<< endl;

    // Mostrar el analisis de Tiempo y Frames
    visualizer->visualizeTimeExec(framesTotal,framesAnalyzed ,(framesAnalyzed * 1.0 / framesTotal), average_time);
//    visualizer->visualizeTimeExec((framesAnalyzed * 1.0 / framesTotal));

    cout << "=====================\n";

}

///
/// \brief CameraCalibrator::initProcessing inicia el proceso de calibración de la cámara
///
void CameraCalibrator::initProcessing(unsigned int pattSelected)
{
    visualizer->cleanImage(PROC1);
    visualizer->cleanImage(PROC2);
    visualizer->cleanImage(PROC3);
    visualizer->cleanImage(PROC9);
    visualizer->cleanImage(PROCFIN);

    pattDetector->setCurrentPattern(pattSelected);

    switch (pattSelected) {
    case PATT_CIRCLE:
        folderOutVideo = pathVideo.substr(pathVideo.size()-21, 17);
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
    saveCamParams = false;
}
