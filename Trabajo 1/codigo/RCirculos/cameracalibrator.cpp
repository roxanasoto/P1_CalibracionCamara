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
        cout << "No se pudo abrir el archivo" << endl;
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

void CameraCalibrator::setCalcDistance(bool status)
{
    distanceActived = status;
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

void CameraCalibrator::setDistanceKeypoints(float size)
{
    distanceKeypoints = size;
}

void CameraCalibrator::setPathOutput(string path)
{
    pathOutput = path;
}

void CameraCalibrator::setPathCameraParams(string path)
{
    if(distanceActived)
        readCameraParameters(path);
}

void CameraCalibrator::setNumFramesToCalibration(unsigned int numFrames)
{
    numFramesToCalibration = numFrames;
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
/// \brief CameraCalibrator::selectFrames se encarga de seleccionar los frames que se usaran para la calibracion de la camara
/// \param mapFrames    Mapa de los frames enlazados con sus centros [idFrame, centros]
/// \param frames       Vector donde se almacenaran los frames seleccionados
/// \param centers      Vector de los frames seleccionados
/// \param imageSize    Tamaño de la imagen
///
void CameraCalibrator::selectFrames(map<uint, vector<Point2f> > mapFrames, vector<Mat> &frames, vector<vector<Point2f> > &centers, Size &imageSize)
{
    centers.clear();
    frames.clear();

    // Selección de frames
    set<int> mySetFrames;
    switch (currFrameSelector) {
        case FRAMESEL_MANUAL:
        {
            vector<vector<int> > framesVideos;
            // 00 se deja vacio porque no hay video A00
            framesVideos.push_back(vector<int>());

            if(pattDetector->getCurrentPattern() == PATT_RING){
                cout << "Entro al Manual Anillos" << endl;
                // El video es de Anillos
                // A01
                int myFrames1[] = {44,56,75,77,81,
                                                  117,120,128,188,256,
                                                  266,288,297,308,402,
                                                  439,480,587,599,608,
                                                  656,664,691,730,762};
                /*int myFrames1[] = {16,36,46,74,84,
                                  159,338,396,474,507,
                                  526,541,555,562,574,
                                  587,617,650,665,675,
                                  709,773,808,824,850};*/
                framesVideos.push_back(vector<int>(myFrames1,myFrames1+25));

                //A02 (falta cargar elementos)
                //framesVideos.push_back(vector<int>());

                // A03
                int myFrames3[80] = {1,2,4,6,8,138,276,288,301,363,426,463,501,551,563,576,588,601,703,779,829,879,985,997,1010,1022,1035,1074,1113,1125,1138,1179,1220,1232,1245,1257,1270,1282,1295,1320,1345,1357,1370,1395,1420,1432,1445,1457,1470,1482,1495,1507,1520,1604,1629,1654,1679,1704,1729,1807,1883,1908,1933,1958,1983,2008,2033,2083,2108,2133,2183,2208,2233,2333,2358,2383,2408,2483,2608,2708};
                //framesVideos.push_back(vector<int>(myFrames3,myFrames3+76));

                // A04
                int myFrames4[89] = {1,4,8,11,15,18,22,25,29,32,36,45,52,60,68,70,75,82,89,96,103,110,120,131,138,146,160,175,189,203,247,276,283,290,297,304,312,319,326,333,343,357,364,369,371,374,378,381,385,388,392,420,443,447,451,465,480,483,487,490,494,497,501,504,508,511,515,518,522,543,550,572,579,586,593,602,624,645,659,666,680,695,702,709,730,737,751,773,788};
                //framesVideos.push_back(vector<int>(myFrames4,myFrames4+89));

                // A05
                int myFrames5[84] = {1,8,16,23,31,38,46,61,76,91,106,113,121,128,136,143,151,158,166,173,181,188,196,203,211,226,318,333,357,363,370,378,393,398,433,440,448,463,470,478,485,493,500,508,515,523,530,538,569,584,599,629,644,659,674,689,704,719,764,779,869,884,929,959,974,989,1049,1064,1169,1184,1259,1274,1289,1319,1349,1364,1394,1409,1424,1439,1484,1514,1529,1544};
                //framesVideos.push_back(vector<int>(myFrames5,myFrames5+84));

            } else {
                cout << "Entro al Manual Circulos" << endl;
                // C01
                int myFrames1[25] = {22,33,45,71,93,
                                    99,115,127,143,175,
                                    184,191,200,223,238,
                                    246,254,261,284,300,
                                    323,335,365,386,591};
                framesVideos.push_back(vector<int>(myFrames1,myFrames1+25));
                // C02
                framesVideos.push_back(vector<int>());
                // C03
                int myFrames3[72] = {1,2,12,21,27,30,31,57,59,71,76,95,103,123,128,130,149,157,162,178,180,189,200,216,262,264,270,273,275,276,277,278,406,457,463,466,467,480,578,582,611,620,622,698,703,717,727,733,871,998,1229,1244,1254,1263,1316,1320,1333,1335,1346,1389,1395,1462,1463,1488,1596,1612,1673,1678,1683,1711,1726,1754};
                framesVideos.push_back(vector<int>(myFrames3,myFrames3+72));
                // C04
                int myFrames4[75] = {1,3,21,29,39,43,65,73,74,75,84,89,135,148,164,176,187,188,199,201,203,220,268,275,283,293,297,300,301,302,305,309,320,324,330,336,345,348,352,356,360,372,386,387,388,396,404,412,421,430,438,447,461,472,476,482,495,503,511,523,543,555,561,570,579,589,594,605,611,620,625,642,676,691,706};
                framesVideos.push_back(vector<int>(myFrames4, myFrames4 + 75));
                // C05
                int myFrames5[75] = {4,13,26,45,60,
                                     69,79,90,103,114,
                                     118,127,138,149,159,
                                     172,182,190,202,219,
                                     231,254,262,271,290,
                                     300,356,391,406,422,
                                     430,446,458,466,476,
                                     498,518,527,570,601,
                                     616,648,675,698,704,
                                     712,730,758,771,818,
                                     905,920,954,988,1005,
                                     1017,1035,1048,1091,1098,
                                     1136,1165,1227,1240,1272,
                                     1286,1376,1391,1404,1482,
                                     1501,1524,1577,1657,1685};
                framesVideos.push_back(vector<int>(myFrames5, myFrames5 + 75));
            }
            int offset = folderOutVideo[folderOutVideo.size()-1]-'0';
            for(size_t i=0; i < numFramesToCalibration; i++) {
                mySetFrames.insert(framesVideos[offset][i]);
            }
            break;
        }
        case FRAMESEL_INTERVAL:
        {
            vector<vector<int> > FramesIndex;
            switch (pattDetector->getCurrentPattern()) {
                case PATT_CIRCLE:
                    ReadFilesCircle(FramesIndex);
                    break;
                case PATT_RING:
                    ReadFiles(FramesIndex);
                    break;
            }
            int MAX_FRAMES = FramesIndex[0][folderOutVideo[folderOutVideo.size()-1]-'0'-1]; // Ojo index 1-0 2-1 3-2 4-3 sacado del nombre del archivo cargado
            int intervalo = (numFramesToCalibration == 0) ? 0 : MAX_FRAMES / numFramesToCalibration;
            int cont = 0;
            for (map<uint, vector<Point2f> >::iterator it = mapFrames.begin(); it != mapFrames.end(); ++it) {
                cont++;
                if((cont % intervalo) == INTERVAL_INC && !isOnArray(FramesIndex,folderOutVideo[folderOutVideo.size()-1]-'0'-1, it->first)) {
                    mySetFrames.insert(it->first);
                }
                if(mySetFrames.size() >= numFramesToCalibration)
                    break;
            }
            break;
        }
        case FRAMESEL_RANSAC:
        {
            // Elegir frames con Ransac
            void RANSAC(Size imageSize,vector<vector<Point2f> > frames,double RMS,vector<int> framesId);
            //RANSAC();
            break;
        }
    }
    // Entregamos los centros elegidos y los frames seleccionados
    int currFrame = 0;
    Mat img;
    loadVideo(pathVideo);
    int idx=0;
    while (true && actived) {
        video >> img;
        if (!img.data)
            break;
        currFrame++;
        imageSize = img.size();

        if(mySetFrames.find(currFrame) != mySetFrames.end() && mapFrames[currFrame].size() == numRows*numCols) {
            frames.push_back(img.clone());              // Pasas la imagen
            centers.push_back(mapFrames[currFrame]);    // Pasas los centros
            cout << "FRAME: " << currFrame << " -> SIZE CENTERS: " << mapFrames[currFrame].size() << endl;
            //dbg2(mapFrames[currFrame].size(),currFrame);
            // Invirtiendo el color del frame que se esta considerando para la calibracion
            //bitwise_not(img,tmp);
            //imshow("FrameToCalibration", tmp);
            // Guardando el frame utilizado para la calibracion
            imwrite("/home/uburoxana/Documentos/Imagenes/PROYECTOFINAL_CALIBRACION/PatronCircular/build-RCirculos-Desktop-Debug/PatronCircular/frame_elegidos_1_" + num2str<int>(idx) + ".png", img);
            idx++;
        }
    }
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

///
/// \brief CameraCalibrator::runOpenCVCalibration   se encarga de realizar la calibracion usando el metodo de OpenCV directamente
/// \param imageSize        Tamaño de la imagen
/// \param imagePoints      Vector que contiene los centros 2D de los frames seleccionados para la calibracion
/// \param objectsPoints    Vector que contiene los centros 3D de los centros del patron en el mundo
/// \param cameraMatrix     Matriz donde se almacenara la matriz de camara obtenida en el proceso de calibracion
/// \param distCoeffs       Vector que almacenara los coeficientes de distorsion encontrados
/// \param rvecs            Vector que almacenara el vector de rotacion encontrado
/// \param tvecs            Vector que almacenara el vector de traslacion encontrado
/// \param resetParams      Booleano que indica si se resetea o no la camara de la matriz y los coeficientes de distorsion
/// \return     valor del RMS obtenido
///
double CameraCalibrator::runOpenCVCalibration(Size imageSize, vector<vector<Point2f> > imagePoints, vector<vector<Point3f> > objectsPoints, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecs, vector<Mat> &tvecs, bool resetParams)
{
    int flag = 0;
    if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if (calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
    if (calibFixAspectRatio)    flag |= CALIB_FIX_ASPECT_RATIO;

    // Si se desea setear la matriz de la camara y los coeficientes de distorsion
    if(resetParams) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        // Si tiene aspect ratio fijo
        if(flag & CALIB_FIX_ASPECT_RATIO)
            cameraMatrix.at<double>(0,0) = 1.0;
        distCoeffs = Mat::zeros(8, 1, CV_64F);
    }
    double rms = calibrateCamera(objectsPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);
    isCalibrated = checkRange(cameraMatrix) && checkRange(distCoeffs);
    return rms;
}

///
/// \brief distort Funcion que se encarga de calcular los puntos 2D distorsionados a partir de puntos 2D no distorsionados
/// \param point        Vector de puntos 2D que seran distorsionados
/// \param cameraMatrix Matriz de calibracion de la camara (parametros intrinsecos)
/// \param distCoeffs   Vector que tiene los coeficientes de distorsion que seran usados
/// \return     Vector de nuevos puntos 2D distorsionados
///
vector<Point2f> distort(vector<Point2f> point, const Mat cameraMatrix, const Mat distCoeffs)
{
    vector<Point2f> ans;
    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
    double cx = cameraMatrix.at<double>(0,2);
    double cy = cameraMatrix.at<double>(1,2);

    double k1 = distCoeffs.at<double>(0,0);
    double k2 = distCoeffs.at<double>(0,1);
    double p1 = distCoeffs.at<double>(0,2);
    double p2 = distCoeffs.at<double>(0,3);
    double k3 = distCoeffs.at<double>(0,4);

    for(size_t i = 0; i < point.size(); i++) {

        double x = (point[i].x - cx) / fx;
        double y = (point[i].y - cy) / fy;

        double r2 = x*x + y*y;

        // Radial distorsion
        double xDistort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
        double yDistort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

        // Tangential distorsion
        xDistort = xDistort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
        yDistort = yDistort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

        // Back to absolute coordinates.
        xDistort = xDistort * fx + cx;
        yDistort = yDistort * fy + cy;

        ans.push_back(Point2f(xDistort, yDistort));
    }
    return ans;
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

void CameraCalibrator::paramsOptimizationAnkurLMSinRotTrasWithLessFNC(vector<vector<Point2f> > centers2D,
                                                           vector<vector<Point3f> > centers3D,
                                         Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> rvecs, vector<Mat> tvecs) {
              cout << " tTrasWithLessFNC " << endl;

              int nFrames  = centers2D.size();
              int m = nFrames;
              int n = 4 + // parametros intrinsecos o camaraMatrix (fx,fy,cx,cy)
                      5; // coeficientes de distorsión (k1,k2,p1,p2,k3)
              int one = 1, info;
              int lwa = m*n + 5*n + m + 10;
              int* iwa = new int[n];

              realMP ftol, fnorm;
              realMP* x = new realMP[n];
              realMP* fvec = new realMP[m];
              realMP* wa = new realMP[lwa];

              xAnte = new realMP[n];
              xAnteOri = new realMP[n];

              cout << " tamanio de m " << m << endl;
              cout << " tamanio de n " << n << endl;

              // pasando los valores de fx, fy, cx, cy al x;
              x[0] = cameraMatrix.at<double>(0,0);
              x[1] = cameraMatrix.at<double>(0,2);
              x[2] = cameraMatrix.at<double>(1,1);
              x[3] = cameraMatrix.at<double>(1,2);

              // pasando los valores de coeficientes de distorsion al x
              x[4] = distCoeffs.at<double>(0);
              x[5] = distCoeffs.at<double>(1);
              x[6] = distCoeffs.at<double>(2);
              x[7] = distCoeffs.at<double>(3);
              x[8] = distCoeffs.at<double>(4);

              for(int i = 0; i < n; i++){
                  xAnte[i] = x[i];
                  xAnteOri[i] = x[i];
              }

              // vector de traslación y rotación, matriz de camara y coeficientes de distorsion
              genParamCameraMatrix = cameraMatrix.clone();
              genDistorsioMatrix = distCoeffs.clone();
              vecGenRot = rvecs;
              vecGenTras = tvecs;
              genCenters2D = centers2D;
              genCenters3D = centers3D;

              // copiando valores antes de la modificacion de variables hecha por Levenberg Marquardt
              cantVeces = 1;
              ftol = sqrt(__minpack_func__(dpmpar)(&one));              
              __minpack_func__(lmdif1)(&fcnWithMoreFcnSinTrRot, &m, &n, x, fvec, &ftol, &info, iwa, wa, &lwa);
              fnorm = __minpack_func__(enorm)(&m, fvec);

              int tempCant = 0;
              for(int i = 0; i < n; i++)
                  if(abs(xAnteOri[i] - x[i]) >= 0.0001)
                      tempCant++;

              cout << " valor de la tolerancia ===> " << ftol << endl;
              //    cout << "LevenbergMarquart sin Rotacion&Traslacion cantidadIteraciones " << cantVeces << endl;
              //    cout << "LevenbergMarquart sin Rotacion&Traslacion cantidadCambios en todo el metodo " << cantActualizaciones << endl;
              //    cout << "LevenbergMarquart sin Rotacion&Traslacion cantidaVariables que cambiaron " << tempCant << endl;

//              cantVeces = 1;
//              cantActualizaciones = 0;

              // guardando el resultado despues de la optimización
              cameraMatrix = genParamCameraMatrix.clone();
              distCoeffs = genDistorsioMatrix.clone();

              // OBTIENEDO LOS RMS DE ESTA FUNCIÓN
              double rmsLM = 0.0;
              //rmsIni = runOpenCVCalibration(imageSize, imagePoints, objectsPoints, genParamCameraMatrix, genDistorsioMatrix, vecGenRot, vecGenTras);

//              int nPoints = numCols * numRows;
//              for(int iFrame = 0; iFrame < nFrames; iFrame++){
//                  // proyectamos los puntos 3d(reales) a puntos 2d usando parametros intrisicos y coeficientes de distorsion
//                  vector<Point2f> project3D2D;
//                  projectPoints(genCenters3D[iFrame], vecGenRot[iFrame], vecGenTras[iFrame], cameraMatrix, distCoef, project3D2D);
//                  double err = norm(Mat(genCenters2D[iFrame]),Mat(project3D2D),NORM_L2);
//                  fvec[iFrame] = (err*err)/nPoints;
//              }

              cout << " valor de la tolerancia " << ftol << endl;
              printf("      FINAL L2 NORM OF THE RESIDUALS%15.7f\n\n",fnorm);
              printf("      EXIT PARAMETER                %10i\n\n", info);

              //return rmsLM;
}

double computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs,
    vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); ++i)
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err / n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}
///new
void create_real_pattern(int h, int w, vector<Point3f>& out_real_centers){

    float margin_h = 70;//50
    float margin_w = 90;
    float distance_points = 110;
    out_real_centers.clear();
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h), 0));

    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h+distance_points*1), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h+distance_points*1), 0));

    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h+distance_points*2), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h+distance_points*2), 0));

    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*0) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*1) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*2) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*3) ,float( margin_h+distance_points*3), 0));
    out_real_centers.push_back(Point3f(  float(margin_w+distance_points*4) ,float( margin_h+distance_points*3), 0));


    Mat real_points_img = Mat::zeros(Size(h,w), CV_8UC3);
    for(int i=0;i<out_real_centers.size();i++){
        circle(real_points_img, Point2f(out_real_centers[i].x,out_real_centers[i].y), 2,  Scalar( 0, 0, 255 ) , 2);
    }
    //save_frame(PATH_DATA_FRAMES,"ideal image", real_points_img);
}
///
/// \brief CameraCalibrator::runAnkurCalibrationSinRotTras realiza la calibracion usando el método propuesto en el paper de Ankur
/// \param imageSize        Tamaño de la imagen
/// \param imagePoints      Vector que contiene los centros 2D de los frames seleccionados para la calibracion
/// \param objectsPoints    Vector que contiene los centros 3D de los centros del patron en el mundo
/// \param frames           Frames seleccionados para realizar la calibracion
/// \param cameraMatrix     Matriz donde se almacenara la matriz de camara obtenida en el proceso de calibracion
/// \param distCoeffs       Vector que almacenara los coeficientes de distorsion encontrados
/// \param rvecs            Vector que almacenara el vector de rotacion encontrado
/// \param tvecs            Vector que almacenara el vector de traslacion encontrado
/// \return     valor del RMS obtenido
///
double CameraCalibrator::runAnkurCalibrationSinRotTras(Size imageSize, vector<vector<Point2f> > imagePoints, vector<vector<Point3f> > objectsPoints, vector<Mat> frames, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecs, vector<Mat> &tvecs)
{
    Size templateSize;
    // Generando las posiciones del template
    vector<Point2f> dstPoints;
    vector<Point2f> control_points_2d;
    float margin_h= 70;
    float margin_w = 90;
    float distance_points = 110;
    float dist = 44;//OFFSET_ANKUR;
    templateSize.width = dist * (numCols + 1);
    switch (pattDetector->getCurrentPattern()) {
        case PATT_CIRCLE:
            templateSize.height = dist * (2 * numRows + 1);
            for (uint j = numCols; j > 0; j--)
                for (size_t i = 1; i <= numRows; i++)
                    dstPoints.push_back(Point2f(j * dist, (2 * i - 1 + (j-1) % 2) * dist));
            break;
        case PATT_RING:
            templateSize.height = dist * (numRows + 1);
            for (uint j = numCols; j > 0; j--)
                for (size_t i = 1; i <= numRows; i++)
                    //dstPoints.push_back(Point2f(j * dist, i * dist));
                    control_points_2d.push_back(Point2f(margin_w + distance_points * j,margin_h + distance_points * i));
            break;
    }

    vector<double> vRms;
    // Calculo de los parametros iniciales para Ankur
    int numIterations = 0;
    double rmsIni, newRms1=0, newRms2, minRms;

    rmsIni = runOpenCVCalibration(imageSize, imagePoints, objectsPoints, cameraMatrix, distCoeffs, rvecs, tvecs);
    minRms = rmsIni;
    newRms2 = rmsIni;
    vRms.push_back(rmsIni);

    // Variables que almacenaran los parametros del RMS menor
    Mat minCameraMatrix = cameraMatrix.clone(),
        minDistCoeffs = distCoeffs.clone();
    vector<Mat> minRvecs = rvecs, minTvecs = tvecs;

    cout << "=====================\n";
    cout << "RatioTemplate: " << templateSize << endl;
    cout << "FramesCalibration: " << numFramesToCalibration << endl;
    cout << "Frames Selected: " << frames.size() << endl;

    iteracionesGen = 1;

    //while(abs(newRms2 - newRms1) > TOL_ANKUR && numIterations < MAX_INTER_ANKUR) {
    while(numIterations < MAX_INTER_ANKUR) {
        Mat frameSize;

        //newRms1 = newRms2;
        numIterations++;

        Mat imgFronPar, imgUnd, aux, H, Hi;
        vector<vector<Point2f> > newCenters;
        vector<Point2f> tmpCenters(numCols * numRows);
        vector<Point2f> tmp1Centers(numCols * numRows);

        bool status = false;
        int cont = 0;

        vector<Point3f> real_centers;
        int h = numCols;
        int w = numRows;
        create_real_pattern(h,w, real_centers);

        //cout<<"imagePoints0"<<imagePoints[0]<<endl;
        //cout<<"imagePoints0"<<imagePoints[0][0] <<endl;

        for(size_t i = 0; actived && i < frames.size(); i++) {
            frameSize = frames[i].clone();
            // Undistort el frame actual
            undistort(frames[i], imgUnd, cameraMatrix, distCoeffs);
            //imwrite(folderOutVideo + "_undistort_" + num2str<int>(i) + ".png", imgUnd);
            imshow("Undistort", imgUnd);
            // Hallando la matriz de homografia para corregir la perspectiva
            H = findHomography(imagePoints[i],real_centers);
            Hi= findHomography(real_centers,imagePoints[i]);
            //H = findHomography(imagePoints[i], dstPoints, noArray(), CV_RANSAC);
            //Hi = findHomography(dstPoints,imagePoints[i], noArray(),CV_RANSAC);
            //H = getPerspectiveTransform( inputQuad, outputQuad );
            // Generando la imagen fronto-paralela con la matriz H
            //warpPerspective(imgUnd, imgFronPar, H, templateSize);
            warpPerspective(imgUnd, imgFronPar, H, frameSize.size());

            //H = getPerspectiveTransform( outputQuad, inputQuad );
            imwrite(folderOutVideo +"/"+ folderOutVideo + "_frontoparalelo_" + num2str<int>(i) + ".png", imgFronPar);
            imshow("Homography transform", imgFronPar);
            // Calculamos los centros del patron canonico (frontoparalelo) ? Puntos de control??
            pattDetector->setImage(imgFronPar.clone());
            switch (pattDetector->getCurrentPattern()) {
                case PATT_CIRCLE:
                    status = pattDetector->processingCirclesPattern(tmp1Centers);
                    break;
                case PATT_RING:
                    status = pattDetector->processingRingsPattern(tmp1Centers);
                    cout<<"status"<<status<<endl;
                    break;
            }
            if(status  && tmp1Centers.size() == numRows * numCols) {
                cont++;
            }
            else {
                cout << "InvalidFrame: i " << i+1 <<  endl;
            }

            visualizer->visualizeImage(PROC5, ImageHelper::convertMatToQimage(imgUnd), "Undistort");
            visualizer->visualizeImage(PROC6, ImageHelper::convertMatToQimage(imgFronPar), "Fronto-parallel");
            aux = imgFronPar.clone();
            // Dibujar circulos de color
            for(size_t i =0; i < tmp1Centers.size(); i++) {
                circle(aux, tmp1Centers[i], 3, Scalar(255,255,0), -1);
            }
            visualizer->visualizeImage(PROC7, ImageHelper::convertMatToQimage(aux.clone()), "Centers fronto-parallel");

            // Reproyeccion de los centros del frontoparalelo a la imagen sin distorsion
            //perspectiveTransform(tmp1Centers, tmpCenters, H.inv());
            perspectiveTransform(tmp1Centers, tmpCenters, Hi);

            aux = imgUnd.clone();
            for(size_t i =0; i < tmpCenters.size(); i++) {
                circle(aux, tmpCenters[i], 3, Scalar(255,255,0), -1);
            }
            imshow("Reprojection", aux);
            visualizer->visualizeImage(PROC8, ImageHelper::convertMatToQimage(aux.clone()), "Reprojection (no distortion)");
            tmp1Centers = distort(tmpCenters,cameraMatrix,distCoeffs);
            newCenters.push_back(tmp1Centers);

            aux = frames[i].clone();
            for(size_t j = 0; j < tmp1Centers.size(); j++){
                circle(aux, imagePoints[i][j], 1, Scalar(255, 255, 255), -1); // Puntos de la imagen original (con distorsion)
                circle(aux, tmp1Centers[j], 1, Scalar(255, 0, 255), -1);    // Centros reprojectados con distorsion
            }
            imshow("Distorsion", aux);
            visualizer->visualizeImage(PROC9, ImageHelper::convertMatToQimage(aux.clone()), "Reprojection (distortion)");
            visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(aux.clone()), "Comparing centers");
            //imwrite(folderOutVideo + "_Ankur_" + num2str<int>(i) + "_iter_" + num2str<int>(numIterations) + ".png", aux);

            waitKey(200);


        }

        vector<vector<Point2f> > newCentersGen;
        vector<vector<Point3f> > objectsPointsGen;
        vector<Mat> trasVecGen;
        vector<Mat> rotVecGen;
        for(int i = 0; i < (int)frames.size(); i++){
            if((int)newCenters[i].size() == numCols * numRows){
                newCentersGen.push_back(newCenters[i]);
                objectsPointsGen.push_back(objectsPoints[i]);
                trasVecGen.push_back(tvecs[i]);
                rotVecGen.push_back(rvecs[i]);
            }
        }
        //Obteniendo el RMS inicial
        cout << " RMS1: " << rmsIni  << endl;
        //Obteneindo los nuevos RMS
        double newRms1 = calibrateCamera(objectsPointsGen, newCentersGen, imageSize, cameraMatrix, distCoeffs, rotVecGen, trasVecGen, 0);
        cout << " cameraMatrix = \n " << cameraMatrix << endl;
        cout << " distorsion Coeff = \n  " << distCoeffs << endl;
        cout << " newRMS1: " << newRms1 << endl;
        vRms.push_back(newRms1);

        //newRms1 = calibrate_camera(objectsPoints, cameraMatrix, distCoeffs, imagePoints);

        // Levenberg - Marquardt
        // obteniendo los frames adecuados:
        /*cout << " # de iteracion " << numIterations << endl;
        cout << "==== Antes de LM ====" << endl;
        cout << " cameraMatrix = \n " << cameraMatrix << endl;
        cout << " distorsion Coeff = \n  " << distCoeffs << endl;
        cout << " newRMS1: " << newRms1 << endl;*/

        // PARA VISUALIZAR LOS PUNTOS
//        for(int j = 0; j < frames.size(); j++){
//            Mat frameTemp = frames[0].clone();
//            for(int i = 0; i < newCenters[0].size(); i++){
//                circle(frameTemp, imagePoints[i][j], 5, Scalar(0,0,255), -1);
//                circle(frameTemp, imagePoints[i][j], 5, Scalar(0,255,255), -1);
//            }
//        }

//        paramsOptimizationAnkurLMSinRotTrasWithLessFNC(newCentersGen, objectsPointsGen, cameraMatrix, distCoeffs, rotVecGen, trasVecGen);
        //vector<float> perViewErrors;
        //newRms2 = computeReprojectionErrors(objectsPointsGen, newCentersGen, rotVecGen, trasVecGen, cameraMatrix, distCoeffs, perViewErrors);

        //newRms2 = paramsOptimizationAnkurLMSinRotTrasWithLessFNC(newCentersGen, objectsPointsGen, cameraMatrix, distCoeffs, rotVecGen, trasVecGen);

        // Nuevamente calibracion con parametros optimizados
        /*cout << "==== Despues de LM ====" << endl;
        cout << " cameraMatrix = \n" << cameraMatrix << endl;
        cout << " distorsion Coeff =\n" << distCoeffs << endl;
        cout << " RMS2: " << newRms2 << endl << endl;*/

        //iteracionesGen++;

        //vRms.push_back(newRms2);

        /*if(newRms2 < minRms) {
            minRms = newRms2;
            minCameraMatrix = cameraMatrix.clone();
            minDistCoeffs = distCoeffs.clone();
            minRvecs = rotVecGen;
            minTvecs = trasVecGen;
        }*/
        //cout << "FRAMES ENVIADOS: " << cont << " / " << numFramesToCalibration << endl;
       // cout << "\n% Analisis: " << (cont * 100.0 / numFramesToCalibration) << "\nDiffRMS: " << abs(newRms2 - newRms1) << endl << endl;
    }
    saveValuesToCSV<double>(folderOutVideo + "/" + folderOutVideo + "_RMSs.csv", vRms);

    /*cout << "\n=====================\n";
    cout << "ANKUR Method:\n -NumIterations: " << numIterations << "\n -Tolerance: " << TOL_ANKUR << "\n -OurRMS: " << rmsIni << "\n -MinRMS: " << minRms << endl;
    cout << "=====================\n";*/
    return newRms1;

}


///
/// \brief CameraCalibrator::runCalibrationAndSave realiza la calibracion de la camara y guarda los datos en un archivo XML
/// \param imageSize    Tamaño de la imagen
/// \param imagePoints  Vector de vector de los centros de los frames seleccionados para calibrar la camara
/// \param cameraMatrix Matriz de salida con los parametros de la camara (distancia focal y centro optico)
/// \param distCoeffs   Vector con los coeficientes de distorsion
///
double CameraCalibrator::runCalibrationAndSave(Size imageSize, vector<vector<Point2f> > imagePoints, vector<Mat> frames, Mat &cameraMatrix, Mat &distCoeffs)
{
    double rms;
    vector<Mat> rvecs, tvecs;
    vector<vector<Point3f> > objectsPoints(1);
    objectsPoints[0] = calcDistanceInWorld();
    objectsPoints.resize(imagePoints.size(), objectsPoints[0]);

    switch (currCalib) {
        case CALIB_OPENCV:
            rms = runOpenCVCalibration(imageSize, imagePoints, objectsPoints, cameraMatrix, distCoeffs, rvecs, tvecs);
            break;
        case CALIB_ANKUR:
            rms = runAnkurCalibrationSinRotTras(imageSize, imagePoints, objectsPoints, frames, cameraMatrix, distCoeffs, rvecs, tvecs);
            break;
        default:
            return 0;
    }
    if(isCalibrated && saveCamParams) {
        saveCameraParameters(imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, imagePoints, rms);

        // Guardamos una imagen del area cubierta por los centros de los frames seleccionados
        Mat imgOut = Mat::zeros(imageSize, CV_8UC3);
        bitwise_not(imgOut,imgOut);
        for(size_t i = 0; i < imagePoints.size(); i++) {
            for(size_t j=0; j<imagePoints[i].size(); j++)
                circle(imgOut, imagePoints[i][j], 5, Scalar(0,0,255), -1);
        }
        imwrite(folderOutVideo + "/" + "AreaCalibracion_" + folderOutVideo + "_" + num2str<int>(numFramesToCalibration) + ".png", imgOut);
        imshow(folderOutVideo + "_" + "AreaCalibracion", imgOut);
    }
    return rms;
}

///
/// \brief CameraCalibrator::saveCameraParameters   almacena los parametros enviados como parametros en un archivo XML
/// \param imageSize        Tamaño de la imagen
/// \param cameraMatrix     Matriz de calibracion de la camara (parametros intrinsecos)
/// \param distCoeffs       Coeficientes de distorsion de la camara
/// \param rvecs            Vector de rotacion
/// \param tvecs            Vector de traslacion
/// \param imagePoints      Vector de los centros de los frames seleccionados para la calibracion de la camara
/// \param rms              RMS obtenido al calibrar la camara
///
void CameraCalibrator::saveCameraParameters(Size &imageSize, Mat &cameraMatrix, Mat &distCoeffs, vector<Mat> &rvecs, vector<Mat> &tvecs, vector<vector<Point2f> > imagePoints, double rms)
{
    FileStorage fs(folderOutVideo + "/" + pathOutput, FileStorage::WRITE);

    time_t tm;
    time(&tm);
    struct tm *t2 = localtime(&tm);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;
    fs << "num_of_frames" << (int)numFramesToCalibration;
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "num_cols" << (int)numCols;
    fs << "num_rows" << (int)numRows;
    fs << "distance_centers" << distanceKeypoints;

    if (calibFixPrincipalPoint) fs << "flag01" << "fix_principal_point";
    if (calibZeroTangentDist)   fs << "flag02" << "zero_tangent_dist";
    if (calibFixAspectRatio)    fs << "flag03" << "fix_aspectRatio";

    fs << "Total_RMS" << rms;
    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    if (!rvecs.empty() && !tvecs.empty())
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int)rvecs.size(); i++)
        {
            Mat r = bigmat(Range(i, i + 1), Range(0, 3));
            Mat t = bigmat(Range(i, i + 1), Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if (!imagePoints.empty())
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int)imagePoints.size(); i++)
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

///
/// \brief CameraCalibrator::readCameraParameters realiza la lectura del XML que contiene los parametros de calibracion de la camara
/// \param path     Ruta donde se encuentra el archivo XML
/// \return     Booleano que indica si se cargo correctamente o no el archivo
///
bool CameraCalibrator::readCameraParameters(string path)
{
    FileStorage fs(path, FileStorage::READ);
    if(!fs.isOpened()) {
        visualizer->visualizeMsg("Archivo de calibracion no encontrado");
        return false;
    }
    fs["num_of_frames"] >> (int)numFramesToCalibration;
    fs["distance_centers"] >> distanceKeypoints;
    fs["Camera_Matrix"] >> paramCameraMatrix;
    fs["Distortion_Coefficients"] >> paramDistCoeffs;
    fs.release();
}

///
/// \brief CameraCalibrator::runComputeDistance realiza el calculo de la distancia entre el patron y la camara
/// \param imagePoints      Array de los centros detectados en el frame actual
/// \param cameraMatrix     Matriz 3x3 de la camara calculada previamente
/// \param distCoeffs       Vector 5x1 de los coeficientes de distorsion calculados previamente
/// \return     Distancia entre el patrón y la cámara en el frame actual
///
float CameraCalibrator::runComputeDistance(vector<Point2f> imagePoints, Mat cameraMatrix, Mat distCoeffs)
{
    Mat rvec, tvec, dstRvec;
    vector<Point3f> objectsPoints = calcDistanceInWorld();
    float distance = 0;

    // Calculamos los parametros extrinsecos y obtenemos RVEC y TVEC (vectores 3x1)
    if(solvePnP(objectsPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec)) {
        // Convierte el vector 3x1 a una matriz 3x3
        Rodrigues(rvec, dstRvec);
        // Transpuesta de la matriz de rotacion
        transpose(dstRvec, dstRvec);
        distance = norm(tvec);
        cout << "||-R^T * t||: " << norm(-dstRvec * tvec) << " == ||T||: " << distance << endl;
        visualizer->visualizeValue("Distancia(mm)", distance);
    }
    return distance;
}

void CameraCalibrator::RANSAC(Size imageSize,vector<vector<Point2f> > frames,double RMS,vector<int> framesId){
    // Escogemos 100 frames distribuidos en todo el video
    vector<vector<Point2f> > frames_aux;
    vector<int> frames_auxId;
    int m = 100; //numero de muestras que se van a escoger como poblacion
    int interval = frames.size()/m;
    //dbg(interval);
    //dbg(frames.size());
    //dbg(framesId.size());
    for(int i=0;i<frames.size();i+=interval){
        frames_aux.push_back(frames[i]);
        frames_auxId.push_back(framesId[i]);
    }

    frames   = frames_aux;
    framesId = frames_auxId;

    cout << "Entro al algoritmo de Ransac con un RMS = " << RMS << endl;
    vector<Mat> rvecs, tvecs;

    int flag = 0;
    if (calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
    if (calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
    if (calibFixAspectRatio)    flag |= CALIB_FIX_ASPECT_RATIO;

    // Empezando la calibracion, creamos una matriz 3x3 para los parametros intrinsecos de la camara
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    // Si tiene aspect ratio fijo
    if(flag & CALIB_FIX_ASPECT_RATIO)
        cameraMatrix.at<double>(0,0) = 1.0;
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);

    // Aqui empieza el algoritmo de RANSAC

    int n=25;  //Numero de elementos que se toman como muestra
    int k = 10; // Numero de iteraciones
    int MOD = frames.size();
    double t = RMS; // RMS inicial
    int d = numFramesToCalibration;
    int iterations = 0;

    vector<vector<Point2f> > bestfit; // Mejor conjunto representativo, inicialmente vacio
    vector<int> bestfitId;
    double besterr = INF;
    double bestSubSetRank = 1.0;
    //dbg(frames.size());

    while(iterations < k){
        //dbg(iterations);
        // Tenemos que escoger n frames aleatoriamente, escogemos los Id's
        vector<int> ids; // arreglo de id seleccionados aleatoriamente
        set<int> setp;
        int nro;
        while(ids.size()<n){
            nro = rand()%MOD;
            if(setp.find(nro)==setp.end()){ // Verificas que no esten repetidos
                ids.push_back(nro);
                setp.insert(nro);
            }
        }

        // Sacamos el RMS del conjunto aleatorio
        vector<vector<Point2f> > framesSeleccionados;
        for(int i=0;i<n;i++){
            framesSeleccionados.push_back(frames[ids[i]]);
        }

        vector<vector<Point3f> > objectsPoints(1);
        objectsPoints[0] = calcDistanceInWorld();
        objectsPoints.resize(framesSeleccionados.size(), objectsPoints[0]);
        double RMS_iteracion =                                          (objectsPoints, framesSeleccionados, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);

        // Si el RMS es alto buscamos otro subconjunto de n elementos con RMS mas bajo.
        if(RMS_iteracion > bestSubSetRank) continue;
        bestSubSetRank = RMS_iteracion+0.04;

        vector<vector<Point2f> > alsoinliers;
        vector<int> alsoinliersId;
        // Confrontamos los puntos del modelo con el resto de puntos
        for(int i=0;i<frames.size();i++){
            if(setp.find(i)!=setp.end()) continue; // verificar que no estemos cogiendo puntos repetidos

            vector<vector<Point2f> > framesPrueba = framesSeleccionados;
            framesPrueba.push_back(frames[i]);

            vector<vector<Point3f> > objectsPoints(1);
            objectsPoints[0] = calcDistanceInWorld();
            objectsPoints.resize(framesPrueba.size(), objectsPoints[0]);
            double RMS_i = calibrateCamera(objectsPoints, framesPrueba, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);

            dbg2(RMS_i,t);
            if(RMS_i < t && RMS_i < RMS_iteracion){ // Este caso se acomoda bien al modelo y se hace la actualizacion de t
                alsoinliers.push_back(frames[i]);
                alsoinliersId.push_back(framesId[i]);
            }
        }

        // Si el numero de elementos de alsoinliers es > d
        //dbg(alsoinliers.size());
        if(alsoinliers.size()>=d){
            // Encontramos un buen modelo
            vector<vector<Point2f> > bettermodel(alsoinliers.begin(),alsoinliers.end());
            vector<int> bettermodelId(alsoinliersId.begin(),alsoinliersId.end());
            for(int i=0;i<ids.size();i++){
                bettermodel.push_back(frames[ids[i]]);
                bettermodelId.push_back(framesId[ids[i]]);
            }
            dbg(bettermodel.size());

            vector<vector<Point3f> > objectsPoints(1);
            objectsPoints[0] = calcDistanceInWorld();
            objectsPoints.resize(bettermodel.size(), objectsPoints[0]);
            double thiserr = calibrateCamera(objectsPoints, bettermodel, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag);
            //dbg2(besterr,thiserr);
            if(thiserr<besterr){
                bestfit = bettermodel;
                bestfitId = bettermodelId;
                besterr = thiserr;
            }
            //dbg(bestfitId.size());
            t = min(besterr,t); // Actualizamos al mejor RMS obtenido en la iteracion y el mejor hasta el momento
        }
        iterations++;
    }
    //dbg(besterr);
    cout << "El mejor valor para RMS que se obtuvo es: " << besterr << endl;
    dbg(bestfit.size());
    // Aqui se muestran los elementos del conjunto
    cout << "El conjunto de frames que tomamos esta compuesto por:" << endl;
    //dbg(bestfitId.size());
    sort(bestfitId.begin(),bestfitId.end());
    for(int i=0;i<bestfitId.size();i++){
       cout << bestfitId[i] << ", ";
    }
    cout << endl;
}

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

    while (actived) {
        // Lectura de cada frame
        video >> img;
        if (!img.data)
            break;
        framesTotal++;

        vector<Point2f> keypoints;
        tmp = img.clone();
        pattDetector->setImage(tmp);
        switch (pattDetector->getCurrentPattern()) {
            case PATT_CIRCLE:
                tm.start();
                status = pattDetector->processingCirclesPattern(keypoints);
                break;
                tm.stop();
                visualizer->visualizetimeReal(tm.getTimeMilli() / tm.getCounter());
                break;
            case PATT_RING:
                tm.start();
                status = pattDetector->processingRingsPattern(keypoints);
                tm.stop();
                visualizer->visualizetimeReal(tm.getTimeMilli() / tm.getCounter());
                break;
        }
        // preguntamos si encontro el patron
        if(status) {
            framesAnalyzed++;
            visualizer->visualizaframesReal(framesAnalyzed);
            mapFrames[framesTotal] = keypoints;
            imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + ".png", tmp);
        }
        else {
            //imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + "_mal.png", tmp);
            continue; // Pasamos al siguiente frame
        }

        // Evaluamos si esta activada la opcion de calcular la distancia de la camara al patron
        if(distanceActived) {
            distances.push_back(runComputeDistance(keypoints, paramCameraMatrix, paramDistCoeffs));
        }
        if (waitKey(10) >= 0)
            break;
    }

    cout << "=====================\n";
    cout<<"TM get counter"<<tm.getCounter()<<endl;
    double average_time = tm.getTimeMilli() / tm.getCounter();
    //cout << "Total Frames: " << framesTotal << "\nFrames Analizados: " << framesAnalyzed << "\n% Analisis: " << (framesAnalyzed * 1.0 / framesTotal) << endl;
    cout << "Total Frames: " << framesTotal << "\nFrames Analizados: " << framesAnalyzed << "\n% Analisis: " << (framesAnalyzed * 1.0 / framesTotal) << "\n AVG: "<<average_time<< endl;
    // Mostrar el analisis de Tiempo y Frames
    visualizer->visualizeTimeExec(framesTotal,framesAnalyzed ,(framesAnalyzed * 1.0 / framesTotal), average_time);
    cout << "=====================\n";

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
            selectFrames(mapFrames, frames, centers, imageSize); // Retorna los frames y los centros
            visualizer->visualizeMsg("Camera calibration ...");
            rms = runCalibrationAndSave(imageSize, centers, frames, paramCameraMatrix, paramDistCoeffs);
            visualizer->visualizeValue("RMS", rms);

            if(showUndistort) {
                Mat img, tmp;
                loadVideo(pathVideo);
                int id = 1;
                while (actived) {
                    video >> img;
                    if (!img.data)
                        break;
                    tmp = img.clone();
                    undistort(img, tmp, paramCameraMatrix, paramDistCoeffs);
                    imshow("After calibration", tmp);
                    visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(tmp), "UNDISTORT IMAGE");
                    //string path = "/home/uburoxana/Documentos/Imagenes/PROYECTOFINAL_CALIBRACION/PatronCircular/build-RCirculos-Desktop-Debug/PadronAnillos_01/Opencv/";

                    imwrite(folderOutVideo + "/frame_" + num2str<int>(framesTotal) + "orifpng", tmp);
                    imwrite(folderOutVideo + "/Calibrados/" + num2str<int>(id++) + ".png", tmp);
                    if (waitKey(20) >= 0)
                        break;
                }
            }
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
    visualizer->cleanImage(PROC5);
    visualizer->cleanImage(PROC6);
    visualizer->cleanImage(PROC7);
    visualizer->cleanImage(PROC8);
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
    currCalib = CALIB_NONE;
    currFrameSelector = FRAMESEL_MANUAL;
    saveCamParams = false;
    showUndistort = false;
}
