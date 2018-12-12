#include "patterndetector.h"
#include "image.h"
#include "math.h"
#include "constantParams.h"
#include "Statistics.h"
#include <algorithm>
#include <string>
#include <sstream>
#include <stack>
#include <fstream>
#include <set>

// Parametros del thresholding
#define DIV_S   12
#define T       0.15f

// Parametros de contornos
#define MIN_SIZE_CONTOUR    6
#define MYEPS 1e-8

// Operaciones
#define dbg(x) cout<<#x<<"="<<x<<endl
#define dbg2(x,y) cout<<#x<<"="<<x<<" "<<#y<<"="<<y<<endl
#define MY_COLOR_YELLOW CV_RGB(255,255,0)
#define MY_COLOR_RED CV_RGB(255,0,0)
#define MY_COLOR_WHITE CV_RGB(255,255,255)
#define MY_COLOR_ORANGE CV_RGB(255,69,0)
#define MY_COLOR_ORANGE1 CV_RGB(100,100,0)
#define MY_COLOR_GREEN CV_RGB(20,150,20)
#define MY_COLOR_BLUE CV_RGB(0,0,205)
#define INF (1<<30)
#define MAXN 100

// Convertir floats a strings
template<typename Te>
string num2str(Te x) { stringstream ss; ss << x; return ss.str();}

// Comparador para ordenar los puntos del patron int
bool cmp(pair<int,int> p1, pair<int,int> p2){
    if(p1.first!=p2.first)
        return p1.first<p2.first;
    return p1.second > p2.second;
}

// Comparador para ordenar los puntos del patron float
bool cmp1(pair<float,float> p1, pair<float,float> p2){
    if(p1.first!=p2.first)
        return p1.first<p2.first;
    return p1.second > p2.second;
}

// Comparador para hallar los puntos en una recta
bool cmp2(pair<float,Point2f>  p1, pair<float,Point2f> p2){
    return p1.first < p2.first;
}

// Comparador para el ordenado y poner numeros
bool cmp3(pair<float,float>  p1, pair<float,float> p2){
    if(p1.second!=p2.second)
        return p1.second>p2.second;
    return p1.first > p2.first;
}

int numeroGlobal = 0;

bool genSide(Point A, Point B, Point C){
    return (A.x-C.x)*(B.y-C.y)-(B.x-C.x)*(A.y-C.y) > 0;
}

int cElDi(vector<Point> ar){
    set<pair<int,int> > setAr;
    for(int i = 0; i < (int)ar.size(); i++){
        setAr.insert(make_pair(ar[i].x, ar[i].y));
    }
    return (int)setAr.size();
}

bool isEqualP(Point a, Point b){
    return ((a.x-b.x)==0) && ((a.y-b.y)==0);
}

bool isEqualP2f(Point2f a, Point2f b){
    return abs(a.x-b.x) <= MYEPS && abs(a.y-b.y) <= MYEPS;
}

float calcDistance(Point2f p1, Point2f p2) {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt((dx * dx) + (dy * dy));
}

bool visit[MAXN];
vector<int> G[MAXN];
int maxnodo=0;
int DFS_Contar(int nodo){
  if(!visit[nodo]){
    visit[nodo]=1;
    int suma=1;
    for(int i=0;i<(int)G[nodo].size();i++){
       suma+=DFS_Contar(G[nodo][i]);
    }
    return suma;
  }
  return 0;
}

void DFS_recoger(int nodo,vector<int> &vi){
  if(!visit[nodo]){
    visit[nodo]=1;
    vi.push_back(nodo);
    for(int i=0;i<(int)G[nodo].size();i++){
       DFS_recoger(G[nodo][i],vi);
    }
  }
}

float cross(pair<float,float> x,pair<float,float> y){
    float dx = abs(x.first - y.first);
    float dy = abs(x.second - y.second);
    return (float)sqrt((dx*dx)+(dy*dy));
}

PatternDetector::PatternDetector()
{
    //kfTracking = new KFTracking(1);
}

void PatternDetector::setCurrentPattern(unsigned int pattType)
{
    currentPattern = pattType;
}

unsigned int PatternDetector::getCurrentPattern()
{
    return currentPattern;
}

void PatternDetector::setImage(Mat im)
{
    img = im;
}

///
/// \brief PatternDetector::setVisualizer asigna el visualizador que se usara para el procesamiento
/// \param vis  MainWindow donde se visualizara los videos
///
void PatternDetector::setVisualizer(MainWindow *vis)
{
    visualizer = vis;
}

///
/// \brief PatternDetector::setSizePattern setea el tamaño del patrón
/// \param nRows    Numero de filas que tiene el patron
/// \param nCols    Numero de columnas que tiene el patron
///
void PatternDetector::setSizePattern(int nRows, int nCols)
{
    numRows = nRows;
    numCols = nCols;
    trackGrid = new TrackingGrid(numRows * numCols);
}

///
/// \brief Funcion que aplica Thresholding Adaptativo, este algo está basado en el paper
/// "Adaptive Thresholding Using the Integral Image" de Derek Bradley y Gerhard Roth
/// \param input    Imagen en escala de grises que será segmentada
/// \return         Imagen binaria resultante
///
Mat PatternDetector::adaptiveThresholdIntegralImage(Mat input)
{
    // Ancho y altura de la imagen
    int w = input.cols;
    int h = input.rows;
    // Tamaño de la ventana S = (w/DIV_S)
    int s2 = (w / DIV_S) / 2;
    // Declaracion de variables auxiliares
    int sum = 0;
    int count = 0;
    int x1, x2, y1, y2;

    // Imagen integral
    unsigned long intImg[h][w];
    // Imagen binaria de salida
    Mat binImg(h, w, CV_8UC1);

    // Calculo de la imagen integral basado en los valores de los pixeles de input
    for (int i = 0; i < h; i++) {
        sum = 0;
        for(int j = 0; j < w; j++) {
            sum += input.at<uchar>(i,j);
            if (i == 0)
                intImg[i][j] = sum;
            else
                intImg[i][j] = intImg[i-1][j] + sum;
        }
    }

    // Se aplica thresholding y se obtiene la imagen binaria
    for (int i = 0; i < h; i++) {
        for(int j = 0; j < w; j++) {
            // Valores (x1,y1) y (x2,y2) de la ventana SxS
            x1 = j - s2;
            x2 = j + s2;
            y1 = i - s2;
            y2 = i + s2;

            // Verificación de bordes
            if(x1 < 0) x1 = 0;
            if(x2 >= w) x2 = w - 1;
            if(y1 < 0) y1 = 0;
            if(y2 >= h) y2 = h - 1;

            count = (x2 - x1) * (y2 - y1);
            sum = intImg[y2][x2] - intImg[y1][x2] - intImg[y2][x1] + intImg[y1][x1];

            // Proceso de binarización
            if((input.at<uchar>(i,j) * count) <= (sum * (1.0 - T)))
                binImg.at<uchar>(i,j) = 0;
            else
                binImg.at<uchar>(i,j) = 255;
        }
    }
    return binImg;
}

///
/// \brief PatternDetector::cleanNoiseCenters  se encarga de eliminar ruido haciendo analisis de los radios de los contornos hallados
/// \param vCenters Vector de centros de los contornos
/// \param vRadius  Vector de radios de los contornos
/// \return Vector de centros con una reduccion de ruido
///
vector<Point2f> PatternDetector::cleanNoiseCenters(vector<Point2f> vCenters, vector<pair<float, int> > vRadius, int maxError)
{
    // Si el numero de centros es el mismo numero de componentes del patron, se regresa el mismo vector
    if(vCenters.size() <= (numCols * numRows + maxError)) {
        // Ordenamiento de los radios en orden descendente para contar las frecuencias por intervalo
        if(vRadius.size()<=2)
            return vCenters;
        sort(vRadius.rbegin(), vRadius.rend());
        radioOptimo = (vRadius[0].first + vRadius[vRadius.size()-1].first) * 0.5;
        return vCenters;
    }

    vector<pair<int, float > > freqs;
    vector<pair<pair<float, float>, pair<int, int> > > extraInfo;
    float avgVal, stdVal;
    int modeVal, posMode;

    // Se obtiene las frecuencias de los datos agrupados
    switch (currentPattern) {
        case PATT_CIRCLE:
            stats::getFrequences<float,int>(vRadius, freqs, extraInfo, false, 9);
            break;
        default:
            stats::getFrequences<float,int>(vRadius, freqs, extraInfo, false);
            break;
    }
    // Se obtiene el promedio y la desviacion estandar
    stats::getAvgStd(freqs, avgVal, stdVal);
    // Se obtiene la moda y el intervalo en que se encuentra
    stats::getMode(freqs, modeVal, posMode);

    // Vector donde se almacenaran los centros del patron
    vector<Point2f> keypoints;
    float minRad, maxRad;

    // Minimo y maximo radio que debe tener un circulo del patron
    if(modeVal == (int)(numCols * numRows)) {
        minRad = extraInfo[posMode].first.first - ERRORF;
        maxRad = extraInfo[posMode].first.second + ERRORF;
    }
    else {
        minRad = freqs[posMode].second - stdVal;
        maxRad = freqs[posMode].second + stdVal;
    }
    for(size_t i = 0; i < vRadius.size(); i++) {
        if(vRadius[i].first >= minRad && vRadius[i].first <= maxRad) {
            keypoints.push_back(vCenters[vRadius[i].second]);
        }
    }
    // El radio optimo es el label del intervalo moda
    radioOptimo = freqs[posMode].second;
    return keypoints;
}

///
/// \brief PatternDetector::findROI_rings    Encuentra los contornos de interes
/// \param image    Imagen binaria de entrada
/// \return Vector de centros de los contornos de interes
///
vector<Point2f> PatternDetector::findROI_rings(Mat image, Mat &imgOut)
{
    // Imagen auxiliar
    imgOut = Mat::zeros(image.size(), CV_8UC3);

    // Obtencion de los contornos con jerarquia
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(image, contours, hierarchy ,CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    // Vector de centros
    vector<Point2f> keypoints;

    // Variables auxiliares
    double areaPar, auxFactorPar, auxFactorCurr;
    int parent, child;
    Point2f centerCurr, centerPar;

    vector<pair<float, int> > vectRadios;
    for(size_t i = 0; i < contours.size(); i++)
    {
        parent = hierarchy[i][3];
        child = hierarchy[i][2];

        if(child == -1) {
            if(parent != -1 && hierarchy[i][0] == -1 && hierarchy[i][1] == -1) {
                // PADRE: Rectangulo donde encaja la elipse o el contorno del padre
                RotatedRect boxPar;
                if(contours[parent].size() < MIN_SIZE_CONTOUR)
                    boxPar = minAreaRect(contours[parent]);
                else {
                    Mat pointsf;
                    Mat(contours[parent]).convertTo(pointsf, CV_32F);
                    boxPar = fitEllipse(pointsf);
                }
                centerPar = boxPar.center;

                // ACTUAL: Rectangulo donde encaja la elipse o el contorno del actual
                RotatedRect boxCurr;
                if(contours[i].size() < MIN_SIZE_CONTOUR)
                    centerCurr = centerPar;
                else {
                    Mat pointsf;
                    Mat(contours[i]).convertTo(pointsf, CV_32F);
                    boxCurr = fitEllipse(pointsf);
                    centerCurr = boxCurr.center;
                }

                // Calculo de areas
                areaPar = contourArea(contours[parent]);

                // Factor de aspect ratio
                auxFactorPar = min(boxPar.size.width, boxPar.size.height) / max(boxPar.size.width, boxPar.size.height);
                auxFactorCurr = min(boxCurr.size.width, boxCurr.size.height) / max(boxCurr.size.width, boxCurr.size.height);
                if(auxFactorPar < R_PAR_MIN_ASPECT_RATIO || auxFactorCurr < R_CHD_MIN_ASPECT_RATIO)
                    continue;

                // Factor de rectangularidad
                auxFactorPar = areaPar / boxPar.size.area();
                if (auxFactorPar < R_PAR_MIN_RECTAN)
                    continue;

                // Almacenamiento del centro de los anillos concentricos
                keypoints.push_back(Point2f((centerCurr.x + centerPar.x) * 0.5, (centerCurr.y + centerPar.y) * 0.5));
                vectRadios.push_back(make_pair(max(boxPar.size.width, boxPar.size.height) * 0.5, keypoints.size() - 1));

                // Grafica de los contornos (padre e hijo)
                drawContours(imgOut, contours, i, Scalar::all(255), 1, 8);
                drawContours(imgOut, contours, parent, Scalar::all(255), 1, 8);
                // Grafica de las elipses
                ellipse(imgOut, boxPar, Scalar(0,0,255), 1, CV_AA);
                ellipse(imgOut, boxCurr, Scalar(0,0,255), 1, CV_AA);
            }
        }
    }
    keypoints = cleanNoiseCenters(keypoints, vectRadios, 2);
    // Grafica de los centros despues de la limpieza
    for(size_t i = 0; i < keypoints.size(); i++) {
        circle(imgOut, keypoints[i], 3, Scalar(255,255,0), -1);
    }
    return keypoints;
}

///
/// \brief PatternDetector::findROI_circles    Encuentra los contornos de interes
/// \param image    Imagen binaria de entrada
/// \return Vector de centros de los contornos de interes
///
vector<Point2f> PatternDetector::findROI_circles(Mat image, Mat &imgOut)
{
    // Imagen auxiliar
    imgOut = Mat::zeros(image.size(), CV_8UC3);

    // Declaracion de variables
    double area, auxFactor;

    // Vector de centros
    vector<Point2f> keypoints;
    vector<pair<float, int> > vectRadios;

    // Obtencion de los contornos más externos
    vector<vector<Point> > contours;
    findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    for(size_t i = 0; i < contours.size(); i++){
        if(contours[i].size() < MIN_SIZE_CONTOUR)
            continue;

        Mat pointsf;
        Mat(contours[i]).convertTo(pointsf, CV_32F);
        // Rectangulo minimo que contiene todo el contorno
        RotatedRect boxmin = minAreaRect(contours[i]);

        // Factor de aspect ratio
        auxFactor = min(boxmin.size.width, boxmin.size.height) / max(boxmin.size.width, boxmin.size.height);
        if(auxFactor < C_MIN_ASPECT_RATIO)
            continue;

        // Factor de rectangularidad
        area = contourArea(contours[i]);
        auxFactor = area / boxmin.size.area();
        if (area < C_MIN_AREA || auxFactor < C_MIN_RECTAN)
            continue;

        // Rectangulo donde encaja la elipse
        RotatedRect box = fitEllipse(pointsf);
        // Almacenamiento del centro de la elipse
        keypoints.push_back(box.center);
        vectRadios.push_back(make_pair(max(box.size.width, box.size.height) * 0.5, keypoints.size()-1));

        // Grafican del contorno
        drawContours(imgOut, contours, (int)i, Scalar::all(255), 1, 8);
        // Grafica de la elipse
        ellipse(imgOut, box, Scalar(0,0,255), 1, CV_AA);
    }
    keypoints = cleanNoiseCenters(keypoints, vectRadios);
    // Grafica de los centros despues de la limpieza
    for(size_t i = 0; i < keypoints.size(); i++) {
        circle(imgOut, keypoints[i], 3, Scalar(255,255,0), -1);
    }
    return keypoints;
}

///
/// \brief PatternDetector::findFinalCenters_circles
/// \param keypoints
/// \param image
/// \return
///
vector<Point2f> PatternDetector::findFinalCenters_circles(vector<Point2f> keypoints, Mat image)
{
    Mat drawImage0 = Mat::zeros(image.size(), CV_8UC3);
    Mat drawImage = Mat::zeros(image.size(), CV_8UC3);
    Mat drawImage2 = Mat::zeros(image.size(), CV_8UC3);
    Mat drawImage1 = Mat::zeros(image.size(), CV_8UC3);

    // FILTRO DE LA DISTANCIA MININA UNO VS TODOS
            // Sacamos los puntos de keypoints para procesarlos
            pair<int,int> par;
            vector<pair<int,int> > Points;     // Utilizamos este arreglo para utilizar ints
            vector<pair<float,float> > Points2;// Utilizamos este arreglo para utilizar floats
            for (size_t i = 0; i < keypoints.size(); ++i){
                int x = keypoints[i].x;
                int y = keypoints[i].y;
                par = make_pair(x,y);
                Points.push_back(par);
                Points2.push_back(make_pair(keypoints[i].x,keypoints[i].y));
            }

            // Ordenacion de los puntos
            sort(Points.begin(),Points.end(),cmp);   // Se ordeno para agarrar mejor los puntos
            sort(Points2.begin(),Points2.end(),cmp1); // y que no esten de manera aleatoria en general

            //Hallamos las distancias mas cercanas de un punto con su universo
            vector<int> id(Points.size()); // arreglo donde almacenamos los id's
            vector<double> Distancias;
            //dbg(numeroGlobal);
            //dbg(Points.size());
            for(int i=0;i<(int)Points.size();i++){
                double dis = INF;
                int idx = -1;
                for(int j=i+1;j<(int)Points.size();j++){
                    // Hallar el mas cercano
                    if(dis>cross(Points[i],Points[j])){
                        dis = cross(Points[i],Points[j]);
                        idx = j;
                    }
                }
                Distancias.push_back(dis); // Distancia minima de un punto P con su universo, para todos los puntos
                id[i]=idx;                 // punto i su mas cercano es idx

                //circle(drawImage0, Point(Points[i].first,Points[i].second), 4, Scalar(255, 0, 255), -1);
                //circle(drawImage0, Point(Points[idx].first,Points[idx].second), 4, Scalar(255, 0, 255), -1);
                //line(drawImage0, Point(Points[i].first,Points[i].second),Point(Points[idx].first,Points[idx].second) ,Scalar(0, 0, 255));

            }
            //imwrite("/home/gerar/Documentos/Vision Computacional/mcs_imagenes_camaracalibration/PatronCircular/cercanos/Frames_" + num2str<int>(numeroGlobal) + ".png", drawImage0);

            //Hallamos las frecuencias de las distancias mas cercanas
            map<double,int> mapa2;
            map<double,int>::iterator it2;
            for(int i=0;i<(int)Distancias.size();i++){
                mapa2[Distancias[i]]++;
            }

            // Pasamos los valores a un vector para su ordenacion de las distancias mas cercanas
            vector<pair<int,double> > frecuenciasD;
            for(it2=mapa2.begin();it2!=mapa2.end();it2++){
                frecuenciasD.push_back(make_pair((*it2).second,(*it2).first));
            }

            sort(frecuenciasD.rbegin(),frecuenciasD.rend()); // ordenamos de manera descendente para hallar los mas frecuentes
            // Promedio 1
            double avg;
            avg=frecuenciasD[0].second;
            // Promedio 2
            vector<pair<int,int> > preDistancias; // Puntos con segmentos que pasan el promedio en tamaño
            vector<double> PreDisSize;       // Distancias de los puntos que pasan el promedio
            int promedio = 0,distancialocal;
            int denominador = 0;
            for(int i=0;i<(int)Distancias.size();i++){
                if(Distancias[i]<=avg*3){
                    preDistancias.push_back(make_pair(i,id[i]));
                    distancialocal = cross(make_pair(Points[i].first,Points[i].second),make_pair(Points[id[i]].first,Points[id[i]].second)) ;
                    PreDisSize.push_back(distancialocal);
                    promedio += distancialocal;
                    denominador++;
                }
            }

            promedio += denominador;
            Distancias = PreDisSize;
            avg = promedio;

            // Promedio 03
            // Declaracion de un grafo para evadir ruido
            memset(visit,0,sizeof(visit)); // llenamos el arreglo de visitados con 0 (no visitado)
            fill(G,G+MAXN,vector<int>());  // Inicializando Grafo para nuevo frame

            for(int i=0;i<(int)Distancias.size();i++){
                if(Distancias[i]<=avg){
                    // Agregando aristas en el grafo
                    G[preDistancias[i].first].push_back(preDistancias[i].second);
                    G[preDistancias[i].second].push_back(preDistancias[i].first);

                    circle(drawImage1, Point(Points[preDistancias[i].first].first,Points[preDistancias[i].first].second), 4, Scalar(255, 0, 255), -1);
                    circle(drawImage1, Point(Points[preDistancias[i].second].first,Points[preDistancias[i].second].second), 4, Scalar(255, 0, 255), -1);
                    line(drawImage1, Point(Points[preDistancias[i].first].first,Points[preDistancias[i].first].second),Point(Points[preDistancias[i].second].first,Points[preDistancias[i].second].second) ,Scalar(0, 0, 255));

                }
            }

            //imshow("Distancia minimo uno vs todos", drawImage1);
            visualizer->visualizeImage(PROC5, ImageHelper::convertMatToQimage(drawImage1), "Distancia minima uno vs todos");
            //imwrite("/home/gerar/Documentos/Vision Computacional/mcs_imagenes_camaracalibration/PatronCircular/UCT/Frames_" + num2str<int>(numeroGlobal) + ".png", drawImage1);

            // FILTRO RECORRIDO DFS
            // Hallando el grafo con mayor numero de nodos
            int idmax, maxnodos=-1;
            for(int i=0;i<MAXN;i++){
                if(!visit[i]){
                    int nroNodo = DFS_Contar(i);
                    if(maxnodos<nroNodo){
                        maxnodos=nroNodo;
                        idmax = i;
                    }
                }
            }

            // Recogemos todos los elementos del grafo con mayor numero de nodos
            memset(visit,0,sizeof(visit));
            vector<int> nodos;
            DFS_recoger(idmax,nodos); // Recogemos los valores de los nodos que recorre el grafo
            Point P1,P2;

            // sacando los segmentos de coste minimo
            vector<pair<int,pair<int,int> > > distancias2;
            for(int i=1;i<(int)nodos.size();i++){
                P1 = Point(Points[nodos[i-1]].first,Points[nodos[i-1]].second);
                P2 = Point(Points[nodos[i]].first,Points[nodos[i]].second);

                // Sacando distancias de dos puntos consecutivos del DFS
                distancias2.push_back(make_pair(cross(make_pair(P1.x,P1.y),make_pair(P2.x,P2.y)),make_pair(nodos[i-1],nodos[i])));

                circle(drawImage2, P1, 4, Scalar(255, 0, 255), -1);
                circle(drawImage2, P2, 4, Scalar(255, 0, 255), -1);

                line(drawImage2,P1,P2,Scalar(0, 0, 255));
            }

            visualizer->visualizeImage(PROC6, ImageHelper::convertMatToQimage(drawImage2), "Recorrido DFS");

            //imwrite("/home/gerar/Documentos/Vision Computacional/mcs_imagenes_camaracalibration/PatronCircular/DFS/Frames_" + num2str<int>(numeroGlobal) + ".png", drawImage2);

            // Sacamos los que tiene los costos minimos de entre las aristas que pertenecen al grafo
            sort(distancias2.begin(),distancias2.end());
            set<int> setp;
            int maximobolitas = 44;
            Mat tmp = img.clone();

            for(int i=0;i<min(maximobolitas,(int)distancias2.size());i++){
                int dx = distancias2[i].second.first;
                int dy = distancias2[i].second.second;

                P1 = Point(Points[dx].first,Points[dx].second);
                P2 = Point(Points[dy].first,Points[dy].second);
                setp.insert(dx);
                if((int)setp.size()>maximobolitas)
                    break;
                circle(drawImage, P1, 4, Scalar(255, 0, 255), -1);
                circle(tmp, P1, 4, Scalar(255, 0, 255), -1);
                setp.insert(dy);
                if((int)setp.size()>maximobolitas)
                    break;
                circle(drawImage, P2, 4, Scalar(255, 0, 255), -1);
                circle(tmp, P2, 4, Scalar(255, 0, 255), -1);
                line(drawImage,P1,P2,Scalar(0, 0, 255));
            }
        // Escribiendo los frames del resultado
        //string str3 = "/home/gerar/Documentos/Vision Computacional/mcs_imagenes_camaracalibration/PatronCircular/MST/Frames_" + num2str<int>(numeroGlobal) + ".png";
        //imwrite(str3, drawImage);
        numeroGlobal++;

        visualizer->visualizeImage(PROC7, ImageHelper::convertMatToQimage(drawImage), "Aristas minimas");

        //FILTRO DE KALMAN USANDO PREDICCION t+1
        Mat frameDraw = Mat::zeros(image.size(), CV_8UC3);
        set<int> setp2;
        TrackingGrid* trackGrid = new TrackingGrid(maximobolitas);
        vector<pair<int,float> > vectorKeyDis;
        for(int i=0;i<min(maximobolitas,(int)distancias2.size());i++){
            int dx = distancias2[i].second.first;
            int dy = distancias2[i].second.second;
            if(setp2.find(dx) == setp2.end()){
                float distx = centNextPred.first - Points[dx].first;
                float disty = centNextPred.second - Points[dx].second;
                vectorKeyDis.push_back(make_pair(dx, distx*distx + disty*disty));
                setp2.insert(dx);
            }
            if(setp2.find(dy) == setp2.end()){
                float distx = centNextPred.first - Points[dy].first;
                float disty = centNextPred.second - Points[dy].second;
                vectorKeyDis.push_back(make_pair(dy, distx*distx + disty*disty));
                setp2.insert(dy);
            }
            if((int)setp2.size() > maximobolitas)
                break;
        }
        vectorKeyDis = trackGrid->sortBySecond(vectorKeyDis);
        patternActual.clear();
        int nVect = (int)vectorKeyDis.size() >= maximobolitas ? maximobolitas : (int)vectorKeyDis.size();
        vector<Point2f> patternActual2;
        for(int i = 0; i < nVect; i++){
            int posX = Points[vectorKeyDis[i].first].first;
            int posY = Points[vectorKeyDis[i].first].second;
            circle(frameDraw, Point(posX,posY),4,MY_COLOR_YELLOW,-1);
            patternActual.push_back(Point(posX,posY));

            patternActual2.push_back(Point2f(Points2[vectorKeyDis[i].first].first,Points2[vectorKeyDis[i].first].second));

        }

        // TRACKING GRID BOLITAS
        if((int)patternActual.size() <= maximobolitas){
            Point2f centroide = trackGrid->getCentroide(patternActual);
            circle(frameDraw, Point(centroide.x, centroide.y), 4, MY_COLOR_GREEN, -1);

            vector<Mat>* arrayMat = new vector<Mat>();
            Mat tempKF(MEASURE_SIZE,1,TYPE_KF);
            tempKF.at<float>(0) = centroide.x;
            tempKF.at<float>(1) = centroide.y;
            tempKF.at<float>(2) = (float)0.0;
            tempKF.at<float>(3) = (float)0.0;
            arrayMat->push_back(tempKF);

            //primera vez que aparece el patron
            /*if(kfTracking->firstFound[0]){
                kfTracking->setStateInit(arrayMat);
                kfTracking->firstFound[0] = false;
            }else{
                kfTracking->predict(dtKFTrac);
                vector<Mat>* correct = kfTracking->kalmanCorrection(arrayMat);
                vector<Mat>* matForPre = kfTracking->futureNTime(1);
                centNextPred = make_pair((*matForPre)[0].at<float>(0), (*matForPre)[0].at<float>(1));
                for(int i =0; i < (int)correct->size(); i++){
                    circle(frameDraw, Point((*correct)[i].at<float>(0),(*correct)[i].at<float>(1)), 4, MY_COLOR_RED,-1);
                    circle(frameDraw, Point((*matForPre)[i].at<float>(0),(*matForPre)[i].at<float>(1)),3, MY_COLOR_WHITE,-1);
                    line(frameDraw,Point(centroide.x,centroide.y), Point((*matForPre)[i].at<float>(0),(*matForPre)[i].at<float>(1)),MY_COLOR_WHITE,1,8,0);
                }
            }
            visualizer->visualizeImage(PROC8, ImageHelper::convertMatToQimage(frameDraw), "Filtro Kalman eliminar ruido");*/
        }

        if((int)patternActual.size() <= maximobolitas){
            patternBefore = patternActual;
        }
        return patternActual2;
}

///
/// \brief PatternDetector::cleanNoiseUsingDistances   Funcion que elimina ruido usando las distancias entre los puntos
/// \param keypoints    Vector de centros de los contornos
/// \param imgOut       Imagen de salida donde se mostrará el procesamiento
/// \return     Vector de centros con el ruido reducido
///
vector<Point2f> PatternDetector::cleanNoiseUsingDistances(vector<Point2f> keypoints, Mat &imgOut)
{
    int maximobolitas = numRows * numCols;
    patternActual.clear();

    if((int)keypoints.size() <= maximobolitas){
        patternActual = keypoints;
    }else {
        int factDist = 2;
        // Calculamos las distancias de uno vs todos
        vector<pair<float, pair<int,int> > > distances;
        float dist = INF;
        int idx = -1;
        for(size_t i = 0; i < keypoints.size() - 1; i++) {
            dist = INF;
            idx = -1;
            for(size_t j = i + 1; j < keypoints.size(); j++) {
                if(dist > calcDistance(keypoints[i], keypoints[j])) {
                    dist = calcDistance(keypoints[i], keypoints[j]);
                    idx = j;
                }
            }
            distances.push_back(make_pair(dist, make_pair(i,idx)));
        }
        // Calculamos la distancia media usando estadista por datos agrupados en 5 intervalos
        vector<pair<int, float> > freqs;
        vector<pair<pair<float,float>, pair<int,int> > > extraInfo;
        int posMode, modeVal;
        stats::getFrequences<float,pair<int,int> >(distances, freqs, extraInfo, false, 5);
        stats::getMode(freqs, modeVal, posMode);
        cout << "MODE: " << freqs[posMode].second << endl;

        float maxVal = freqs[posMode].second * factDist;
        // Usando la moda eliminamos los puntos que se encuentren muy alejados
        set<pair<float,float> > pointsSelected;
        for(size_t i = 0; i < distances.size(); i++) {
            if(distances[i].first < maxVal) {
                pointsSelected.insert(make_pair(keypoints[distances[i].second.first].x, keypoints[distances[i].second.first].y));
                pointsSelected.insert(make_pair(keypoints[distances[i].second.second].x, keypoints[distances[i].second.second].y));
            }
        }
        // Insertamos los puntos en la variable patternActual para que sea evaluado en Kalman
        set<pair<float,float> >::iterator it;
        for(it = pointsSelected.begin(); it != pointsSelected.end(); ++it) {
            patternActual.push_back(Point2f((*it).first, (*it).second));
        }
        cout << "KeySize: " << keypoints.size() << " - Patt: " << patternActual.size() << endl;
    }

    //FILTRO DE KALMAN USANDO PREDICCION t+1
    imgOut = Mat::zeros(imgOut.size(), CV_8UC3);
    TrackingGrid* trackGrid = new TrackingGrid(maximobolitas);
    vector<pair<int,float> > vectorKeyDis;
    for(int i=0;i<(int)patternActual.size();i++){
        float distx = centNextPred.first - patternActual[i].x;
        float disty = centNextPred.second - patternActual[i].y  ;
        vectorKeyDis.push_back(make_pair(i, distx*distx + disty*disty));
    }

    vectorKeyDis = trackGrid->sortBySecond(vectorKeyDis);
    vector<Point2f> tempVector = patternActual;
    patternActual.clear();
    int nVect = (int)vectorKeyDis.size() >= maximobolitas ? maximobolitas : (int)vectorKeyDis.size();
    for(int i = 0; i < nVect; i++){
        float posX = tempVector[vectorKeyDis[i].first].x;
        float posY = tempVector[vectorKeyDis[i].first].y;
        circle(imgOut, Point2f(posX,posY),4,MY_COLOR_YELLOW,-1);
        patternActual.push_back(Point2f(posX,posY));
    }

    // TRACKING GRID ANILLOS
    if((int)patternActual.size() <= maximobolitas){
        Point2f centroide = trackGrid->getCentroide(patternActual);
        circle(imgOut, Point2f(centroide.x, centroide.y), 4, MY_COLOR_GREEN, -1);

        vector<Mat>* arrayMat = new vector<Mat>();
        Mat tempKF(MEASURE_SIZE,1,TYPE_KF);
        tempKF.at<float>(0) = centroide.x;
        tempKF.at<float>(1) = centroide.y;
        tempKF.at<float>(2) = (float)0.0;
        tempKF.at<float>(3) = (float)0.0;
        arrayMat->push_back(tempKF);

        //primera vez que aparece el patron
        /*if(kfTracking->firstFound[0]){
            kfTracking->setStateInit(arrayMat);
            kfTracking->firstFound[0] = false;
        }else{
            kfTracking->predict(dtKFTrac);
            vector<Mat>* correct = kfTracking->kalmanCorrection(arrayMat);
            vector<Mat>* matForPre = kfTracking->futureNTime(1);
            centNextPred = make_pair((*matForPre)[0].at<float>(0), (*matForPre)[0].at<float>(1));
            for(int i =0; i < (int)correct->size(); i++){
                circle(imgOut, Point2f((*correct)[i].at<float>(0),(*correct)[i].at<float>(1)), 4, MY_COLOR_RED,-1);
                circle(imgOut, Point2f((*matForPre)[i].at<float>(0),(*matForPre)[i].at<float>(1)),3, MY_COLOR_WHITE,-1);
                line(imgOut,Point2f(centroide.x,centroide.y), Point2f((*matForPre)[i].at<float>(0),(*matForPre)[i].at<float>(1)),MY_COLOR_WHITE,1,8,0);
            }
        }*/
    }
    if((int)patternActual.size() <= maximobolitas){
        patternBefore = patternActual;
    }
    return patternActual;
}

void PatternDetector::drawZigZagPattern(vector<Point2f> gridPoints, vector<StruSegme> vectSeg, Mat imageRaw, int gross, int flag){

    // DIBUJAMOS EL PATRON ORIENTADO de acuerdo a flag

    //dibujamos los puntos
    for(int i = 0; i < (int)gridPoints.size(); i++){
        stringstream sstr;
        sstr << i;
        putText(imageRaw, sstr.str(), Point2f(gridPoints[i].x + 3, gridPoints[i].y - 3), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 1);
        circle(imageRaw, gridPoints[i], gross, MY_COLOR_YELLOW, CV_FILLED, 8, 0);
    }

    //dibujamos a rectas para las filas
    TrackingGrid* trackingGrid  = new TrackingGrid(0);
    vector<Scalar> colors;
    colors.push_back(MY_COLOR_GREEN);
    colors.push_back(MY_COLOR_ORANGE);
    colors.push_back(MY_COLOR_BLUE);
    colors.push_back(MY_COLOR_YELLOW);
    colors.push_back(MY_COLOR_WHITE);
    colors.push_back(MY_COLOR_RED);

    if(flag == 0){
        for(int i = 0;i < (int)vectSeg.size();i++)
            vectSeg[i].vectorPoint = trackingGrid->sortUpToDown(vectSeg[i].vectorPoint);
    }else if(flag == 1){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortRightToLeft(vectSeg[i].vectorPoint);
    }else if(flag == 2){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortLeftToRight(vectSeg[i].vectorPoint);
    }else if(flag == 3){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortDownToUp(vectSeg[i].vectorPoint);
    }else if(flag == 4){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortRightToLeft(vectSeg[i].vectorPoint);
    }else if(flag == 5){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortLeftToRight(vectSeg[i].vectorPoint);
    }

    for(int i = 0;i < (int)vectSeg.size(); i++){
        for(int j = 1; j < (int)vectSeg[i].vectorPoint.size(); j++){
            Point2f P1 = Point2f(vectSeg[i].vectorPoint[j-1].first,vectSeg[i].vectorPoint[j-1].second);
            Point2f P2 = Point2f(vectSeg[i].vectorPoint[j].first,vectSeg[i].vectorPoint[j].second);
            circle(imageRaw, P1, gross, colors[i%colors.size()], CV_FILLED,8,0);
            circle(imageRaw, P2, gross, colors[i%colors.size()], CV_FILLED,8,0);
            line(imageRaw,P1,P2,colors[i%colors.size()]);
        }
        if(i+1 != (int)vectSeg.size()){
            int sizeCol1 = (int)vectSeg[i].vectorPoint.size();
            Point2f P1 = Point2f(vectSeg[i].vectorPoint[sizeCol1-1].first,vectSeg[i].vectorPoint[sizeCol1-1].second);
            Point2f P2 = Point2f(vectSeg[i+1].vectorPoint[0].first,vectSeg[i+1].vectorPoint[0].second);
            line(imageRaw,P1,P2,colors[i%colors.size()]);
        }
    }
    Mat tmp = imageRaw.clone();
    //imshow("RESULTADO FINAL", tmp);
    visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(imageRaw));
}

bool PatternDetector::processingRingsPattern(std::vector<Point2f> &keypoints)
{
    // Variables auxiliares
    Mat tmp;

    // Conversion de imagen a escala de grises
    cvtColor(img, tmp, CV_BGR2GRAY);
    // Aplicacion de filtro gaussiano
    GaussianBlur(tmp, tmp, Size(3,3), 0.5, 0.5);
    visualizer->visualizeImage(PROC1, ImageHelper::convertMatToQimage(tmp.clone()), "Filtro gaussiano");
    // Segmentacion de imagen usando threshold adaptativo
    tmp = adaptiveThresholdIntegralImage(tmp);
    visualizer->visualizeImage(PROC2, ImageHelper::convertMatToQimage(tmp.clone()), "Threshold adaptativo (paper)");
    // Obtención del ROI
    cout << "findROI_rings " << endl;
    keypoints = findROI_rings(tmp.clone(), tmp);
    visualizer->visualizeImage(PROC3, ImageHelper::convertMatToQimage(tmp.clone()), "ROI");
    // Obtención de los centros finales
    cout << "cleanNoiseUsingDistances " << endl;
    keypoints = cleanNoiseUsingDistances(keypoints, tmp);
    visualizer->visualizeImage(PROC4, ImageHelper::convertMatToQimage(tmp), "Reduccion de ruido con distancias y KF");
    // Si no se tiene completo el patron, se descarta el frame
    cout << "trackingPoints Rings" << endl;
    bool trackCorrect = trackingRingsPoints(keypoints);
    cout << endl ;
    return trackCorrect;
}


bool PatternDetector::processingCirclesPattern(std::vector<Point2f> &keypoints)
{
    // Variables auxiliares
    Mat tmp;

    // Conversión de imagen a escala de grises
    cvtColor(img, tmp, CV_BGR2GRAY);
    // Aplicación de filtro gaussiano
    GaussianBlur(tmp, tmp, Size(5,5), 2.5, 3);
    visualizer->visualizeImage(PROC1, ImageHelper::convertMatToQimage(tmp.clone()), "Filtro gaussiano");
    // Segmentación de imagen usando threshold adaptativo
    tmp = adaptiveThresholdIntegralImage(tmp);
    visualizer->visualizeImage(PROC2, ImageHelper::convertMatToQimage(tmp.clone()), "Threshold adaptativo (paper)");
    // Aplicación del algoritmo de Canny: Ratio max:min, 2:1 o 3:1
    Canny(tmp, tmp, C_THRES_CANNY, C_THRES_CANNY * C_FACTOR_CANNY);
    visualizer->visualizeImage(PROC3, ImageHelper::convertMatToQimage(tmp.clone()), "Canny");
    // Obtención del ROI
    //cout << "FindROI_circles " << endl;
    keypoints = findROI_circles(tmp.clone(), tmp);
    //cout << "Salida de nro point findROI en el frame: " << numeroGlobal << endl;
    visualizer->visualizeImage(PROC4, ImageHelper::convertMatToQimage(tmp.clone()), "ROI");
    // Obtención de los centros finales
    //cout << "findFinalCenters_circles " << endl;
    keypoints = findFinalCenters_circles(keypoints, tmp);
    // Tracking&Etiquetado de los centros finales o puntos
    //cout << "trackingPoints Circles " << endl;
    bool trackCorrect = trackingCirclePoints(keypoints);
    //cout << endl << endl;
    return trackCorrect;
}

bool PatternDetector::trackingCirclePoints(vector<Point2f> &keypoints) {
    // ETIQUETADO - TRACKING
    Mat img2 = img.clone();
    vector<StruSegme> vecCol;
    int flag = 0;
    if(keypoints.size() != numCols * numRows) {
        return false;
    }

    // SE HALLA EL CONVEX HULL
    vector<vector<Point2f> > keys;
    keys.push_back(keypoints);
    vector<vector<Point2f> > hull(1);
    convexHull(Mat(keypoints), hull[0], false);
    vector<int> posCornes = trackGrid->getPosCornes(hull);

    // HALLAMOS LOS BORDES O PUNTOS EXTREMOS QUE CONFORMAN EL CONVEX HULL
    vector<pair<double, pair<int,int> > > vectorDis;

    for(int i = 0; i < (int)posCornes.size(); i++){
        pair<int,int> value = make_pair(posCornes[i],posCornes[i+1==(int)posCornes.size()?0:i+1]);
        double dist = (hull[0][posCornes[i]].x - hull[0][posCornes[i+1==(int)posCornes.size()?0:i+1]].x)*(hull[0][posCornes[i]].x - hull[0][posCornes[i+1==(int)posCornes.size()?0:i+1]].x)
                + (hull[0][posCornes[i]].y - hull[0][posCornes[i+1==(int)posCornes.size()?0:i+1]].y)*(hull[0][posCornes[i]].y - hull[0][posCornes[i+1==(int)posCornes.size()?0:i+1]].y) ;

        vectorDis.push_back(make_pair(dist,value));
    }

    // ESCOGEMOS LA PRIMERA Y ULTIMA COLUMNA DEL PATRON EN BASE A LA DISTANCIA
    vectorDis = trackGrid->sortByDistWithPoint(vectorDis);
    if(vectorDis.size() != 5){
        return false;
    }else{
        for(int i = 0; i < (int)posCornes.size(); i++){
            circle(img2, hull[0][posCornes[i]], 10, CV_RGB(155,10,230), CV_FILLED,8,0);
        }
        for(int i = 0; i < (int)vectorDis.size(); i++){
            line(img2,hull[0][vectorDis[i].second.first], hull[0][vectorDis[i].second.second], MY_COLOR_GREEN,5);
        }

        int firstCol = -1, lastCol = -1;
        pair<int, int> key0 = vectorDis[0].second;
        pair<int, int> key1 = vectorDis[1].second;

        for(int i = 0; i < (int)posCornes.size(); i++){
            pair<int,int> xPair = make_pair(posCornes[i],posCornes[i+1==(int)posCornes.size()?0:i+1]);
            if((key0.first == xPair.first && key0.second == xPair.second) || (key1.first == xPair.first && key1.second == xPair.second))
                continue;
            if((xPair.first==key0.second && xPair.second==key1.first) || (xPair.first==key1.second && xPair.second==key0.first))
                continue;

            if(xPair.second==key0.first || xPair.second==key1.first){
                for(int j = 0; j < (int)vectorDis.size(); j++){
                    pair<int,int> val = vectorDis[j].second;
                    if(val.first == xPair.first && val.second == xPair.second){
                        firstCol = j;
                        break;
                    }
                }
            }

            if(xPair.first==key0.second || xPair.first==key1.second){
                for(int j = 0; j < (int)vectorDis.size(); j++){
                    pair<int,int> val = vectorDis[j].second;
                    if(val.first == xPair.first && val.second == xPair.second){
                        lastCol = j;
                        break;
                    }
                }
            }

        }

        line(img2,hull[0][vectorDis[firstCol].second.first], hull[0][vectorDis[firstCol].second.second], MY_COLOR_ORANGE,5);
        line(img2,hull[0][vectorDis[lastCol].second.first], hull[0][vectorDis[lastCol].second.second], MY_COLOR_ORANGE,5);

        set<pair<float,float> > setBordes;
        setBordes.insert(make_pair(hull[0][vectorDis[firstCol].second.first].x,hull[0][vectorDis[firstCol].second.first].y));
        setBordes.insert(make_pair(hull[0][vectorDis[firstCol].second.second].x,hull[0][vectorDis[firstCol].second.second].y));
        setBordes.insert(make_pair(hull[0][vectorDis[lastCol].second.first].x,hull[0][vectorDis[lastCol].second.first].y));
        setBordes.insert(make_pair(hull[0][vectorDis[lastCol].second.second].x,hull[0][vectorDis[lastCol].second.second].y));

        // SE OBTIENE LA ULTIMA FILA DEL PATRON(FILA DISCRIMINADORA), FILA QUE GENERA LA ASIMETRIA EN EL PATRON
        vector<Point> borderSobrante;
        for(int i = 0; i < (int)posCornes.size(); i++){
            if(setBordes.find(make_pair(hull[0][posCornes[i]].x,hull[0][posCornes[i]].y)) == setBordes.end())
                borderSobrante.push_back(hull[0][posCornes[i]]);
        }

        if(!borderSobrante.empty()){
            // HALLAMOS UNA RECTA PERPENDICULAR A LA "FILA DISCRIMINADORA" QUE ATRIEVESE AL PATRON Y QUE EMPIEZE EN LA MITAD DE LA "FILA DISCRIMINADORA"
            // HALLAMOS EL ANGULO(ANGULO-ORIENTACION) ENTRE LA RECTA PERPENDICULAR Y EJE X POSITIVO CUYO CENTRO ES EL PUNTO MEDIO DE LA "FILA DISCRIMINADORA"

            int fSob = 0, lSob = borderSobrante.size()>0?borderSobrante.size()-1:0;
            if(borderSobrante[fSob].x > borderSobrante[lSob].x)
                swap(fSob,lSob);
            Point perpenRec(-(borderSobrante[lSob].y-borderSobrante[fSob].y), borderSobrante[lSob].x-borderSobrante[fSob].x);
            Point midP((borderSobrante[fSob].x+borderSobrante[lSob].x)/2, (borderSobrante[fSob].y+borderSobrante[lSob].y)/2);
            Point perMid(midP.x - perpenRec.x, midP.y - perpenRec.y);
            Point perMidOp(midP.x + perpenRec.x, midP.y + perpenRec.y);

            double distFirsToPer = (hull[0][vectorDis[firstCol].second.first].x - perMid.x)*(hull[0][vectorDis[firstCol].second.first].x - perMid.x)
                    + (hull[0][vectorDis[firstCol].second.first].y - perMid.y)*(hull[0][vectorDis[firstCol].second.first].y - perMid.y);
            double distFirsToPerOp = (hull[0][vectorDis[firstCol].second.first].x - perMidOp.x)*(hull[0][vectorDis[firstCol].second.first].x - perMidOp.x)
                    + (hull[0][vectorDis[firstCol].second.first].y - perMidOp.y)*(hull[0][vectorDis[firstCol].second.first].y - perMidOp.y);
            if(distFirsToPerOp < distFirsToPer){
                Point temp(perMid.x,perMid.y);
                perMid.x = perMidOp.x;
                perMid.y = perMidOp.y;
                perMidOp.x = temp.x;
                perMidOp.y = temp.y;
            }

            bool sideA = genSide(midP,perMid,hull[0][vectorDis[firstCol].second.first]);

            line(img2, midP, Point(0, midP.y) ,MY_COLOR_BLUE,1);
            line(img2, midP, Point(midP.x, 0),MY_COLOR_BLUE,1);
            line(img2, midP, Point(img2.cols-1, midP.y),MY_COLOR_BLUE,1);
            line(img2, midP, Point(midP.x, img2.rows-1),MY_COLOR_BLUE,1);

            if(sideA == (perMid.y < midP.y)){
                swap(firstCol,lastCol);
            }
            else if(perMid.y >= midP.y){
                swap(firstCol, lastCol);
            }

            line(img2, midP,perMid,MY_COLOR_BLUE,3);
            line(img2, midP,perMidOp,MY_COLOR_RED,3);
            line(img2,borderSobrante[0], borderSobrante[1], MY_COLOR_BLUE,5);
            /* DIBUJANDO CUADRANTES */

            line(img2, midP, Point(0, midP.y) ,MY_COLOR_BLUE,1);
            line(img2, midP, Point(midP.x, 0),MY_COLOR_BLUE,1);
            line(img2, midP, Point(img2.cols-1, midP.y),MY_COLOR_BLUE,1);
            line(img2, midP, Point(midP.x, img2.rows-1),MY_COLOR_BLUE,1);

            double angleError = 28;
            double angleOri = trackGrid->getAnglesBetweenRects(Point(perMid.x-midP.x,perMid.y-midP.y),Point((img2.cols-1)-midP.x,0));

            //cout << "angulo ==> " << trackGrid->getAnglesBetweenRects(Point(perMid.x-midP.x,perMid.y-midP.y),Point((img2.cols-1)-midP.x,0))<< endl;
            Point pFirstCol0 = hull[0][vectorDis[firstCol].second.first];
            Point pFirstCol1 = hull[0][vectorDis[firstCol].second.second];
            Point pLastCol0 = hull[0][vectorDis[lastCol].second.first];
            Point pLastCol1 = hull[0][vectorDis[lastCol].second.second];

            // SE CONSIDERA 6 REGIONES EN EL QUE PUEDE ESTAR EL "ANGULO-ORIENTACION"
            // DE ACUERDO A LA REGION PRIMERO ORDENA LOS PUNTOS EXTREMOS DE LA SUPUESTA PRIMERA COLUMNA Y SUPUESTA ULTIMA COLUMNA
            // SEGUNDO SE OBTIENE LA VERDADERA PRIMERA FILA Y VERDADERA SEGUNDA COLUMNA
            // EL FLAG SE USARA MAS ADELANTE PARA LA ORDENACION DE LOS ELEMENTOS QUE CONFORMNA UNA COLUMNA
            if(angleOri >= 0 && angleOri <= angleError){

                if(pFirstCol1.x > pFirstCol0.x) pFirstCol0 = pFirstCol1;
                if(pLastCol1.x > pLastCol0.x) pLastCol0 = pLastCol1;
                if(pLastCol0.y < pFirstCol0.y) swap(firstCol,lastCol);
                flag = 1;
            }else if(angleOri > angleError && angleOri < 180-angleError){

                if(pFirstCol1.y < pFirstCol0.y) pFirstCol0 = pFirstCol1;
                if(pLastCol1.y < pLastCol0.y) pLastCol0 = pLastCol1;
                if(pFirstCol0.x > pLastCol0.x) swap(firstCol,lastCol);
                flag = 0;
            }else if(angleOri >= 180-angleError && angleOri <= 180){

                if(pFirstCol1.x < pFirstCol0.x) pFirstCol0 = pFirstCol1;
                if(pLastCol1.x < pLastCol0.x) pLastCol0 = pLastCol1;
                if(pLastCol0.y > pFirstCol0.y) swap(firstCol,lastCol);
                flag = 2;
            }else if(angleOri < 0 && angleOri >= -angleError){

                if(pFirstCol1.x > pFirstCol0.x) pFirstCol0 = pFirstCol1;
                if(pLastCol1.x > pLastCol0.x) pLastCol0 = pLastCol1;
                if(pLastCol0.y < pFirstCol0.y) swap(firstCol,lastCol);
                flag = 4;
            }else if(angleOri <= -angleError && angleOri > -(180-angleError)){
                if(pFirstCol1.y > pFirstCol0.y) pFirstCol0 = pFirstCol1;
                if(pLastCol1.y > pLastCol0.y) pLastCol0 = pLastCol1;
                if(pLastCol0.x > pFirstCol0.x) swap(firstCol,lastCol);
                flag = 3;
            }else if(angleOri <= -(180-angleError) && angleOri >= -180){
                if(pFirstCol1.x < pFirstCol0.x) pFirstCol0 = pFirstCol1;
                if(pLastCol1.x < pLastCol0.x) pLastCol0 = pLastCol1;
                if(pLastCol0.y > pFirstCol0.y) swap(firstCol,lastCol);
                flag = 5;
            }

            // SE OBTIENE LAS COLUMNAS QUE CONFORMAN EL PATRON
            // SE REALIZA DISTANCIA DE TODOS LOS PUNTOS CONTRA LA RECTA FORMADO POR LOS BORDES DE LA PRIMERA COLUMNA
            // SE ORDENA LAS DISTANCIAS, LOS 2 PRIMEROS SON AGREGADOS A LA PRIMERA COLUMNA
            // PARA OBTENER LAS SIGUIENTES FILAS SE AGRUPAN LOS 4 ELEMENTOS SIGUIENTES, Y ASI CONSECUITAVAMENTE

            vector<StruSegme> tempVecCOl(11);
            vecCol = tempVecCOl;
            vector<pair<double,int> > vDisRecPoi;

            vector<Scalar> colors1;
            colors1.push_back(MY_COLOR_GREEN);
            colors1.push_back(MY_COLOR_ORANGE);
            colors1.push_back(MY_COLOR_BLUE);
            colors1.push_back(MY_COLOR_YELLOW);
            colors1.push_back(MY_COLOR_WHITE);
            colors1.push_back(MY_COLOR_RED);

            int index = 0;
            vecCol[index].vectorPoint.push_back(make_pair(hull[0][vectorDis[firstCol].second.first].x,  hull[0][vectorDis[firstCol].second.first].y));
            vecCol[index].vectorPoint.push_back(make_pair(hull[0][vectorDis[firstCol].second.second].x, hull[0][vectorDis[firstCol].second.second].y));
            for(int j = 0; j < (int)keypoints.size(); j++){
                    if(!isEqualP2f(keypoints[j], Point2f(vecCol[index].vectorPoint[0].first,vecCol[index].vectorPoint[0].second))
                            && !isEqualP2f(keypoints[j], Point2f(vecCol[index].vectorPoint[1].first,vecCol[index].vectorPoint[1].second))){
                        double dist = trackGrid->getDistanPointToRect(Point2f(vecCol[index].vectorPoint[0].first,vecCol[index].vectorPoint[0].second),
                                                                      Point2f(vecCol[index].vectorPoint[1].first,vecCol[index].vectorPoint[1].second),keypoints[j]);
                    vDisRecPoi.push_back(make_pair(abs(dist),j));
                }
            }
            vDisRecPoi = trackGrid->sortRectVSPoint(vDisRecPoi,0);
            for(int j = 0; j < 2; j++)
                vecCol[index].vectorPoint.push_back(make_pair(keypoints[vDisRecPoi[j].second].x,keypoints[vDisRecPoi[j].second].y));

            for(int i = 1, k = 2; i < 6; i++, k+=4){
                for(int j = k; j < k + 4; j++){
                    circle(img2, keypoints[vDisRecPoi[j].second],5,colors1[i%6],CV_FILLED,8,0);
                    vecCol[i].vectorPoint.push_back(make_pair(keypoints[vDisRecPoi[j].second].x, keypoints[vDisRecPoi[j].second].y));
                }
            }

            index = 10;
            vecCol[index].vectorPoint.push_back(make_pair(hull[0][vectorDis[lastCol].second.first].x,  hull[0][vectorDis[lastCol].second.first].y));
            vecCol[index].vectorPoint.push_back(make_pair(hull[0][vectorDis[lastCol].second.second].x, hull[0][vectorDis[lastCol].second.second].y));
            vDisRecPoi.clear();
            for(int j = 0; j < (int)keypoints.size(); j++){
                    if(!isEqualP2f(keypoints[j], Point2f(vecCol[index].vectorPoint[0].first,vecCol[index].vectorPoint[0].second))
                            && !isEqualP2f(keypoints[j], Point2f(vecCol[index].vectorPoint[1].first,vecCol[index].vectorPoint[1].second))){
                        double dist = trackGrid->getDistanPointToRect(Point2f(vecCol[index].vectorPoint[0].first,vecCol[index].vectorPoint[0].second),
                                                                      Point2f(vecCol[index].vectorPoint[1].first,vecCol[index].vectorPoint[1].second),keypoints[j]);
                    vDisRecPoi.push_back(make_pair(abs(dist),j));
                }
            }
            vDisRecPoi = trackGrid->sortRectVSPoint(vDisRecPoi,0);
            for(int j = 0; j < 2; j++){
                vecCol[index].vectorPoint.push_back(make_pair(keypoints[vDisRecPoi[j].second].x,keypoints[vDisRecPoi[j].second].y));
            }

            for(int i = 9, k = 2; i >= 6; i--, k+=4){
                for(int j = k; j < k + 4; j++){
                    circle(img2, keypoints[vDisRecPoi[j].second],5,colors1[i%6],CV_FILLED,8,0);
                    vecCol[i].vectorPoint.push_back(make_pair(keypoints[vDisRecPoi[j].second].x, keypoints[vDisRecPoi[j].second].y));
                }
            }

            for(int i = 0; i < 4; i++){
                circle(img2, Point2f(vecCol[0].vectorPoint[i].first, vecCol[0].vectorPoint[i].second) ,5,colors1[0],CV_FILLED,8,0);
                circle(img2, Point2f(vecCol[10].vectorPoint[i].first, vecCol[10].vectorPoint[i].second), 5,colors1[4],CV_FILLED,8,0);
            }
            sortPoints(vecCol,keypoints,flag);
        }
    }

    //imshow("convex hull ", img2);
    visualizer->visualizeImage(PROC9, ImageHelper::convertMatToQimage(img2), "ConvexHull, Firt&Last Row, Orientation");
    drawZigZagPattern(keypoints,vecCol,img,2,flag);
    return true;
}

bool PatternDetector::trackingRingsPoints(vector<Point2f> &keypoints){
    if(keypoints.size() != numCols * numRows) {
        return false;
    }

    // Declaracion de vectores de colores
    vector<Scalar> colors;
    colors.push_back(MY_COLOR_RED);
    colors.push_back(MY_COLOR_BLUE);
    colors.push_back(MY_COLOR_YELLOW);
    colors.push_back(MY_COLOR_GREEN);
    colors.push_back(MY_COLOR_ORANGE);
    colors.push_back(MY_COLOR_WHITE);

    // En esta parte se empieza a utilizar el convexhull para hallar los segmentos de arriba y abajo
    if(keypoints.size() > 0) {

        Mat imgCH = img.clone();
        vector<vector<Point2f> > keys;
        keys.push_back(keypoints);
        vector<vector<Point2f> > hull(1);
        convexHull(Mat(keypoints), hull[0], false);

        //Obteniendo las esquinas del convexhull en el patron
        vector<int> posCornes = trackGrid->getPosCornes(hull);

        // escribiendo esquinas del convexhull que son puntos consecutivos de un cuadrilatero
        for(int i = 0; i < (int)posCornes.size(); i++){
            circle(imgCH, hull[0][posCornes[i]], 10, colors[i], CV_FILLED,8,0);
        }
        visualizer->visualizeImage(PROC5, ImageHelper::convertMatToQimage(imgCH), "Convex Hull");

        vector<pair<Point2f,Point2f> > extremosUpDown;
        // Hallando extremos, arriba y abajo
        for(int i = 0; i < (int)posCornes.size(); i++){
            extremosUpDown.push_back(make_pair(hull[0][posCornes[i]],hull[0][posCornes[(i+1) % 4]]));
        }

        // Hallando una recta con 6 puntos en su contenido extremosUpDown
        vector<vector<pair<float,float> > > ans;
        for(int i=0;i<(int)extremosUpDown.size();i++){
            Point2f A = extremosUpDown[i].first;
            Point2f B = extremosUpDown[i].second;
            Point2f P;
            // Interseccion de la recta AB con el punto P
            vector<pair<float,float> > aux;
            for(int k=0;k<(int)keypoints.size();k++){

                // Vemos que no sean los mismo puntos para evitar overflow
                if( (keypoints[k].x == A.x && keypoints[k].y == A.y ) || (keypoints[k].x == B.x && keypoints[k].y == B.y )) continue;
                P = keypoints[k];
                // Hallando la distancia del punto P a la recta AB
                double numerador = (P.x-A.x) * (B.y-A.y) - (P.y-A.y) * (B.x-A.x);
                double denominador = sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
                double distancia = numerador / denominador;
                if(abs((int)distancia) < 6){ // se escoge 6 como tolerancia de precision
                    aux.push_back(make_pair(keypoints[k].x,keypoints[k].y));
                }
            }
            aux.push_back(make_pair(A.x,A.y));
            aux.push_back(make_pair(B.x,B.y));

            if((int)aux.size()==4){
                //Ordenando Ascendentemente x, descendentemente y
                sort(aux.begin(),aux.end(),cmp);
                ans.push_back(aux);
            }
        }

        vector<pair<float,float> > SortPoints;
        stack<vector<pair<float,float> > > pila;
        // escribir lineas de colores
        if(ans.size()>1){

            Point2f PPP = Point2f(ans[0][0].first,ans[0][0].second);
            for(int j=0;j<min((int)ans[0].size(),(int)ans[1].size());j++){
                // Escribiendo los puntos extremos y el segmento entre ellos
                circle(img, Point2f(ans[0][j].first,ans[0][j].second), 5, colors[j%colors.size()], CV_FILLED,8,0);
                circle(img, Point2f(ans[1][j].first,ans[1][j].second), 5, colors[j%colors.size()], CV_FILLED,8,0);
                line(img,Point2f(ans[0][j].first,ans[0][j].second),Point2f(ans[1][j].first,ans[1][j].second),colors[j%colors.size()]);

                SortPoints.push_back(make_pair(ans[0][j].first,ans[0][j].second));
                SortPoints.push_back(make_pair(ans[1][j].first,ans[1][j].second));

                vector<pair<float,Point2f> > distanciaRecta; // Distancia a la recta AB del punto P
                // Hallando los puntos de la recta AB
                Point2f A =  Point2f(ans[0][j].first,ans[0][j].second);
                Point2f B =  Point2f(ans[1][j].first,ans[1][j].second);
                Point2f P;
                // Keypoints tiene todos los puntos del patron
                for(int k=0;k<(int)keypoints.size();k++){
                    //Vemos que no sean los mismo puntos para evitar overflow
                    if( (keypoints[k].x == A.x && keypoints[k].y == A.y ) || (keypoints[k].x == B.x && keypoints[k].y == B.y )) continue;
                    P = keypoints[k];
                    // Hallando la distancia del punto P a la recta AB
                    double numerador = (P.x-A.x) * (B.y-A.y) - (P.y-A.y) * (B.x-A.x);
                    double denominador = sqrt((B.x - A.x)*(B.x - A.x) + (B.y - A.y)*(B.y - A.y));
                    double distancia = numerador / denominador;
                    distanciaRecta.push_back(make_pair(abs((float)distancia),P));
                }

                // Ordenamos las distancias, para escoger los 3 mas cercanos
                sort(distanciaRecta.begin(),distanciaRecta.end(),cmp2);
                for(int i=0;i<3;i++){
                    SortPoints.push_back(make_pair(distanciaRecta[i].second.x,distanciaRecta[i].second.y));
                    circle(img, distanciaRecta[i].second, 5, colors[j%colors.size()], CV_FILLED,8,0);
                }

                circle(img, Point(ans[1][j].first,ans[1][j].second), 10, CV_RGB(0,0,0), CV_FILLED,8,0);
                sort(SortPoints.rbegin(),SortPoints.rend(),cmp3);
                // Almacenando los puntos de una recta
                pila.push(SortPoints);
                SortPoints.clear();

                // Escribiendo linea para la siguiente columna del patron
                line(img,Point2f(ans[0][j].first,ans[0][j].second),PPP,colors[(j-1+colors.size())%colors.size()]);
                PPP = Point2f(ans[1][j].first,ans[1][j].second);
            }

            // Escribiendo las rectas de manera descendente
            int counter = 0;  // Contador para etiquetar los puntos
            keypoints.clear();
            // Extraendo los elementos de la pila
            while(!pila.empty()){
                // Escribiendo numeros
                for(int i=0;i<pila.top().size();i++){
                    stringstream sstr;
                    sstr<<counter;
                    counter++;
                    cv::putText(img,sstr.str(),Point2f(pila.top()[i].first,pila.top()[i].second),cv::FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255),2);
                    keypoints.push_back(Point2f(pila.top()[i].first,pila.top()[i].second));
                }
                pila.pop();
            }
        }
        imshow("RESULTADO FINAL", img);
    }

    visualizer->visualizeImage(PROCFIN, ImageHelper::convertMatToQimage(img), "Resultado Final");

    return true;
}

void PatternDetector::sortPoints(vector<StruSegme> vectSeg,vector<Point2f> &keyPoints, int flag){
    TrackingGrid* trackingGrid  = new TrackingGrid(0);
    if(flag == 0){
        for(int i = 0;i < (int)vectSeg.size();i++)
            vectSeg[i].vectorPoint = trackingGrid->sortUpToDown(vectSeg[i].vectorPoint);
    }else if(flag == 1){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortRightToLeft(vectSeg[i].vectorPoint);
    }else if(flag == 2){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortLeftToRight(vectSeg[i].vectorPoint);
    }else if(flag == 3){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortDownToUp(vectSeg[i].vectorPoint);
    }else if(flag == 4){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortRightToLeft(vectSeg[i].vectorPoint);
    }else if(flag == 5){
        for(int i = 0; i < (int)vectSeg.size(); i++)
            vectSeg[i].vectorPoint = trackingGrid->sortLeftToRight(vectSeg[i].vectorPoint);
    }

    for(int i = vectSeg.size()-1, k = 0; i >= 0; i--){
        for(int j = 0; j < (int)vectSeg[i].vectorPoint.size(); j++, k++){
            keyPoints[k] = Point2f(vectSeg[i].vectorPoint[j].first, vectSeg[i].vectorPoint[j].second);
        }
    }
}
