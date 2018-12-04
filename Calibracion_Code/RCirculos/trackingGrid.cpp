#ifndef TRACKINGGRID_CPP
#define TRACKINGGRID_CPP
#include "trackingGrid.h"
#include "Geometria.h"
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace cv;

bool compareUpToDown(const pair<int,int>&i, const pair<int,int>&j){
    return i.second < j.second;
}

bool compareDownToUp(const pair<int,int>&i, const pair<int,int>&j){
    return i.second > j.second;
}

bool compareLeftToRight(const pair<int,int>&i, const pair<int,int>&j){
    return i.first < j.first;
}

bool compareRightToLeft(const pair<int,int>&i, const pair<int,int>&j){
    return i.first > j.first;
}

bool compareBySecond(const pair<int,int>&i, const pair<int,int>&j){
    return i.second < j.second;
}

bool compareSortColumsLeftToRight(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].first < j.vectorPoint[0].first;
}

bool compareSortColumsRightToLeft(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].first > j.vectorPoint[0].first;
}

bool compareSortRowsUpToDown(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].second < j.vectorPoint[0].second;
}

bool compareSortRowsDownToUp(const StruSegme&i, const StruSegme&j){
    return i.vectorPoint[0].second > j.vectorPoint[0].second;
}

bool compareDistWithPoint(const pair<double, pair<int,int> >&i, const pair<double, pair<int,int> >&j){
    return i.first < j.first;
}

bool compareByDist(const pair<double,int>&i, const pair<double,int>&j){
    return i.first < j.first;
}

TrackingGrid::TrackingGrid(int n){
    nPoints = n;
}

vector<pair<float,float> > TrackingGrid::sortUpToDown(vector<pair<float,float> > array){
    sort(array.begin(), array.end(), compareUpToDown);
    return array;
}

vector<pair<float,float> > TrackingGrid::sortDownToUp(vector<pair<float,float> > array){
    sort(array.begin(), array.end(), compareDownToUp);
    return array;
}

vector<pair<float,float> > TrackingGrid::sortLeftToRight(vector<pair<float, float> > array){
    sort(array.begin(), array.end(), compareLeftToRight);
    return array;
}

vector<pair<float,float> > TrackingGrid::sortRightToLeft(vector<pair<float, float> > array){
    sort(array.begin(), array.end(), compareRightToLeft);
    return array;
}

vector<StruSegme> TrackingGrid::sortColumnsLeftToRight(vector<StruSegme> array){
    sort(array.begin(), array.end(), compareSortColumsLeftToRight);
    return array;
}

vector<StruSegme> TrackingGrid::sortColumnsRightToLeft(vector<StruSegme> array){
    sort(array.begin(), array.end(), compareSortColumsRightToLeft);
    return array;
}

vector<StruSegme> TrackingGrid::sortRowsUpToDown(vector<StruSegme> array){
    sort(array.begin(), array.end(), compareSortRowsUpToDown);
    return array;
}

vector<StruSegme> TrackingGrid::sortRowsDownToUp(vector<StruSegme> array){
    sort(array.begin(), array.end(), compareSortRowsDownToUp);
    return array;
}

vector<pair<int,float> > TrackingGrid::sortBySecond(vector<pair<int,float> > array){
    sort(array.begin(), array.end(), compareUpToDown);
    return array;
}

Point2f TrackingGrid::getCentroide(vector<Point2f> array){
    float x = 0, y = 0;
    int n = array.size();
    for(int i = 0; i < n; i++){
        x += array[i].x;
        y += array[i].y;
    }
    return Point2f(x/n, y/n);
}

vector<pair<int,int> > TrackingGrid::transformVectorPointToPair(vector<Point> array){
    vector<pair<int,int> > res;
    for(int i = 0; i < (int)array.size(); i++)
        res.push_back(make_pair(array[i].x, array[i].y));
    return res;
}

vector<StruSegme> TrackingGrid::getRowsByGridUsingPendents(vector<Point> array, int numCols, int numRows, float minDist){
    vector<pair<double,double> > auxCol;
    set<vector<pair<double, double> > > columns2;
    double distancia, distancia2;

    // Obtencion de todas las rectas que tienen el mismo numero de array en la recta (numRows)

    PointFrz A, B, P;
    for(size_t i = 0; i < array.size(); i++){
        for(size_t j = i+1; j < array.size(); j++){
            A.x = array[i].x;
            A.y = array[i].y;
            B.x = array[j].x;
            B.y = array[j].y;
            auxCol.clear();
            for(size_t k = 0; k < array.size(); k++){
                if(k == i || k == j) continue;
                P.x = array[k].x;
                P.y = array[k].y;
                distancia = cross((P - A), (B - A)) / (B - A).mod();
                distancia2 = cross((P - B), (A - B)) / (A - B).mod();

                if(abs(distancia) <= minDist && abs(distancia2) <= minDist) {
                    auxCol.push_back(make_pair(P.x, P.y));
                }
            }
            if((int)auxCol.size() == (numRows - 2)){
                auxCol.push_back(make_pair(array[i].x, array[i].y));
                auxCol.push_back(make_pair(array[j].x, array[j].y));
                sort(auxCol.begin(), auxCol.end());
                columns2.insert(auxCol);
            }
        }
    }

    // Colocamos las columnas en un vector
    vector<vector<pair<double, double> > > rectas;
    for (std::set<vector<pair<double,double> > >::iterator it = columns2.begin(); it != columns2.end(); it++){
        rectas.push_back((*it));
    }

    // Calculamos la pendiente de todas las rectas
    double pendiente = 0;
    vector<pair<double,int> > freq;
    for (int i=0; i<(int)rectas.size(); i++){
        if((rectas[i][0].first - rectas[i][1].first) == 0)
            pendiente = 10000;
        else
            pendiente = (rectas[i][0].second - rectas[i][1].second) * 1.0 / (rectas[i][0].first - rectas[i][1].first);
        freq.push_back(make_pair(pendiente, i));
    }

    sort(freq.begin(), freq.end());
    double dist, diffPen = 0.25;
     vector<vector<pair<double, double> > > rectSelected;
    rectSelected.push_back(rectas[freq[0].second]);
    for(int i=1; i < (int)freq.size(); i++)
    {
        dist = abs(freq[i].first-freq[i-1].first);
        if(dist < diffPen) {
             rectSelected.push_back(rectas[freq[i].second]);
             if((int)rectSelected.size() == numCols)
                 break;
        }
        else {
            if((int)rectSelected.size() < numCols) {
                rectSelected.clear();
                 rectSelected.push_back(rectas[freq[i].second]);
            }
            else
                break;
        }
    }

    vector<StruSegme> res;
    for(int i = 0; i < (int)rectSelected.size(); i++){
        StruSegme tmp;
        vector<pair<float,float> > vecPoin;
        for(int j = 0; j < numRows; j++){
            float x = rectSelected[i][j].first;
            float y = rectSelected[i][j].second;
            vecPoin.push_back(make_pair(x,y));
        }
        tmp.vectorPoint = vecPoin;
        res.push_back(tmp);
    }
    return res;
}


vector<int> TrackingGrid::getPosCornes(vector<vector<Point2f> > hull){
    vector<int> posCornes;
    double piValue = 3.14159265, maxAngle = 148;
    int nHull = (int)hull[0].size();
    for(int i = 0; i < nHull; i++){
        float angle;
        PointFrz A(hull[0][i].x - hull[0][(i==0?nHull:i)-1].x, hull[0][i].y - hull[0][(i==0?nHull:i)-1].y);
        PointFrz B(hull[0][i].x - hull[0][i==nHull-1?0:i+1].x, hull[0][i].y - hull[0][i==nHull-1?0:i+1].y);
        angle = atan2(cross(A,B),dot(A,B)) * 180 / piValue;
        if(abs(angle) <= maxAngle)
            posCornes.push_back(i);
    }
    return posCornes;
}

vector<pair<double, pair<int,int> > > TrackingGrid::sortByDistWithPoint(vector<pair<double, pair<int,int> > > array){
    sort(array.begin(), array.end(), compareDistWithPoint);
    return array;
}

vector<pair<double,int> > TrackingGrid::sortRectVSPoint(vector<pair<double,int> > array, int pos){
    sort(array.begin() + pos, array.end(), compareByDist);
    return array;
}

double TrackingGrid::getAnglesBetweenRects(Point a, Point b){
    PointFrz A(a.x,a.y);
    PointFrz B(b.x,b.y);
    double piValue = 3.14159265;
    return atan2(cross(A,B),dot(A,B)) * 180 / piValue;
}

double TrackingGrid::getDistanPointToRect(Point a,Point b,Point c){
    PointFrz A(a.x,a.y);
    PointFrz B(b.x,b.y);
    PointFrz P(c.x,c.y);
    return cross((P - A), (B - A)) / (B - A).mod();
}

#endif // TRACKINGGRID_CPP

