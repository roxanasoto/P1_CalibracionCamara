#ifndef TRACKINGGRID_H
#define TRACKINGGRID_H

#include <vector>
#include "opencv2/opencv.hpp"

struct StruSegme{
    std::vector<std::pair<float,float> > vectorPoint;
    std::pair<float,float> direc;
};

class TrackingGrid{
    public:
        int nPoints;
        TrackingGrid(int);
        std::vector<std::pair<float, float> > sortUpToDown(std::vector<std::pair<float,float> >);
        std::vector<std::pair<float, float> > sortDownToUp(std::vector<std::pair<float,float> >); // new
        std::vector<std::pair<float, float> > sortLeftToRight(std::vector<std::pair<float,float> >);
        std::vector<std::pair<float, float> > sortRightToLeft(std::vector<std::pair<float,float> >); //new
        std::vector<StruSegme> sortColumnsLeftToRight(std::vector<StruSegme>);
        std::vector<StruSegme> sortColumnsRightToLeft(std::vector<StruSegme>); //new
        std::vector<StruSegme> sortRowsUpToDown(std::vector<StruSegme>);
        std::vector<StruSegme> sortRowsDownToUp(std::vector<StruSegme>);

        std::vector<std::pair<int,float> > sortBySecond(std::vector<std::pair<int,float> >);
        cv::Point2f getCentroide(std::vector<cv::Point2f>);
        std::vector<StruSegme> getRowsByGridUsingPendents(std::vector<cv::Point>,int,int,float);
        std::vector<std::pair<int,int> > transformVectorPointToPair(std::vector<cv::Point>);                
        std::vector<int> getPosCornes(std::vector<std::vector<cv::Point2f> >);
        std::vector<std::pair<double, std::pair<int,int> > > sortByDistWithPoint(std::vector<std::pair<double, std::pair<int,int> > >);
        std::vector<std::pair<double,int> > sortRectVSPoint(std::vector<std::pair<double,int> >,int);
        double getAnglesBetweenRects(cv::Point, cv::Point);
        double getDistanPointToRect(cv::Point,cv::Point,cv::Point);
};

#endif // TRACKINGGRID_H
