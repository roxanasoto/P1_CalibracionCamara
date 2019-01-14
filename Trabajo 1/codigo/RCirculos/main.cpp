#include <QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

//#include <stdio.h>
//#include <math.h>
//#include <minpack.h>
//#include <iostream>
//#define real __minpack_real__

//using namespace std;

//void fcn1(const int*,const int*,const real*,real*,int*);

//real computarFunction(real y,real x0,real x1,real x2,real tmp1,real tmp2,real tmp3){
//    return y - (x0 + tmp1*x1/tmp2 - tmp3*x2);
//}

//void fcn1(const int* m,const int* n, const real *x, real* fvec, int *iflag){
//    real y[15]={1.4e-1,1.8e-1,2.2e-1,2.5e-1,2.9e-1,3.2e-1,3.5e-1,3.9e-1,
//          3.7e-1,5.8e-1,7.3e-1,9.6e-1,1.34e0,2.1e0,4.39e0};
//    cout << " x[0] = " << x[0] << endl;
//    for (int i=0; i< *m; i++){
//        real tmp1 = i+1;
//        real tmp2 = 15 - i;
//        real tmp3 = tmp1 < tmp2 ? tmp1 : tmp2;
//        fvec[i] = y[i] -(x[0] + tmp1*x[1]/tmp2 - tmp3*x[2]);
//        fvec[i] = computarFunction(y[i],x[0],x[1],x[2],tmp1,tmp2,tmp3);
//      }
//}

//int main(){
//    int m = 15, n = 3, one = 1, info;
//    real ftol, fnorm;
//    int lwa = m*n + 5*n + m + 10;
//    int* iwa = new int[n];
//    real* x = new real[n];
//    real* fvec = new real[m];
//    real* wa = new real[lwa];

//    x[0] = 0.e0; x[1] = 1.e0; x[2] = 1.e0;

//    ftol = sqrt(__minpack_func__(dpmpar)(&one));
//    __minpack_func__(lmdif1)(&fcn1, &m, &n, x, fvec, &ftol, &info, iwa, wa, &lwa);
//    fnorm = __minpack_func__(enorm)(&m, fvec);

//    printf("      FINAL L2 NORM OF THE RESIDUALS%15.7f\n\n",fnorm);
//    printf("      EXIT PARAMETER                %10i\n\n", info);
//    printf("      FINAL APPROXIMATE SOLUTION\n\n %15.7f%15.7f%15.7f\n",
//       x[0], x[1], x[2]);
//    return 0;
//}
