#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "cameracalibrator.h"

class CameraCalibrator;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);    
    ~MainWindow();

    void visualizeMsg(std::string msg);
    void visualizeValue(std::string label, double value);
    void visualizeImage(int id, QImage img, std::string title="");
    void cleanImage(int id);
    void visualizeTimeExec(int totalFrames, int framesAna, double accu ,double timeValue);
    void visualizetimeReal(double tframe);

private slots:
    void on_btnLoadVideo_clicked();
    void on_btnInitProc_clicked();
    void on_rbCircle_clicked();
    void on_rbRing_clicked();
    void closeEvent(QCloseEvent *event);


private:
    Ui::MainWindow *ui;
    CameraCalibrator *calibrator;
    unsigned int pattSelected;
    unsigned int currCalibrator;
    unsigned int currFrameSelector;

    //void activateCalibrationParams(bool status);
};

#endif // MAINWINDOW_H
